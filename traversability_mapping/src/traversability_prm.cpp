#include "utility.h"

class TraversabilityPRM : public rclcpp::Node {
private:

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPRMGraph; // publish PRM nodes and edges
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPRMPath; // path extracted from roadmap
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGlobalPath; // path is published in pose array format
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubSingleSourcePaths; // publish paths to al states in roadmap

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPRMNodes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPRMGraph;

    rclcpp::Subscription<elevation_msgs::msg::OccupancyElevation>::SharedPtr subElevationMap; // 2d local height map from mapping package

    elevation_msgs::msg::OccupancyElevation elevationMap; // this is received from mapping package. it is a 2d local map that includes height info

    float map_min[3]; // 0 - x, 1 - y, 2 - z
    float map_max[3];
 
    ///////////// Planner ////////////
    vector<state_t*> nodeList;
    vector<state_t*> pathList;

    // (compositeCost, state) entry in bfsSearch's priority queue. std::greater on a pair orders
    // by the float first and only falls back to the pointer to break exact ties, which keeps
    // the ordering total and deterministic.
    typedef std::pair<float, state_t*> QueueEntry;

    nav_msgs::msg::Path globalPath;
    nav_msgs::msg::Path displayGlobalPath;

    bool planningFlag; // true only when a NEW PRM plan is required

    // Final-goal / intermediate-waypoint state.
    // The final goal stays fixed, while bfsSearch() selects one reachable PRM
    // endpoint at a time. That endpoint is latched until the robot reaches it.
    bool haveFinalGoal;
    bool activeWaypoint;
    double waypointX;
    double waypointY;
    double waypointZ;

    // Replan after getting this close to the currently latched PRM endpoint.
    // A repeated /prm_goal closer than newGoalDistanceThreshold to the stored
    // final goal is treated as the same goal and does NOT trigger replanning.
    double waypointReachDistance;
    double newGoalDistanceThreshold;

    state_t *robotState;
    state_t *goalState;
    state_t *nearestGoalState;
    state_t *mapCenter;

    kdtree_t *kdtree;

    bool costUpdateFlag[NUM_COSTS];

    std::mutex mtx;

    rclcpp::Clock::SharedPtr steadyClock;

public:
    explicit TraversabilityPRM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
        Node("traversability_prm", options),
        planningFlag(false),
        haveFinalGoal(false),
        activeWaypoint(false),
        waypointX(0.0),
        waypointY(0.0),
        waypointZ(0.0),
        waypointReachDistance(1.0),
        newGoalDistanceThreshold(0.2){
        loadRuntimeConfig(*this);

        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        steadyClock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        robotState = new state_t;
        goalState = new state_t;
        mapCenter = new state_t;

        // Optional ROS parameters; defaults implement the requested behavior.
        this->get_parameter_or("waypoint_reach_distance", waypointReachDistance, 1.0);
        this->get_parameter_or("new_goal_distance_threshold", newGoalDistanceThreshold, 0.2);

        subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            goalTopic, 5, std::bind(&TraversabilityPRM::goalPosHandler, this, std::placeholders::_1));
        subElevationMap = this->create_subscription<elevation_msgs::msg::OccupancyElevation>(
            elevationTopic, 5, std::bind(&TraversabilityPRM::elevationMapHandler, this, std::placeholders::_1));

        pubPRMGraph = this->create_publisher<visualization_msgs::msg::MarkerArray>(prmGraphTopic, 5);
        pubPRMPath = this->create_publisher<visualization_msgs::msg::MarkerArray>(prmPathTopic, 5);
        pubSingleSourcePaths = this->create_publisher<visualization_msgs::msg::MarkerArray>(prmSingleSourcePathsTopic, 5);

        pubCloudPRMNodes = this->create_publisher<sensor_msgs::msg::PointCloud2>(prmNodesTopic, 5);
        pubCloudPRMGraph = this->create_publisher<sensor_msgs::msg::PointCloud2>(prmGraphCloudTopic, 5);

        // Same QoS as the mapping node's grid outputs, so a nav2 global-planner plugin that
        // wraps this node (see traversability_global_planner in this package) always has a
        // path to hand back the instant it is asked, even before the first bfsSearch() call.
        rclcpp::QoS pathQoS(5);
        pathQoS.reliable();
        pathQoS.transient_local();
        pubGlobalPath = this->create_publisher<nav_msgs::msg::Path>(globalPathTopic, pathQoS);

        allocateMemory(); 
    }

    ~TraversabilityPRM(){}

    void allocateMemory(){

        // 2D, not 3D. The map is 2.5D - one height per (x,y) - so the roadmap is a ground graph
        // and every nearest/near query on it is really a ground-plane query.
        kdtree = kd_create(2);

        for (int i = 0; i < NUM_COSTS; ++i)
            costUpdateFlag[i] = false;
        for (size_t i = 0; i < costHierarchy.size(); ++i)
            costUpdateFlag[costHierarchy[i]] = true;
    }

    void elevationMapHandler(const elevation_msgs::msg::OccupancyElevation::ConstSharedPtr mapMsg){

        std::lock_guard<std::mutex> lock(mtx);

        elevationMap = *mapMsg;

        updateMapBoundary();

        updateCostMap();

        buildRoadMap();
    }

    void updateMapBoundary(){
        map_min[0] = elevationMap.occupancy.info.origin.position.x; 
        map_min[1] = elevationMap.occupancy.info.origin.position.y;
        map_min[2] = elevationMap.occupancy.info.origin.position.z;
        map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.width; 
        map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.height; 
        map_max[2] = elevationMap.occupancy.info.origin.position.z;
    }

    void updateCostMap(){
        int sizeMap = elevationMap.occupancy.data.size();
        int inflationSize = int(costmapInflationRadius / elevationMap.occupancy.info.resolution);
        for (int i = 0; i < sizeMap; ++i) {
            int idX = int(i % elevationMap.occupancy.info.width);
            int idY = int(i / elevationMap.occupancy.info.width);
            // Threshold before inflating. occupancy is a graded 0..100 score now, so the old
            // "> 0" test fired on virtually every observed cell and would inflate the entire map.
            if (elevationMap.occupancy.data[i] >= obstacleOccupancyThreshold){
                for (int m = -inflationSize; m <= inflationSize; ++m) {
                    for (int n = -inflationSize; n <= inflationSize; ++n) {
                        int newIdX = idX + m;
                        int newIdY = idY + n;
                        if (newIdX < 0 || newIdX >= (int)elevationMap.occupancy.info.width || newIdY < 0 || newIdY >= (int)elevationMap.occupancy.info.height)
                            continue;
                        int index = newIdX + newIdY * elevationMap.occupancy.info.width;
                        elevationMap.cost_map[index] = std::max(elevationMap.cost_map[index], std::sqrt(float(m*m+n*n)));
                    }
                }
            }
        }
    }

    void buildRoadMap(){
        // 1. Keep expanding/updating the rolling roadmap continuously.
        generateSamples();

        // 2. Update robot pose, node heights and graph edges as the map changes.
        // updateStatesAndEdges() calls getRobotState() first.
        updateStatesAndEdges();

        // No goal has been received yet: only maintain/visualize the roadmap.
        if (!haveFinalGoal){
            publishPRM();
            publishRoadmap2Cloud();
            return;
        }

        // If the robot is already at the actual final goal, finish the whole
        // waypoint sequence instead of selecting yet another PRM endpoint.
        double finalDx = robotState->x[0] - goalState->x[0];
        double finalDy = robotState->x[1] - goalState->x[1];
        double finalGoalDistance = sqrt(finalDx*finalDx + finalDy*finalDy);

        if (finalGoalDistance <= waypointReachDistance){
            if (activeWaypoint || planningFlag){
                RCLCPP_INFO(this->get_logger(), "Final PRM goal reached (%.2f m).", finalGoalDistance);
            }
            activeWaypoint = false;
            planningFlag = false;
        }

        // A PRM endpoint is currently latched. Do NOT run bfsSearch() while
        // travelling toward it. Only unlock planning after reaching it.
        if (activeWaypoint){
            double dx = robotState->x[0] - waypointX;
            double dy = robotState->x[1] - waypointY;
            double waypointDistance = sqrt(dx*dx + dy*dy);

            if (waypointDistance <= waypointReachDistance){
                RCLCPP_INFO(this->get_logger(), "Reached latched PRM waypoint [%.2f, %.2f] (%.2f m). Replanning toward final goal [%.2f, %.2f].",
                         waypointX, waypointY, waypointDistance,
                         goalState->x[0], goalState->x[1]);

                activeWaypoint = false;
                planningFlag = true;
            }
        }

        // Plan only when there is no active intermediate waypoint. On success,
        // bfsSearch() latches the newly selected endpoint before returning.
        if (planningFlag && !activeWaypoint){
            if (bfsSearch()){
                planningFlag = false;
                publishCurrentPath();
            }
            else{
                // Keep planningFlag true so the next map update can try again
                // after more samples / traversability information arrive.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *steadyClock, 2000, "PRM could not find a reachable waypoint yet; will retry on the next map update.");
            }
        }
        else if (activeWaypoint){
            // Re-publish the SAME stored path for downstream consumers, but do
            // not recompute it. The latched waypoint therefore stays fixed.
            publishCurrentPath();
        }

        // 4. Visualize roadmap and currently stored path.
        publishPRM();

        // 5. Convert PRM graph into point cloud for external usage.
        publishRoadmap2Cloud();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Planner /////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void goalPosHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal){

        // If /prm_goal is periodically republished, do not let the same final
        // goal cancel the currently latched intermediate waypoint.
        if (haveFinalGoal){
            double dx = goal->pose.position.x - goalState->x[0];
            double dy = goal->pose.position.y - goalState->x[1];
            double goalChange = sqrt(dx*dx + dy*dy);

            if (goalChange < newGoalDistanceThreshold){
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *steadyClock, 2000, "Ignoring repeated /prm_goal; current final goal is unchanged.");
                return;
            }
        }

        // Store the FINAL requested destination. This remains unchanged while
        // the planner advances through one reachable PRM waypoint at a time.
        goalState->x[0] = goal->pose.position.x;
        goalState->x[1] = goal->pose.position.y;
        goalState->x[2] = goal->pose.position.z;

        haveFinalGoal = true;

        // A genuinely new final goal cancels the old intermediate waypoint and
        // requests one fresh plan on the next elevation-map update.
        activeWaypoint = false;
        planningFlag = true;

        RCLCPP_INFO(this->get_logger(), "New FINAL PRM goal received: [%.2f, %.2f, %.2f]",
                 goalState->x[0], goalState->x[1], goalState->x[2]);
    }

    
    
    bool bfsSearch(){

        pathList.clear();
        globalPath.poses.clear();
        // 1. reset costs, parents and settled flags
        for (size_t i = 0; i < nodeList.size(); ++i){
            for (int j = 0; j < NUM_COSTS; ++j)
                nodeList[i]->costsToRoot[j] = FLT_MAX;
            nodeList[i]->parentState = NULL;
            nodeList[i]->closedFlag = false;
        }
        // 2. find the state that is the closest to the robot
        state_t *startState = NULL;

        vector<state_t*> nearRobotStates;
        getNearStates(robotState, nearRobotStates, 2);
        if (nearRobotStates.size() == 0)
            return false;

        float nearRobotDist = FLT_MAX;
        for (size_t i = 0; i < nearRobotStates.size(); ++i){
            float dist = distance(nearRobotStates[i]->x, robotState->x);
            if (dist < nearRobotDist && nearRobotStates[i]->neighborList.size() != 0){
                nearRobotDist = dist;
                startState = nearRobotStates[i];
            }
        }

        if (startState == NULL || startState->neighborList.size() == 0)
            return false;

        for (int i = 0; i < NUM_COSTS; ++i)
            startState->costsToRoot[i] = 0;

        // 3. Resolve the goal node up front, so the search can stop as soon as it is settled.
        nearestGoalState = getNearestState(goalState);
        bool earlyExitEnabled = (nearestGoalState != NULL && nearestGoalState != startState);

        // 4. Dijkstra over the roadmap, ordered by compositeCost.
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry> > Queue;
        Queue.push(QueueEntry(0.0f, startState));

        while(!Queue.empty() && rclcpp::ok()){

            state_t *fromState = Queue.top().second;
            Queue.pop();

            if (fromState->closedFlag) // stale heap entry, superseded by a cheaper pop
                continue;
            fromState->closedFlag = true;

            // stop searching if minimum cost path to goal is found
            if (earlyExitEnabled && fromState == nearestGoalState)
                break;

            // loop through all neighbors of this state
            for (size_t i = 0; i < fromState->neighborList.size(); ++i){
                state_t *toState = fromState->neighborList[i].neighbor;

                if (toState->closedFlag) // already settled, its cost cannot improve
                    continue;

                float newCosts[NUM_COSTS];
                for (int j = 0; j < NUM_COSTS; ++j)
                    newCosts[j] = fromState->costsToRoot[j] + fromState->neighborList[i].edgeCosts[j];

                float newCost = compositeCost(newCosts);

                if (newCost < compositeCost(toState->costsToRoot)) {
                    updateCosts(fromState, toState, i);
                    toState->parentState = fromState;
                    Queue.push(QueueEntry(newCost, toState));
                }
            }
        }

        // 5. If the goal node was never reached, fall back to the reachable roadmap node
        // closest to the goal.
        if (nearestGoalState != NULL && nearestGoalState->parentState == NULL){
            vector<state_t*> nearGoalStates;
            getNearStates(nearestGoalState, nearGoalStates, 20);
            float nearGoalDist = FLT_MAX;
            for (size_t i = 0; i < nearGoalStates.size(); ++i){
                float dist = distance(nearGoalStates[i]->x, goalState->x);
                if (dist < nearGoalDist && nearGoalStates[i]->parentState != NULL){
                    nearGoalDist = dist;
                    nearestGoalState = nearGoalStates[i];
                }
            }
        }
        // the nearest goal state is invalid
        if (nearestGoalState == NULL || nearestGoalState->parentState == NULL) // no path to the nearestGoalState is found
            return false;

        // 6. Extract path
        state_t *thisState = nearestGoalState;
        while (thisState->parentState != NULL){
            pathList.insert(pathList.begin(), thisState);
            thisState = thisState->parentState;
        }
        pathList.insert(pathList.begin(), robotState); // add current robot state

        // 7. Smooth path
        smoothPath();

        // Latch this exact PRM endpoint.
        waypointX = nearestGoalState->x[0];
        waypointY = nearestGoalState->x[1];
        waypointZ = nearestGoalState->x[2];
        activeWaypoint = true;

        RCLCPP_INFO(this->get_logger(), "Latched PRM waypoint: [%.2f, %.2f, %.2f] -> final goal [%.2f, %.2f, %.2f]",
                 waypointX, waypointY, waypointZ,
                 goalState->x[0], goalState->x[1], goalState->x[2]);

        return true;
    }

    // The single scalar bfsSearch minimizes. Used for BOTH the priority queue ordering and the
    // relaxation test.
    static float compositeCost(const float costs[NUM_COSTS]){
        if (costs[0] == FLT_MAX) // unreached; avoid overflowing to inf
            return FLT_MAX;
        return pathCostWeightTraversability * (costs[0] / pathCostScaleTraversability)
             + pathCostWeightElevation      * (costs[1] / pathCostScaleElevation)
             + pathCostWeightDistance       * (costs[2] / pathCostScaleDistance);
    }

    void updateCosts(state_t* fromState, state_t* toState, int neighborInd){
        for (int i = 0; i < NUM_COSTS; ++i)
            toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
    }

    void smoothPath(){

        if (pathList.size() <= 1)
            return;
        // Cubic Spline
        nav_msgs::msg::Path originPath;
        nav_msgs::msg::Path splinePath;
        
        originPath.header.frame_id = mapFrame;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = mapFrame;

        originPath.poses.clear();

        for (size_t i = 0; i < pathList.size(); i++){
            pose.pose.position.x = pathList[i]->x[0];
            pose.pose.position.y = pathList[i]->x[1];
            pose.pose.position.z = pathList[i]->x[2];
            pose.pose.orientation.w = 1.0; // yaw = 0
            originPath.poses.push_back(pose);
        }

        // Defaults match the original ROS-param defaults (points_per_unit=5.0, skip_points=0,
        // use_end_conditions=false, use_middle_conditions=false); see cubic_spline_interpolator.h.
        path_smoothing::CubicSplineInterpolator csi(5.0, 0, false, false);
        csi.interpolatePath(originPath, splinePath);

        globalPath = splinePath;
        displayGlobalPath = globalPath; // displayGlobalPath is only changed during planning
    }    

    void generateSamples(){

        double sampling_start_time = steadyClock->now().seconds();
        while (steadyClock->now().seconds() - sampling_start_time < samplingTimeBudget && rclcpp::ok()){

            state_t* newState = new state_t;

            if (sampleState(newState)){
                // 1.1 Too close discard
                if (nodeList.size() != 0 && stateTooClose(newState) == true){
                    delete newState;
                    continue;
                }
                // 1.2 Save new state and insert to KD-tree
                newState->stateId = nodeList.size(); // mark state ID
                nodeList.push_back(newState);
                insertIntoKdtree(newState);
            }
            else
                delete newState;
        }
    }

    bool stateTooClose(state_t *stateIn){
        // Minimum spacing between roadmap nodes, measured on the GROUND PLANE.
        state_t *nearestState = getNearestState(stateIn);
        if (nearestState == NULL)
            return false;

        return distance2D(stateIn->x, nearestState->x) <= neighborSampleRadius;
    }

    void updateStatesAndEdges(){
        getRobotState();
        // 0. find local map center
        mapCenter->x[0] = (map_min[0] + map_max[0]) / 2;
        mapCenter->x[1] = (map_min[1] + map_max[1]) / 2;
        mapCenter->x[2] = robotState->x[2];
        // 1. add edges for the nodes that are within a certain radius of mapCenter
        vector<state_t*> nearStates;
        getNearStates(mapCenter, nearStates, neighborSearchRadius);
        if (nearStates.size() == 0)
            return;
        // 3. update states height values (because the map is changing all the time)
        for (size_t i = 0; i < nearStates.size(); ++i){
            float thisHeight = getStateHeight(nearStates[i]);
            if (thisHeight != -FLT_MAX) // new height can be -FLT_MAX since the map is shifted a bit every time (round error)
                nearStates[i]->x[2]  = thisHeight;
        }
        // 4. loop through all neighbors
        float edgeCosts[NUM_COSTS];
        neighbor_t thisNeighbor;
        for (size_t i = 0; i < nearStates.size(); ++i){
            for (size_t j = i+1; j < nearStates.size(); ++j){
                // 4.1 height difference larger than threshold, too steep to connect
                if (abs(nearStates[i]->x[2] - nearStates[j]->x[2]) > neighborConnectHeight){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.2 distance larger than x, too far to connect
                float distanceBetween = distance(nearStates[i]->x, nearStates[j]->x);
                if (distanceBetween > neighborConnectRadius || distanceBetween < 0.3){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.3 this edge is connectable
                if(edgePropagation(nearStates[i], nearStates[j], edgeCosts) == true){
                    // even if edge already exists, we still need to update costs (casuse elevation may change)
                    deleteEdge(nearStates[i], nearStates[j]);
                    for (int k = 0; k < NUM_COSTS; ++k)
                        thisNeighbor.edgeCosts[k] = edgeCosts[k];

                    thisNeighbor.neighbor = nearStates[j];
                    nearStates[i]->neighborList.push_back(thisNeighbor);
                    thisNeighbor.neighbor = nearStates[i];
                    nearStates[j]->neighborList.push_back(thisNeighbor);
                }else{ // edge is not connectable, delete old edge if it exists
                    deleteEdge(nearStates[i], nearStates[j]);
                }
            } 
        }
    }

    void deleteEdge(state_t* stateA, state_t* stateB){
        compareState = stateB;
        stateA->neighborList.erase(std::remove_if(stateA->neighborList.begin(), stateA->neighborList.end(), isStateExsiting), stateA->neighborList.end());
        compareState = stateA;
        stateB->neighborList.erase(std::remove_if(stateB->neighborList.begin(), stateB->neighborList.end(), isStateExsiting), stateB->neighborList.end());
    }

    

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]){
        // 0. initialize edgeCosts
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;
        // 1. segment the edge for collision checking
        int steps = floor(distance(state_from->x, state_to->x) / (mapResolution));
        float stepX = (state_to->x[0]-state_from->x[0]) / steps;
        float stepY = (state_to->x[1]-state_from->x[1]) / steps;
        float stepZ = (state_to->x[2]-state_from->x[2]) / steps;
        // 2. allocate memory for a state, this state must be deleted after collision checking
        state_t *stateCurr = new state_t;
        stateCurr->x[0] = state_from->x[0];
        stateCurr->x[1] = state_from->x[1];
        stateCurr->x[2] = state_from->x[2];

        int rounded_x, rounded_y, indexInLocalMap;

        // Elevation cost is the accumulated |height change| ALONG the edge, so it needs the
        // previous step's height; seed it from the start node's cell.
        int fromIndex = (int)((state_from->x[0] - map_min[0]) / mapResolution)
                      + (int)((state_from->x[1] - map_min[1]) / mapResolution) * elevationMap.occupancy.info.width;
        float prevElevation = (fromIndex >= 0 && fromIndex < (int)elevationMap.height.size())
                            ? elevationMap.height[fromIndex] : -FLT_MAX;

        // 3. collision checking loop
        for (int stepCount = 0; stepCount < steps; ++stepCount){
            stateCurr->x[0] += stepX;
            stateCurr->x[1] += stepY;
            stateCurr->x[2] += stepZ;

            rounded_x = (int)((stateCurr->x[0] - map_min[0]) / mapResolution);
            rounded_y = (int)((stateCurr->x[1] - map_min[1]) / mapResolution);
            indexInLocalMap = rounded_x + rounded_y * elevationMap.occupancy.info.width;

            if (isIncollision(rounded_x, rounded_y, indexInLocalMap)){
                delete stateCurr;
                return false;
            }

            // Costs are LINE INTEGRALS along the edge, not endpoint differences.
            if (costUpdateFlag[0]) {
                int occ = elevationMap.occupancy.data[indexInLocalMap];
                // Unknown reads as -1. Left raw it would make unobserved ground the cheapest
                // terrain on the map and actively attract the planner into it.
                if (occ < 0) occ = unknownCellTraversability;
                edgeCosts[0] += float(occ) * mapResolution;
            }

            if (costUpdateFlag[1]) {
                float thisElevation = elevationMap.height[indexInLocalMap];
                if (thisElevation != -FLT_MAX){
                    if (prevElevation != -FLT_MAX)
                        edgeCosts[1] += std::abs(thisElevation - prevElevation);
                    prevElevation = thisElevation;
                }
            }

            // costs propagation
            if (costUpdateFlag[2])
                edgeCosts[2] = edgeCosts[2] + mapResolution; // distance cost
        }
        delete stateCurr;
        return true;
    }

    // Collision check (using rounded index for input)
    bool isIncollision(int rounded_x, int rounded_y, int index){
        if (rounded_x < 0 || rounded_x >= localMapArrayLength ||
            rounded_y < 0 || rounded_y >= localMapArrayLength )
            return true;

        // Test the INFLATED map, not the raw occupancy. updateCostMap spreads every obstacle
        // cell by costmapInflationRadius precisely so the planner keeps the robot's width away
        // from it.
        if (elevationMap.cost_map[index] != 0)
            return true;

        if (planningUnknown == false){
            // stateIn->x is on an unknown grid
            if (elevationMap.height[index] == -FLT_MAX)
                return true;
        }

        return false;
    }

    void getNearStates(state_t *stateIn, vector<state_t*>& vectorNearStatesOut, double radius){
        kdres_t *kdres = kd_nearest_range (kdtree, stateIn->x, radius);
        vectorNearStatesOut.clear();
        // Create the vector data structure for storing the results
        int numNearVertices = kd_res_size (kdres);
        if (numNearVertices == 0) {
            kd_res_free (kdres);
            return;
        }
        // Place pointers to the near vertices into the vector 
        kd_res_rewind (kdres);
        while (!kd_res_end(kdres)) {
            state_t *stateCurr = (state_t *) kd_res_item_data (kdres);
            vectorNearStatesOut.push_back(stateCurr);
            kd_res_next(kdres);
        }
        // Free temporary memory
        kd_res_free (kdres);
    }


    bool sampleState(state_t *stateCurr){
        // random x and y
        for (int i = 0; i < 2; ++i)
            stateCurr->x[i] = (double)rand()/(RAND_MAX + 1.0)*(map_max[i] - map_min[i]) 
                - (map_max[i] - map_min[i])/2.0 + (map_max[i] + map_min[i])/2.0;
        // random heading
        stateCurr->theta = (double)rand()/(RAND_MAX + 1.0) * 2 * M_PI - M_PI;
        // collision checking before getting height info
        if (isIncollision(stateCurr))
            return false;
        // z is not random since the robot is constrained to move on the ground
        stateCurr->x[2] = getStateHeight(stateCurr);
        if (stateCurr->x[2] == -FLT_MAX)
            return false;

        return true;
    }

    double getStateHeight(state_t* stateIn){
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        return elevationMap.height[rounded_x + rounded_y * elevationMap.occupancy.info.width];
    }

    // Collision check (using state for input)
    bool isIncollision(state_t* stateIn){
        // if the state is outside the map, discard this state
        if (stateIn->x[0] <= map_min[0] || stateIn->x[0] >= map_max[0] 
            || stateIn->x[1] <= map_min[1] || stateIn->x[1] >= map_max[1])
            return true;
        // if the distance to the nearest obstacle is less than xxx, in collision
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        int index = rounded_x + rounded_y * elevationMap.occupancy.info.width;

        // inflated map - see the index-based overload above
        if (elevationMap.cost_map[index] != 0)
            return true;

        if (planningUnknown == false){
            // stateIn->x is on an unknown grid
            if (elevationMap.height[index] == -FLT_MAX)
                return true;
        }

        return false;
    }

    void insertIntoKdtree(state_t *stateCurr){
        kd_insert(kdtree, stateCurr->x, stateCurr);
    }

    state_t* getNearestState(state_t *stateIn){
        kdres_t *kdres = kd_nearest(kdtree, stateIn->x);
        if (kd_res_end (kdres)){
            kd_res_free (kdres);
            return NULL;
        }
        state_t* nearestState = (state_t*) kd_res_item_data(kdres);
        kd_res_free (kdres);
        return nearestState;
    }

    // Euclidean distance between two samples (3D - correct measure for edge LENGTH; an edge
    // climbing a slope really is longer).
    float distance(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) +
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }

    // Ground-plane distance - the right measure for node SPACING.
    float distance2D(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) +
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]));
    }


    void publishPRM(){        

        // Path
        if (pubPRMPath->get_subscription_count() != 0){

            visualization_msgs::msg::MarkerArray markerArray;
            geometry_msgs::msg::Point p;

            // path visualization
            visualization_msgs::msg::Marker markerPath;
            markerPath.header.frame_id = mapFrame;
            markerPath.header.stamp = this->now();
            markerPath.action = visualization_msgs::msg::Marker::ADD;
            markerPath.type = visualization_msgs::msg::Marker::LINE_STRIP;
            markerPath.ns = "path";
            markerPath.id = 0;
            markerPath.scale.x = 0.2;
            markerPath.color.r = 0.0; markerPath.color.g = 0; markerPath.color.b = 1.0;
            markerPath.color.a = 1.0;

            for (size_t i = 0; i < displayGlobalPath.poses.size(); ++i){
                p.x = displayGlobalPath.poses[i].pose.position.x;
                p.y = displayGlobalPath.poses[i].pose.position.y;
                p.z = displayGlobalPath.poses[i].pose.position.z + 0.3;
                markerPath.points.push_back(p);
            }
            
            // goal point visualization
            visualization_msgs::msg::Marker markerGoal;
            markerGoal.header.frame_id = mapFrame;
            markerGoal.header.stamp = this->now();
            markerGoal.action = visualization_msgs::msg::Marker::ADD;
            markerGoal.type= visualization_msgs::msg::Marker::SPHERE_LIST;
            markerGoal.ns = "goal";
            markerGoal.id = 1;
            markerGoal.scale.x = 0.5;
            markerGoal.color.r = 0.0; markerGoal.color.g = 0.0; markerGoal.color.b = 1.0;
            markerGoal.color.a = 1.0;

            if (displayGlobalPath.poses.size() != 0){
                p.x = displayGlobalPath.poses.back().pose.position.x;
                p.y = displayGlobalPath.poses.back().pose.position.y;
                p.z = displayGlobalPath.poses.back().pose.position.z + 0.3;
                markerGoal.points.push_back(p);
            }

            // push to markerarray and publish
            markerArray.markers.push_back(markerPath);
            markerArray.markers.push_back(markerGoal);
            pubPRMPath->publish(markerArray);
        }


        if (pubPRMGraph->get_subscription_count() != 0){

            visualization_msgs::msg::MarkerArray markerArray;
            geometry_msgs::msg::Point p;

            // PRM nodes visualization
            visualization_msgs::msg::Marker markerNode;
            markerNode.header.frame_id = mapFrame;
            markerNode.header.stamp = this->now();
            markerNode.action = visualization_msgs::msg::Marker::ADD;
            markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            markerNode.ns = "nodes";
            markerNode.id = 2;
            markerNode.scale.x = 0.2;
            markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
            markerNode.color.a = 1;

            for (size_t i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.13;
                markerNode.points.push_back(p);
            }

            // PRM edge visualization with cost-based coloring
            visualization_msgs::msg::Marker markerEdge;
            markerEdge.header.frame_id = mapFrame;
            markerEdge.header.stamp = this->now();
            markerEdge.action = visualization_msgs::msg::Marker::ADD;
            markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
            markerEdge.ns = "edges";
            markerEdge.id = 3;
            markerEdge.scale.x = 0.05;
            markerEdge.pose.orientation.w = 1.0;

            for (size_t i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;

                for (size_t j = 0; j < nodeList[i]->neighborList.size(); ++j){
                    state_t* neighbor = nodeList[i]->neighborList[j].neighbor;
                    float* edgeCosts = nodeList[i]->neighborList[j].edgeCosts;

                    // Combine all 3 costs: traversability (0), elevation (1), and distance (2)
                    float combinedCost = edgeCosts[0] + edgeCosts[1] + edgeCosts[2];

                    // Normalize combined cost (adjust denominator based on expected max total cost)
                    float maxCombinedCost = 100.0f;  // example: max traversability + elevation + distance
                    float t = std::min(std::max(combinedCost / maxCombinedCost, 0.0f), 1.0f);

                    // Use color to reflect combined cost: red = high cost, green = low cost
                    std_msgs::msg::ColorRGBA color;
                    color.a = 1.0;
                    color.r = t;
                    color.g = 1.0 - t;
                    color.b = 0.0;

                    geometry_msgs::msg::Point p1, p2;
                    p1.x = nodeList[i]->x[0]; p1.y = nodeList[i]->x[1]; p1.z = nodeList[i]->x[2] + 0.1;
                    p2.x = neighbor->x[0];    p2.y = neighbor->x[1];    p2.z = neighbor->x[2] + 0.1;

                    markerEdge.points.push_back(p1);
                    markerEdge.colors.push_back(color);
                    markerEdge.points.push_back(p2);
                    markerEdge.colors.push_back(color);
                }
            }


            // push to markerarray and publish
            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerEdge);
            pubPRMGraph->publish(markerArray);

        }

        // 4. Single Source Shortest Paths
        if (pubSingleSourcePaths->get_subscription_count() != 0){

            visualization_msgs::msg::MarkerArray markerArray;
            geometry_msgs::msg::Point p;

             // publish empty single-source paths
            if (planningFlag == false){
                pubSingleSourcePaths->publish(markerArray);
                return;
            }

            // single source path visualization
            visualization_msgs::msg::Marker markersPath;
            markersPath.header.frame_id = mapFrame;
            markersPath.header.stamp = this->now();
            markersPath.action = visualization_msgs::msg::Marker::ADD;
            markersPath.type = visualization_msgs::msg::Marker::LINE_LIST;
            markersPath.ns = "path";
            markersPath.id = 4;
            markersPath.scale.x = 0.05;
            markersPath.color.r = 0.3; markersPath.color.g = 0; markersPath.color.b = 1.0;
            markersPath.color.a = 1.0;

            for (size_t i = 0; i < nodeList.size(); ++i){
                if (nodeList[i]->parentState == NULL)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.2;
                markersPath.points.push_back(p);
                p.x = nodeList[i]->parentState->x[0];
                p.y = nodeList[i]->parentState->x[1];
                p.z = nodeList[i]->parentState->x[2]+0.2;
                markersPath.points.push_back(p);
            }
            // push to markerarray and publish
            markerArray.markers.push_back(markersPath);
            pubSingleSourcePaths->publish(markerArray);
        }
    }

    void publishCurrentPath(){
        // Publishing and planner-state transitions are intentionally separated.
        globalPath.header.frame_id = mapFrame;
        globalPath.header.stamp = this->now();
        pubGlobalPath->publish(globalPath);
    }

    void publishRoadmap2Cloud(){
        if (pubCloudPRMNodes->get_subscription_count() == 0 && pubCloudPRMGraph->get_subscription_count() == 0)
            return;

        int sizeCloud = nodeList.size();
        // PRM nodes to point cloud
        PointType thisPoint;
        pcl::PointCloud<PointType> nodeCloud; nodeCloud.resize(sizeCloud);
        pcl::PointCloud<PointType> adjacencyCloud; adjacencyCloud.resize(sizeCloud*sizeCloud);
        for (int i = 0; i < sizeCloud; ++i){
            // save node
            thisPoint.x = nodeList[i]->x[0];
            thisPoint.y = nodeList[i]->x[1];
            thisPoint.z = nodeList[i]->x[2]+0.15;
            thisPoint.intensity = i;
            nodeCloud.points[i] = thisPoint;
            // extract adjacency matrix into cloud
            int numNeighbors = nodeList[i]->neighborList.size();
            for (int j = 0; j < numNeighbors; ++j){
                int index = nodeList[i]->stateId + sizeCloud * nodeList[i]->neighborList[j].neighbor->stateId;
                adjacencyCloud.points[index].intensity = 1;
            }
        }
        // Publish
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(nodeCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = mapFrame;
        laserCloudTemp.header.stamp = this->now();
        pubCloudPRMNodes->publish(laserCloudTemp);
        pcl::toROSMsg(adjacencyCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = mapFrame;
        laserCloudTemp.header.stamp = this->now();
        pubCloudPRMGraph->publish(laserCloudTemp);
    }

    void getRobotState(){
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tfBuffer->lookupTransform(mapFrame, baseFrame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex){ /*RCLCPP_ERROR(this->get_logger(), "Transfrom Failure.");*/ return; }

        robotState->x[0] = transform.transform.translation.x;
        robotState->x[1] = transform.transform.translation.y;
        robotState->x[2] = transform.transform.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        robotState->theta = yaw + M_PI; // change from -PI~PI to 0~1*PI
    }

};


int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto TPRM = std::make_shared<TraversabilityPRM>(
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO(TPRM->get_logger(), "\033[1;32m---->\033[0m Traversability Planner Started.");

    rclcpp::spin(TPRM);
    rclcpp::shutdown();
    return 0;
}