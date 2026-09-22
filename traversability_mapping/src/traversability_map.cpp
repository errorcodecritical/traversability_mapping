#include "utility.h"

class TraversabilityMapping : public rclcpp::Node {

private:

    // Mutex Memory Lock
    std::mutex mtx;
    // Transform Listener
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subFilteredGroundCloud;
    // Publisher
    // nav2's StaticLayer / a topic-based costmap plugin expects a persistent, "latched-like"
    // grid: reliable + transient_local so a late-joining subscriber (e.g. the costmap plugin,
    // started after this node) still receives the most recent map on connection.
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMapLocal;
    rclcpp::Publisher<elevation_msgs::msg::OccupancyElevation>::SharedPtr pubOccupancyMapLocalHeight;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubElevationCloud;
    // Point Cloud Pointer
    pcl::PointCloud<PointType>::Ptr laserCloud; // save input filtered laser cloud for mapping
    pcl::PointCloud<PointType>::Ptr laserCloudElevation; // a cloud for publishing elevation map
    // Occupancy Grid Map
    nav_msgs::msg::OccupancyGrid occupancyMap2D; // local occupancy grid map
    elevation_msgs::msg::OccupancyElevation occupancyMap2DHeight; // customized message that includes occupancy map and elevation info

    int pubCount;
    
    // Map Arrays
    int mapArrayCount;
    int **mapArrayInd; // it saves the index of this submap in vector mapArray
    int **predictionArrayFlag;
    vector<childMap_t*> mapArray;

    // Local Map Extraction
    PointType robotPoint;
    PointType localMapOriginPoint;
    grid_t localMapOriginGrid;

    // Global Variables for Traversability Calculation
    cv::Mat matCov, matEig, matVec;

    // Lists for New Scan
    vector<mapCell_t*> observingList1; // thread 1: save new observed cells
    vector<mapCell_t*> observingList2; // thread 2: calculate traversability of new observed cells

    std::thread predictionThread;

public:
    explicit TraversabilityMapping(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
        Node("traversability_map", options),
        pubCount(1),
        mapArrayCount(0){
        loadRuntimeConfig(*this);

        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        // subscribe to traversability filter
        subFilteredGroundCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            filteredCloudTopic, 5,
            std::bind(&TraversabilityMapping::cloudHandler, this, std::placeholders::_1));

        // publish local occupancy and elevation grid map
        // QoS chosen so this topic can be consumed directly by nav2 (e.g. nav2_costmap_2d's
        // StaticLayer, or the bundled traversability_costmap_layer plugin in this package):
        // reliable + transient_local (latched) + depth 1, matching nav2's map-topic convention.
        rclcpp::QoS mapQoS(1);
        mapQoS.reliable();
        mapQoS.transient_local();
        pubOccupancyMapLocal = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancyTopic, mapQoS);
        pubOccupancyMapLocalHeight = this->create_publisher<elevation_msgs::msg::OccupancyElevation>(elevationTopic, mapQoS);
        // publish elevation map for visualization
        pubElevationCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(elevationCloudTopic, 5);

        allocateMemory();

        // Traversability scoring runs on its own thread at 10 Hz, decoupled from the point
        // cloud callback rate - identical to the ROS1 std::thread(&Class::Method, &instance).
        predictionThread = std::thread(&TraversabilityMapping::TraversabilityThread, this);
    }

    ~TraversabilityMapping(){
        if (predictionThread.joinable())
            predictionThread.join();
    }

    

    void allocateMemory(){
        // allocate memory for point cloud
        laserCloud.reset(new pcl::PointCloud<PointType>());
        laserCloudElevation.reset(new pcl::PointCloud<PointType>());
        
        // initialize array for cmap
        mapArrayInd = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i)
            mapArrayInd[i] = new int[mapArrayLength];

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                mapArrayInd[i][j] = -1;

        // initialize array for predicting elevation sub-maps
        predictionArrayFlag = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i)
            predictionArrayFlag[i] = new int[mapArrayLength];

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                predictionArrayFlag[i][j] = false;

        // Matrix Initialization
        matCov = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
        matEig = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
        matVec = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

        initializeLocalOccupancyMap();
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// Register Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg){
        // Lock thread
        std::lock_guard<std::mutex> lock(mtx);
        // Get Robot Position
        if (getRobotPosition() == false) 
            return;
        // Convert Point Cloud
        pcl::fromROSMsg(*laserCloudMsg, *laserCloud);
        // Register New Scan
        updateElevationMap();
        // publish local occupancy grid map
        publishMap();
    }

    void updateElevationMap(){
        int cloudSize = laserCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            laserCloud->points[i].z -= 0.2; // for visualization
            updateElevationMap(&laserCloud->points[i]);
        }
    }

    void updateElevationMap(PointType *point){
        // Find point index in global map
        grid_t thisGrid;
        if (findPointGridInMap(&thisGrid, point) == false) return;
        // Get current cell pointer
        mapCell_t *thisCell = grid2Cell(&thisGrid);
        // update elevation
        updateCellElevation(thisCell, point);
        // accumulate this scan's binary obstacle evidence (does NOT touch occupancy - see below)
        updateCellObstacleBelief(thisCell, point);
        // update observation time
        updateCellObservationTime(thisCell);
    }

    void updateCellObservationTime(mapCell_t *thisCell){
        ++thisCell->observeTimes;
        if (thisCell->observeTimes >= traversabilityObserveTimeTh)
            observingList1.push_back(thisCell);
    }

    // The scan pipeline (traversability_filter) can only report binary evidence: a filtered point is
    // either an obstacle (intensity 100) or free (intensity 0). Feed that into a smoothed belief
    // instead of writing it straight into occupancy - occupancy is continuous and is owned by
    // traversabilityMapCalculation(). Having both write it made every new scan flatten the
    // continuous score back to 0 or 100.
    void updateCellObstacleBelief(mapCell_t *thisCell, PointType *point){
        thisCell->updateObstacleBelief(point->intensity >= 50 ? 100.0f : 0.0f);
    }

    void updateCellElevation(mapCell_t *thisCell, PointType *point){
        // Kalman Filter: update cell elevation using Kalman filter
        // https://www.cs.cornell.edu/courses/cs4758/2012sp/materials/MI63slides.pdf

        // cell is observed for the first time, no need to use Kalman filter
        if (thisCell->elevation == -FLT_MAX){
            thisCell->elevation = point->z;
            thisCell->elevationVar = pointDistance(robotPoint, *point);
            return;
        }

        // Predict:
        float x_pred = thisCell->elevation; // x = F * x + B * u
        float P_pred = thisCell->elevationVar + 0.01; // P = F*P*F + Q
        // Update:
        float R_factor = (thisCell->observeTimes > 20) ? 10 : 1;
        float R = pointDistance(robotPoint, *point) * R_factor; // measurement noise: R, scale it with dist and observed times
        float K = P_pred / (P_pred + R);// Gain: K  = P * H^T * (HPH + R)^-1
        float y = point->z; // measurement: y
        float x_final = x_pred + K * (y - x_pred); // x_final = x_pred + K * (y - H * x_pred)
        float P_final = (1 - K) * P_pred; // P_final = (I - K * H) * P_pred
        // Update cell
        thisCell->updateElevation(x_final, P_final);
    }

    mapCell_t* grid2Cell(grid_t *thisGrid){
        return mapArray[mapArrayInd[thisGrid->cubeX][thisGrid->cubeY]]->cellArray[thisGrid->gridX][thisGrid->gridY];
    }

    bool findPointGridInMap(grid_t *gridOut, PointType *point){
        // Calculate the cube index that this point belongs to. (Array dimension: mapArrayLength * mapArrayLength)
        grid_t thisGrid;
        getPointCubeIndex(&thisGrid.cubeX, &thisGrid.cubeY, point);
        // Decide whether a point is out of pre-allocated map
        if (thisGrid.cubeX >= 0 && thisGrid.cubeX < mapArrayLength && 
            thisGrid.cubeY >= 0 && thisGrid.cubeY < mapArrayLength){
            // Point is in the boundary, but this sub-map is not allocated before
            // Allocate new memory for this sub-map and save it to mapArray
            if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1){
                childMap_t *thisChildMap = new childMap_t(mapArrayCount, thisGrid.cubeX, thisGrid.cubeY);
                mapArray.push_back(thisChildMap);
                mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] = mapArrayCount;
                ++mapArrayCount;
            }
        }else{
            // Point is out of pre-allocated boundary, report error (you should increase map size)
            RCLCPP_ERROR(this->get_logger(), "Point cloud is out of elevation map boundary. Change params ->mapArrayLength<-. The program will crash!");
            return false;
        }
        // sub-map id
        thisGrid.mapID = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
        // Find the index for this point in this sub-map (grid index)
        thisGrid.gridX = (int)((point->x - mapArray[thisGrid.mapID]->originX) / mapResolution);
        thisGrid.gridY = (int)((point->y - mapArray[thisGrid.mapID]->originY) / mapResolution);
        if (thisGrid.gridX < 0 || thisGrid.gridY < 0 || thisGrid.gridX >= mapCubeArrayLength || thisGrid.gridY >= mapCubeArrayLength)
            return false;

        *gridOut = thisGrid;
        return true;
    }

    void getPointCubeIndex(int *cubeX, int *cubeY, PointType *point){
        *cubeX = int((point->x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        *cubeY = int((point->y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;

        if (point->x + mapCubeLength/2.0 < 0)  --*cubeX;
        if (point->y + mapCubeLength/2.0 < 0)  --*cubeY;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Traversability Calculation ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void TraversabilityThread(){

        rclcpp::Rate rate(10); // Hz
        
        while (rclcpp::ok()){

            traversabilityMapCalculation();

            rate.sleep();
        }
    }

    void traversabilityMapCalculation(){

        PointType robotPointCopy;
        {
            // observingList1 and robotPoint are written by cloudHandler() on the ROS spinner thread.
            // Swapping the list without this lock is a data race: a concurrent push_back can
            // reallocate the vector while it is being copied, yielding dangling mapCell_t pointers.
            std::lock_guard<std::mutex> lock(mtx);

            // no new scan, return
            if (observingList1.size() == 0)
                return;

            observingList2.swap(observingList1);
            observingList1.clear();
            robotPointCopy = robotPoint;
        }

        int listSize = observingList2.size();

        for (int i = 0; i < listSize; ++i){

            mapCell_t *thisCell = observingList2[i];
            // convert this cell to a point for convenience
            PointType thisPoint;
            thisPoint.x = thisCell->xyz->x;
            thisPoint.y = thisCell->xyz->y;
            thisPoint.z = thisCell->xyz->z;
            // too far, not accurate
            if (pointDistance(thisPoint, robotPointCopy) >= traversabilityCalculatingDistance)
                continue;
            // Find neighbor cells of this center cell
            vector<float> xyzVector = findNeighborElevations(thisCell);

            if (xyzVector.size() <= 2)
                continue;

            Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xyzVector.data(), xyzVector.size() / 3, 3);

            int numNeighbors = matPoints.rows();

            // min and max elevation
            float minElevation = matPoints.col(2).minCoeff();
            float maxElevation = matPoints.col(2).maxCoeff();
            float maxDifference = maxElevation - minElevation;

            // find slope. Note: "centered.adjoint() * centered" is the scatter matrix, not the
            // covariance - it has to be divided by the sample count before its entries mean
            // anything in metric units. The eigenvectors are unaffected by that scaling, and
            // the eigenvalues are scaled by numNeighbors (divided out where they are used).
            Eigen::MatrixXf centered = matPoints.rowwise() - matPoints.colwise().mean();
            Eigen::MatrixXf scatter = (centered.adjoint() * centered);
            cv::eigen2cv(scatter, matCov); // copy data from eigen to cv::Mat
            cv::eigen(matCov, matEig, matVec); // find eigenvalues and eigenvectors for the covariance matrix

            float slopeAngle = std::acos(std::abs(matVec.at<float>(2, 2))) / M_PI * 180;

            if (std::isnan(slopeAngle))
                continue;

            // Residuals about the FITTED PLANE, not about the mean elevation. Without this the
            // three terms below all measure the same raw elevation spread over the footprint,
            // so a smooth slope is charged three times.
            //
            // cv::eigen returns eigenvalues descending with eigenvectors as rows, so row 2 is
            // the plane normal and eigenvalue 2 is the residual variance. matEig came from the
            // scatter matrix, hence the division by numNeighbors.
            Eigen::Vector3f planeNormal(matVec.at<float>(2, 0),
                                        matVec.at<float>(2, 1),
                                        matVec.at<float>(2, 2));
            Eigen::VectorXf residuals = centered * planeNormal;
            float maxResidual = residuals.cwiseAbs().maxCoeff();
            float residualVariance = std::max(matEig.at<float>(2), 0.0f) / float(numNeighbors);

            // Three normalized 0..1 traversability terms, now measuring three different things.
            // vSlope: sigmoid centered on filterAngleLimit.
            // vStep:  worst departure from the fitted plane inside the footprint.
            // vRough: RMS departure from the fitted plane, i.e. how non-planar the patch is.
            float vSlope = 1.0f / (1.0f + std::exp(-(slopeAngle - filterAngleLimit) / travSlopeSigmoidWidth));
            float vStep  = std::min(maxResidual / filterHeightLimit, 1.0f);
            float vRough = std::min(std::sqrt(residualVariance) / travRoughnessScale, 1.0f);

            float v = travWeightSlope * vSlope + travWeightStep * vStep + travWeightRoughness * vRough;
            v = std::max(0.0f, std::min(v, 1.0f));

            float traversability = v * 100.0f;

            // Keep hard obstacles hard. The weighted score alone caps a wall at roughly
            // travWeightStep * 100, which would silently downgrade obstacles the scan filter
            // already detected reliably.
            if (thisCell->obstacleBelief > obstacleBeliefThreshold ||
                maxDifference > hardObstacleStepFactor * filterHeightLimit)
                traversability = 100.0f;

            thisCell->updateOccupancy(traversability);
        }
    }

    vector<float> findNeighborElevations(mapCell_t *centerCell){

        vector<float> xyzVector;

        grid_t centerGrid = centerCell->grid;
        grid_t thisGrid;

        int footprintRadiusLength = int(robotRadius / mapResolution);

        for (int k = -footprintRadiusLength; k <= footprintRadiusLength; ++k){
            for (int l = -footprintRadiusLength; l <= footprintRadiusLength; ++l){
                // skip grids too far
                if (std::sqrt(float(k*k + l*l)) * mapResolution > robotRadius)
                    continue;
                // the neighbor grid
                thisGrid.cubeX = centerGrid.cubeX;
                thisGrid.cubeY = centerGrid.cubeY;
                thisGrid.gridX = centerGrid.gridX + k;
                thisGrid.gridY = centerGrid.gridY + l;
                // If the checked grid is in another sub-map, update it's indexes
                if(thisGrid.gridX < 0){ --thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX + mapCubeArrayLength;
                }else if(thisGrid.gridX >= mapCubeArrayLength){ ++thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX - mapCubeArrayLength; }
                if(thisGrid.gridY < 0){ --thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY + mapCubeArrayLength;
                }else if(thisGrid.gridY >= mapCubeArrayLength){ ++thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY - mapCubeArrayLength; }
                // If the sub-map that the checked grid belongs to is empty or not
                int mapInd = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
                if (mapInd == -1) continue;
                // the neighbor cell
                mapCell_t *thisCell = grid2Cell(&thisGrid);
                // save neighbor cell for calculating traversability
                if (thisCell->elevation != -FLT_MAX){
                    xyzVector.push_back(thisCell->xyz->x);
                    xyzVector.push_back(thisCell->xyz->y);
                    xyzVector.push_back(thisCell->xyz->z);
                }
            }
        }

        return xyzVector;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////// Occupancy Map (local) //////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void publishMap(){
        // Publish Occupancy Grid Map and Elevation Map
        pubCount++;
        if (pubCount > visualizationFrequency){
            pubCount = 1;
            publishLocalMap();
            publishTraversabilityMap();
        }
    }

    void publishLocalMap(){

        if (pubOccupancyMapLocal->get_subscription_count() == 0 &&
            pubOccupancyMapLocalHeight->get_subscription_count() == 0)
            return;

        // 1.3 Initialize local occupancy grid map to unknown, height to -FLT_MAX
        std::fill(occupancyMap2DHeight.occupancy.data.begin(), occupancyMap2DHeight.occupancy.data.end(), -1);
        std::fill(occupancyMap2DHeight.height.begin(), occupancyMap2DHeight.height.end(), -FLT_MAX);
        std::fill(occupancyMap2DHeight.cost_map.begin(), occupancyMap2DHeight.cost_map.end(), 0);
        
        // local map origin x and y
        localMapOriginPoint.x = robotPoint.x - localMapLength / 2;
        localMapOriginPoint.y = robotPoint.y - localMapLength / 2;
        localMapOriginPoint.z = robotPoint.z;
        // local map origin cube id (in global map)
        localMapOriginGrid.cubeX = int((localMapOriginPoint.x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        localMapOriginGrid.cubeY = int((localMapOriginPoint.y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        if (localMapOriginPoint.x + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeX;
        if (localMapOriginPoint.y + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeY;
        // local map origin grid id (in sub-map)
        float originCubeOriginX, originCubeOriginY; // the orign of submap that the local map origin belongs to (note the submap may not be created yet, cannot use originX and originY)
        originCubeOriginX = (localMapOriginGrid.cubeX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originCubeOriginY = (localMapOriginGrid.cubeY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        localMapOriginGrid.gridX = int((localMapOriginPoint.x - originCubeOriginX) / mapResolution);
        localMapOriginGrid.gridY = int((localMapOriginPoint.y - originCubeOriginY) / mapResolution);

        // 2 Calculate local occupancy grid map root position
        occupancyMap2DHeight.header.stamp = this->now();
        occupancyMap2DHeight.occupancy.header.stamp = occupancyMap2DHeight.header.stamp;
        occupancyMap2DHeight.occupancy.info.origin.position.x = localMapOriginPoint.x;
        occupancyMap2DHeight.occupancy.info.origin.position.y = localMapOriginPoint.y;
        occupancyMap2DHeight.occupancy.info.origin.position.z = localMapOriginPoint.z + 10; // add 10, just for visualization

        // extract all info
        for (int i = 0; i < localMapArrayLength; ++i){
            for (int j = 0; j < localMapArrayLength; ++j){

                int indX = localMapOriginGrid.gridX + i;
                int indY = localMapOriginGrid.gridY + j;

                grid_t thisGrid;

                thisGrid.cubeX = localMapOriginGrid.cubeX + indX / mapCubeArrayLength;
                thisGrid.cubeY = localMapOriginGrid.cubeY + indY / mapCubeArrayLength;

                thisGrid.gridX = indX % mapCubeArrayLength;
                thisGrid.gridY = indY % mapCubeArrayLength;

                // if sub-map is not created yet
                if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1) {
                    continue;
                }
                
                mapCell_t *thisCell = grid2Cell(&thisGrid);

                // skip unknown grid
                if (thisCell->elevation != -FLT_MAX){
                    int index = i + j * localMapArrayLength; // index of the 1-D array 
                    occupancyMap2DHeight.height[index] = thisCell->elevation;
                    occupancyMap2DHeight.occupancy.data[index] = thisCell->occupancy;
                }
            }
        }

        pubOccupancyMapLocalHeight->publish(occupancyMap2DHeight);
        pubOccupancyMapLocal->publish(occupancyMap2DHeight.occupancy);
    }
    

    void initializeLocalOccupancyMap(){
        // initialization of customized map message
        occupancyMap2DHeight.header.frame_id = mapFrame;
        occupancyMap2DHeight.occupancy.header.frame_id = mapFrame;
        occupancyMap2DHeight.occupancy.info.width = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.height = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.resolution = mapResolution;
        
        occupancyMap2DHeight.occupancy.info.origin.orientation.x = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.y = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.z = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.w = 1.0;

        occupancyMap2DHeight.occupancy.data.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.height.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.cost_map.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
    }    

    bool getRobotPosition(){
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tfBuffer->lookupTransform(mapFrame, baseFrame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex){ RCLCPP_ERROR(this->get_logger(), "Transfrom Failure."); return false; }

        robotPoint.x = transform.transform.translation.x;
        robotPoint.y = transform.transform.translation.y;
        robotPoint.z = transform.transform.translation.z;

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Point Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void publishTraversabilityMap(){

        if (pubElevationCloud->get_subscription_count() == 0)
            return;
        // 1. Find robot current cube index
        int currentCubeX, currentCubeY;
        getPointCubeIndex(&currentCubeX, &currentCubeY, &robotPoint);
        // 2. Loop through all the sub-maps that are nearby
        int visualLength = int(visualizationRadius / mapCubeLength);
        for (int i = -visualLength; i <= visualLength; ++i){
            for (int j = -visualLength; j <= visualLength; ++j){

                if (sqrt(float(i*i+j*j)) >= visualLength) continue;

                int idx = i + currentCubeX;
                int idy = j + currentCubeY;

                if (idx < 0 || idx >= mapArrayLength ||  idy < 0 || idy >= mapArrayLength) continue;

                if (mapArrayInd[idx][idy] == -1) continue;

                *laserCloudElevation += mapArray[mapArrayInd[idx][idy]]->cloud;
            }
        }
        // 3. Publish elevation point cloud
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudElevation, laserCloudTemp);
        laserCloudTemp.header.frame_id = mapFrame;
        laserCloudTemp.header.stamp = this->now();
        pubElevationCloud->publish(laserCloudTemp);
        // 4. free memory
        laserCloudElevation->clear();
    }
};




int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto tMapping = std::make_shared<TraversabilityMapping>(
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO(tMapping->get_logger(), "\033[1;32m---->\033[0m Traversability Mapping Started.");
    RCLCPP_INFO(tMapping->get_logger(), "\033[1;32m---->\033[0m Traversability Mapping Scenario: %s.",
        urbanMapping == true ? "\033[1;31mUrban\033[0m" : "\033[1;31mTerrain\033[0m");

    rclcpp::spin(tMapping);

    rclcpp::shutdown();
    return 0;
}