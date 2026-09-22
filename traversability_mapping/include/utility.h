#ifndef _UTILITY_TM_H_
#define _UTILITY_TM_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <array> // c++11
#include <thread> // c++11
#include <mutex> // c++11
#include <stdexcept>
#include <type_traits>
#include <map>

#include "planner/kdtree.h"
#include "planner/cubic_spline_interpolator.h"

#include "elevation_msgs/msg/occupancy_elevation.hpp"

using namespace std;

typedef pcl::PointXYZI  PointType;
typedef struct kdtree kdtree_t;
typedef struct kdres kdres_t;

// Runtime configuration. These names intentionally match the legacy constants so the
// processing code remains readable while parameters can be changed in the ROS parameter file.
inline bool useSimTime = false;
inline bool urbanMapping = false;
inline int N_SCAN = 128;
inline int Horizon_SCAN = 1800;
inline float ang_res_x = 360.0f / Horizon_SCAN;
inline float ang_res_y = 90.0f / (N_SCAN - 1);
inline float ang_bottom = 0.0f;
inline bool useCloudRing = true;
inline float sensorMinimumRange = 0.5f;
inline bool useHeightCut = true;
inline float heightCutAboveSensor = 1.5f;
inline int maxRingIndex = 64;
inline float mapResolution = 0.1f;
inline float mapCubeLength = 1.0f;
inline float globalMapLength = 2000.0f;
inline float localMapLength = 20.0f;
inline int mapCubeArrayLength = 10;
inline int mapArrayLength = 2000;
inline int rootCubeIndex = 1000;
inline int scanNumCurbFilter = 8;
inline int scanNumSlopeFilter = 10;
inline int scanNumMax = 10;
inline float sensorRangeLimit = 20.0f;
inline float filterHeightLimit = 0.25f;
inline float filterAngleLimit = 25.0f;
inline int filterHeightMapArrayLength = 400;
inline bool predictionEnableFlag = true;
inline float predictionKernalSize = 0.2f;
inline float p_occupied_when_laser = 0.9f;
inline float p_occupied_when_no_laser = 0.2f;
inline float large_log_odds = 100.0f;
inline float max_log_odds_for_belief = 20.0f;
inline float occupancyEmaAlpha = 0.3f;
inline float obstacleBeliefAlpha = 0.3f;
inline float obstacleBeliefThreshold = 80.0f;
inline float hardObstacleStepFactor = 2.0f;
inline int localMapArrayLength = 200;
inline float visualizationRadius = 50.0f;
inline float visualizationFrequency = 2.0f;
inline float robotRadius = 0.2f;
inline float sensorHeight = 0.5f;
inline int traversabilityObserveTimeTh = 10;
inline float traversabilityCalculatingDistance = 15.0f;
inline float travWeightSlope = 0.5f;
inline float travWeightStep = 0.3f;
inline float travWeightRoughness = 0.2f;
inline float travSlopeSigmoidWidth = 5.0f;
inline float travRoughnessScale = 0.2f;
inline float pathCostScaleTraversability = 2000.0f;
inline float pathCostScaleElevation = 2.0f;
inline float pathCostScaleDistance = 20.0f;
inline float pathCostWeightTraversability = 0.34f;
inline float pathCostWeightElevation = 0.33f;
inline float pathCostWeightDistance = 0.33f;
inline bool planningUnknown = true;
inline int obstacleOccupancyThreshold = 75;
inline int unknownCellTraversability = 50;
inline float costmapInflationRadius = 0.15f;
inline double samplingTimeBudget = 0.01;
inline float neighborSampleRadius = 0.5f;
inline float neighborConnectHeight = 1.0f;
inline float neighborConnectRadius = 1.0f;
inline float neighborSearchRadius = 10.0f;
inline float waypointReachDistance = 1.0f;
inline float newGoalDistanceThreshold = 0.2f;
inline int pathDepthConfig = 4; inline float angularVelocityMaxDegConfig = 7.0f; inline float angularVelocityResolutionDegConfig = 0.5f; inline float angularVelocityMaxSecondaryDegConfig = 0.5f; inline float angularVelocityResolutionSecondaryDegConfig = 0.25f; inline float forwardVelocityConfig = 0.1f; inline float simulationDeltaTimeConfig = 1.0f; inline int simulationTimeStepsConfig = 30;

inline std::string mapFrame = "map";
inline std::string baseFrame = "base_link";
inline std::string lidarFrame = "hesai_lidar";
inline std::string inputCloudTopic = "/hesai/points_decimated";
inline std::string filteredCloudTopic = "/filtered_pointcloud";
inline std::string occupancyTopic = "/occupancy_map_local";
inline std::string elevationTopic = "/occupancy_map_local_height";
inline std::string goalTopic = "/prm_goal";
inline std::string globalPathTopic = "/global_path";
inline std::string filteredCloudVisualHighTopic = "/filtered_pointcloud_visual_high_res";
inline std::string filteredCloudVisualLowTopic = "/filtered_pointcloud_visual_low_res";
inline std::string laserScanTopic = "/pointcloud_2_laserscan";
inline std::string elevationCloudTopic = "/elevation_pointcloud";
inline std::string pathTrajectoryTopic = "/path_trajectory";
inline std::string pathLibraryValidTopic = "/path_library_valid";
inline std::string pathLibraryOriginTopic = "/path_library_origin";
inline std::string prmGraphTopic = "/prm_graph";
inline std::string prmPathTopic = "/prm_path";
inline std::string prmSingleSourcePathsTopic = "/prm_single_source_paths";
inline std::string prmNodesTopic = "/prm_cloud_nodes";
inline std::string prmGraphCloudTopic = "/prm_cloud_graph";

inline void loadRuntimeConfig(rclcpp::Node &node){
    auto get = [&node](const char *name, auto &value){
        using T = std::decay_t<decltype(value)>;
        if (!node.has_parameter(name)) {
            if constexpr (std::is_floating_point_v<T>) {
                node.declare_parameter(name, static_cast<double>(value));
            } else {
                node.declare_parameter<T>(name, value);
            }
        }
        const auto parameter = node.get_parameter(name);
        if constexpr (std::is_same_v<T, bool>) {
            value = parameter.as_bool();
        } else if constexpr (std::is_same_v<T, std::string>) {
            value = parameter.as_string();
        } else if constexpr (std::is_floating_point_v<T>) {
            value = static_cast<T>(parameter.as_double());
        } else if constexpr (std::is_integral_v<T>) {
            value = static_cast<T>(parameter.as_int());
        }
    };
    get("use_sim_time", useSimTime);
    get("map_frame", mapFrame); get("base_frame", baseFrame); get("lidar_frame", lidarFrame);
    get("input_cloud_topic", inputCloudTopic); get("filtered_cloud_topic", filteredCloudTopic);
    get("occupancy_topic", occupancyTopic); get("elevation_topic", elevationTopic);
    get("goal_topic", goalTopic); get("global_path_topic", globalPathTopic);
    get("filtered_cloud_visual_high_topic", filteredCloudVisualHighTopic);
    get("filtered_cloud_visual_low_topic", filteredCloudVisualLowTopic);
    get("laser_scan_topic", laserScanTopic);
    get("elevation_cloud_topic", elevationCloudTopic);
    get("path_trajectory_topic", pathTrajectoryTopic);
    get("path_library_valid_topic", pathLibraryValidTopic);
    get("path_library_origin_topic", pathLibraryOriginTopic);
    get("prm_graph_topic", prmGraphTopic);
    get("prm_path_topic", prmPathTopic);
    get("prm_single_source_paths_topic", prmSingleSourcePathsTopic);
    get("prm_nodes_topic", prmNodesTopic);
    get("prm_graph_cloud_topic", prmGraphCloudTopic);
    get("urban_mapping", urbanMapping); get("n_scan", N_SCAN); get("horizon_scan", Horizon_SCAN);
    get("use_cloud_ring", useCloudRing); get("sensor_minimum_range", sensorMinimumRange);
    get("use_height_cut", useHeightCut); get("height_cut_above_sensor", heightCutAboveSensor); get("max_ring_index", maxRingIndex);
    get("map_resolution", mapResolution); get("map_cube_length", mapCubeLength); get("global_map_length", globalMapLength); get("local_map_length", localMapLength);
    get("scan_num_curb_filter", scanNumCurbFilter); get("scan_num_slope_filter", scanNumSlopeFilter); get("sensor_range_limit", sensorRangeLimit); get("filter_height_limit", filterHeightLimit); get("filter_angle_limit", filterAngleLimit);
    get("prediction_enable", predictionEnableFlag); get("prediction_kernel_size", predictionKernalSize);
    get("occupied_probability_laser", p_occupied_when_laser); get("occupied_probability_no_laser", p_occupied_when_no_laser); get("large_log_odds", large_log_odds); get("max_log_odds_belief", max_log_odds_for_belief);
    get("occupancy_ema_alpha", occupancyEmaAlpha); get("obstacle_belief_alpha", obstacleBeliefAlpha); get("obstacle_belief_threshold", obstacleBeliefThreshold); get("hard_obstacle_step_factor", hardObstacleStepFactor);
    get("visualization_radius", visualizationRadius); get("visualization_frequency", visualizationFrequency); get("robot_radius", robotRadius); get("sensor_height", sensorHeight);
    get("traversability_observe_time_threshold", traversabilityObserveTimeTh); get("traversability_calculating_distance", traversabilityCalculatingDistance); get("traversability_weight_slope", travWeightSlope); get("traversability_weight_step", travWeightStep); get("traversability_weight_roughness", travWeightRoughness); get("traversability_slope_sigmoid_width", travSlopeSigmoidWidth); get("traversability_roughness_scale", travRoughnessScale);
    get("planning_unknown", planningUnknown); get("obstacle_occupancy_threshold", obstacleOccupancyThreshold); get("unknown_cell_traversability", unknownCellTraversability); get("costmap_inflation_radius", costmapInflationRadius); get("path_cost_scale_traversability", pathCostScaleTraversability); get("path_cost_scale_elevation", pathCostScaleElevation); get("path_cost_scale_distance", pathCostScaleDistance); get("path_cost_weight_traversability", pathCostWeightTraversability); get("path_cost_weight_elevation", pathCostWeightElevation); get("path_cost_weight_distance", pathCostWeightDistance);
    get("path_depth", pathDepthConfig); get("angular_velocity_max_deg", angularVelocityMaxDegConfig); get("angular_velocity_resolution_deg", angularVelocityResolutionDegConfig); get("angular_velocity_max_secondary_deg", angularVelocityMaxSecondaryDegConfig); get("angular_velocity_resolution_secondary_deg", angularVelocityResolutionSecondaryDegConfig); get("forward_velocity", forwardVelocityConfig); get("simulation_delta_time", simulationDeltaTimeConfig); get("simulation_time_steps", simulationTimeStepsConfig);
    get("sampling_time_budget", samplingTimeBudget); get("neighbor_sample_radius", neighborSampleRadius); get("neighbor_connect_height", neighborConnectHeight); get("neighbor_connect_radius", neighborConnectRadius); get("neighbor_search_radius", neighborSearchRadius); get("waypoint_reach_distance", waypointReachDistance); get("new_goal_distance_threshold", newGoalDistanceThreshold);
    if (N_SCAN < 2 || Horizon_SCAN < 1 || mapResolution <= 0 || mapCubeLength <= 0 || globalMapLength <= 0 || localMapLength <= 0 || sensorRangeLimit <= 0 || filterHeightLimit <= 0 || predictionKernalSize <= 0)
        throw std::invalid_argument("invalid traversability_mapping geometry/range parameter");
    if (scanNumCurbFilter < 1 || scanNumSlopeFilter < 1 || mapCubeLength < mapResolution)
        throw std::invalid_argument("invalid traversability_mapping filter/map parameter");
    if (p_occupied_when_laser < 0 || p_occupied_when_laser > 1 || p_occupied_when_no_laser < 0 || p_occupied_when_no_laser > 1 ||
        occupancyEmaAlpha < 0 || occupancyEmaAlpha > 1 || obstacleBeliefAlpha < 0 || obstacleBeliefAlpha > 1 ||
        travWeightSlope < 0 || travWeightStep < 0 || travWeightRoughness < 0 ||
        std::abs(travWeightSlope + travWeightStep + travWeightRoughness - 1.0f) > 1e-3f ||
        visualizationFrequency < 0 || robotRadius < 0 || samplingTimeBudget < 0)
        throw std::invalid_argument("invalid traversability_mapping probability/weight parameter");
    mapCubeArrayLength = std::max(1, int(std::lround(mapCubeLength / mapResolution)));
    mapArrayLength = std::max(1, int(std::lround(globalMapLength / mapCubeLength)));
    rootCubeIndex = mapArrayLength / 2;
    localMapArrayLength = std::max(1, int(std::lround(localMapLength / mapResolution)));
    filterHeightMapArrayLength = std::max(1, int(std::lround(sensorRangeLimit * 2.0f / mapResolution)));
    scanNumMax = std::max(scanNumCurbFilter, scanNumSlopeFilter);
    neighborSearchRadius = std::max(0.0f, neighborSearchRadius);
    ang_res_x = 360.0f / Horizon_SCAN; ang_res_y = 90.0f / (N_SCAN - 1);
}

extern const int NUM_COSTS = 3;
extern const int tmp[] = {0, 1, 2};
extern const std::vector<int> costHierarchy(tmp, tmp + sizeof(tmp) / sizeof(int));

struct grid_t;
struct mapCell_t;
struct childMap_t;
struct state_t;
struct neighbor_t;

/*
    This struct is used to send map from mapping package to prm package
    */
struct grid_t{
    int mapID;
    int cubeX;
    int cubeY;
    int gridX;
    int gridY;
    int gridIndex;
};

/*
    Cell Definition:
    a cell is a member of a grid in a sub-map
    a grid can have several cells in it. 
    a cell represent one height information
    */

struct mapCell_t{

    PointType *xyz; // it's a pointer to the corresponding point in the point cloud of submap

    grid_t grid;

    float log_odds;

    int observeTimes;

    float occupancy, occupancyVar;
    float elevation, elevationVar;

    // Per-scan binary obstacle evidence (0..100), smoothed over time. Kept separate from
    // "occupancy": the scan pipeline can only say obstacle/free, while occupancy is the continuous
    // traversability score owned solely by TraversabilityMapping::traversabilityMapCalculation().
    float obstacleBelief;
    bool occupancyInit; // has occupancy ever been scored by the traversability thread?

    mapCell_t(){

        log_odds = 0.5;
        observeTimes = 0;

        elevation = -FLT_MAX;
        elevationVar = 1e3;

        occupancy = 0; // initialized as unkown
        occupancyVar = 1e3;

        obstacleBelief = 0;
        occupancyInit = false;
    }

    void updatePoint(){
        // Never publish a cell without a valid elevation: -FLT_MAX in the visualization cloud
        // destroys RViz's autocomputed bounds (and any consumer that reads z).
        if (elevation == -FLT_MAX)
            return;
        xyz->z = elevation;
        xyz->intensity = occupancy;
    }
    void updateElevation(float elevIn, float varIn){
        elevation = elevIn;
        elevationVar = varIn;
        updatePoint();
    }
    // Continuous replacement for the old log-odds filter: exponential moving average, no saturation.
    void updateOccupancy(float occupIn){
        if (occupancyInit == false){
            occupancy = occupIn;
            occupancyInit = true;
        }else{
            occupancy = (1.0f - occupancyEmaAlpha) * occupancy + occupancyEmaAlpha * occupIn;
        }
        updatePoint();
    }
    // Smoothed obstacle evidence from a single scan point (evidenceIn is 0 or 100).
    void updateObstacleBelief(float evidenceIn){
        if (observeTimes == 0)
            obstacleBelief = evidenceIn;
        else
            obstacleBelief = (1.0f - obstacleBeliefAlpha) * obstacleBelief + obstacleBeliefAlpha * evidenceIn;
    }
};


/*
    Sub-map Definition:
    childMap_t is a small square. We call it "cellArray". 
    It composes the whole map
    */
struct childMap_t{

    vector<vector<mapCell_t*> > cellArray;
    int subInd; //sub-map's index in 1d mapArray
    int indX; // sub-map's x index in 2d array mapArrayInd
    int indY; // sub-map's y index in 2d array mapArrayInd
    float originX; // sub-map's x root coordinate
    float originY; // sub-map's y root coordinate
    pcl::PointCloud<PointType> cloud;

    childMap_t(int id, int indx, int indy){

        subInd = id;
        indX = indx;
        indY = indy;
        originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;

        // allocate and initialize each cell
        cellArray.resize(mapCubeArrayLength);
        for (int i = 0; i < mapCubeArrayLength; ++i)
            cellArray[i].resize(mapCubeArrayLength);

        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j] = new mapCell_t;
        // allocate point cloud for visualization
        cloud.points.resize(mapCubeArrayLength*mapCubeArrayLength);

        // initialize cell pointer to cloud point
        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j]->xyz = &cloud.points[i + j*mapCubeArrayLength];

        // initialize each point in the point cloud, also each cell
        for (int i = 0; i < mapCubeArrayLength; ++i){
            for (int j = 0; j < mapCubeArrayLength; ++j){
                
                // point cloud initialization
                int index = i + j * mapCubeArrayLength;
                cloud.points[index].x = originX + i * mapResolution;
                cloud.points[index].y = originY + j * mapResolution;
                cloud.points[index].z = std::numeric_limits<float>::quiet_NaN();
                cloud.points[index].intensity = cellArray[i][j]->occupancy;

                // cell position in the array of submap
                cellArray[i][j]->grid.mapID = subInd;
                cellArray[i][j]->grid.cubeX = indX;
                cellArray[i][j]->grid.cubeY = indY;
                cellArray[i][j]->grid.gridX = i;
                cellArray[i][j]->grid.gridY = j;
                cellArray[i][j]->grid.gridIndex = index;
            }
        }
    }
};



/*
    Robot State Defination
    */


struct state_t{
    double x[3]; //  1 - x, 2 - y, 3 - z
    float theta;
    int stateId;
    float cost;
    bool validFlag;
    // # Cost types
    // # 0. obstacle cost
    // # 1. elevation cost
    // # 2. distance cost
    float costsToRoot[NUM_COSTS];
    float costsToParent[NUM_COSTS]; // used in RRT*
    float costsToGo[NUM_COSTS];

    state_t* parentState; // parent for this state in PRM and RRT*
    vector<neighbor_t> neighborList; // PRM adjencency list with edge costs
    vector<state_t*> childList; // RRT*

    // Set once the graph search has settled this state, i.e. popped it from the priority queue
    // with its final cost. Only valid during a search; meaningless between searches.
    bool closedFlag;

    // default initialization
    state_t(){
        parentState = NULL;
        closedFlag = false;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = FLT_MAX;
            costsToGo[i] = FLT_MAX;
        }
    }
    // use a state input to initialize new state
    
    state_t(state_t* stateIn){
        // pose initialization
        for (int i = 0; i < 3; ++i)
            x[i] = stateIn->x[i];
        theta = stateIn->theta;
        // regular initialization
        parentState = NULL;
        closedFlag = false;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = stateIn->costsToParent[i];
        }
    }
};


struct neighbor_t{
    state_t* neighbor;
    float edgeCosts[NUM_COSTS]; // the cost from this state to neighbor
    neighbor_t(){
        neighbor = NULL;
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = FLT_MAX;
    }
};

/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

// RS-BPearl vertical angle ring mapping (top-down)
std::map<int, int> ringToRow = {
    {1, 0}, {9, 1}, {2, 2}, {10, 3}, {3, 4}, {11, 5}, {4, 6}, {12, 7},
    {5, 8}, {13, 9}, {6,10}, {14,11}, {7,12}, {15,13}, {8,14}, {16,15},
    {17,16}, {25,17}, {18,18}, {26,19}, {19,20}, {27,21}, {20,22}, {28,23},
    {21,24}, {29,25}, {22,26}, {30,27}, {23,28}, {31,29}, {24,30}, {32,31}
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////      Some Functions    ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
state_t *compareState;
bool isStateExsiting(neighbor_t neighborIn){
    return neighborIn.neighbor == compareState ? true : false;
}

float pointDistance(PointType p1, PointType p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

// ROS 2 replacement for pcl_ros::transformPointCloud(frame, cloud_in, cloud_out, listener)
// convenience overload, which no longer exists in the same form. Round-trips through
// sensor_msgs::msg::PointCloud2 and tf2_sensor_msgs::doTransform so it works with any PCL point
// type that pcl::toROSMsg/fromROSMsg supports, without depending on a specific pcl_ros API.
template <typename PointT>
inline void tmTransformPointCloud(const pcl::PointCloud<PointT> &cloudIn,
                                   pcl::PointCloud<PointT> &cloudOut,
                                   const geometry_msgs::msg::TransformStamped &transform){
    sensor_msgs::msg::PointCloud2 msgIn, msgOut;
    pcl::toROSMsg(cloudIn, msgIn);
    tf2::doTransform(msgIn, msgOut, transform);
    pcl::fromROSMsg(msgOut, cloudOut);
    cloudOut.header = cloudIn.header;
}

#endif