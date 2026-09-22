#include "utility.h"

class TraversabilityFilter : public rclcpp::Node {

private:

    // ROS subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud;
    // ROS publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudVisualHiRes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudVisualLowRes;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaserScan;
    // Point Cloud
    pcl::PointCloud<PointType>::Ptr laserCloudIn; // projected full velodyne cloud
    pcl::PointCloud<PointType>::Ptr laserCloudOut; // filtered and downsampled point cloud
    pcl::PointCloud<PointType>::Ptr laserCloudObstacles; // cloud for saving points that are classified as obstables, convert them to laser scan
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;
    // Transform Listener
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std_msgs::msg::Header cloudHeader;
    // A few points
    PointType robotPoint;
    PointType localMapOrigin;
    // Sentinel used to pre-fill the organized cloud. Beams with no return must be distinguishable
    // from a real measurement; a default-constructed PointType is (0,0,0) with intensity 0, which
    // reads as "valid free point at range 0 sitting on the sensor origin".
    PointType nanPoint;
    // point cloud saved as N_SCAN * Horizon_SCAN form
    vector<vector<PointType>> laserCloudMatrix;
    // Matrice
    cv::Mat obstacleMatrix; // -1 - invalid, 0 - free, 1 - obstacle
    cv::Mat rangeMatrix; // -1 - invalid, >0 - valid range value
    // laser scan message
    sensor_msgs::msg::LaserScan laserScan;
    // for downsample
    float **minHeight;
    float **maxHeight;
    bool **obstFlag;
    bool **initFlag;


public:
    explicit TraversabilityFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
        Node("traversability_filter", options){
        loadRuntimeConfig(*this);

        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        subCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            inputCloudTopic, rclcpp::SensorDataQoS(),
            std::bind(&TraversabilityFilter::cloudHandler, this, std::placeholders::_1));

        pubCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(filteredCloudTopic, 5);
        pubCloudVisualHiRes = this->create_publisher<sensor_msgs::msg::PointCloud2>(filteredCloudVisualHighTopic, 5);
        pubCloudVisualLowRes = this->create_publisher<sensor_msgs::msg::PointCloud2>(filteredCloudVisualLowTopic, 5);
        pubLaserScan = this->create_publisher<sensor_msgs::msg::LaserScan>(laserScanTopic, 5);

        allocateMemory();

        pointcloud2laserscanInitialization();
    }



   void rebuildObstacleCloudFromHeightMap(){
    laserCloudObstacles->clear();
    for (int i = 0; i < filterHeightMapArrayLength; ++i){
        for (int j = 0; j < filterHeightMapArrayLength; ++j){
            if (obstFlag[i][j] == false)
                continue;
            PointType p;
            p.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
            p.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
            p.z = maxHeight[i][j];
            p.intensity = 100;
            laserCloudObstacles->push_back(p);
             }
        }
    }




    void allocateMemory(){

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1; // "no range value" marker, matches rangeMatrix's -1 convention

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());
        laserCloudObstacles.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        laserCloudMatrix.resize(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].resize(Horizon_SCAN);

        initFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            initFlag[i] = new bool[filterHeightMapArrayLength];

        obstFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            obstFlag[i] = new bool[filterHeightMapArrayLength];

        minHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            minHeight[i] = new float[filterHeightMapArrayLength];

        maxHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            maxHeight[i] = new float[filterHeightMapArrayLength];

        resetParameters();
    }

    void resetParameters(){

        laserCloudIn->clear();
        laserCloudOut->clear();
        laserCloudObstacles->clear();
        laserCloudInRing->clear();

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                initFlag[i][j] = false;
                obstFlag[i][j] = false;
            }
        }
    }

    ~TraversabilityFilter(){}


    void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg){

        copyPointCloud(laserCloudMsg);

        projectPointCloud();
        
        extractRawCloud();

        if (transformCloud() == false) return;

        cloud2Matrix();

        applyFilter();

        extractFilteredCloud();

        downsampleCloud();

        predictCloudBGK();

        // rebuildObstacleCloudFromHeightMap();   // <-- if DWA requried

        publishCloud();

        publishLaserScan();

        resetParameters();
    }

    void copyPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = this->now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        // Remove Nan points
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        // have "ring" channel in the cloud
        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                laserCloudInRing->is_dense = true;
                // RCLCPP_ERROR(this->get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
                // rclcpp::shutdown();
            }  
        }
    }

    void projectPointCloud() {

        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index;
        PointType thisPoint;

        size_t cloudSize = laserCloudIn->points.size();

        // Create a copy of unordered input for safe processing
        pcl::PointCloud<PointType>::Ptr inputCloud(new pcl::PointCloud<PointType>(*laserCloudIn));

        // Resize output cloud to organized structure and mark every cell as "no return" up front
        laserCloudIn->points.clear();
        laserCloudIn->points.resize(N_SCAN * Horizon_SCAN);
        std::fill(laserCloudIn->points.begin(), laserCloudIn->points.end(), nanPoint);
        laserCloudIn->width = Horizon_SCAN;
        laserCloudIn->height = N_SCAN;
        laserCloudIn->is_dense = false; // it now legitimately contains NaN cells

        for (size_t i = 0; i < cloudSize; ++i) {
            thisPoint = inputCloud->points[i];

            // Get row index (vertical beam)
            if (useCloudRing) {
                rowIdn = laserCloudInRing->points[i].ring;  // assumes alignment with inputCloud
            } else {
                if (!pcl::isFinite(thisPoint)) continue;
                float verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180.0 / M_PI;
                rowIdn = static_cast<int>((verticalAngle + ang_bottom) / ang_res_y);
            }

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // Drop beams pointing well above the horizon before any further work - see
            // maxRingIndex in utility.h. Cheapest possible place to do it: this discards
            // roughly 80% of a QT128 scan before projection, filtering or transformation.
            // Skipped when the height cut is in use, since that one runs in the map frame.
            if (useHeightCut == false && (int)rowIdn > maxRingIndex)
                continue;

            // Get column index (horizontal angle)
            horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180.0 / M_PI;
            columnIdn = static_cast<int>(round(horizonAngle / ang_res_x));
            columnIdn = (columnIdn + Horizon_SCAN) % Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;  // this condition now should never hit


            // Range filtering
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;

            // Project to organized cloud
            index = columnIdn + rowIdn * Horizon_SCAN;
            laserCloudIn->points[index] = thisPoint;
            laserCloudIn->points[index].intensity = range;  // encode range in intensity
        }
    }

    void extractRawCloud(){
        // ROS msg -> PCL cloud
        // pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // extract range info
        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                // skip beams with no return. NOTE: the original test compared intensity against
                // quiet_NaN(), which is always false (NaN never compares equal to anything), so
                // empty cells were silently registered as valid free points at range 0.
                if (laserCloudIn->points[index].intensity == -1) continue;
                // save range info
                rangeMatrix.at<float>(i, j) = laserCloudIn->points[index].intensity;
                // reset obstacle status to 0 - free 
                obstacleMatrix.at<int>(i, j) = 0;
            }
        }
    }

    bool transformCloud(){
        // Listen to the TF transform and prepare for point cloud transformation
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tfBuffer->lookupTransform(mapFrame, lidarFrame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex){ /*RCLCPP_ERROR(this->get_logger(), "Transfrom Failure.");*/ return false; }

        robotPoint.x = transform.transform.translation.x;
        robotPoint.y = transform.transform.translation.y;
        robotPoint.z = transform.transform.translation.z;

        laserCloudIn->header.frame_id = lidarFrame; // ??? this originally was "base"
        laserCloudIn->header.stamp = 0; // don't use the latest time, we don't have that transform in the queue yet

        pcl::PointCloud<PointType> laserCloudTemp;
        tmTransformPointCloud(*laserCloudIn, laserCloudTemp, transform);
        *laserCloudIn = laserCloudTemp;

        return true;
    }

    void cloud2Matrix(){

        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                PointType p = laserCloudIn->points[index];
                laserCloudMatrix[i][j] = p;
            }
        }
    }

    void applyFilter(){

        if (urbanMapping == true){
            positiveCurbFilter();
            negativeCurbFilter();
        }

        slopeFilter();
    }

    void positiveCurbFilter(){
        int rangeCompareNeighborNum = 3;
        std::vector<float> diff(Horizon_SCAN - 1);

        for (int i = 0; i < scanNumCurbFilter; ++i){
            // calculate range difference
            for (int j = 0; j < Horizon_SCAN - 1; ++j)
                diff[j] = rangeMatrix.at<float>(i, j) - rangeMatrix.at<float>(i, j+1);

            for (int j = rangeCompareNeighborNum; j < Horizon_SCAN - rangeCompareNeighborNum; ++j){

                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;

                bool breakFlag = false;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // make sure all points have valid range info
                for (int k = -rangeCompareNeighborNum; k <= rangeCompareNeighborNum; ++k)
                    if (rangeMatrix.at<float>(i, j+k) == -1){
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true) continue;
                // range difference should be monotonically increasing or decresing
                for (int k = -rangeCompareNeighborNum; k < rangeCompareNeighborNum-1; ++k)
                    if (diff[j+k] * diff[j+k+1] <= 0){
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true) continue;
                // the range difference between the start and end point of neighbor points is smaller than a threashold, then continue
                if (abs(rangeMatrix.at<float>(i, j-rangeCompareNeighborNum) - rangeMatrix.at<float>(i, j+rangeCompareNeighborNum)) /rangeMatrix.at<float>(i, j) < 0.03)
                    continue;
                // if "continue" is not used at this point, it is very likely to be an obstacle point
                obstacleMatrix.at<int>(i, j) = 1;
            }
        }
    }

    void negativeCurbFilter(){
        int rangeCompareNeighborNum = 3;

        for (int i = 0; i < scanNumCurbFilter; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                // point without range value cannot be verified
                if (rangeMatrix.at<float>(i, j) == -1)
                    continue;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // check neighbors
                for (int m = -rangeCompareNeighborNum; m <= rangeCompareNeighborNum; ++m){
                    int k = j + m;
                    if (k < 0 || k >= Horizon_SCAN)
                        continue;
                    if (rangeMatrix.at<float>(i, k) == -1)
                        continue;
                    // height diff greater than threashold, might be a negative curb
                    if (laserCloudMatrix[i][j].z - laserCloudMatrix[i][k].z > 0.1
                        && pointDistance(laserCloudMatrix[i][j], laserCloudMatrix[i][k]) <= 1.0){
                        obstacleMatrix.at<int>(i, j) = 1;
                        break;
                    }
                }
            }
        }
    }

    void slopeFilter(){
        
        for (int i = 0; i < scanNumMax; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                // point without range value cannot be verified
                if (rangeMatrix.at<float>(i, j) == -1 || rangeMatrix.at<float>(i+1, j) == -1)
                    continue;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // Calculate slope angle
                float diffX = laserCloudMatrix[i+1][j].x - laserCloudMatrix[i][j].x;
                float diffY = laserCloudMatrix[i+1][j].y - laserCloudMatrix[i][j].y;
                float diffZ = laserCloudMatrix[i+1][j].z - laserCloudMatrix[i][j].z;
                float angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;
                // Slope angle is larger than threashold, mark as obstacle point
                if (angle < -filterAngleLimit || angle > filterAngleLimit){
                    obstacleMatrix.at<int>(i, j) = 1;
                    continue;
                }
            }
        }
    }

    

    void extractFilteredCloud(){
        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                // invalid points and points too far are skipped
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit ||
                    rangeMatrix.at<float>(i, j) == -1)
                    continue;
                // Height cut - see useHeightCut in utility.h. laserCloudMatrix is in the
                // gravity-aligned map frame by this point and robotPoint is the sensor origin,
                // so this is a true vertical height above the sensor at any range and under
                // any robot attitude. Rejects canopy/ceiling returns that would otherwise share
                // an (x,y) cell with the ground beneath them and blow up its height spread.
                if (useHeightCut == true &&
                    laserCloudMatrix[i][j].z - robotPoint.z > heightCutAboveSensor)
                    continue;
                // update point intensity (occupancy) into
                PointType p = laserCloudMatrix[i][j];
                p.intensity = obstacleMatrix.at<int>(i,j) == 1 ? 100 : 0;
                // save updated points
                laserCloudOut->push_back(p);
                // extract obstacle points and convert them to laser scan
                if (p.intensity == 100)
                    laserCloudObstacles->push_back(p);
            }
        }

        // Publish laserCloudOut for visualization (before downsample and BGK prediction)
        if (pubCloudVisualHiRes->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = this->now();
            laserCloudTemp.header.frame_id = mapFrame;
            pubCloudVisualHiRes->publish(laserCloudTemp);
        }
    }

    void downsampleCloud(){

        float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
        float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;
        // height map origin
        localMapOrigin.x = roundedX - sensorRangeLimit;
        localMapOrigin.y = roundedY - sensorRangeLimit;
        
        // convert from point cloud to height map
        int cloudSize = laserCloudOut->points.size();
        for (int i = 0; i < cloudSize; ++i){

            int idx = (laserCloudOut->points[i].x - localMapOrigin.x) / mapResolution;
            int idy = (laserCloudOut->points[i].y - localMapOrigin.y) / mapResolution;
            // points out of boundry
            if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                continue;
            // obstacle point (decided by curb or slope filter)
            if (laserCloudOut->points[i].intensity == 100)
                obstFlag[idx][idy] = true;
            // save min and max height of a grid
            if (initFlag[idx][idy] == false){
                minHeight[idx][idy] = laserCloudOut->points[i].z;
                maxHeight[idx][idy] = laserCloudOut->points[i].z;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudOut->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudOut->points[i].z);
            }
        }
        // intermediate cloud
        pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>());
        // convert from height map to point cloud
        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                // no point at this grid
                if (initFlag[i][j] == false)
                    continue;
                // convert grid to point
                PointType thisPoint;
                thisPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                thisPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                thisPoint.z = maxHeight[i][j];

                if (obstFlag[i][j] == true || maxHeight[i][j] - minHeight[i][j] > filterHeightLimit){
                    obstFlag[i][j] = true;
                    thisPoint.intensity = 100; // obstacle
                    laserCloudTemp->push_back(thisPoint);
                }else{
                    thisPoint.intensity = 0; // free
                    laserCloudTemp->push_back(thisPoint);
                }
            }
        }

        *laserCloudOut = *laserCloudTemp;

        // Publish laserCloudOut for visualization (after downsample but beforeBGK prediction)
        if (pubCloudVisualLowRes->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp2;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp2);
            laserCloudTemp2.header.stamp = this->now();
            laserCloudTemp2.header.frame_id = mapFrame;
            pubCloudVisualLowRes->publish(laserCloudTemp2);
        }
    }

    void predictCloudBGK(){

        if (predictionEnableFlag == false)
            return;

        int kernelGridLength = int(predictionKernalSize / mapResolution);

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                // skip observed point
                if (initFlag[i][j] == true)
                    continue;
                PointType testPoint;
                testPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                testPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                testPoint.z = robotPoint.z; // this value is not used except for computing distance with robotPoint
                // skip grids too far
                if (pointDistance(testPoint, robotPoint) > sensorRangeLimit)
                    continue;
                // Training data
                vector<float> xTrainVec; // training data x and y coordinates
                vector<float> yTrainVecElev; // training data elevation
                vector<float> yTrainVecOccu; // training data occupancy
                // Fill trainig data (vector)
                for (int m = -kernelGridLength; m <= kernelGridLength; ++m){
                    for (int n = -kernelGridLength; n <= kernelGridLength; ++n){
                        // skip grids too far
                        if (std::sqrt(float(m*m + n*n)) * mapResolution > predictionKernalSize)
                            continue;
                        int idx = i + m;
                        int idy = j + n;
                        // index out of boundry
                        if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                            continue;
                        // save only observed grid in this scan
                        if (initFlag[idx][idy] == true){
                            xTrainVec.push_back(localMapOrigin.x + idx * mapResolution + mapResolution / 2.0);
                            xTrainVec.push_back(localMapOrigin.y + idy * mapResolution + mapResolution / 2.0);
                            yTrainVecElev.push_back(maxHeight[idx][idy]);
                            yTrainVecOccu.push_back(obstFlag[idx][idy] == true ? 1 : 0);
                        }
                    }
                }
                // no training data available, continue
                if (xTrainVec.size() == 0)
                    continue;
                // convert from vector to eigen
                Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
                Eigen::MatrixXf yTrainElev = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecElev.data(), yTrainVecElev.size(), 1);
                Eigen::MatrixXf yTrainOccu = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecOccu.data(), yTrainVecOccu.size(), 1);
                // Test data (current grid)
                vector<float> xTestVec;
                xTestVec.push_back(testPoint.x);
                xTestVec.push_back(testPoint.y);
                Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);
                // Predict
                Eigen::MatrixXf Ks; // covariance matrix
                covSparse(xTest, xTrain, Ks); // sparse kernel

                Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
                Eigen::MatrixXf ybarOccu = (Ks * yTrainOccu).array();
                Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

                // Update Elevation with Prediction
                if (std::isnan(ybarElev(0,0)) || std::isnan(ybarOccu(0,0)) || std::isnan(kbar(0,0)))
                    continue;

                if (kbar(0,0) == 0)
                    continue;

                float elevation = ybarElev(0,0) / kbar(0,0);
                float occupancy = ybarOccu(0,0) / kbar(0,0);

                PointType p;
                p.x = xTestVec[0];
                p.y = xTestVec[1];
                p.z = elevation;
                p.intensity = occupancy*100;

                laserCloudOut->push_back(p);
            }
        }
    }

    void dist(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &d) const {
        d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
        for (int i = 0; i < xStar.rows(); ++i) {
            d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
        }
    }

    void covSparse(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &Kxz) const {
        dist(xStar/(predictionKernalSize+0.1), xTrain/(predictionKernalSize+0.1), Kxz);
        Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
              (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * 1.0f;
        // Clean up for values with distance outside length scale, possible because Kxz <= 0 when dist >= predictionKernalSize
        for (int i = 0; i < Kxz.rows(); ++i)
            for (int j = 0; j < Kxz.cols(); ++j)
                if (Kxz(i,j) < 0) Kxz(i,j) = 0;
    }

    void publishCloud(){
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
        laserCloudTemp.header.stamp = this->now();
        laserCloudTemp.header.frame_id = mapFrame;
        pubCloud->publish(laserCloudTemp);
    }

    void publishLaserScan(){

        updateLaserScan();

        laserScan.header.stamp = this->now();
        pubLaserScan->publish(laserScan);
        // initialize laser scan for new scan
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0f);
    }

    void updateLaserScan(){

        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tfBuffer->lookupTransform(baseFrame, mapFrame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex){ /*RCLCPP_ERROR(this->get_logger(), "Transfrom Failure.");*/ return; }

        laserCloudObstacles->header.frame_id = mapFrame;
        laserCloudObstacles->header.stamp = 0;
        // transform obstacle cloud back to "base" frame
        pcl::PointCloud<PointType> laserCloudTemp;
        tmTransformPointCloud(*laserCloudObstacles, laserCloudTemp, transform);
        //convert point to scan
        int cloudSize = laserCloudTemp.points.size();
        for (int i = 0; i < cloudSize; ++i){
            PointType *point = &laserCloudTemp.points[i];
            float x = point->x;
            float y = point->y;
            float range = std::sqrt(x*x + y*y);
            float angle = std::atan2(y, x);
            int index = (angle - laserScan.angle_min) / laserScan.angle_increment;
            if (index >= 0 && index < (int)laserScan.ranges.size())
                laserScan.ranges[index] = std::min(laserScan.ranges[index], range);
        } 
    }

    void pointcloud2laserscanInitialization(){

        laserScan.header.frame_id = baseFrame; // assume laser has the same frame as the robot

        laserScan.angle_min = -M_PI;
        laserScan.angle_max =  M_PI;
        laserScan.angle_increment = 1.0f / 180 * M_PI;
        laserScan.time_increment = 0;

        laserScan.scan_time = 0.1;
        laserScan.range_min = 0.3;
        laserScan.range_max = 100;

        int range_size = std::ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment);
        laserScan.ranges.assign(range_size, laserScan.range_max + 1.0f);
    }
};


int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto TFilter = std::make_shared<TraversabilityFilter>(
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO(TFilter->get_logger(), "\033[1;32m---->\033[0m Traversability Filter Started.");

    rclcpp::spin(TFilter);

    rclcpp::shutdown();
    return 0;
}