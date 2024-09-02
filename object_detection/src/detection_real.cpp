#include <detection.h>

Object_Detection::Object_Detection(ros::NodeHandle* nodeHandle) : counter(0){
    ROS_INFO("start detection");
    nh = *nodeHandle;  
    
    // Publisher 
    detection_image = nh.advertise<sensor_msgs::Image>("/calibration_image", 1);   
    detection_msg = nh.advertise<detect_msgs::detected_array>("/calibration_msg", 1);  
    cloud_filter = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filter", 1);  
    cloud_cluster = nh.advertise<sensor_msgs::PointCloud2>("/cloud_cluster", 1); 
    cloud_centeroid = nh.advertise<sensor_msgs::PointCloud2>("/cloud_centeroid", 1); 
    tracking_id = nh.advertise<visualization_msgs::MarkerArray>("/tracking_id", 1);

    // Parameter
    // 1) Topic 
    nh.param<std::string>("lidar_topic", lidar_topic, "/livox/lidar"); 
    nh.param<std::string>("camera_topic", camera_topic, "/camera/image_raw");
    nh.param<std::string>("yolo_topic", yolo_topic, "/yolov8_pub"); 
    nh.param<std::string>("frame_name", frame_name, "livox_frame");  

    // 2) ROI Filtering 
    nh.param<double>("xMinRange", xMinRange,  0);
    nh.param<double>("xMaxRange", xMaxRange,  20);
    nh.param<double>("yMinRange", yMinRange, -6);
    nh.param<double>("yMaxRange", yMaxRange,  6);
    nh.param<double>("zMinRange", zMinRange, -0.5);
    nh.param<double>("zMaxRange", zMaxRange,  0.0);

    // 3) Cluster
    nh.param<double>("cluster_tolerance", cluster_tolerance, 1.0); 
    nh.param<int>("cluster_min", cluster_min, 4); 
    nh.param<int>("cluster_max", cluster_max, 100); 

    // Subscriber 
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidar_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> camera_sub(nh, camera_topic, 10);
    message_filters::Subscriber<detect_msgs::Yolo_Objects> yolo_sub(nh, yolo_topic, 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, detect_msgs::Yolo_Objects> Sync;
    boost::shared_ptr<message_filters::Synchronizer<Sync>> sync;
    sync.reset(new message_filters::Synchronizer<Sync>(Sync(10), lidar_sub, camera_sub, yolo_sub));
    sync->registerCallback(boost::bind(&Object_Detection::Detection_Callback, this, _1, _2, _3));

    ROS_INFO("lidar_topic: %s", lidar_topic.c_str());
    ROS_INFO("camera_topic: %s", camera_topic.c_str());
    ROS_INFO("yolo_topic: %s", yolo_topic.c_str());
    ROS_INFO("frame_name: %s", frame_name.c_str());

    ROS_INFO("xMinRange: %f", xMinRange);
    ROS_INFO("xMaxRange: %f", xMaxRange);
    ROS_INFO("yMinRange: %f", yMinRange);
    ROS_INFO("yMaxRange: %f", yMaxRange);
    ROS_INFO("zMinRange: %f", zMinRange);
    ROS_INFO("zMaxRange: %f", zMaxRange);
    
    filter_range.reset(new pcl::ConditionAnd<pcl::PointXYZI>());
    filter_range->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, xMaxRange)));
    filter_range->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, xMinRange)));
    filter_range->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, yMaxRange)));
    filter_range->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, yMinRange)));
    filter_range->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, zMaxRange)));
    filter_range->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, zMinRange)));

    Object_Detection::read_projection_matrix();

    ros::spin();
}

Object_Detection::~Object_Detection(){
    ROS_INFO("finish detection");
}

void Object_Detection::Detection_Callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg, 
                                         const sensor_msgs::Image::ConstPtr& camera_msg, 
                                         const detect_msgs::Yolo_Objects::ConstPtr& yolo_msg){
    
    ROS_INFO("Callback...");

    camera_image = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8)->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*lidar_msg, *pointcloud);

    // Filtering
    filter.setCondition(filter_range);
    filter.setInputCloud(pointcloud);
    filter.setKeepOrganized(false);
    filter.filter(*pointcloud);

    // Publish Filltered
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*pointcloud, filtered_msg);
    filtered_msg.header.frame_id = frame_name;
    cloud_filter.publish(filtered_msg);

    // Save Distance
    for (int i = 0; i < pointcloud->size(); i++){
        cv::Point3d point;
        point.x = pointcloud->points[i].x;
        point.y = pointcloud->points[i].y;
        point.z = pointcloud->points[i].z;
        lidar_points.push_back(point);
        intensity_list.push_back(pointcloud->points[i].intensity);

        double distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        distance_list.push_back(distance);
    }

    // Save Projected Points
    if (lidar_points.size() > 0){
        cv::perspectiveTransform(lidar_points, projected_list, projection_matrix);

        for (int i = 0; i < projected_list.size(); i++){
            double u1 = projected_list[i].x;
            double v1 = projected_list[i].y;
            cv::circle(camera_image, cv::Point2d(u1, v1), 2, Object_Detection::scalarHSV2BGR(intensity_list[i]*(180.0/255.0), 255, 255), -1);
        }
        
        convert_msg(yolo_msg, lidar_msg->header);
    }

    std::vector<double>().swap(distance_list);
    std::vector<double>().swap(intensity_list);
    std::vector<cv::Point3d>().swap(lidar_points);
    std::vector<cv::Point2d>().swap(projected_list);

    // Image Publish
    cv_bridge::CvImage image_bridge;
    sensor_msgs::Image image_msg;
    image_bridge = cv_bridge::CvImage(camera_msg->header, sensor_msgs::image_encodings::BGR8, camera_image);
    image_bridge.toImageMsg(image_msg);
    detection_image.publish(image_msg);
}

void Object_Detection::convert_msg(const detect_msgs::Yolo_Objects::ConstPtr& yolo_msg, std_msgs::Header header){
    detect_msgs::detected_array object_array;
    visualization_msgs::MarkerArray text_array;
    std::vector<object_struct> object_struct_list;

    // Add ID Text Marker
    for (int i = 0; i < counter; i++){
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.action = visualization_msgs::Marker::DELETE;
        text_array.markers.push_back(marker);
    }

    counter = 0;
    
    // YOLO 2D bbox
    for (int i = 0; i < yolo_msg->yolo_objects.size(); i++){
        int Class = yolo_msg->yolo_objects[i].Class;
        int id = yolo_msg->yolo_objects[i].id;

        int x_min = yolo_msg->yolo_objects[i].x1;
        int y_min = yolo_msg->yolo_objects[i].y1;
        int x_max = yolo_msg->yolo_objects[i].x2;
        int y_max = yolo_msg->yolo_objects[i].y2;
        double min_distance = 1e3;
   
        object_struct object_info = {999};
        
        if (x_max  - x_min < 30) continue; // Shorter than min width
        cv::rectangle(camera_image, cv::Rect(cv::Point2d(x_min, y_min), cv::Point2d(x_max, y_max)), cv::Scalar(255, 255, 255), 2);
        cv::rectangle(camera_image, cv::Rect(cv::Point2d (x_min + 1, y_min + 1), cv::Point2d(x_max -1 , y_max - 1)), cv::Scalar(255, 255, 0), 1);

        for (int j = 0; j < projected_list.size(); j++){
            double u = projected_list[j].x;
            double v = projected_list[j].y;
            if (u < x_min + 10 || u > x_max - 10|| v < y_min + 10 || v > y_max - 10) continue; // Points in bbox

            double dist = distance_list[j];
            if (dist < min_distance) min_distance = dist;
        }

        // Projected Points
        for (int j = 0; j < projected_list.size(); j++){
            double u = projected_list[j].x;
            double v = projected_list[j].y;
            if (u < x_min + 10 || u > x_max - 10|| v < y_min + 10 || v > y_max - 10) continue; // Points in bbox

            double dist = distance_list[j];
            if (abs(min_distance - dist) > 1.5) continue; // Points around min distance

            object_info.x = lidar_points[j].x;
            object_info.y = lidar_points[j].y;
            object_info.z = lidar_points[j].z;
            object_info.u = projected_list[j].x;
            object_info.v = projected_list[j].y;
            object_info.id = id;
            object_info.x_min = x_min;
            object_info.y_min = y_min;
            object_info.x_max = x_max;
            object_info.y_max = y_max;
            
            object_struct_list.push_back(object_info);

            cv::circle(camera_image, cv::Point2d(u, v), 3, cv::Scalar(255, 255, 255), -1);
        }
    }

    // Save Cloud of 2D bbox
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < object_struct_list.size(); i++){
        object_struct object_info = object_struct_list[i];
        pcl::PointXYZ point;
        point.x = object_info.x;
        point.y = object_info.y;
        point.z = object_info.z;
        cloud->points.push_back(point);
    }

    // Ground Filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filtered = ground_filter(*cloud);

    // Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;

    if (!filtered->empty()){
        tree->setInputCloud(filtered);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(cluster_min);   
        ec.setMaxClusterSize(cluster_max);  
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered);
        ec.extract(cluster_indices);
    }

    // Find Cluster Centeroid
    pcl::PointCloud<pcl::PointXYZI> cluster_list; 
    pcl::PointCloud<pcl::PointXYZI> centeroid_list; 
    std::vector<cv::Point3d> centeroid_list_cv;
    int intensity = 0;
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        pcl::PointXYZI centeroid;
        cv::Point3d centeroid_cv;
        int total_points = 0;

    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        	pcl::PointXYZ point = filtered->points[*pit];
            centeroid.x += point.x;
            centeroid.y += point.y;
            centeroid.z += point.z;
            total_points++;

            pcl::PointXYZI cluster_point;
            cluster_point.x = point.x;
            cluster_point.y = point.y;
            cluster_point.z = point.z;
            cluster_point.intensity = (float)(intensity + 1);
            cluster_list.push_back(cluster_point);
    	}

        centeroid.intensity = (float)(intensity + 1);
        centeroid.x /= total_points;
        centeroid.y /= total_points;
        centeroid.z /= total_points;
        centeroid_list.push_back(centeroid);
        intensity++;

        centeroid_cv.x = centeroid.x;
        centeroid_cv.y = centeroid.y;
        centeroid_cv.z = centeroid.z;
        centeroid_list_cv.push_back(centeroid_cv);
    }
    
    // Publish Cluster
    sensor_msgs::PointCloud2 cluster_msg;
    pcl::toROSMsg(cluster_list, cluster_msg);
    cluster_msg.header.frame_id = frame_name;
    cloud_cluster.publish(cluster_msg);

    // Publish Centeroid
    sensor_msgs::PointCloud2 centeroid_msg;
    pcl::toROSMsg(centeroid_list, centeroid_msg);
    centeroid_msg.header.frame_id = frame_name;
    cloud_centeroid.publish(centeroid_msg);

    // Project Centeroid to 2D Points
    std::vector<cv::Point2d> projected_centeroid_list;

    if (centeroid_list_cv.size() > 0){
        cv::perspectiveTransform(centeroid_list_cv, projected_centeroid_list, projection_matrix);
    }   
    
    // Find Centeroid ID
    for (int i = 0; i < projected_centeroid_list.size(); i++){ 
        cv::Point2d projected_centeroid = projected_centeroid_list[i];
        int u = projected_centeroid.x;
        int v = projected_centeroid.y;
        double min_distance = 1e6;
        int min_distance_id = -1;

        for (int j = 0; j < object_struct_list.size(); j++){ // Points in 2D bbox
            object_struct object_info = object_struct_list[j];
            int x_min = object_info.x_min; 
            int y_min = object_info.y_min; 
            int x_max = object_info.x_max; 
            int y_max = object_info.y_max; 
            int id = object_info.id;
           
            int center_x = (x_min + x_max) / 2;
            int center_y = (y_min + y_max) / 2;
            int dist = pow(center_x - u, 2) + pow(center_y - v, 2);

            if (dist < min_distance){ // Find min distance ID
                min_distance = dist;
                min_distance_id = id;
            }
        }

        cv::circle(camera_image, cv::Point2d(u, v), 10, cv::Scalar(255, 0, 255), -1);

        visualization_msgs::Marker id_text = create_text(min_distance_id, centeroid_list_cv[i].x, centeroid_list_cv[i].y, counter);
        text_array.markers.push_back(id_text);

        // Save Centeroid
        detect_msgs::detected_object object;
        object.id = min_distance_id;
        object.world_point.position.x = centeroid_list_cv[i].x;
        object.world_point.position.y = centeroid_list_cv[i].y;
        object.world_point.position.z = centeroid_list_cv[i].z;
        object_array.objects.push_back(object);
        counter++; 
    }

    // Publish Centeroid Info
    tracking_id.publish(text_array);
    detection_msg.publish(object_array);
}

// LiDAR-Camera Projection Matrix
void Object_Detection::read_projection_matrix(){
    // Real Parameter
    double fx = 1.8555e+03;
    double fy = 1.8549e+03;
    double cx = 950.6236;
    double cy = 575.6943;

    double k1 = -0.2981;
    double k2 = 0.1189;
    double p1 = 0.0000;
    double p2 = 0.0000;
    double k3 = 0.0000;

    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
        fx, 0,  cx, 
        0,  fy, cy, 
        0,  0,  1);

    cv::Mat T = (cv::Mat_<double>(3 ,4) <<
        -0.0259,    -0.9994,    0.0212,     0.0269,
        -0.0177,    -0.0207,    -0.9996,    -0.1107,
        0.9995,     -0.0263,    -0.0172,    -0.0358,
        0.0000,     0.0000,     0.0000,     1.0000);

    projection_matrix = camera_matrix * T;   
}

cv::Scalar Object_Detection::scalarHSV2BGR(uchar h, uchar s, uchar v){
    cv::Mat RGB;
    cv::Mat HSV(1,1, CV_8UC3, cv::Scalar(h, s, v));
    cvtColor(HSV, RGB, CV_HSV2BGR);

    return cv::Scalar(RGB.data[0], RGB.data[1], RGB.data[2]);
}

// LiDAR Ground Filter
pcl::PointCloud<pcl::PointXYZ>::Ptr Object_Detection::ground_filter(pcl::PointCloud<pcl::PointXYZ> cloud){
    double height_thresh = 0.0; 
    int grid_dim = 320; 
    double per_cell = 0.2; // Increase the interval between Points 

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filtered->points.resize(cloud.size());

    float min[grid_dim][grid_dim];
    float max[grid_dim][grid_dim];
    bool init[grid_dim][grid_dim];
    size_t obstacle_count = 0;

    memset(&min, 0, grid_dim * grid_dim * sizeof(float));
    memset(&max, 0, grid_dim * grid_dim * sizeof(float));
    memset(&init, false, grid_dim * grid_dim * sizeof(bool));

    for (int i = 0; i < cloud.size(); i++){
        int x = ((grid_dim / 2) + cloud[i].x / per_cell); 
        int y = ((grid_dim / 2) + cloud[i].y / per_cell);

        if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim){
            if (!init[x][y]){
                min[x][y] = cloud[i].z;
                max[x][y] = cloud[i].z;
                init[x][y] = true;
            }

            else{
                min[x][y] = MIN(min[x][y], cloud[i].z);
                max[x][y] = MAX(max[x][y], cloud[i].z);
            }
        }
    }

    double grid_offset = grid_dim / 2.0 * per_cell;
    for (int x = 0; x < grid_dim; x++)
        for (int y = 0; y < grid_dim; y++){
            if ((max[x][y] - min[x][y]) > height_thresh){
                filtered->points[obstacle_count].x = -grid_offset + (x * per_cell + per_cell / 2.0);
                filtered->points[obstacle_count].y = -grid_offset + (y * per_cell + per_cell / 2.0);
                filtered->points[obstacle_count].z = -0.1; // height set
                obstacle_count++;
            }
        }

    filtered->points.resize(obstacle_count);
   
    return filtered;
}

// Text ID Marker
visualization_msgs::Marker Object_Detection::create_text(int id, double x, double y, int counter){
    visualization_msgs::Marker id_text;
    id_text.header.frame_id = frame_name;
    id_text.header.stamp = ros::Time::now();
    id_text.text = "ID: " + std::to_string(id);
    id_text.color.r = 1.0;
    id_text.color.g = 1.0;
    id_text.color.b = 1.0;
    id_text.color.a = 1.0;
    id_text.scale.z = 1.0;
    id_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    id_text.id = counter;
    id_text.action = visualization_msgs::Marker::ADD;
    id_text.pose.orientation.w = 1.0;
    id_text.pose.position.x = x; 
    id_text.pose.position.y = y; 
    return id_text;
}