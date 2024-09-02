#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <detect_msgs/detected_object.h>
#include <detect_msgs/detected_array.h>
#include <detect_msgs/Yolo_Objects.h>
#include <detect_msgs/Objects.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <morai_msgs/EgoVehicleStatus.h>
#include <std_msgs/Int32.h>

#include <kalman_filter.h>

struct object_struct{
    double x;
    double y;
    double z;
    double u;
    double v;
    int id;
    int x_min;
    int y_min;
    int x_max;
    int y_max;
};

class Object_Detection{
private:    
    ros::NodeHandle nh;
    ros::Publisher detection_image;
    ros::Publisher detection_msg;
    ros::Publisher cloud_filter;
    ros::Publisher cloud_cluster;
    ros::Publisher cloud_centeroid;
    ros::Publisher tracking_id;
    ros::Publisher tracking_vel;
    
    // ROS message Topic
    std::string lidar_topic, camera_topic, yolo_topic, frame_name;
    
    // LiDAR Filtering
    pcl::ConditionalRemoval<pcl::PointXYZI> filter;
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr filter_range;
    double xMinRange, xMaxRange, yMinRange, yMaxRange, zMinRange, zMaxRange;

    // Clustering Parameter
    double cluster_tolerance;
    int cluster_min;
    int cluster_max;
    
    // Projection Matrix, Image
    cv::Mat projection_matrix;
    cv::Mat camera_image;

    std::vector<double> distance_list;          // Distance info
    std::vector<double> intensity_list;         // intensity_list info
    std::vector<cv::Point3d> lidar_points;      // 3D(X,Y,Z) info
    std::vector<cv::Point2d> projected_list;    // 2D(u,v) info
    
    // Function
    void read_projection_matrix();
    cv::Scalar scalarHSV2BGR(uchar h, uchar s, uchar v);
    void convert_msg(const detect_msgs::Yolo_Objects::ConstPtr& yolo_msg, std_msgs::Header header);

    // Callback
    void Detection_Callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg, 
                           const sensor_msgs::Image::ConstPtr& camera_msg, 
                           const detect_msgs::Yolo_Objects::ConstPtr& yolo_msg);
    void Ego_Callback(const morai_msgs::EgoVehicleStatus& ego_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filter(pcl::PointCloud<pcl::PointXYZ> cloud);
    visualization_msgs::Marker create_text(int id, double x, double y, int counter);

    int counter;
    double point_height;
    double current_vel;

    int bbox_width;
    double dist_thresh;

    // Kalman filter
    KalmanFilter kf;
    double dt;            
    double p1; 
    double p2; 
    double q1; 
    double q2; 
    double r;

    double prev_vel;
    double max_vel;
    double min_vel;
 
public:
    Object_Detection(ros::NodeHandle* nodeHandle);
    ~Object_Detection();
};

#endif 