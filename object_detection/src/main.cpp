#include <detection.h>

int main(int argc, char** argv){
    ROS_INFO("Object Detection");

    ros::init(argc, argv, "Object_Detection");
    ros::NodeHandle nodeHandle("~");

    Object_Detection Object_Detection(&nodeHandle);

    return 0;
}