#include "ros/ros.h"
#include "openni2_tracker.h"



int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "nite_tracking");
    
    OpenNI2Tracker ot(ros::this_node::getName());
    
    ROS_INFO("... and we're spinning in the main thread!");
    ros::MultiThreadedSpinner().spin();
    
    return 0;
}