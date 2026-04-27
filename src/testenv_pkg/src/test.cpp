#include "ros/ros.h"

int main(int argc , char* argv[]){
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"HelloVsCode");

    ROS_INFO("[INFO]:hello vs code");

    return 0;
}