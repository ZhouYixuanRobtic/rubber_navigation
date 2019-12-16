//
// Created by xcy on 2019/12/12.
//
#include "ros/ros.h"
#include "rubber_navigation/BaseController.h"
#include "ros/package.h"
#include "../../Visual-Servo/include//visual_servo/JoyTeleop.h"
int main(int argc,char* argv[])
{
    ros::init(argc,argv,"baseOnly");
    ros::NodeHandle nh_("~");
    std::string base_foot_print,odom_frame,map_frame;
    nh_.param("base_foot_print",base_foot_print,(std::string)"/base_link");
    nh_.param("odom_frame",odom_frame,(std::string)"/odom");
    const std::string parameter_addr{ros::package::getPath("rubber_navigation")+"/config/BaseModel.yaml"};

    BaseController baseController("/dev/ttyUSB0",B115200,base_foot_print,odom_frame);
    baseController.setBaseModel(parameter_addr);

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Rate loop_rate(30);



    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}
