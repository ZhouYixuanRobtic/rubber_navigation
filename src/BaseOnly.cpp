//
// Created by xcy on 2019/12/12.
//
#include "ros/ros.h"
#include "rubber_navigation/BaseController.h"
#include "ros/package.h"
#include "rubber_navigation/JoyTeleop.h"
#include "BaseController.cpp"
int main(int argc,char* argv[])
{
    ros::init(argc,argv,"baseOnly");
    ros::NodeHandle nh_("~");
    const std::string parameter_addr{ros::package::getPath("rubber_navigation")+"/config/BaseModel.yaml"};
    double max_linear_velocity,max_angular_velocity;
    std::string base_foot_print,odom_frame,map_frame,serial_addr;
    bool publish_tf;

    nh_.param("publish_tf",publish_tf,(bool)false);
    nh_.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh_.param("odom_frame",odom_frame,(std::string)"odom");
    nh_.param("max_linear_velocity",max_linear_velocity,(double)0.8);
    nh_.param("max_angular_velocity",max_angular_velocity,(double)0.5);
    nh_.param("serial_addr",serial_addr,(std::string)"/dev/ttyS0");

    JOYTELEOP::JoyTeleop joyTeleop("joy",true,max_linear_velocity,max_angular_velocity);

    BaseController baseController(serial_addr,B115200,base_foot_print,odom_frame,publish_tf);
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
