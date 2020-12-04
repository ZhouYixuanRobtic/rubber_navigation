#include <iostream> //标准输入输出流
#include "rubber_navigation/NavCore.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include "geometry_msgs/Pose2D.h"

#include "../../Visual-Servo/include/visual_servo/JoyTeleop.h"

struct targetPose{
    geometry_msgs::Pose2D pose;
    enum TargetAction{
        DEFAULT,
        TAP,
        CHARGE,
        NAV,
        TURN
    }targetAction{};
};

void getPoseArray(std::vector<targetPose> & targetPoseArray)
{
    const std::string pose_addr{ros::package::getPath("rubber_navigation")+"/config/treeTarget.txt"};
    std::ifstream input_file(pose_addr.c_str());
    targetPoseArray.clear();
    if(input_file.is_open())
    {
        std::string str;
        targetPose target_pose{};
        while(getline(input_file,str)&&!str.empty())
        {
            std::istringstream stringGet(str);
            int temp;
            stringGet>>temp>>target_pose.pose.x>>target_pose.pose.y>>target_pose.pose.theta;
            target_pose.targetAction=targetPose::TargetAction(temp);
            targetPoseArray.push_back(target_pose);
        }
        input_file.close();
    }
}
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"rubber_navigation");
    ros::NodeHandle nh_("~");
    ros::Rate loop_rate(60);

    const std::string parameter_addr{ros::package::getPath("rubber_navigation")+"/config/BaseModel.yaml"};
    std::string base_foot_print,odom_frame,map_frame,serial_addr;
    bool publish_tf;

    nh_.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh_.param("odom_frame",odom_frame,(std::string)"odom");
    nh_.param("map_frame",map_frame,(std::string)"map");
    nh_.param("publish_tf",publish_tf,(bool)false);
    nh_.param("serial_addr",serial_addr,(std::string)"/dev/ttyS0");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    BaseController baseController(serial_addr,B115200,base_foot_print,odom_frame,publish_tf);
    baseController.setBaseModel(parameter_addr);

    NavCore navCore(base_foot_print,map_frame);

    double max_linear_velocity,max_angular_velocity;
    nh_.param("max_linear_velocity",max_linear_velocity,(double)0.8);
    nh_.param("max_angular_velocity",max_angular_velocity,(double)0.5);

    JOYTELEOP::JoyTeleop joyTeleop("joy",true,max_linear_velocity,max_angular_velocity);

    bool nav_on{},nav_pause{},newGoal{true};
    std::vector<targetPose> targetPoseArray{};
    getPoseArray(targetPoseArray);
    auto iter = targetPoseArray.begin();
    while(ros::ok())
    {
        switch (joyTeleop.getControlTrigger())
        {
            case JOYTELEOP::NavOn:
            {
                if(nav_pause&&iter!=targetPoseArray.end())
                {
                    if(navCore.isGoalPassed((*iter).pose))
                        iter++;
                    newGoal = true;
                }
                nav_on = true;
                ROS_INFO("NavOn received, ready to go");
                nav_pause=false;
                break;
            }
            case JOYTELEOP::NavPause:
            {
                nav_on=false;
                nav_pause=true;
                ROS_INFO("NavPause received, cancel all goals");
                navCore.cancelAllGoals();
                break;
            }
            default:
                break;
        }

        switch (navCore.getMoveBaseActionResult())
        {
            case NavCore::SUCCEEDED:
            {
                iter++;
                newGoal=true;
                break;
            }
            case NavCore::ABORTED:
            {
                ROS_ERROR_STREAM("ROBOT ABORTED ");
                if(navCore.clearCostMap())
                {
                    iter++;
                    newGoal=true;
                }
                else
                    ROS_ERROR_STREAM("Failed to call clear cost map service");

                break;
            }
            default:
                break;
        }


        if(nav_on && !targetPoseArray.empty()&&newGoal)
        {
            if(iter != targetPoseArray.end())
            {
                ROS_INFO_STREAM("now goal: x is "<<(*iter).pose.x<<" y is "<<(*iter).pose.y<<" theata is "<<(*iter).pose.theta);
                navCore.setGoal((*iter).pose);
            }
            else
            {
                getPoseArray(targetPoseArray);
                iter = targetPoseArray.begin();
                ROS_INFO("Reach the end of the goal list, reload ree target");
                nav_on=false;
            }
            newGoal=false;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}