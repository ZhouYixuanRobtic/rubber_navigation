#ifndef RUBBER_NAVIGATION_NAVCORE_H
#define RUBBER_NAVIGATION_NAVCORE_H

#include "ros/ros.h"
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <move_base_msgs/MoveBaseGoal.h>
#include "std_srvs/Empty.h"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "ros/time.h"

#include "rubber_navigation/BaseController.h"
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <Eigen/Core>

class NavCore {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
public:
    enum MoveBaseActionResult {
        PENDDING,
        ACTIVE,
        PREEMPTED,
        SUCCEEDED,
        ABORTED,
        REJECTED,
        PREEMPTING,
        RECALLING,
        RECALLED,
        LOST,
        EMPTY,
    };
private:
    const std::string BASE_FOOT_PRINT_;
    const std::string MAP_FRAME_;

    ros::NodeHandle nh;
    ros::Subscriber action_result_sub;

    bool isMoveBaseClientConnected_{};
    MoveBaseClient *moveBaseClient;
    MoveBaseActionResult moveBaseActionResult_{EMPTY};
    move_base_msgs::MoveBaseGoal goal_{};

    std_srvs::Empty clear_costmap_srv_;
    ros::ServiceClient client;

    geometry_msgs::Pose2D current_pose_{};
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    void actionResultCallback(const move_base_msgs::MoveBaseActionResult &msg);


public:

    NavCore(std::string base_foot_print, std::string map_frame);

    ~NavCore();

    bool isMoveBaseClientConnected()const {return isMoveBaseClientConnected_;};
    bool clearCostMap();
    void cancelAllGoals();

    void setGoal(const geometry_msgs::Pose2D &goal2d);

    MoveBaseActionResult getMoveBaseActionResult() { MoveBaseActionResult temp{moveBaseActionResult_}; moveBaseActionResult_=MoveBaseActionResult::EMPTY;return temp;};

    const geometry_msgs::Pose2D &getCurrentPose(const std::string &target_frame, const std::string &source_frame);

    bool isGoalPassed(const geometry_msgs::Pose2D &goal_pose);
};


#endif //RUBBER_NAVIGATION_NAVCORE_H
