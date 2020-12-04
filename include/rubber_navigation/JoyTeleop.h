#ifndef VISUAL_SERVO_JOYTELEOP_H
#define VISUAL_SERVO_JOYTELEOP_H
#include <utility>

#include"ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "atomic"
namespace JOYTELEOP
{
    enum ControlTrigger{
        Default,
        NavOn,
        NavPause,
        ManipulateOn,
        ChargeOn,
        UpOn,
        HomeOn,
        AntiClockGo,
        ClockGo,
        SingleAntiClockGo,
        SingleClockGo,
        KnifeOn,
        KnifeOff,
        KnifeUnplug,
        LightOn,
        LightOff,
		SteeringIn,
		SteeringOut,
        MappingOn,
        MappingOff,
        RobotArmOn,
        NavigationOn,
        ShutDown,
        SaveCutPoint,
        SaveChargePoint,
        SaveNavPoint,
        SaveTurnPoint,
        ArmEmergencyChange,
		LinearUp,
		LinearDown,
		LinearForward,
		LinearBack,
		CutBack
    };
    class JoyTeleop {
    private:
        const std::string TOPIC_NAME_;
        const double WATCHDOG_PERIOD_=2.0;

        ros::NodeHandle nh_;
        ros::Timer watchdog_timer_;
        ros::Subscriber Joy_sub_;
        ros::Publisher joy_vel_pub;
        bool publish_vel_{};
        ControlTrigger control_trigger_{};
        double max_linear_velocity_{};
        double max_angular_velocity_{};

        std::atomic_bool joy_alive_{};
        mutable boost::shared_mutex control_trigger_mutex_;
    public:
        explicit JoyTeleop(std::string topic_name,bool publish_vel=false,double max_linear_velocity=0.8,double max_angular_velocity=0.5);
        ~JoyTeleop();
        void JoyCallback(const sensor_msgs::JoyConstPtr & msg);
        void watchdog(const ros::TimerEvent &e);
        ControlTrigger getControlTrigger()
        {
            boost::unique_lock<boost::shared_mutex> writLock(control_trigger_mutex_);
            ControlTrigger temp{control_trigger_};
            control_trigger_=Default;
            return temp;
        };
    };
}



#endif //VISUAL_SERVO_JOYTELEOP_H