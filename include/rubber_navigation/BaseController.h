#ifndef RUBBER_NAVIGATION_BASECONTROLLER_H
#define RUBBER_NAVIGATION_BASECONTROLLER_H

#include <yaml-cpp/yaml.h>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <climits>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/timer.h"
#include "ros/wall_timer.h"
#include "rubber_navigation/NaviSerialManager.h"
#include "rubber_navigation/WheelStatus.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node& node, T& i) {
  i = node.as<T>();
}
#endif
#define _USE_MATH_DEFINES

//割胶导轨控制协议  电压第六位，第七位时间 /50ms
unsigned char clockGo[COMMAND_SIZE] = {0x53, 0x02, 0x39, 0x03,
                                       0x20, 0x50, 0x00, 0x00};
unsigned char antiClockGo[COMMAND_SIZE] = {0x53, 0x02, 0x39, 0x04,
                                           0x20, 0x50, 0x00, 0x00};
// unsigned char clockGo[COMMAND_SIZE] = {0x53, 0x02, 0x39, 0x01,
//                                        0x80, 0x60, 0x00, 0x00};
// unsigned char antiClockGo[COMMAND_SIZE] = {0x53, 0x02, 0x39, 0x02,
//                                            0x80, 0x60, 0x00, 0x00};
unsigned char singleClockGo[COMMAND_SIZE] = {0x53, 0x02, 0x39, 0x03,
                                             0x80, 0x00, 0x37, 0x00};
unsigned char singleAntiClockGo[COMMAND_SIZE] = {0x53, 0x02, 0x39, 0x04,
                                                 0x80, 0x00, 0x37, 0x00};
//割胶刀控制协议    电压第六位，第七位延时 /500ms
unsigned char knifeOn[COMMAND_SIZE] = {0x53, 0x02, 0x33, 0xFF,
                                       0x00, 0x00, 0x00, 0x00};
unsigned char knifeOff[COMMAND_SIZE] = {0x53, 0x02, 0x33, 0x00,
                                        0x00, 0x00, 0x00, 0x00};
unsigned char knifeUnplug[COMMAND_SIZE] = {0x53, 0x02, 0x38, 0xFF,
                                           0x00, 0x00, 0x00, 0x00};

unsigned char turn_right[COMMAND_SIZE] = {0x53, 0x13, 0x10, 0x03,
                                          0x00, 0xf8, 0xf8, 0x34};
unsigned char turn_left[COMMAND_SIZE] = {0x53, 0x13, 0x10, 0x03,
                                         0x00, 0x08, 0x08, 0x34};
unsigned char moving_forward[COMMAND_SIZE] = {0x53, 0x13, 0x10, 0x03,
                                              0x00, 0xf0, 0x10, 0x34};
unsigned char moving_back[COMMAND_SIZE] = {0x53, 0x13, 0x10, 0x03,
                                           0x00, 0x10, 0xf0, 0x34};
unsigned char stop_smooth[COMMAND_SIZE] = {0x53, 0x13, 0x11, 0x02,
                                           0x00, 0x00, 0x00, 0x00};
unsigned char get_pos[COMMAND_SIZE] = {0x53, 0x13, 0x13, 0x00,
                                       0x00, 0x00, 0x00, 0x34};
unsigned char get_switch[COMMAND_SIZE] = {0x53, 0x02, 0x60, 0x02,
                                          0x00, 0x00, 0x00, 0x00};

class BaseController {
 public:
  enum Command {
    DEFAULT,
    STOP,
    TURN_RIGHT,
    TURN_LEFT,
    FORWARD,
    BACK,
    CLOCK_GO,
    ANTI_CLOCK_GO,
    SINGLE_ANTI_CLOCK_GO,
    SINGLE_CLOCK_GO,
    KNIFE_ON,
    KNIFE_OFF,
    KNIFE_UNPLUG,
    GET_POSE,
    STEERING_IN,
    STEERING_OUT,
    GET_SWITCH,
  };
  struct Encoder {
    int right_encoder;
    int left_encoder;
    double interval;
    bool encoderWrong;
  };
  struct Cmd_vel {
    char cmd_right;
    char cmd_left;
  };
  struct Base_model {
    double Wheel_Diameter{120.0};
    double Encoder_to_Distance{1080.0};
    double Wheel_Base{450.0};
    double Wheel_Center_X_Offset{0.0};
    double Wheel_Center_Y_OffSet{0.0};
  };
  struct User_command {
    enum Command user_command;
    double parameter;
  };

 private:
  mutable boost::shared_mutex send_vel_mutex_{};
  mutable boost::shared_mutex command_mutex_{};

  char vel[COMMAND_SIZE] = {0x53, 0x13, 0x10, 0x03, 0x00, 0x00, 0x00, 0x34};
  //舵机控制协议  运行时间第四五位 单位US 总角度300
  unsigned char steeringGoByAngle[COMMAND_SIZE] = {0x53, 0x02, 0x40, 0x02,
                                                   0x00, 0x00, 0x00, 0x00};
  const int TIMER_SPAN_RATE_ = 120;
  const int ODOM_TIMER_SPAN_RATE_ = 30;
  const std::string BASE_FOOT_PRINT_;
  const std::string ODOM_FRAME_;
  Encoder ENCODER_{};
  Base_model BASE_MODEL_{};
  User_command USER_COMMAND_{};

  ros::NodeHandle nh_;

  NaviSerialManager* serialManager;

  char message_[COMMAND_SIZE];

  ros::Publisher wheel_status_pub;
  ros::Publisher odom_raw_pub;
  tf::TransformBroadcaster broad_caster;

  ros::Subscriber cmd_vel_sub;
  ros::Subscriber joy_vel_sub;

  ros::Timer read_timer_;
  ros::Timer send_timer_;
  ros::Timer odom_publish_timer_;

  ros::Time cmd_vel_watch_;
  ros::Time encoder_pre{}, encoder_after{}, encoder_stop{};

  std::atomic<double> linear_velocity_{}, angular_velocity_{};
  std::atomic<double> global_x{0.0}, global_y{0.0}, global_theta{0.0};

  bool joy_vel_received_{};
  bool cmd_vel_received_{};
  std::atomic_int battery;

  bool publish_tf_{};

  bool right_updated{}, left_updated{};

  bool knife_right_end{}, knife_left_end{};

  unsigned char xor_msgs(unsigned char* msg);
  void init_send_msgs();
  void readTimerCallback(const ros::TimerEvent& e);
  void sendTimerCallback(const ros::TimerEvent& e);
  void odom_publish_timer_callback(const ros::TimerEvent& e);
  void cmd_velCallback(const geometry_msgs::TwistConstPtr& msg);
  void joy_velCallback(const geometry_msgs::TwistConstPtr& msg);
  int parsingMsg();
  void odom_parsing();
  void steeringControlByAngle(int angle);
  void sendCommand();

 public:
  BaseController(const std::string& serial_addr, unsigned int baudrate,
                 std::string base_foot_print, std::string odom_frame,
                 bool publish_tf = false);
  ~BaseController();
  void passCommand(Command user_command, double parameter = 0.0);
  void sendVelocity();
  void setBaseModel(const std::string& param_addr);
};

#endif  // RUBBER_NAVIGATION_BASECONTROLLER_H
