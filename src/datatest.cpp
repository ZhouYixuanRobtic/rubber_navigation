/**
 * 本节点用于记录机器人测试日志。
 * 本节点订阅话题“robot_log”，消息类型为std_msgs::String。
 * 功能概述：在运行本节点时，首先检测/home/xcy/data目录是否存在，若不存在，则先创建目录；若存在,则在/home/xcy/data目录下生成以节点开始运行时间为文件名的日志文件，例如“20190904151322.txt”，即2019年9月4日15时13分22秒运行该节点产生的日志文件。每次运行节点会删除最近修改时间在7天以上的日志文件。本程序订阅“robot_log”话题，并将话题内容保存在日志文件中。每次记录的格式为“时间+字符串内容”，例如“2019.09.04
 * 15:13:22 start charging”,其中“start
 * charging”即其他节点发布的需要记录的日志消息。
 *
 * 发布器示例
 * ros::Publisher log_pub_ = nh_.advertise<std_msgs::String>("robot_log",1);
 * std_msgs::String log;
 * log.data = "start charging";
 * log_pub_.publish(log);
 * find /home/xcy/.robot_log/ -name "*.txt" -mtime +7 -exec rm -fv {} \;
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>

#include <ctime>
#include <fstream>
#include <iostream>
using namespace std;
ofstream outputfile;
ostringstream temp;
struct tm* local_time() {
  time_t now = time(0);
  tm* ltm = localtime(&now);
  ltm->tm_year += 1900;
  ltm->tm_mon += 1;
  //     cout << ltm->tm_year << endl;
  //     cout << ltm->tm_mon << endl;
  //     cout << ltm->tm_mday << endl;
  //     cout << ltm->tm_hour << endl;
  //     cout << ltm->tm_min << endl;
  //     cout << ltm->tm_sec << endl;
  return ltm;
}
int openNewFile() {
  tm* _p_fopen_time = local_time();
  ostringstream temp;
  string filename;
  temp << "/home/xcy/data/" << _p_fopen_time->tm_year << setfill('0') << setw(2)
       << _p_fopen_time->tm_mon << setfill('0') << setw(2)
       << _p_fopen_time->tm_mday << setfill('0') << setw(2)
       << _p_fopen_time->tm_hour << setfill('0') << setw(2)
       << _p_fopen_time->tm_min << setfill('0') << setw(2)
       << _p_fopen_time->tm_sec << ".log";
  filename = temp.str();
  temp.clear();
  temp.str("");
  outputfile.open(filename.c_str(), ios::trunc);
}

int step = 0;
std::vector<int> vec{
    1,  2,  301, 302, 303, 6,  7,  301, 302, 303, 11, 12, 301, 302, 303,
    16, 17, 301, 302, 303, 21, 22, 301, 302, 303, 26, 27, 301, 302, 303,
    31, 32, 301, 302, 303, 36, 37, 301, 302, 303, 41, 42, 301, 302, 303,
    46, 47, 301, 302, 303, 51, 52, 301, 302, 303, 56, 57, 301, 302, 303,
    61, 62, 301, 302, 303, 66, 67, 301, 302, 303, 71, 72, 301, 302, 303,
    76, 77, 301, 302, 303, 81, 82, 301, 302, 303, 86, 87};
std::vector<int>::iterator iter = vec.begin();
ros::Publisher joy_pub;
void robotLogCB(const std_msgs::Int8::ConstPtr& msg);

int main(int argc, char** argv) {
  ros::init(argc, argv, "datatest");
  ros::NodeHandle nh_;
  if (access("/home/xcy/data", 00)) system("mkdir /home/xcy/data");
  ros::Subscriber log_sub_ =
      nh_.subscribe<std_msgs::Int8>("datatest", 10, robotLogCB);
  joy_pub = nh_.advertise<sensor_msgs::Joy>("joy", 10);
  // system("find /home/xcy/.robot_log -name \"*.txt\" -mtime +7 -exec rm -fv {}
  // '\;'");
  ros::spin();
  if (outputfile.is_open()) {
    outputfile << endl;
    outputfile.close();
  }
  return 0;
}
void robotLogCB(const std_msgs::Int8::ConstPtr& msg) {
  if (msg->data == (*vec.begin())) {  // 1代表开始
    if (outputfile
            .is_open()) {  // 如果开始的时候文件是打开的，说明上一次最后没有关文件
      outputfile << endl;
      outputfile.close();
      iter = vec.begin();
      ROS_WARN("The file is not closed, now close the file.");
    }
    openNewFile();  // 开始后正常打开新文件
    ROS_INFO("New loop open new file.");
  }  //每次发1说明开始了新一轮测试
  // TODO: 如果没有正确关闭文件，又跳过了开始，那么就只能人工检查了
  if (!outputfile
           .is_open()) {  // 此时如果没打开文件，说明跳过了开始的1，那么打开一个新文件
    openNewFile();
    ROS_WARN("Open file unexpectly, because 1st step is skipped.");
  }
  if ((*iter) != msg->data) {
    //如果不相等，说明有点数被跳过了，用星号代替
    while ((*iter) != msg->data) {
      outputfile << "*"
                 << " ";
      iter++;
    }
  }
  //两边步数相等，说明没有被跳过，正常记录时间
  outputfile << fixed << setprecision(3) << ros::Time::now().toSec() << " ";

  if (msg->data != vec.back()) {
    ROS_INFO("iter++");
    iter++;
  } else {  //一共要记录23个时间点
    outputfile << endl;
    outputfile.close();
    iter = vec.begin();
    ROS_INFO("File close, end.");
    // sensor_msgs::Joy a;
    // a.header.stamp = ros::Time::now();
    // a.header.frame_id = "/dev/input/js0";
    // a.axes = std::vector<float>{0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    // a.buttons = std::vector<int>{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    // sleep(10);
    // joy_pub.publish(a);
    // std::cout << "joy" << std::endl;
  }
}