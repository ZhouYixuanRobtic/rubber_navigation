/**
 * 本节点用于记录机器人运行日志。
 * 本节点订阅话题“robot_log”，消息类型为std_msgs::String。
 * 功能概述：在运行本节点时，首先检测/home/xcy/.robot_log目录是否存在，若不存在，则先创建目录；若存在,则在/home/xcy/.robot_log目录下生成以节点开始运行时间为文件名的日志文件，例如“20190904151322.txt”，即2019年9月4日15时13分22秒运行该节点产生的日志文件。每次运行节点会删除最近修改时间在7天以上的日志文件。本程序订阅“robot_log”话题，并将话题内容保存在日志文件中。每次记录的格式为“时间+字符串内容”，例如“2019.09.04 15:13:22 start charging”,其中“start charging”即其他节点发布的需要记录的日志消息。
 *
 * 发布器示例
 * ros::Publisher log_pub_ = nh_.advertise<std_msgs::String>("robot_log",1);
 * std_msgs::String log;
 * log.data = "start charging";
 * log_pub_.publish(log);
 * find /home/xcy/.robot_log/ -name "*.txt" -mtime +7 -exec rm -fv {} \;
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
using namespace std;
ofstream outputfile;
ostringstream temp;
struct tm* local_time()
{
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
void robotLogCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    //string log_ = msg->data;
    //tm* current_time = local_time();
    /*outputfile <<current_time->tm_year <<"."<<
         setfill('0') << setw(2) << current_time->tm_mon <<"."<<
         setfill('0') << setw(2) << current_time->tm_mday <<" "<<
         setfill('0') << setw(2) << current_time->tm_hour <<":"<<
         setfill('0') << setw(2) << current_time->tm_min <<":"<<
         setfill('0') << setw(2) << current_time->tm_sec <<" "
         << msg->data << endl;*/
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    double r,p,y;
    tf::Matrix3x3(quat).getRPY(r,p,y);
    outputfile << "1 "<< msg->pose.pose.position.x <<" "<< msg->pose.pose.position.y<<" "<<y<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initialPoseWriter");
    ros::NodeHandle nh_;
    if(access("/home/xcy/.robot_log", 00))
    system("mkdir /home/xcy/.robot_log");
    tm* _p_fopen_time = local_time();
    ostringstream temp;
    string filename;
    temp <<
         "/home/xcy/.robot_log/" <<
         _p_fopen_time->tm_year <<
         setfill('0') << setw(2) << _p_fopen_time->tm_mon <<
         setfill('0') << setw(2) << _p_fopen_time->tm_mday <<
         setfill('0') << setw(2) << _p_fopen_time->tm_hour <<
         setfill('0') << setw(2) << _p_fopen_time->tm_min <<
         setfill('0') << setw(2) << _p_fopen_time->tm_sec <<
         "target.txt";
    filename = temp.str();
    temp.clear();
    temp.str("");
    outputfile.open(filename.c_str(), ios::trunc);
    ros::Subscriber log_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10, robotLogCB);
    //system("find /home/xcy/.robot_log -name \"*.txt\" -mtime +7 -exec rm -fv {} '\;'");
    ros::spin();
    outputfile.close();
    return 0;
}
