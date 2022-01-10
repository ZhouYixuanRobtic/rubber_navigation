/**
 * 用来测试日志记录
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>

using namespace std;

std::vector<int> vec{
    1,   2,   101, 110, 102, 103, 6,   6,   6,   6,   7,   101, 111, 102, 103,
    11,  12,  101, 112, 102, 103, 16,  17,  101, 113, 102, 103, 21,  22,  101,
    114, 102, 103, 26,  27,  101, 115, 102, 103, 31,  32,  101, 116, 102, 103,
    36,  37,  101, 117, 102, 103, 41,  42,  101, 118, 102, 103, 46,  47,  101,
    119, 102, 103, 51,  52,  56,  57,  101, 120, 102, 103, 61,  62,  101, 121,
    102, 103, 66,  67,  101, 122, 102, 103, 71,  72,  101, 123, 102, 103, 76,
    77,  101, 124, 102, 103, 81,  82,  101, 125, 102, 103, 86,  87};

int main(int argc, char** argv) {
  ros::init(argc, argv, "testtest");
  ros::NodeHandle nh_;
  ros::Publisher pub = nh_.advertise<std_msgs::Int8>("datatest", 10);
  sleep(1);
  for (auto iter : vec) {
    std_msgs::Int8 tmp;
    cout << iter << endl;
    tmp.data = iter;
    pub.publish(tmp);
    sleep(1);
  }
  return 0;
}
