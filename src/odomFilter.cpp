#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"
class Odometry{
private:
    ros::NodeHandle nh;
    ros::Subscriber poseCombined_sub;
    ros::Publisher odom_pub;
    nav_msgs::Odometry odom;
    const std::string ODOM_FRAME_;
    const std::string BASE_FOOT_PRINT_;

public:
    Odometry(std::string base_foot_print,std::string odom_frame);
    ~Odometry();
    double gx,gy,gth,vx,vth;
    void poseCombined_callback(const geometry_msgs::PoseWithCovarianceStamped & msg);
};

Odometry::Odometry(std::string base_foot_print,std::string odom_frame):BASE_FOOT_PRINT_(std::move(base_foot_print)),ODOM_FRAME_(std::move(odom_frame))
{
    poseCombined_sub=nh.subscribe("robot_pose_ekf/odom_combined",100,&Odometry::poseCombined_callback,this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",100);
}
Odometry::~Odometry()
{
}

void Odometry::poseCombined_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{

    gx=msg.pose.pose.position.x;
    gy=msg.pose.pose.position.y;
    gth=tf::getYaw(msg.pose.pose.orientation);

    static double last_gx{gx},last_gy{gy},last_th{gth};
    static ros::Time last_time{ros::Time::now()};

    //this means the first time, the dx will be zero
    double dx=gx-last_gx;
    double dy=gy-last_gy;
    double dth=gth-last_th;
    double dt=(ros::Time::now()-last_time).toSec();
    vx=sqrt((dx/dt)*(dx/dt)+(dy/dt)*(dy/dt));
    vth=dth/dt;

    last_gx=msg.pose.pose.position.x;
    last_gy=msg.pose.pose.position.y;
    last_th=tf::getYaw(msg.pose.pose.orientation);
    last_time=ros::Time::now();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(gth);

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = gx;
    odom.pose.pose.position.y = gy;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance=msg.pose.covariance;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"odometry");
    ros::NodeHandle nh_private("~");
    std::string odom_frame;
    std::string base_foot_print;
    nh_private.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh_private.param("odom_frame",odom_frame,(std::string)"odom");

    Odometry odometry(base_foot_print,odom_frame);

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
