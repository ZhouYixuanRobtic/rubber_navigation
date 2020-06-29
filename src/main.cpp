#include <iostream> //标准输入输出流
#include "rubber_navigation/NavCore.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "visual_servo/manipulate.h"
#include "../../Visual-Servo/include/visual_servo/VisualServoMetaType.h"

#include "../../Visual-Servo/include/visual_servo/JoyTeleop.h"
#include "../../Visual-Servo/include/visual_servo/parameterTeleop.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "tr1/memory"
using std::tr1::shared_ptr;

class RubberNav{
private:
    ros::NodeHandle nh;
    struct targetPose{
        geometry_msgs::Pose2D pose;
        enum TargetAction{
            DEFAULT,
            TAP,
            CHARGE,
            NAV,
            TURN,
        }targetAction{};
    };
    bool nav_on{},nav_pause{},newGoal{true};
    const std::vector<std::string> parameterNames{"/visual_servo/clockGo","/visual_servo/antiClockGo","/visual_servo/knifeOn","/visual_servo/knifeOff","/visual_servo/knifeUnplug",
                                                  "/visual_servo/singleClockGo","/visual_servo/singleAntiClockGo","/visual_servo/steeringIn","/visual_servo/steeringOut","/visual_servo/getSwitch"};
    BaseController *baseController;
    NavCore *navCore;
    visual_servo_namespace::ServiceCaller* serviceCaller;
    JOYTELEOP::JoyTeleop * joyTeleop;
    ParameterListener* parameterListener;

    ros::Subscriber vs_status_sub;
	ros::Subscriber log_sub;
    /** log **/
    ros::Publisher log_pub;
    std::vector<targetPose> targetPoseArray{};
    std::vector<targetPose>::iterator iter;

	bool tappingDone{false};

    void checkSrvFinish();
    void getPoseArray();
    void statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg);
	void logCallback(const std_msgs::String & log_data);
    void processCommand();
    void setBySignal();
    void setGoalInOrder();
public:
    RubberNav(std::string base_foot_print,std::string odom_frame,std::string map_frame,std::string serial_addr,bool publish_tf);
    ~RubberNav();
    void run();
};
RubberNav::RubberNav(std::string base_foot_print,std::string odom_frame,std::string map_frame,std::string serial_addr,bool publish_tf)
{
    baseController = new BaseController(serial_addr,B115200,base_foot_print,odom_frame,publish_tf);
    navCore = new NavCore(base_foot_print,map_frame);
	serviceCaller = new visual_servo_namespace::ServiceCaller;
    parameterListener = new ParameterListener(40,8);
    joyTeleop = new JOYTELEOP::JoyTeleop("joy");
    vs_status_sub = nh.subscribe("VisualServoStatus",100,&RubberNav::statusCallback,this);
	log_sub=nh.subscribe("robot_log",10,&RubberNav::logCallback,this);
    /** log **/
    log_pub = nh.advertise<std_msgs::String>("robot_log", 10);
    
    parameterListener->registerParameterCallback(parameterNames,false);
    const std::string parameter_addr{ros::package::getPath("rubber_navigation")+"/config/BaseModel.yaml"};
    baseController->setBaseModel(parameter_addr);
    getPoseArray();
    iter = targetPoseArray.begin();
}
RubberNav::~RubberNav()
{
    delete joyTeleop;
    delete parameterListener;
    delete serviceCaller;
    delete navCore;
    delete baseController;
}
void RubberNav::getPoseArray()
{
    const std::string pose_addr{ros::package::getPath("rubber_navigation")+"/config/treeTarget.txt"};
    std::ifstream input_file(pose_addr.c_str());
    if(input_file.is_open())
    {
        targetPoseArray.clear();
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
        std_msgs::String log_string;
        log_string.data="The total number of targets is"+std::to_string(targetPoseArray.size());
        log_pub.publish(log_string);
    }
}
void RubberNav::statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg)
{
    if(!msg.RobotAllRight)
    {
        navCore->cancelAllGoals();
        baseController->sendCommand(BaseController::STOP);
        nav_on=false;
        newGoal=false;
    }
}
void RubberNav::logCallback(const std_msgs::String& log_data)
{
	if(log_data.data=="Rubber Tapping Done")
		tappingDone=true;
	ROS_INFO("LOG CALL BACK");

}
void RubberNav::processCommand() 
{
    if((bool)parameterListener->parameters()[0])
    {
        baseController->sendCommand(BaseController::CLOCK_GO); 
		ros::param::set(parameterNames[0],(double)false);
    }
    else if((bool)parameterListener->parameters()[1])
    {
        baseController->sendCommand(BaseController::ANTI_CLOCK_GO);
        ros::param::set(parameterNames[1],(double)false);
    }
    else if((bool)parameterListener->parameters()[2])
    {
        baseController->sendCommand(BaseController::KNIFE_ON);
        ros::param::set(parameterNames[2],(double)false);
    }
    else if((bool)parameterListener->parameters()[3])
    {
        baseController->sendCommand(BaseController::KNIFE_OFF);
        ros::param::set(parameterNames[3],(double)false);
    }
    else if((bool)parameterListener->parameters()[4])
    {
        baseController->sendCommand(BaseController::KNIFE_UNPLUG);
        ros::param::set(parameterNames[4],(double)false);
    }
    else if((bool)parameterListener->parameters()[5])
    {
        baseController->sendCommand(BaseController::SINGLE_CLOCK_GO);
        ros::param::set(parameterNames[5],(double)false);
    }
    else if((bool)parameterListener->parameters()[6])
    {
        baseController->sendCommand(BaseController::SINGLE_ANTI_CLOCK_GO);
        ros::param::set(parameterNames[6],(double)false);
    }
	 else if((bool)parameterListener->parameters()[7])
    {
        baseController->sendCommand(BaseController::STEERING_IN,parameterListener->parameters()[7]);
        ros::param::set(parameterNames[7],(double)false);
    }
	 else if((bool)parameterListener->parameters()[8])
    {
        baseController->sendCommand(BaseController::STEERING_OUT);
        ros::param::set(parameterNames[8],(double)false);
    }
	else if((bool)parameterListener->parameters()[9])
	{
		baseController->sendCommand(BaseController::GET_SWITCH,parameterListener->parameters()[9]);
		ros::param::set(parameterNames[9],(double)false);		
	}
}
void RubberNav::setBySignal()
{
    switch (joyTeleop->getControlTrigger())
    {
        case JOYTELEOP::NavOn:
        {
            if(nav_pause&&iter!=targetPoseArray.end())
            {
                if(navCore->isGoalPassed((*iter).pose))
                    iter++;
                newGoal = true;
                nav_pause=false;
            }
            nav_on = true;
            ROS_INFO("NavOn received, ready to go");
            break;
        }
        case JOYTELEOP::NavPause:
        {
            nav_on=false;
            nav_pause=true;
            ROS_INFO("NavPause received, cancel all goals");
            navCore->cancelAllGoals();
            baseController->sendCommand(BaseController::STOP);
            break;
        }
        default:
            break;
    }
}
void RubberNav::setGoalInOrder()
{
    if(nav_on && !targetPoseArray.empty()&&newGoal)
    {
        if(iter != targetPoseArray.end())
        {
            ROS_INFO_STREAM("now goal: x is "<<(*iter).pose.x<<" y is "<<(*iter).pose.y<<" theata is "<<(*iter).pose.theta);
            navCore->setGoal((*iter).pose);
            newGoal=false;
        }
        else
        {
            getPoseArray();
            iter = targetPoseArray.begin();
            ROS_INFO("Reach the end of the goal list, reload tree target");
            nav_on=false;
            newGoal=true;
        }
    }
}
void RubberNav::checkSrvFinish()
{
    if(serviceCaller->srvFinished())
    {    
        if(serviceCaller->getSrvResponseStatus().status!=visual_servo_namespace::SERVICE_STATUS_EMPTY&&!tappingDone)
        {
            navCore->cancelAllGoals();
            newGoal=false;
        }
        else
        {
            iter++;
            newGoal=true;
        }
    }
}
void RubberNav::run()
{
    processCommand();
    setBySignal();
    std_msgs::String log_string;
	static int counter{0};
    switch (navCore->getMoveBaseActionResult())
    {
        case NavCore::SUCCEEDED:
        {
            switch ((*iter).targetAction)
            {
                case targetPose::TAP:
                {
                    if(!serviceCaller->srvCalling())
                        serviceCaller->callSrv(visual_servo::manipulate::Request::CUT);
					tappingDone=false;					
                    break;
                }
                case targetPose::CHARGE:
                {
                    if(!serviceCaller->srvCalling())
                        serviceCaller->callSrv(visual_servo::manipulate::Request::CHARGE);
                    break;
                }
                case targetPose::NAV:
                {
					if(!serviceCaller->srvCalling())
	                        serviceCaller->callSrv(visual_servo::manipulate::Request::EMPTY);
					 //iter++;
					 //newGoal=true;			                   
					 break;
                }
                case targetPose::TURN:
                {
                    double temp_inverse;
					ros::param::get("/user/inverse",temp_inverse);
					usleep(200000);
					//reverse
					temp_inverse = (bool)temp_inverse ? 0.0 :1.0;
					ros::param::set("/user/inverse",temp_inverse);
                    usleep(200000);
                    if(!serviceCaller->srvCalling())
                        serviceCaller->callSrv(visual_servo::manipulate::Request::HOME);
                    break;
                }
                default:
                    break;
            }
            log_string.data="The "+std::to_string(std::distance(targetPoseArray.begin(),iter))+"th target is arrived";
			//sleep(15);
            break;
        }
        case NavCore::ABORTED:
        {
            /*ROS_ERROR_STREAM("ROBOT ABORTED ");
            if(navCore->clearCostMap())
            {
                iter++;
                newGoal=true;
            }
            else
                ROS_ERROR_STREAM("Failed to call clear cost map service");
			*/
			iter++;
            newGoal=true;
            log_string.data="The "+std::to_string(std::distance(targetPoseArray.begin(),iter))+"th target is aborted";
			break;
        }
        default:
            //log_string.data="The "+std::to_string(std::distance(targetPoseArray.begin(),iter))+"th target is others";
            break;
    }
    checkSrvFinish();
    setGoalInOrder();
    //log_pub.publish(log_string);
}



int main(int argc, char* argv[])
{
    ros::init(argc,argv,"rubber_navigation");
    ros::NodeHandle nh_("~");
    ros::Rate loop_rate(30);


    std::string base_foot_print,odom_frame,map_frame,serial_addr;
    bool publish_tf;
    nh_.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh_.param("odom_frame",odom_frame,(std::string)"odom");
    nh_.param("map_frame",map_frame,(std::string)"map");
    nh_.param("serial_addr",serial_addr,(std::string)"/dev/ttyS1");
    nh_.param("publish_tf",publish_tf,(bool)false);

    ros::AsyncSpinner spinner(5);
    spinner.start();

    RubberNav rubberNav(base_foot_print,odom_frame,map_frame,serial_addr,publish_tf);

    while(ros::ok())
    {
        rubberNav.run();
        loop_rate.sleep();
        ros::spinOnce();
    }
}
