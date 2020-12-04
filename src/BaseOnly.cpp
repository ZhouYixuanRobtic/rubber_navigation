//
// Created by xcy on 2019/12/12.
//
#include "ros/ros.h"
#include "rubber_navigation/BaseController.h"
#include "ros/package.h"
#include "../../Visual-Servo/include/visual_servo/parameterTeleop.h"
int main(int argc,char* argv[])
{
    ros::init(argc,argv,"baseOnly");
    ros::NodeHandle nh_("~");
    const std::string parameter_addr{ros::package::getPath("rubber_navigation")+"/config/BaseModel.yaml"};
    std::string base_foot_print,odom_frame,map_frame,serial_addr;
    bool publish_tf;

    nh_.param("publish_tf",publish_tf,(bool)false);
    nh_.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh_.param("odom_frame",odom_frame,(std::string)"odom");
    nh_.param("serial_addr",serial_addr,(std::string)"/dev/ttyS1");

	const std::vector<std::string> parameterNames{"/visual_servo/clockGo","/visual_servo/antiClockGo","/visual_servo/knifeOn","/visual_servo/knifeOff","/visual_servo/knifeUnplug","/visual_servo/singleClockGo","/visual_servo/singleAntiClockGo","/visual_servo/steeringIn","/visual_servo/steeringOut","/visual_servo/getSwitch"};

 	ParameterListener parameterListener(40,9);
	parameterListener.registerParameterCallback(parameterNames,false);

    BaseController baseController(serial_addr,B115200,base_foot_print,odom_frame,publish_tf);
    baseController.setBaseModel(parameter_addr);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate loop_rate(30);



    while(ros::ok())
    {
		if((bool)parameterListener.parameters()[0])
		{
		    baseController.passCommand(BaseController::CLOCK_GO);
			ros::param::set(parameterNames[0],(double)false);
		}
		else if((bool)parameterListener.parameters()[1])
		{
		    baseController.passCommand(BaseController::ANTI_CLOCK_GO);
		    ros::param::set(parameterNames[1],(double)false);
		}
		else if((bool)parameterListener.parameters()[2])
		{
		    baseController.passCommand(BaseController::KNIFE_ON);
		    ros::param::set(parameterNames[2],(double)false);
		}
		else if((bool)parameterListener.parameters()[3])
		{
		    baseController.passCommand(BaseController::KNIFE_OFF);
		    ros::param::set(parameterNames[3],(double)false);
		}
		else if((bool)parameterListener.parameters()[4])
		{
		    baseController.passCommand(BaseController::KNIFE_UNPLUG);
		    ros::param::set(parameterNames[4],(double)false);
		}
		else if((bool)parameterListener.parameters()[5])
		{
		    baseController.passCommand(BaseController::SINGLE_CLOCK_GO);
		    ros::param::set(parameterNames[5],(double)false);
		}
		else if((bool)parameterListener.parameters()[6])
		{
		    baseController.passCommand(BaseController::SINGLE_ANTI_CLOCK_GO);
		    ros::param::set(parameterNames[6],(double)false);
		}
	 	else if((bool)parameterListener.parameters()[7])
    	{
        	baseController.passCommand(BaseController::STEERING_IN,parameterListener.parameters()[7]);
        	ros::param::set(parameterNames[7],(double)false);
    	}
	 	else if((bool)parameterListener.parameters()[8])
    	{
		    baseController.passCommand(BaseController::STEERING_OUT);
		    ros::param::set(parameterNames[8],(double)false);
    	}
		else if((bool)parameterListener.parameters()[9])
		{
			baseController.passCommand(BaseController::GET_SWITCH,parameterListener.parameters()[9]);
			ros::param::set(parameterNames[9],(double)false);		
		}
        ros::spinOnce();
        loop_rate.sleep();
    }

}
