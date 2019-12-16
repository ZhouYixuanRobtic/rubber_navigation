
#include "rubber_navigation/BaseController.h"

BaseController::BaseController(std::string serial_addr, unsigned int baudrate,std::string base_foot_print,std::string odom_frame):BASE_FOOT_PRINT_(std::move(base_foot_print)),ODOM_FRAME_(std::move(odom_frame))
{
    serialManager = new NaviSerialManager(serial_addr,baudrate);
    if(serialManager->openSerial())
    {
        serialManager->registerAutoReadThread(TIMER_SPAN_RATE_);
        encoder_start=ros::WallTime::now();

        wheel_status_pub = nh_.advertise<rubber_navigation::WheelStatus>("/wheel_status",100);
        odom_raw_pub = nh_.advertise<nav_msgs::Odometry>("/odom_raw",100);
        cmd_vel_sub = nh_.subscribe("joy_vel",100,&BaseController::joy_velCallback,this);
        joy_vel_sub = nh_.subscribe("cmd_vel",100,&BaseController::cmd_velCallback,this);
        init_send_msgs();

        timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / TIMER_SPAN_RATE_),&BaseController::timerCallback,this);
        timer_.start();
        odom_publish_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/ODOM_TIMER_SPAN_RATE_),&BaseController::odom_publish_timer_callback,this);
        odom_publish_timer_.start();
        ROS_INFO_STREAM("BASE READY!!!");
    }
    else
        ROS_ERROR_STREAM("Can't open "<<"SERIAL "<<serial_addr<<std::endl);


}
BaseController::~BaseController()
{
    delete serialManager;
}
unsigned char BaseController::xor_msgs(unsigned char *msg)
{
    unsigned char check=msg[1];
    for(int i=2;i<7;i++)
        check=check ^ msg[i];
    return check;
}
void BaseController::init_send_msgs()
{
    get_pos[7]=xor_msgs(get_pos);
}
void BaseController::odom_parsing()
{
    static int right_encoder_pre{ENCODER_.right_encoder},left_encoder_pre{ENCODER_.left_encoder};
    static ros::WallTime last_time{ros::WallTime::now()};

    int right_delta{ENCODER_.right_encoder - right_encoder_pre};
    int left_delta{ENCODER_.left_encoder - left_encoder_pre};


    double dt = (ros::WallTime::now()-last_time).toSec();
    double right_distance{},left_distance{};
    double theta{};

    // the robot is not moving
    if(abs(left_delta)<2&&abs(right_delta)<2)
    {
        linear_velocity_=0;
        angular_velocity_=0;
        //global position don't change

        //update value
        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::WallTime::now();
        return;
    }

    //in case of wrong data poisoning;
    //Noted, this is because  value bigger than the int limit;
    if(abs(left_delta)>1000||abs(right_delta)>1000)
    {
        //we hope to keep the same value of last time
        //so just update value and return;

        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::WallTime::now();
        return ;
    }
    right_distance = M_PI * BASE_MODEL_.Wheel_Diameter * right_delta / BASE_MODEL_.Encoder_to_Distance;//mm
    left_distance  = M_PI * BASE_MODEL_.Wheel_Diameter * left_delta / BASE_MODEL_.Encoder_to_Distance;//mm
    theta = (right_distance-left_distance)/BASE_MODEL_.Wheel_Base;
    if(left_delta==right_delta)
    {
        //moving forward
        double vertical_robot{((right_distance+left_distance)/2.0)};//mm
        vertical_robot /= 1000.0; //mm to m;
        double horizontal_robot{};//m

        global_x += vertical_robot*cos(global_theta) - horizontal_robot*sin(global_theta);//m
        global_y += horizontal_robot*cos(global_theta) + vertical_robot*sin(global_theta);//m
        global_theta +=theta;

        linear_velocity_ = (right_distance+left_distance)/(2000.0*dt);
        angular_velocity_ = theta/dt;
    }
    else
    {
        //turing round
        double turning_radius;
        turning_radius =(BASE_MODEL_.Wheel_Base*(right_distance+left_distance))/(2.0*(right_distance-left_distance));//m

        double vertical_robot{turning_radius*sin(theta)};//mm
        double horizontal_robot{turning_radius*(1-cos(theta))};//mm

        vertical_robot /= 1000.0;//m
        horizontal_robot /= 1000.0;//m

        global_x += vertical_robot*cos(global_theta) - horizontal_robot*sin(global_theta);//m
        global_y += horizontal_robot*cos(global_theta) + vertical_robot*sin(global_theta);//m
        global_theta +=theta;

        linear_velocity_ = (right_distance+left_distance)/(2000.0*dt);
        angular_velocity_ = theta/dt;
    }

    //update value
    right_encoder_pre = ENCODER_.right_encoder;
    left_encoder_pre = ENCODER_.left_encoder;
    last_time = ros::WallTime::now();

}
int BaseController::parsingMsg()
{
    static bool right_updated{},left_updated{};
    if(0x35!=message_[0])
    {
        memset(message_,0,COMMAND_SIZE);
        return -1;
    }
    else
    {
        switch (message_[1])
        {
            case 0x31:
                /*preserved*/
                break;
            case 0x21:
                /*poistion of right wheel*/
                if(0x13==message_[2])
                {
                    encoder_after = ros::WallTime::now();
                    //right encodisk parsing
                    char* pchar = (char*)&ENCODER_.right_encoder;
                    *(pchar+3) = message_[3];
                    *(pchar+2) = message_[4];
                    *(pchar+1) = message_[5];
                    *(pchar+0) = message_[6];

                    if (INT_MIN + 1000 <= ENCODER_.right_encoder <= INT_MAX - 1000) {} else ENCODER_.right_encoder = 0;
                    //make sure the consistency of left and right
                    right_updated= true;
                }
                break;
            case 0x11:
                /*position of left wheel*/
                if(0x13==message_[2])
                {
                    encoder_after = ros::WallTime::now();

                    char* pchar = (char*)&ENCODER_.left_encoder;
                    *(pchar+3) = message_[3];
                    *(pchar+2) = message_[4];
                    *(pchar+1) = message_[5];
                    *(pchar+0) = message_[6];

                    //按旧小车，向前进时，左轮码盘为负，要乘负一
                    ENCODER_.left_encoder *=-1;
                    if (INT_MIN + 1000 <= ENCODER_.left_encoder <= INT_MAX - 1000) {} else ENCODER_.left_encoder = 0;
                    //make sure the consistency of left and right
                    left_updated=true;
                }
                break;
            case 0x32:
                /*preserved for knife move end*/
                break;
            case 0x13:
            {
                if((message_[2]==0x50)&&(message_[3]==0x01))
                {
                    battery = ((message_[5]*256 + message_[6])-2350)/5;
                    if (battery<0)
                        battery=0;
                }
                break;
            }
            default:
                break;
        }
    }
    if(right_updated&&left_updated)
    {
        odom_parsing();
        right_updated=false;
        left_updated=false;
        return 1;
    }
    return 0;
}
void BaseController::timerCallback(const ros::WallTimerEvent &e)
{

    //try to get encodere data
    sendCommand(GET_POSE);
    //keep consistency
    if(!serialManager->isSerialAlive())
    {
        ROS_ERROR_STREAM("SERIAL WRONG!!!!!");
    }
    NaviSerialManager::ReadResult self_results{serialManager->getReadResult()};

    encoder_pre = ros::WallTime::now();
    if(self_results.read_bytes>=COMMAND_SIZE)
    {
        for(int i=0;i<self_results.read_bytes;i+=COMMAND_SIZE)
        {
            memcpy(message_, &self_results.read_result[i], COMMAND_SIZE);
            /*std::cout<<"receive  ";
            for(int j=0;j<8;j++)
            {
                printf("%x ",message_[j]);
            }
            std::cout<<std::endl<<"---------"<<std::endl;
            */
            parsingMsg();
            ENCODER_.interval=(encoder_after-encoder_pre).toSec();
        }
    }
    else
        memset(message_, 0, COMMAND_SIZE);
    //ignore 3 seconds after brought up;
    if((encoder_pre-encoder_start).toSec()>3.0)
    {
        if(ENCODER_.interval>1.0||ENCODER_.interval<0.0)
        {
            if(!ENCODER_.encoderWrong)
                encoder_stop=ros::WallTime::now();
            ENCODER_.encoderWrong=true;
        }
        else
        {
            if(ENCODER_.encoderWrong)
            {
                if((encoder_pre-encoder_stop).toSec()>=3.0)
                    ENCODER_.encoderWrong=false;
            }
        }
        rubber_navigation::WheelStatus wheelStatus{};
        wheelStatus.right_encoder=ENCODER_.right_encoder;
        wheelStatus.left_encoder = ENCODER_.left_encoder;
        wheelStatus.encoderWrong = ENCODER_.encoderWrong;
        wheelStatus.interval = ENCODER_.interval;
        wheel_status_pub.publish(wheelStatus);
    }
}
void BaseController::odom_publish_timer_callback(const ros::WallTimerEvent &e)
{
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(global_theta);

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = ODOM_FRAME_;
    odom.child_frame_id = BASE_FOOT_PRINT_;

    odom.pose.pose.position.x = global_x;
    odom.pose.pose.position.y = global_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance=	{1e-3,0,0,0,0,0,
                              0,1e-3,0,0,0,0,
                              0,0,1e6,0,0,0,
                              0,0,0,1e6,0,0,
                              0,0,0,0,1e6,0,
                              0,0,0,0,0,1e3};

    odom.twist.twist.linear.x = linear_velocity_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_velocity_;

    odom.twist.covariance={1e-3,0,0,0,0,0,
                           0,1e-3,0,0,0,0,
                           0,0,1e6,0,0,0,
                           0,0,0,1e6,0,0,
                           0,0,0,0,1e6,0,
                           0,0,0,0,0,1e3};

    odom_raw_pub.publish(odom);
}
void BaseController::cmd_velCallback(const geometry_msgs::TwistConstPtr  &msg)
{
    //manual control first
    if(joy_vel_received_)
        return;

    Cmd_vel user_cmd_vel{};
    double linear_velocity{msg->linear.x};
    double angular_velocity{msg->angular.z};
    double right_vel{}, left_vel{};
    if(linear_velocity == 0)
    {
        right_vel = angular_velocity * BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel  = -user_cmd_vel.cmd_right;
    }
    else if(angular_velocity == 0)
        right_vel = left_vel = linear_velocity;
    else
    {
        right_vel  = linear_velocity - angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel   = linear_velocity + angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
    }
    user_cmd_vel.cmd_right = right_vel*60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
    user_cmd_vel.cmd_left  = left_vel *60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);

    //if always the same velocity, send frequency decrease by three times
    static int same_no_zero_vel_counter;
    if((linear_velocity!=0||angular_velocity!=0) && vel[5] == -user_cmd_vel.cmd_left && vel[6] == user_cmd_vel.cmd_right )
        same_no_zero_vel_counter++;

    //only send changed(may zero) or same no zero vel(decrease by three times)
    if(vel[5] != -user_cmd_vel.cmd_left || vel[6]!= user_cmd_vel.cmd_right || same_no_zero_vel_counter >= 3)
    {
        sendVelocity(user_cmd_vel);
        same_no_zero_vel_counter = 0;
    }

}
void BaseController::joy_velCallback(const geometry_msgs::TwistConstPtr  &msg)
{
    Cmd_vel user_cmd_vel{};
    double linear_velocity{msg->linear.x};
    double angular_velocity{msg->angular.z};
    double right_vel{}, left_vel{};
    if(linear_velocity == 0)
    {
        right_vel = angular_velocity * BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel  = -user_cmd_vel.cmd_right;
    }
    else if(angular_velocity == 0)
        right_vel = left_vel = linear_velocity;
    else
    {
        right_vel  = linear_velocity - angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel   = linear_velocity + angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
    }
    user_cmd_vel.cmd_right = right_vel*60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
    user_cmd_vel.cmd_left  = left_vel *60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);

    static int same_no_zero_vel_counter;
    if(linear_velocity!=0||angular_velocity!=0)
    {
        joy_vel_received_ = true;
        //if always the same velocity, decrease by three times
        if(vel[5] == -user_cmd_vel.cmd_left && vel[6] == user_cmd_vel.cmd_right )
            same_no_zero_vel_counter++;
    }
    else
        joy_vel_received_=false;

    //only send changed(may zero) or same no zero vel(decrease by three times)
    if(vel[5] != -user_cmd_vel.cmd_left || vel[6]!= user_cmd_vel.cmd_right || same_no_zero_vel_counter >= 3)
    {
        sendVelocity(user_cmd_vel);
        same_no_zero_vel_counter = 0;
    }

}
void BaseController::sendCommand(enum BaseController::Command user_command )
{
    switch (user_command)
    {
        case STOP:
            serialManager->send(stop_smooth,COMMAND_SIZE);
            break;
        case FORWARD:
            serialManager->send(moving_forward,COMMAND_SIZE);
            break;
        case BACK:
            serialManager->send(moving_back,COMMAND_SIZE);
            break;
        case TURN_RIGHT:
            serialManager->send(turn_right,COMMAND_SIZE);
            break;
        case TURN_LEFT:
            serialManager->send(turn_left,COMMAND_SIZE);
            break;
        case CLOCK_GO:
            serialManager->send(clockGo,COMMAND_SIZE);
            break;
        case ANTI_CLOCK_GO:
            serialManager->send(antiClockGo,COMMAND_SIZE);
            break;
        case KNIFE_ON:
            serialManager->send(knifeOn,COMMAND_SIZE);
            break;
        case KNIFE_OFF:
            serialManager->send(knifeOff,COMMAND_SIZE);
            break;
        case GET_POSE:
            serialManager->send(get_pos,COMMAND_SIZE);
            break;
        default:
            break;
    }
}
void BaseController::sendVelocity(Cmd_vel user_cmd_vel)
{
    vel[5] = -user_cmd_vel.cmd_left;
    vel[6] = user_cmd_vel.cmd_right;
    serialManager->send(vel,COMMAND_SIZE);
}
void BaseController::setBaseModel(const std::string & param_addr)
{
    global_x -= BASE_MODEL_.Wheel_Center_X_Offset;
    global_y -= BASE_MODEL_.Wheel_Center_Y_OffSet;
    YAML::Node doc = YAML::LoadFile(param_addr);
    try
    {
        BASE_MODEL_.Wheel_Diameter=doc["WheelDiameter"].as<double>();
        BASE_MODEL_.Wheel_Base = doc["WheelBase"].as<double>();
        BASE_MODEL_.Encoder_to_Distance = doc["EncoderToDistance"].as<double>();
        BASE_MODEL_.Wheel_Center_X_Offset = doc["WheelCenterXOffset"].as<double>();
        BASE_MODEL_.Wheel_Center_Y_OffSet = doc["WheelCenterYOffset"].as<double>();
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("tagParam.yaml is invalid.");
    }
    global_x+=BASE_MODEL_.Wheel_Center_X_Offset;
    global_y+=BASE_MODEL_.Wheel_Center_Y_OffSet;
}