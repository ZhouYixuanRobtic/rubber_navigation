//
// Created by xcy on 2019/12/9.
//
#include "rubber_navigation/NaviSerialManager.h"
#include "ros/ros.h"
NaviSerialManager::NaviSerialManager(std::string serial_addr, unsigned int baudrate):SerialManager(serial_addr,baudrate)
{

}
NaviSerialManager::NaviSerialManager(const SerialManager & serialManager):SerialManager(serialManager)
{

}
NaviSerialManager::~NaviSerialManager()
{
    if(isAutoThreadRegistered_)
    {
        thread_ptr_->interrupt();
        thread_ptr_->join();
    }
}
void NaviSerialManager::registerAutoReadThread(int rate)
{
    thread_ptr_.reset(new boost::thread(boost::bind(&NaviSerialManager::readWorker, this, rate)));
    isAutoThreadRegistered_=true;
}
void NaviSerialManager::readWorker(int rate)
{
    static ros::Rate loop_rate(rate);
    try
    {
        boost::this_thread::interruption_enabled();
        while(true)
        {
            //boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::universal_time();
            boost::this_thread::interruption_point();
            this->receive();
            //boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::universal_time();
            //boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate - (endTime - startTime).total_microseconds()));
            loop_rate.sleep();
        }
    }
    catch (boost::thread_interrupted&e )
    {
        std::cout<<"now quit the read thread"<<std::endl;
    }
}
int NaviSerialManager::getCommandBeginIndex(int check_begin_index)
{
    for(int i=0;i<read_used_bytes-check_begin_index;++i)
    {
        if(read_buffer[i+check_begin_index]==COMMAND_HEAD)
            return  i+check_begin_index;
    }
    return read_used_bytes;
}
void NaviSerialManager::receive()
{
    boost::unique_lock<boost::shared_mutex> writLock(queue_mutex_);
    int receiveNumbers=read(m_dFd,&read_buffer[read_used_bytes],BUFFER_SIZE);
    //ROS_INFO_STREAM("the receive number is "<<receiveNumbers);
    if(receiveNumbers>0)
    {
        serial_alive_ =true;
        read_used_bytes +=receiveNumbers;
        if(read_used_bytes>=COMMAND_SIZE)
        {
            int commandBeginIndex{-COMMAND_SIZE};
            ReadResult temp{};
            for(int i=0;i<read_used_bytes;i+=COMMAND_SIZE)
            {
                commandBeginIndex=getCommandBeginIndex(commandBeginIndex+COMMAND_SIZE);
                if(commandBeginIndex<=read_used_bytes-COMMAND_SIZE)
                {
                    memcpy(&temp.read_result[i], &read_buffer[commandBeginIndex], COMMAND_SIZE);
                    temp.read_bytes += COMMAND_SIZE;
                }
                else
                   break;
            }
            read_result_queue.push(temp);
            if(commandBeginIndex<read_used_bytes&&commandBeginIndex>read_used_bytes-COMMAND_SIZE)
            {
                char transfer_buffer[read_used_bytes-commandBeginIndex];
                memcpy(&transfer_buffer[0],&read_buffer[commandBeginIndex],read_used_bytes-commandBeginIndex);
                memset(read_buffer,0,BUFFER_SIZE);
                memcpy(&read_buffer[0],&transfer_buffer[0],read_used_bytes-commandBeginIndex);
                read_used_bytes=read_used_bytes-commandBeginIndex;
            }
            else
            {
                memset(read_buffer,0,BUFFER_SIZE);
                read_used_bytes=0;
            }
        }
        //Warning, ignore some data when the STM32 send wrong many times
        if(read_used_bytes>=BUFFER_SIZE-COMMAND_SIZE)
        {
            read_used_bytes=0;
            memset(read_buffer,0,BUFFER_SIZE);
        }
    }

}
