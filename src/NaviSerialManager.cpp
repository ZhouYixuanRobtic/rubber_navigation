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
    thread_ptr_->interrupt();
    thread_ptr_->join();
}
void NaviSerialManager::registerAutoReadThread(int rate)
{
    thread_ptr_.reset(new boost::thread(boost::bind(&NaviSerialManager::readWorker, this, rate)));
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
void NaviSerialManager::receive()
{
    //serial_mutex_.lock();
    int receiveNumbers=read(m_dFd,&read_buffer[read_used_bytes],BUFFER_SIZE);
    //serial_mutex_.unlock();
    if(receiveNumbers>0)
    {
        serial_alive_ =true;
        read_used_bytes +=receiveNumbers;
        if(read_used_bytes%COMMAND_SIZE==0)
        {
            ReadResult temp{};
            for(int i=0;i<read_used_bytes;i+=COMMAND_SIZE)
            {
                //WARNING, MAY IGNORE SOME DATA
                if(i<=RESULT_SIZE-COMMAND_SIZE)
                {
                    memcpy(&temp.read_result[i], &read_buffer[i], COMMAND_SIZE);
                    temp.read_bytes += COMMAND_SIZE;
                }
            }
            read_result_queue.push(temp);
            memset(read_buffer,0,BUFFER_SIZE);
            read_used_bytes = 0;
        }
        if(read_used_bytes>=BUFFER_SIZE-COMMAND_SIZE)
        {
            read_used_bytes=0;
            memset(read_buffer,0,BUFFER_SIZE);
        }
    }
    else if(receiveNumbers<0)
    {
        serial_alive_ =false;
        read_used_bytes= 0;
        memset(read_buffer,0,BUFFER_SIZE);
    }

}