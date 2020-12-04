//
// Created by xcy on 2019/12/9.
//

#ifndef VISUAL_SERVO_NAVISERIALMANAGER_H
#define VISUAL_SERVO_NAVISERIALMANAGER_H

#include "../../Visual-Servo/include/visual_servo/SerialManager.h"
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#define COMMAND_SIZE 8
#define RESULT_SIZE COMMAND_SIZE*20
#define COMMAND_HEAD 0x35

class NaviSerialManager : public SerialManager{
public:
    struct ReadResult{
        char read_result[RESULT_SIZE];
        int read_bytes;
    };

private:
    std::queue<ReadResult> read_result_queue{};
    int read_used_bytes{};
    bool isAutoThreadRegistered_{};
    std::tr1::shared_ptr<boost::thread> thread_ptr_;
    mutable boost::shared_mutex queue_mutex_{};

    void readWorker(int rate);
    int getCommandBeginIndex(int check_begin_index=0);
public:
    NaviSerialManager(std::string serial_addr, unsigned int baudrate);
    explicit NaviSerialManager(const SerialManager & serialManager);
    ~NaviSerialManager();
    void registerAutoReadThread(int rate);
    void receive() override;
    ReadResult getReadResult()
    {
        boost::unique_lock<boost::shared_mutex> writeLock(queue_mutex_);
        ReadResult read_result{};
        if(!read_result_queue.empty())
        {
            read_result = read_result_queue.front();
            read_result_queue.pop();
        }
        return read_result;
    };
};


#endif //VISUAL_SERVO_NAVISERIALMANAGER_H