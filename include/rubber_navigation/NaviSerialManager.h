//
// Created by xcy on 2019/12/9.
//

#ifndef VISUAL_SERVO_NAVISERIALMANAGER_H
#define VISUAL_SERVO_NAVISERIALMANAGER_H

#include "../../Visual-Servo/include/visual_servo/SerialManager.h"
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
    ReadResult read_results_;
    int read_used_bytes{};
    bool isAutoThreadRegistered_{};
    std::mutex queue_mutex_{};
    std::tr1::shared_ptr<boost::thread> thread_ptr_;
    void readWorker(int rate);
    int getCommandBeginIndex(int check_begin_index=0);
public:
    NaviSerialManager(std::string serial_addr, unsigned int baudrate);
    NaviSerialManager(const SerialManager & serialManager);
    ~NaviSerialManager();
    void registerAutoReadThread(int rate);
    void receive();
    ReadResult & getReadResult()
    {
        if(!read_result_queue.empty())
        {
            queue_mutex_.lock();
            read_results_ = read_result_queue.front();
            read_result_queue.pop();
            queue_mutex_.unlock();
        }
        else
            memset(&read_results_,0, sizeof(ReadResult));
        return read_results_;
    };
};


#endif //VISUAL_SERVO_NAVISERIALMANAGER_H