/*
* Copyright (c) 2022, Autonics Co.,Ltd.
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met:
* 
*   * Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*
*   * Redistributions in binary form must reproduce the above 
*     copyright notice, this list of conditions and the following 
*     disclaimer in the documentation and/or other materials provided 
*     with the distribution.
*
*   * Neither the name of the Autonics Co.,Ltd nor the names of its 
*     contributors may be used to endorse or promote products derived 
*     from this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*/



#include "parser.hpp"

#include <chrono>

#include <netinet/in.h>

#define NUM_OF_CMD_TYPE 7  
#define NUM_OF_CMD 8

#define NUMBER_OF_PARAM 13
#define MAX_NUM_OF_PACKET_PARAM 16

#define INTENSITY_MAX 50000.0


const char* cmd_type_list[NUM_OF_CMD_TYPE] = 
{"sMC",
"sMA",
"sRC",
"sRA",
"sWC",
"sWA",
"sCA"
};

const char* cmd_list[NUM_OF_CMD] = 
{"None",
"SetAccessLevel",
"SensorScanInfo",
"SensorStart",
"SensorStop",
"ScanData",
"FirstConnectDummySend",
"LSScanDataConfig"
};

enum CmdListNum 
{
    NONE = 0,
    SET_ACCESS_LEVEL,
    SENSOR_SCAN_INFO,
    SENSOR_START,
    SENSOR_STOP,
    SCAN_DATA,
    FIRST_CONNECT_DUMMY_SEND,
    SCAN_DATA_CONFIG,
};

Parser::Parser()
{
    readQueue_ = std::make_shared<std::queue<unsigned char>>();
    readMtx_ = std::make_shared<std::mutex>();
}

void Parser::setReadQueuePtr(std::shared_ptr<std::queue<unsigned char>> readQueuePtr)
{
    readQueue_ = readQueuePtr;
}

std::shared_ptr<std::queue<unsigned char>> Parser::getReadQueuePtr()
{
    return readQueue_;
}

void Parser::setReadMutexPtr(std::shared_ptr<std::mutex> readMutexPtr)
{
    readMtx_ = readMutexPtr;
}

std::shared_ptr<std::mutex> Parser::getReadMutexPtr(void)
{
    return readMtx_;
}

std::queue<std::vector<unsigned char>>* Parser::getRecvMsgQueuePtr(void)
{
    return &recvMsgQueue_;
}

std::mutex* Parser::getRecvMsgMtxPtr(void)
{
    return &recvMsgMtx_;
}

int Parser::makeCommand(unsigned char* buf, std::string cmd)
{
    std::vector<unsigned char> v_cmd_frame;
    int cnt = 0, cmd_num = 0;
    std::string target = " ";
    std::string rep = ",";
    std::string temp_str;

    for(int i = 0; cmd.find(target) != std::string::npos && i < MAX_NUM_OF_PACKET_PARAM; i++)
    {
        cmd.replace(cmd.find(target), target.length(), rep);
    }

    temp_str = cmd;
    char* ptr_tok_cmd = strtok(&temp_str[0], ",");

    // searching matched cmd
    for(int i = 0; i < NUM_OF_CMD; i++)
    {
        if(strcmp(cmd_list[i], ptr_tok_cmd) == 0)
        {
            cmd_num = i;
            break;  
        }
    }

    v_cmd_frame.push_back(',');

    switch(cmd_num)
    {
        case SET_ACCESS_LEVEL :
        case SENSOR_START :
        case SENSOR_STOP :
            v_cmd_frame.push_back('s');
            v_cmd_frame.push_back('M');
            v_cmd_frame.push_back('C');
            break;
        
        case SENSOR_SCAN_INFO :
        case SCAN_DATA_CONFIG:
            v_cmd_frame.push_back('s');
            v_cmd_frame.push_back('R');
            v_cmd_frame.push_back('C');
            break;

        case NONE :
        default :
            std::cout << "invalid command, type correct command " << cmd_num << std::endl;
            return -1; 
            break;
    }
    v_cmd_frame.push_back(',');

    for(int i = 0; i < cmd.size() + 1; i++)
    {
        v_cmd_frame.push_back(cmd[i]);
    }

    memcpy(buf, &v_cmd_frame[0], v_cmd_frame.size());

    return v_cmd_frame.size() - 1;
}

// 231029
void* thread_parsingMsg(void* arg)
{
    Parser* parser = (Parser*) arg;
    std::vector<unsigned char> msgFrame;

    unsigned char recvdata = 0;
    char tempAsciidata[5] = {'\0', };
    unsigned char cmdtype[3] = {0, };
    unsigned char cmd[30] = {0, };
    uint16_t datasize = 0;
    uint8_t tempIndex = 0;
    uint32_t packetsize = 0;

    bool flag_cmdBegin = false;
    bool flag_cmdEnd = false;
    bool flag_dataBegin = false;

    bool flag_frameBegin = false;

    uint64_t popcnt = 0;
    parser->parsingStep_ = SEARCHSTX;

    struct timespec sleeptime, rem;
    sleeptime.tv_nsec = 100;
    sleeptime.tv_sec = 0;

    uint64_t timesum = 0;

    std::chrono::steady_clock::time_point time_begin;
    std::chrono::steady_clock::time_point time_end;

    std::chrono::steady_clock::time_point time_begin2;
    std::chrono::steady_clock::time_point time_end2;

    std::chrono::steady_clock::time_point time_begin3;
    std::chrono::steady_clock::time_point time_end3;
    
    uint64_t sizeWaitCnt = 0;

    while(1)
    {
        if(!(parser->readQueue_->empty()))                
        {
#if true
            parser->readMtx_->lock();
            recvdata = parser->readQueue_->front();
            parser->readQueue_->pop();
            parser->readMtx_->unlock();
            popcnt++;

            if(flag_frameBegin == true)
            {
                msgFrame.push_back(recvdata); // read and push

                if(parser->readQueue_->size() >= (packetsize - msgFrame.size()))
                {   
                    // std::cout << "sizeWaitCnt : " << sizeWaitCnt << std::endl;
                    sizeWaitCnt = 0;
                    time_end3 = std::chrono::steady_clock::now();
                    // std::cout << "time from start to while : " << std::chrono::duration_cast<std::chrono::microseconds> (time_end3 - time_begin3).count() << "us" << std::endl;

                    time_begin2 = std::chrono::steady_clock::now();
                    while(msgFrame.size() < packetsize)
                    {
                        parser->readMtx_->lock();
                        recvdata = parser->readQueue_->front();
                        parser->readQueue_->pop();
                        parser->readMtx_->unlock();
                        msgFrame.push_back(recvdata);
                        // popcnt++;
                    }
                    time_end2 = std::chrono::steady_clock::now();
                    // std::cout << "time in while : " << std::chrono::duration_cast<std::chrono::microseconds> (time_end2 - time_begin2).count() << "us" << std::endl;
                }
                else
                {
                    sizeWaitCnt++;
                    // std::cout << "queue size : " << parser->readQueue_->size() << ", remained packet size : " << (packetsize - msgFrame.size()) << std::endl;
                }
                
                if(msgFrame.size() == packetsize)
                {
                    flag_frameBegin = false;
                    if(recvdata == 0x03)
                    {
                        parser->recvMsgMtx_.lock();
                        parser->recvMsgQueue_.push(msgFrame);
                        parser->recvMsgMtx_.unlock();
                        msgFrame.clear();
                        time_end = std::chrono::steady_clock::now();
                        // std::cout << "time to put to recvMsgQueue : " << std::chrono::duration_cast<std::chrono::microseconds> (time_end - time_begin).count() << "us" << std::endl;
                        popcnt = 0;
                    }
                    else
                    {
                        msgFrame.clear();
                    }
                }
                else if(msgFrame.size() > packetsize)
                {
                    std::cout << "frame size mismatch" << msgFrame.size() << " packetsize : " << packetsize << std::endl;
                    flag_frameBegin = false;
                    msgFrame.clear();
                }
            }

            if(recvdata == 0x02)
            {                
                // std::cout << "popcnt : " << popcnt << std::endl;
                // popcnt = 0;
                time_begin = std::chrono::steady_clock::now();
                time_begin3 = std::chrono::steady_clock::now();
                flag_frameBegin = true;
                msgFrame.clear();

                msgFrame.push_back(recvdata);

                for(int i = 0; i < 4; i++)
                {
                    parser->readMtx_->lock();
                    recvdata = parser->readQueue_->front();
                    parser->readQueue_->pop();
                    parser->readMtx_->unlock();

                    tempAsciidata[i] = recvdata;

                    msgFrame.push_back(recvdata);

                    // std::cout <'/0'< "size" << i << " : " << tempAsciidata[i] << std::endl; ////
                }

                // std::cout << "data : "; ////
                // for(int i = 0; i < parser->msgFrame.size(); i++)////
                // {
                //     std::cout << parser->msgFrame[i] << " ";
                // }
                // std::cout << "" << std::endl;////

                packetsize = strtoul(tempAsciidata, NULL, 16);
                // std::cout << "packetsize : " << packetsize << std::endl;////
            }   
            
            
            
#else
            switch(parser->parsingStep)
            {
                case SEARCHSTX:
                    parser->readQueue->Mtx.lock();
                    recvdata = parser->readQueue->Queue.front();
                    parser->readQueue->Queue.pop();
                    parser->readQueue->Mtx.unlock();

                    if(recvdata == 0x02)
                    {
                        parser->msgFrame.push_back(recvdata);
                        packetsize = 0;
                        parser->parsingStep_ = PACKETSIZE;
                        tempIndex = 0;packetsize
                    }
                    break;    parser->readQueue->Mtx.lock();
                        recvdata = parser->readQueue->Queue.front();
                        parser->readQueue->Queue.pop();
                        parser->readQueue->Mtx.unlock();
                        parser->msgFrame.push_back(recvdata);

                case PACKETSIZE:
                    // for(int i = 0; i < 4; i++)
                    // {
                    //     parser->readQueue->mtx.lock();
                    //     recvdata = parser->readQueue->Queue.front();
                    //     parser->readQueue->Queue.pop();
                    //     parser->readQueue->mtx.unlock();

                    //     tempAsciidata[i] = recvdata
                    
                    //     parser->msgFrame.push_back(recvdata);
                    // }

                    parser->readQueue->Mtx.lock();
                    recvdata = parser->readQueue->Queue.front();
                    parser->readQueue->Queue.pop();
                    parser->readQueue->Mtx.unlock();
                    parser->msgFrame.push_back(recvdata);

                    tempAsciidata[tempIndex++] = recvdata;
                
                    if(tempIndex >= 4)
                    {
                        for(int i = 0; i < 4; i++)
                        {
                            std::cout << tempAsciidata[i];
                        }
                        std::cout << "" << std::endl;
                        packetsize = strtoul(tempAsciidata, NULL, 16);
                        std::cout << "packetsize : " << packetsize << std::endl;
                        parser->parsingStep_ = CMDTYPE;
                        tempIndex = 0;
                    }
                    break;        

                case CMDTYPE:
                    parser->readQueue->Mtx.lock();
                    recvdata = parser->readQueue->Queue.front();
                    parser->readQueue->Queue.pop();
                    parser->readQueue->Mtx.unlock();

                    parser->msgFrame.push_back(recvdata);

                    if(recvdata == ',')
                    {
                        
                    }
                    else
                    {
                        cmdtype[tempIndex++] = recvdata;

                        if(tempIndex >= 3)
                        {
                            parser->parsingStep_ = COMMAND;
                            tempIndex = 0;packetsize
                        }
                    }
                    break;

                case COMMAND:
                    parser->readQueue->Mtx.lock();
                    recvdata = parser->readQueue->Queue.front();
                    parser->readQueue->Queue.pop();
                    parser->readQueue->Mtx.unlock();
                    parser->msgFrame.push_back(recvdata);

                    if(recvdata == ',')
                    {
                        if(fla    ag_cmdEnd = false;
                        }
                        else
                        {
                            flag_cmdBegin = false;
                            flag_cmdEnd = true;
                            parser->parsingStep_ = DATA;
                            datasize = packetsize - 1 - 4 - (3+1) - (tempIndex+1) - 1;
                        }   
                    }
                    else if(recvdata == 0x03)
                    {
                        if(parser->msgFrame.size() == packetsize)
                        {
                            parser->recvMsgQueue.push(parser->msgFrame);
                            tempIndex = 0;
                            parser->parsingStep_ = SEARCHSTX;
                            parser->msgFrame.clear();
                            
                        }
                        else
                        {
                            std::cout << "CMD packetsize mismatch, msg : " << parser->msgFrame.size() << ", packetsize : " << packetsize << std::endl;
                            for(int i = 0; i < parser->msgFrame.size(); i++)
                            {
                                std::cout << parser->msgFrame[i];
                            }
                            std::cout << "" << std::endl;
                            tempIndex = 0;
                            parser-__requested_time,>parsingStep_ = SEARCHSTX;
                            parser->msgFrame.clear();
                        }
                        break;
                    }
                    else
                    {
                        if(flag_cmdBegin == true)
                        {
                            cmd[tempIndex++] = recvdata;
                        }
                    }
                    break;

                case DATA:
                    parser->readQueue->Mtx.lock();
                    recvdata = parser->readQueue->Queue.front();
                    parser->readQueue->Queue.pop();
                    parser->readQueue->Mtx.unlock();
                    parser->msgFrame.push_back(recvdata);

                    if(recvdata == ',')
                    {
                        flag_dataBegin = true;
                    }
                    else if(recvdata == 0x03)
                    {
                        if(parser->msgFrame.size() == packetsize)
                        {
                            parser->recvMsgQueue.push(parser->msgFrame);
                            tempIndex = 0;
                            parser->parsingStep_ = SEARCHSTX;
                            parser->msgFrame.clear();
                            
                        }
                        else
                        {
                            std::cout << "DATA packetsize mismatch, msg : " << parser->msgFrame.size() << ", packetsize : " << packetsize << std::endl;
                            for(int i = 0; i < parser->msgFrame.size(); i++)
                            {
                                std::cout << parser->msgFrame[i];
                            }
                            std::cout << "" << std::endl;
                            tempIndex = 0;
                            parser->parsingStep_ = SEARCHSTX;
                            parser->msgFrame.clear();
                        }
                        break;
                    }
                    else
                    {
                        if(parser->msgFrame.size() > packetsize)
                        {
                            std::cout << "DATA msgsize over, msg : " << parser->msgFrame.size() << ", packetsize : " << packetsize << std::endl;

                            for(int i = 0; i < parser->msgFrame.size(); i++)
                            {
                                std::cout << parser->msgFrame[i];
                            }
                            std::cout << "" << std::endl;

                            tempIndex = 0;
                            parser->parsingStep_ = SEARCHSTX;    parser->readQueue->Mtx.lock();
                        recvdata = parser->readQueue->Queue.front();
                        parser->readQueue->Queue.pop();
                        parser->readQueue->Mtx.unlock();
                        parser->msgFrame.push_back(recvdata);
                            parser->msgFrame.clear();
                        }
                    }

                    if(flag_dataBegin == true)
                    {

                    }

                    if(parser->msgFrame.size() >= datasize-1)
                    {
                        flag_dataBegin = true;
                        parser->parsingStep_ = ETX;
                    }
                    break;

                case ETX:
                    parser->readQueue->Mtx.lock();
                    recvdata = parser->readQueue->Queue.front();
                    parser->readQueue->Queue.pop();
                    parser->readQueue->Mtx.unlock();
                    parser->msgFrame.push_back(recvdata);


                    if(recvdata == 0x03)ROS_HOSTNAME
                    {
                        // parser->recvMsgQueue.push(parser->msgFrame);
                        // tempIndex = 0;
                        // parser->parsingStep_ = SEARCHSTX;
                        // parser->msgFrame.clear();

                        if(parser->msgFrame.size() == packetsize)
                        {
                            parser->recvMsgQueue.push(parser->msgFrame);
                            tempIndex = 0;
                            parser->parsingStep_ = SEARCHSTX;
                            parser->msgFrame.clear();
                            
                        }
                        else
                        {
                            std::cout << "ETX packetsize mismatch, msg : " << parser->msgFrame.size() << ", packetsize : " << packetsize << std::endl;
                            tempIndex = 0;
                            parser->parsingStep_ = SEARCHSTX;
                            parser->msgFrame.clear();
                        }
                    }
                    else if(recvdata == 0x02)
                    {
                        parser->msgFrame.clear();
                        std::cout << "Parsing Err, get STX before One frame is finished" << std::endl;
                        parser->msgFrame.push_back(recvdata);
                        parser->parsingStep_ = PACKETSIZE;
                        tempIndex = 0;
                    }
                    else
                    {
                        if(parser->msgFrame.size() > packetsize)
                        {
                            tempIndex = 0;
                            parser->parsingStep_ = SEARCHSTX;
                            std::cout << "Parsing Err, miss ETX" << std::endl;
                            std::cout << "msgsize : " << parser->msgFrame.size() << ", packetsize : " << packetsize << std::endl;

                            // for(int i = 0; i < parser->msgFrame.size(); i ++)
                            // {
                            //     std::cout << parser->msgFrame[i];
                            // }
                            // std::cout << "" << std::endl;

                            parser->msgFrame.clear();
                        }
                    }

                    break;
            }
#endif
        }
        usleep(1000*1);
    }
}


// 231029 parsing start
int Parser::parsingStart()
{    
    if(pthread_create(&pthrd_parser_id, NULL, thread_parsingMsg, this) < 0)
    {
        std::cout << "parsing pthread create failed" << std::endl;
        return -1;
    }
    else
    {
        pthread_parsing_running = true;
        return 0;
    }
}

void Parser::parsingMsg(std::vector<unsigned char> raw_msg, sensor_msgs::LaserScan::Ptr msg, Lsc_t* lsc)
{
    uint16_t msg_size = 0, index= 0, field_size = 0, index_base = 0;
    int cmd_type = 0, cmd = 0, version = 0, status = 0;
    float chk_intensity = 0;
    std::vector<char *> field;
    char* ptr;
    
    if(raw_msg[0] == 0x02)
    {        
        ptr = strtok((char*) &raw_msg[1], ",");

        if(raw_msg[strtoul(ptr,NULL, 16) - 1] == 0x03)
        {
            raw_msg[strtoul(ptr, NULL, 16) - 1] = '\0';
        }
        else
        {
            for(int i = 0; i < raw_msg.size(); i++)
            {
                std::cout << raw_msg[i] <<std::endl;
            }
            return;
        }

        while(ptr != NULL)
        {
            field.push_back(ptr);
            ptr = strtok(NULL, ",");
        }
    
        // get packet size
        msg_size = strtoul(field[index++], NULL, 16);
        field_size = field.size();

        // get command type
        for(int i = 0; i < NUM_OF_CMD_TYPE; i++)
        {
            int temp_ret = strcmp(field[index], cmd_type_list[i]);

            if(temp_ret == 0)
            {
                cmd_type = i;
                break;
            }
        }
        index++;

        // get command
        for(int i = 0; i < NUM_OF_CMD; i++)
        {
            int temp_ret = strcmp(field[index], cmd_list[i]);

            if(temp_ret == 0)
            {
                cmd = i;
                break;
            }
        }
        index++;

        switch(cmd)
        {
            case SENSOR_SCAN_INFO :
                lsc->scan_info.angle_start = strtoul(field[6], NULL, 32);
                lsc->scan_info.angle_end = strtoul(field[7], NULL, 32);
                lsc->scan_info.fw_ver = strtoul(field[8], NULL, 16);         // fw_ver
                lsc->scan_info.model_name = field[13];                        // model name
                
                #ifdef PRINT_PARSE_DATA                
                std::cout << "Command : " << cmd_list[cmd] << std::endl;
                std::cout << "fw_ver : " << lsc->scan_info.fw_ver << std::endl;
                std::cout << "Model_name : " << lsc->scan_info.model_name << std::endl;
                #endif

                break;

            case SCAN_DATA :
                for(int i = 0; i < NUMBER_OF_PARAM; i++)
                {
                    ptr = strtok(NULL, ",");
                    field.push_back(ptr);
                }
                lsc->scan_mea.scan_counter = strtoul(field[7], NULL, 16);       // scan counter
                lsc->scan_mea.scan_freq = strtoul(field[10], NULL, 16);          // scan freq
                lsc->scan_mea.meas_freq = strtoul(field[11], NULL, 16);          // meas freq
                lsc->scan_mea.angle_begin = strtoul(field[12], NULL, 16);        // angle_begin
                lsc->scan_mea.angle_resol = strtoul(field[13], NULL, 16);        // angle_resol
                lsc->scan_mea.amnt_of_data = strtoul(field[14], NULL, 16);       // amount of data
                index = 16;

                #ifdef PRINT_PARSE_DATA                
                std::cout << "Cmd type : " << cmd_type_list[cmd_type] << std::endl;
                std::cout << "Command : " << cmd_list[cmd] << std::endl;
                std::cout << "Scan counter : " << lsc->scan_mea.scan_counter << std::endl;
                std::cout << "Scan frequency : " << std::dec << lsc->scan_mea.scan_freq << std::endl;
                std::cout << "Measuring frequency : " << lsc->scan_mea.meas_freq << std::endl;
                std::cout << "Angle begin : " << lsc->scan_mea.angle_begin << std::endl;
                std::cout << "Angle resolution : " << lsc->scan_mea.angle_resol << std::endl;
                std::cout << "Amount of data : " << lsc->scan_mea.amnt_of_data << std::endl;
                #endif

                msg->angle_min = lsc->scan_mea.angle_begin / 10000.0 * DEG2RAD;
                msg->angle_max = (lsc->scan_mea.angle_begin / 10000.0 + (lsc->scan_mea.angle_resol / 10000.0 * lsc->scan_mea.amnt_of_data)) * DEG2RAD;
                msg->angle_increment = lsc->scan_mea.angle_resol / 10000.0 * DEG2RAD;
                msg->time_increment = (60.0 / lsc->scan_mea.meas_freq) / lsc->scan_mea.scan_counter;
                msg->scan_time = (60.0 / lsc->scan_mea.meas_freq); 

                #ifdef PRINT_PARSE_DATA 
                std::cout << "" << std::endl;
                std::cout << "msg->angle_min = " << msg->angle_min << std::endl;
                std::cout << "msg->angle_max = " << msg->angle_max << std::endl;
                std::cout << "msg->angle_increment = " << msg->angle_increment << std::endl;
                std::cout << "msg->range_max = " << msg->range_max << std::endl;
                std::cout << "msg->range_min = " << msg->range_min << std::endl;
                std::cout << "msg->scan_time = " << msg->scan_time << std::endl;
                std::cout << "msg->time_increment = " << msg->time_increment << std::endl;
                #endif
                
                if(strcmp(field[index++], "DIST1") == 0)
                {
                    index_base = index;

                    msg->ranges.clear();
                    for(uint16_t i = 0; i < lsc->scan_mea.amnt_of_data; i++)
                    {
                        msg->ranges.push_back(strtoul(field[index++], NULL, 16) / 1000.0);
                        
                    #ifdef PRINT_PARSE_DATA
                        std::cout << msg->ranges[i] << ", ";
                    }
                    std::cout << "" << std::endl;
                    #else
                    }                   
                    #endif
                }

                msg->intensities.clear();
                if(field[index] == field.back())
                {   
                }
                else
                {
                    if(strcmp(field[index++], "RSSI1") == 0 )
                    {
                        index_base = index;

                        for(uint16_t i = 0; i < lsc->scan_mea.amnt_of_data; i++)
                        { 
                            chk_intensity = strtoul(field[index++], NULL, 16);                            

                            msg->intensities.push_back(chk_intensity);

                        #ifdef PRINT_PARSE_DATA
                            std::cout << std::hex << std::uppercase << msg->intensities[i] << ",";
                        }
                        std::cout << "" << std::endl;
                        #else
                        }
                        #endif
                    }
                } 
                break;

            case SCAN_DATA_CONFIG:
                if(cmd_type == 3)
                {
                    lsc->scan_data_config.angle_start = strtol(field[3], NULL, 16);
                    lsc->scan_data_config.angle_end = strtol(field[4], NULL, 16);
                    lsc->scan_data_config.rssi_activate = strtoul(field[5], NULL, 16);
                    lsc->scan_data_config.scan_interval = strtoul(field[6], NULL, 16);
                    lsc->scan_data_config.fieldset_output_activate = strtoul(field[7], NULL, 16);
                }
                else if(cmd_type == 5)
                {
                    int resp = strtoul(field[0], NULL, 16);
                    std::cout << "LSScanDataConfig sWA " << resp << std::endl;
                }
                else
                {
                    std::cout << "LSScanDataConfig ??? " << std::endl;
                }

                break;

            case SENSOR_START :
            case SENSOR_STOP :
            case SET_ACCESS_LEVEL :
            case FIRST_CONNECT_DUMMY_SEND :
                for(int i = 2; i < field.size(); i++)
                {
                    std::cout << field[i] << " ";
                }
                break;
                
            default :
                for(int i = 2; i < field.size(); i++)
                {
                    std::cout << field[i] << " ";
                }
                std::cout << "" << std::endl;
                break;
        }
    }
}





UdpParser::UdpParser()
{
    pthrd_parser_id_ = 0;
    pthread_parsing_running_ = false;
}

UdpParser::~UdpParser()
{

}

bool UdpParser::getPthreadRunning(void)
{
    return pthread_parsing_running_;
}

void UdpParser::setPthreadRunning(bool flag)
{
    pthread_parsing_running_ = flag;
}

void UdpParser::setReadQueuePtr(std::shared_ptr<std::queue<unsigned char>> readQueuePtr)
{
    readQueue_ = readQueuePtr;
}

std::shared_ptr<std::queue<unsigned char>> UdpParser::getReadQueuePtr()
{
    return readQueue_;
}

void UdpParser::setReadMutexPtr(std::shared_ptr<std::mutex> readMutexPtr)
{
    readMtx_ = readMutexPtr;
}

std::shared_ptr<std::mutex> UdpParser::getReadMutexPtr(void)
{
    return readMtx_;
}

std::queue<std::vector<unsigned char>>* UdpParser::getRecvMsgQueuePtr(void)
{
    return &recvMsgQueue_;
}

std::mutex* UdpParser::getRecvMsgMtxPtr(void)
{
    return &recvMsgMtx_;
}

unsigned char makeCheckSum(unsigned char* pData, uint16_t nLength)
{
    uint32_t nSum = 0;
    for(int i=0 ; i<nLength; i++) nSum += pData[i];
    return ~(nSum & 0xFF) +1;
}

bool VerifyCheckSum(unsigned char* pData, uint16_t nLength)
{
    uint32_t nSum = 0;
    for(int i=0 ; i<nLength ; i++) nSum += pData[i];
    return ((nSum & 0xFF) == 0);
}

int UdpParser::makeUdpCmd(int cmd, unsigned char* sendbuf, UdpSet info)
{
    uint32_t length = 0;
    uint8_t checksum = 0;
    int i = 0;
    int index = 0;

    // stx
    sendbuf[index++] = 0xFC;
    sendbuf[index++] = 0xF3;

    // command 
    sendbuf[index++] = 0;
    sendbuf[index++] = cmd;

    // length
    index += 4;

    switch(cmd)
    {
        case 1:
            break;

        case 2:
            // MAC
            for(i = 0; i < 8; i++)
            {
                sendbuf[index++] = info.MAC[i];
            }

            // Old IP
            for(i = 0; i < 4; i++)
            {
                sendbuf[index++] = info.OldIp[i];
            }

            // New IP
            for(i = 0; i < 4; i++)
            {
                sendbuf[index++] = info.NewIp[i];
            }

            // SubnetMask
            for(i = 0; i < 4; i++)
            {
                sendbuf[index++] = info.SubnetMask[i];
            }

            // GateWay
            for(i = 0; i < 4; i++)
            {
                sendbuf[index++] = info.GateWay[i];
            }

            // Port
            for(i = 0; i < 4; i++)
            {
                sendbuf[index++] = info.Port[i];
            }
            break;

        default:
            std::cout << "cmdMake : invalid cmd" << std::endl;
            break;
    }

    // checksum
    index += 4;

    // calculate size
    length = index;

    sendbuf[4] = (length >> 24) & 0xFF;
    sendbuf[5] = (length >> 16) & 0xFF;
    sendbuf[6] = (length >> 8) & 0xFF;
    sendbuf[7] = (length) & 0xFF;

    // calculate checksum
    checksum = makeCheckSum(sendbuf, length);
    sendbuf[length-1] = checksum;

    return length;
}

int UdpParser::parsingMsg(std::vector<unsigned char> raw_msg, std::string recv_addr, UdpInfo* info)
{
    uint16_t msg_size = 0, index= 0, l_byte2 = 0;
    int cmd = 0, resp = 0, ret = 0;

    if(raw_msg[0] == 0xF3 && raw_msg[1] == 0xFC)
    {        
        index += 2;

        // get command
        memcpy(&l_byte2, &raw_msg[index], sizeof(l_byte2));
        cmd = htons(l_byte2);
        index += 2;
    
        // get packet size
        index += 2;
        memcpy(&l_byte2, &raw_msg[index], sizeof(l_byte2));
        msg_size = htons(l_byte2);
        index += 2;


        memset(info->IpAddr, 0x00, sizeof(info->IpAddr));

        if(recv_addr.size() == 0)
        {
            ;
        }
        else
        {
            info->IpAddr[0] = std::stoi(strtok(&recv_addr[0], "."));

            for(int i = 1; i < 4; i++)
            {
                char* temp_tok = strtok(NULL, ".");
                if(temp_tok != NULL)
                {
                    info->IpAddr[i] = std::stoi(temp_tok);
                }
                else
                {
                    break;
                }
            }
        }

        switch(cmd)
        {
            case 1 :
                for(int i = 0; i < 4; i++)
                {
                    info->BroadcastVersion[i] = raw_msg[index];
                    index++;
                }
            
                for(int i = 0; i < 4; i++)
                {
                    memcpy(&l_byte2, &raw_msg[index], sizeof(l_byte2));
                    info->HWVersion[i] = htons(l_byte2);
                    index += 2;
                }
                
                for(int i = 0; i < 4; i++)
                {
                    memcpy(&l_byte2, &raw_msg[index], sizeof(l_byte2));
                    info->SWVersion[i] = htons(l_byte2);
                    index += 2;
                }

                for(int i = 0; i < 4; i++)
                {
                    memcpy(&l_byte2, &raw_msg[index], sizeof(l_byte2));
                    info->FPGAVersion[i] = htons(l_byte2);
                    index += 2;
                }

                for(int i = 0; i < 4; i++)
                {
                    info->SubnetMask[i] = raw_msg[index];
                    index++;
                }

                for(int i = 0; i < 4; i++)
                {
                    info->GateWay[i] = raw_msg[index];
                    index++;
                }

                for(int i = 0; i < 4; i++)
                {
                    info->Port[i] = raw_msg[index];
                    index++;
                }

                for(int i = 0; i < 8; i++)
                {
                    info->MAC[i] = raw_msg[index];
                    index++;
                }

                for(int i = 0; i < 32; i++)
                {
                    if(raw_msg[index] == 0x00)
                    {
                        index += 32 - i;
                        info->ModelName[i] = '\n';
                        break;
                    }
                    else
                    {
                        info->ModelName[i] = raw_msg[index];
                        index++;
                    }
                }

                index += 3;
                info->InUse = raw_msg[index];
                ret = 1;
                
                std::cout << "parsed network info" << std::endl;
                break;

            case 2 :
                index += 3;
                resp = raw_msg[index];

                if(resp == 0)
                {
                    ret = -1;
                    std::cout << "NetWork infomation change failed" << std::endl;
                }
                else if(resp == 1)
                {
                    ret = 1;
                    std::cout << "NetWork infomation change complete" << std::endl;
                }
                else
                {
                    ret = -1;
                    std::cout << "invalied response value : " << resp << std::endl;
                }
                break;

            default :
                std::cout << "invalid cmd value : " << cmd << std::endl;
                break;
        }
    }
    else
    {
        std::cout << "stx isn't matched" << std::endl;
    }

    return ret;
}
    

void* thread_udp_parsingMsg(void* arg)
{
    UdpParser* parser = (UdpParser*) arg;

    std::cout << "udp_parsing entered" << std::endl;
    std::vector<unsigned char> msgFrame;

    unsigned char recvdata = 0;
    unsigned char recvdata2 = 0;

    unsigned char cmdtype[3] = {0, };
    char tempcmd[3] = {'\0', };
    char tempAsciidata[4] = {'\0', };
    uint16_t cmd = 0;
    uint16_t datasize = 0;
    uint8_t tempIndex = 0;

    uint32_t packetsize = 0;
    uint32_t l_packetsize = 0;

    bool flag_frameBegin = false;

    uint8_t checksum = 0;
    bool bool_checksum = false;

    uint64_t popcnt = 0;

    uint64_t sizeWaitCnt = 0;


    struct timespec sleeptime, rem;
    sleeptime.tv_nsec = 100;
    sleeptime.tv_sec = 0;

    uint64_t timesum = 0;

    std::chrono::steady_clock::time_point time_begin;
    std::chrono::steady_clock::time_point time_end;

    std::chrono::steady_clock::time_point time_begin2;
    std::chrono::steady_clock::time_point time_end2;

    std::chrono::steady_clock::time_point time_begin3;
    std::chrono::steady_clock::time_point time_end3;

    while(parser->getPthreadRunning())
    {
        if(!(parser->readQueue_->empty()))                
        {
            parser->readMtx_->lock();
            recvdata = parser->readQueue_->front();
            parser->readQueue_->pop();
            parser->readMtx_->unlock();
            popcnt++;

            if(flag_frameBegin == true)
            {
                msgFrame.push_back(recvdata); // read and push

                if(parser->readQueue_->size() >= (packetsize - msgFrame.size()))
                {   
                    // std::cout << "sizeWaitCnt : " << sizeWaitCnt << std::endl;
                    sizeWaitCnt = 0;
                    time_end3 = std::chrono::steady_clock::now();
                    // std::cout << "time from start to while : " << std::chrono::duration_cast<std::chrono::microseconds> (time_end3 - time_begin3).count() << "us" << std::endl;

                    time_begin2 = std::chrono::steady_clock::now();
                    while(msgFrame.size() < packetsize)
                    {
                        parser->readMtx_->lock();
                        recvdata = parser->readQueue_->front();
                        parser->readQueue_->pop();
                        parser->readMtx_->unlock();
                        msgFrame.push_back(recvdata);
                    }
                    time_end2 = std::chrono::steady_clock::now();
                }
                else
                {
                    sizeWaitCnt++;
                }
                
                if(msgFrame.size() == packetsize)
                {
                    flag_frameBegin = false;
                    
                    bool_checksum = VerifyCheckSum(&msgFrame[0], msgFrame.size());
                    checksum = makeCheckSum(&msgFrame[0], msgFrame.size());

                    if(bool_checksum > 0)
                    {
                        parser->recvMsgMtx_.lock();
                        parser->recvMsgQueue_.push(msgFrame);
                        parser->recvMsgMtx_.unlock();
                        msgFrame.clear();
                        time_end = std::chrono::steady_clock::now();
                        popcnt = 0;
                    }
                    else
                    {
                        msgFrame.clear();
                    }
                }
                else if(msgFrame.size() > packetsize)
                {
                    std::cout << "frame size mismatch" << msgFrame.size() << " packetsize : " << packetsize << std::endl;
                    flag_frameBegin = false;
                    msgFrame.clear();
                }
            }

            if(recvdata == 0xF3)
            {                
                parser->readMtx_->lock();
                recvdata2 = parser->readQueue_->front();
                // parser->readQueue_->pop();
                parser->readMtx_->unlock();

                if(recvdata2 == 0xFC)
                {
                    parser->readMtx_->lock();
                    parser->readQueue_->pop();
                    parser->readMtx_->unlock();
                    popcnt++;

                    time_begin = std::chrono::steady_clock::now();
                    time_begin3 = std::chrono::steady_clock::now();

                    flag_frameBegin = true;
                    msgFrame.clear();

                    msgFrame.push_back(recvdata);
                    msgFrame.push_back(recvdata2);

                    for(int i = 0; i < 2; i++)
                    {
                        parser->readMtx_->lock();
                        recvdata = parser->readQueue_->front();
                        parser->readQueue_->pop();
                        parser->readMtx_->unlock();

                        msgFrame.push_back(recvdata);
                    }

                    for(int i = 0; i < 4; i++)
                    {
                        parser->readMtx_->lock();
                        recvdata = parser->readQueue_->front();
                        parser->readQueue_->pop();
                        parser->readMtx_->unlock();

                        tempAsciidata[i] = recvdata;

                        msgFrame.push_back(recvdata);
                    }

                    memcpy(&l_packetsize, tempAsciidata, sizeof(tempAsciidata));
                    packetsize = htonl(l_packetsize);
                }
                else
                {
                    ;
                }
            }   
        }
        usleep(1000*1);
    }
    std::cout << "escape parser read while loop" << std::endl;

    return 0;
}


int UdpParser::parsingStart()
{    
    pthread_parsing_running_ = true;

    if(pthread_create(&pthrd_parser_id_, NULL, thread_udp_parsingMsg, this) < 0)
    {
        pthread_parsing_running_ = false;
        std::cout << "udp parsing pthread create failed" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "udp parsing thread is created" << std::endl;
        return 0;
    }
}


void UdpParser::parserClose()
{
    pthread_parsing_running_ = false;

    if(pthrd_parser_id_)   
    {
        pthread_join(pthrd_parser_id_, NULL);
    }

    std::cout << "udp parser closed" << std::endl;
}