#ifndef FORKLIFT_DRIVE_HPP
#define FORKLIFT_DRIVE_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include "forklift_drive/modbus_client.h"
#include "forklift_drive/modbus_server.h"
#include "forklift_drive/chargingCmd.h"
#include "forklift_drive/chargingStatus.h"
#include <ros/time.h>
#include <ros/duration.h>
#include <numeric>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <filesystem>
#include <algorithm>

#define MAX_BUF_SIZE 30

namespace driver
{

class ForkliftDrive
{
private:
  std::string m_serverTopic;
  std::string m_clientTopic;
  std::string m_chargingService;
  ros::Publisher serverPub;
  ros::Subscriber clientSub;
  ros::ServiceServer chargeService;
  int8_t chargingCmd;
  int8_t chargingStatus;
  bool chargingRunning;
  ros::Timer timer1;
  ros::Timer timer2;
  std::string port;
  int baudrate;
  int loopRate;
  serial::Serial ser;
  bool serialConnect;
  unsigned char serialSendBuf[MAX_BUF_SIZE];
  unsigned char serialReceiveBuf[MAX_BUF_SIZE];
  int errorReceiveCount;
  
  // 热插拔相关成员变量
  std::chrono::steady_clock::time_point lastReconnectAttempt;
  int reconnectInterval; // 重连间隔(毫秒)
  int maxReconnectAttempts; // 最大重连尝试次数
  int currentReconnectAttempts; // 当前重连尝试次数
  bool hotPlugEnabled; // 热插拔功能开关
  struct send_Data
  {
    unsigned char head0;
    unsigned char head1;
    unsigned char ack;
    char driveSpeed_H;
    char driveSpeed_L;
    char steerAngle_H;
    char steerAngle_L;
    unsigned char liftControl;   //0 stop; 1 up; 2 down
    unsigned char ledRed;
    unsigned char ledGreen;
    unsigned char ledYellow;
    unsigned char ledBuzzer;
    unsigned char leftObstacleLidarMap;
    unsigned char rightObstacleLidarMap;
    unsigned char autoChargeIR;
    unsigned char autoChargeSwitch;
    unsigned char Mp3Channel;
    unsigned char wingsMotorControl;
    unsigned char warningLightControl;
    unsigned char reserve1;
    unsigned char reserve2;
    unsigned char reserve3;
    unsigned char reserve4;
    unsigned char reserve5;
    unsigned char reserve6;
    unsigned char reserve7;
    unsigned char reserve8;
    unsigned char reserve9;
    unsigned char checksum;
    unsigned char end;
  };
  struct send_Data s_Data;

  struct receive_Data1
  {
    int16_t driveSpeed;
    int16_t steerAngle;
    int8_t emergencyStop;
    int8_t bumper;
    int8_t buttonStop;
    int8_t buttonStart;
    int8_t leftForkTipAvoid;
    int8_t rightForkTipAvoid;
    int8_t forkUpLimitStatus;
    int8_t forkDownLimitStatus;
    int8_t cargoStatus;
    int8_t chargeStatus;
    int8_t isAuto;  
  };
  struct receive_Data1 r_Data1;

  struct receive_Data2
  {
    int8_t leftObstacleLidarTrigger1;
    int8_t leftObstacleLidarTrigger2;
    int8_t leftObstacleLidarTrigger3;
    int8_t rightObstacleLidarTrigger1;
    int8_t rightObstacleLidarTrigger2;
    int8_t rightObstacleLidarTrigger3;
    int8_t driveMotorErrorCode;
    int8_t steeringMotorErrorCode;
    int8_t liftMotorErrorCode;
    int8_t hardwareStatus;
    int8_t batteryPctg;
    int8_t batteryVal;
    int16_t batteryCurrent;
  };
  struct receive_Data2 r_Data2;

  struct receive_Data3
  {
    int16_t leftMiddleUltrasonicRadarValue;
    int16_t rightMiddleUltrasonicRadarValue;
    int32_t dirveMotorEncoder;
    int16_t leftUpUltrasonicRadarValue;
    int16_t leftDownUltrasonicRadarValue;
    int16_t rightUpUltrasonicRadarValue;
    int16_t rightDownUltrasonicRadarValue;
    int8_t wingsStatus;
  };
  struct receive_Data3 r_Data3;
  forklift_drive::modbus_server serverData;
  enum
  {
    StopPower = 0,
    StartCharging,
    EndCharging
  };

  enum
  {
    Disable = 0,
    Enable
  };

public:
  ForkliftDrive(ros::NodeHandle& node, ros::NodeHandle& private_nh);
  ~ForkliftDrive();
  bool Poll();
  void timer1Callback(const ros::TimerEvent &);
  void timer2Callback(const ros::TimerEvent &);
  bool chargingServiceCallback(forklift_drive::chargingCmd::Request &req,
        forklift_drive::chargingCmd::Response &res);
  void clientTopicCallback(const forklift_drive::modbus_client::ConstPtr &clientTopicData);
private: 
  void initVariable();
  void initSerial(ros::NodeHandle& private_nh);
  void initTopic(ros::NodeHandle& private_nh);
  void setServerTopic(std::string topic);
  void setClientTopic(std::string topic);
  void setChargeService(std::string service);
  bool connectController();
  void updateSerialSendData();
  void updateSerialReceiveData();
  int16_t BitCovTo16(unsigned char (&arry)[MAX_BUF_SIZE] , uint8_t index);
  int32_t BitCovTo32(unsigned char (&arry)[MAX_BUF_SIZE] , uint8_t index);
  bool chargingProcess(int8_t chargingCmd);
  bool startChargingProcess();
  bool endChargingProcess();
  bool stopPowerProcess();
  void warnLightProcess();
  
  // 热插拔相关方法
  void initHotPlug();
  bool isSerialConnected();
  bool attemptReconnect();
  void handleSerialDisconnection();
  bool shouldAttemptReconnect();
};

}
#endif  //FORKLIFT_DRIVE_HPP