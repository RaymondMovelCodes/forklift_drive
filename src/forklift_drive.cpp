/*
*****************************************************************************
* COPYRIGHT STATEMENT
* Copyright (c) 2025, Raymond Co.,Ltd. 
* All Rights Reserved.
* You can not use, copy or spread without official authorization.
*****************************************************************************
* Author: kuang
* Version: 1.0.1
* Date: 2025.09
* DESCRIPTION
* Raymond forklift drive module.
*/

#include "forklift_drive/forklift_drive.hpp"

namespace {
constexpr size_t SERIAL_BUF_SIZE = 30;
constexpr uint8_t SERIAL_HEAD0 = 255;
constexpr uint8_t SERIAL_HEAD1 = 1;
constexpr uint8_t SERIAL_END = 7;
constexpr uint8_t SERIAL_ACK_MIN = 1;
constexpr uint8_t SERIAL_ACK_MAX = 3;
constexpr uint8_t SERIAL_RECEIVE_HEAD1 = 2;
} 

namespace driver
{

ForkliftDrive::ForkliftDrive(ros::NodeHandle& node, ros::NodeHandle& private_nh)
  : serialConnect(false),
    chargingCmd(0),
    chargingStatus(0),
    chargingRunning(false)
{
  initVariable();
  initSerial(private_nh);
  initHotPlug();
  initTopic(node);
}

void ForkliftDrive::initVariable()
{
  serialConnect = false;
  chargingCmd = 0;
  chargingStatus = 0;
  chargingRunning = false;
  errorReceiveCount = 0;
  s_Data = {}; // 结构体清零
  s_Data.head0 = SERIAL_HEAD0;
  s_Data.head1 = SERIAL_HEAD1;
  s_Data.ack = SERIAL_ACK_MIN;
  s_Data.reserve1 = 0;
  s_Data.reserve2 = 0;
  s_Data.reserve3 = 0;
  s_Data.reserve4 = 0;
  s_Data.reserve5 = 0;
  s_Data.reserve6 = 0;
  s_Data.reserve7 = 0;
  s_Data.reserve8 = 0;
  s_Data.reserve9 = 0;
  s_Data.end = SERIAL_END;
  updateSerialSendData();
}

void ForkliftDrive::initSerial(ros::NodeHandle& private_nh)
{
  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<int>("baudrate", baudrate, 115200);
  private_nh.param<int>("loopRate", loopRate, 40);
}

void ForkliftDrive::clientTopicCallback(const forklift_drive::modbus_client::ConstPtr &clientTopicData)
{
    s_Data.driveSpeed_H = (clientTopicData->driveSpeed & 0xff00) >> 8;
    s_Data.driveSpeed_L = clientTopicData->driveSpeed & 0xff;
    s_Data.liftControl = clientTopicData->liftControl;
    s_Data.steerAngle_H = (clientTopicData->steerAngle & 0xff00) >> 8;
    s_Data.steerAngle_L = clientTopicData->steerAngle & 0xff;
    s_Data.ledRed = clientTopicData->lightRed;
    s_Data.ledGreen = clientTopicData->lightGreen;
    s_Data.ledYellow = clientTopicData->lightYellow;
    s_Data.leftObstacleLidarMap = clientTopicData->leftObstacleLidarMap;
    s_Data.rightObstacleLidarMap = clientTopicData->rightObstacleLidarMap;
    s_Data.Mp3Channel = clientTopicData->mp3Channel;
    s_Data.ledBuzzer = clientTopicData->buzzer;
    s_Data.wingsMotorControl = clientTopicData->wingsMotorControl;
    //s_Data.warningLightControl = clientTopicData->warningLightControl;
}

void ForkliftDrive::timer1Callback(const ros::TimerEvent &)
{
  updateSerialReceiveData();
  serverPub.publish(serverData);
}

void ForkliftDrive::timer2Callback(const ros::TimerEvent &)
{
  chargingProcess(chargingCmd);
}

bool ForkliftDrive::startChargingProcess()
{
  s_Data.autoChargeIR = Enable;
  if(r_Data1.chargeStatus==1)
  {
    s_Data.autoChargeSwitch = Enable;
    chargingStatus = 1;
    chargingRunning = false;
  }
  return true;
}

bool ForkliftDrive::endChargingProcess()
{
  s_Data.autoChargeSwitch = Disable;
  s_Data.autoChargeIR = Disable;
  if(r_Data1.chargeStatus==1)
  {
    chargingStatus = 2;
    chargingRunning = false;
  }
  return true;
}

bool ForkliftDrive::stopPowerProcess()
{
  s_Data.autoChargeSwitch = Disable;
  s_Data.autoChargeIR = Disable;
  chargingRunning = false;
  chargingStatus = 2;
  return true;
}

bool ForkliftDrive::chargingProcess(int8_t chargingCmd)
{
  if(!chargingRunning)
    return false;
  switch (chargingCmd)
  {
    case StartCharging:
      return startChargingProcess();
    case EndCharging:
      return endChargingProcess();
    case StopPower:
      return stopPowerProcess();
    default:
      chargingRunning = false;
      return false;
  }
}

bool ForkliftDrive::chargingServiceCallback(forklift_drive::chargingCmd::Request &req,
         forklift_drive::chargingCmd::Response &res)
{
  ROS_INFO("chargingCmd =%d", (int)req.chargingCmd);
  chargingCmd = req.chargingCmd;
  chargingRunning = true;
  res.result = true;
  return true;
}

void ForkliftDrive::initTopic(ros::NodeHandle& node)
{
  setServerTopic("forklift_drive_publisher");
  setClientTopic("forklift_drive_subscriber");
  setChargeService("charging");
  serverPub = node.advertise<forklift_drive::modbus_server>(m_serverTopic, 1);
  clientSub = node.subscribe<forklift_drive::modbus_client>(m_clientTopic, 1, boost::bind(&ForkliftDrive::clientTopicCallback, this, _1));
  timer1 = node.createTimer(ros::Duration(0.02), &ForkliftDrive::timer1Callback, this);
  timer2 = node.createTimer(ros::Duration(0.05), &ForkliftDrive::timer2Callback, this);
  chargeService = node.advertiseService(m_chargingService, &ForkliftDrive::chargingServiceCallback,this);
}

void ForkliftDrive::setServerTopic(std::string topic)
{
    m_serverTopic = std::move(topic);
}

void ForkliftDrive::setClientTopic(std::string topic)
{
    m_clientTopic = std::move(topic);
}

void ForkliftDrive::setChargeService(std::string service)
{
    m_chargingService = std::move(service);
}

ForkliftDrive::~ForkliftDrive() = default;

bool ForkliftDrive::connectController()
{
    constexpr int max_retry = 5;
    constexpr int retry_interval_ms = 2000;
    int retry_count = 0;

    while (retry_count < max_retry)
    {
        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(timeout);
            ser.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port: " << e.what() << ", retrying (" << retry_count + 1 << "/" << max_retry << ")...");
            serialConnect = false;
            ++retry_count;
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
            continue;
        }
        if (ser.isOpen())
        {
            ROS_INFO_STREAM("Serial Port initialized ok");
            serialConnect = true;
            ser.flushInput();
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Serial Port initialized failed! Retrying (" << retry_count + 1 << "/" << max_retry << ")...");
            serialConnect = false;
            ++retry_count;
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
        }
    }
    ROS_ERROR_STREAM("Serial Port could not be opened after " << max_retry << " attempts. Will continue with hot-plug detection.");
    serialConnect = false;
    return false;
}

void ForkliftDrive::updateSerialReceiveData()
{
  serverData.header.seq = 0;
  serverData.header.stamp = ros::Time::now();
  serverData.header.frame_id = "forklift_drive";
  serverData.driveSpeed = r_Data1.driveSpeed;
  serverData.liftControl = s_Data.liftControl;
  serverData.steerAngle = r_Data1.steerAngle;
  serverData.emergencyStop = r_Data1.emergencyStop;
  serverData.bumper = r_Data1.bumper;
  serverData.buttonStop = r_Data1.buttonStop;
  serverData.buttonStart = r_Data1.buttonStart;
  serverData.leftForkTipAvoid = r_Data1.leftForkTipAvoid;
  serverData.rightForkTipAvoid = r_Data1.rightForkTipAvoid;
  serverData.forkUpLimitStatus = r_Data1.forkUpLimitStatus;
  serverData.forkDownLimitStatus = r_Data1.forkDownLimitStatus;
  serverData.cargoStatus = r_Data1.cargoStatus;
  serverData.chargeStatus = r_Data1.chargeStatus;
  serverData.mp3Channel = s_Data.Mp3Channel;
  serverData.buzzer = s_Data.ledBuzzer;
  serverData.isAuto = r_Data1.isAuto;

  serverData.leftObstacleLidarTrigger1 = r_Data2.leftObstacleLidarTrigger1;
  serverData.leftObstacleLidarTrigger2 = r_Data2.leftObstacleLidarTrigger2;
  serverData.leftObstacleLidarTrigger3 = r_Data2.leftObstacleLidarTrigger3;
  serverData.rightObstacleLidarTrigger1 = r_Data2.rightObstacleLidarTrigger1;
  serverData.rightObstacleLidarTrigger2 = r_Data2.rightObstacleLidarTrigger2;
  serverData.rightObstacleLidarTrigger3 = r_Data2.rightObstacleLidarTrigger3;
  serverData.driveMotorErrorCode = r_Data2.driveMotorErrorCode;
  serverData.steeringMotorErrorCode = r_Data2.steeringMotorErrorCode;
  serverData.liftMotorErrorCode = r_Data2.liftMotorErrorCode; 
  serverData.hardwareStatus = r_Data2.hardwareStatus;
  serverData.batteryPctg = r_Data2.batteryPctg;
  serverData.batteryVal = r_Data2.batteryVal; 
  serverData.batteryCurrent = r_Data2.batteryCurrent;

  serverData.leftMiddleUltrasonicRadarValue = r_Data3.leftMiddleUltrasonicRadarValue;
  serverData.rightMiddleUltrasonicRadarValue = r_Data3.rightMiddleUltrasonicRadarValue;
  serverData.dirveMotorEncoder = r_Data3.dirveMotorEncoder; 
  serverData.leftUpUltrasonicRadarValue = r_Data3.leftUpUltrasonicRadarValue;
  serverData.leftDownUltrasonicRadarValue = r_Data3.leftDownUltrasonicRadarValue;
  serverData.rightUpUltrasonicRadarValue = r_Data3.rightUpUltrasonicRadarValue;
  serverData.rightDownUltrasonicRadarValue = r_Data3.rightDownUltrasonicRadarValue;
  serverData.wingsStatus = r_Data3.wingsStatus; 
}

void ForkliftDrive::updateSerialSendData()
{
  s_Data.checksum = 0x00;
  serialSendBuf[0] = s_Data.head0;
  serialSendBuf[1] = s_Data.head1;
  serialSendBuf[2] = s_Data.ack;
  serialSendBuf[3] = s_Data.driveSpeed_H;
  serialSendBuf[4] = s_Data.driveSpeed_L;
  serialSendBuf[5] = s_Data.steerAngle_H;
  serialSendBuf[6] = s_Data.steerAngle_L;
  serialSendBuf[7] = s_Data.liftControl;
  serialSendBuf[8] = s_Data.ledRed;
  serialSendBuf[9] = s_Data.ledGreen;
  serialSendBuf[10] = s_Data.ledYellow;
  serialSendBuf[11] = s_Data.ledBuzzer;
  serialSendBuf[12] = s_Data.leftObstacleLidarMap;
  serialSendBuf[13] = s_Data.rightObstacleLidarMap;
  serialSendBuf[14] = s_Data.autoChargeIR;
  serialSendBuf[15] = s_Data.autoChargeSwitch;
  serialSendBuf[16] = s_Data.Mp3Channel;
  serialSendBuf[17] = s_Data.wingsMotorControl;
  serialSendBuf[18] = s_Data.warningLightControl;
  serialSendBuf[19] = s_Data.reserve1;  
  serialSendBuf[20] = s_Data.reserve2;  
  serialSendBuf[21] = s_Data.reserve3;
  serialSendBuf[22] = s_Data.reserve4;
  serialSendBuf[23] = s_Data.reserve5;
  serialSendBuf[24] = s_Data.reserve6;
  serialSendBuf[25] = s_Data.reserve7;
  serialSendBuf[26] = s_Data.reserve8;
  serialSendBuf[27] = s_Data.reserve9;
  for (size_t i = 0; i < 28; i++)
  {
    s_Data.checksum ^= serialSendBuf[i];
  }
  serialSendBuf[28] = s_Data.checksum;
  serialSendBuf[29] = s_Data.end;
}

int16_t ForkliftDrive::BitCovTo16(unsigned char (&arry)[SERIAL_BUF_SIZE] , uint8_t index)
{
  unsigned char tempbuf[2];
  int16_t value = 0;

  tempbuf[0] = arry[index + 1];
  tempbuf[1] = arry[index];
  std::memcpy(&value, tempbuf, sizeof(value));
  return value;
}

int32_t ForkliftDrive::BitCovTo32(unsigned char (&arry)[SERIAL_BUF_SIZE] , uint8_t index)
{
  unsigned char tempbuf[4];
  int32_t value = 0;

  tempbuf[0] = arry[index + 3];
  tempbuf[1] = arry[index + 2];
  tempbuf[2] = arry[index + 1];
  tempbuf[3] = arry[index];
  std::memcpy(&value, tempbuf, sizeof(value));
  return value;
}

void ForkliftDrive::warnLightProcess()
{
  static int count = 0;

  if(r_Data1.driveSpeed != 0)
  {
    count++;
    if(count > 10) 
    {
      count = 0;
      s_Data.warningLightControl = 1;
    }
  }
  else
  {
    count = 0;
    s_Data.warningLightControl = 0;
  }
}

bool ForkliftDrive::Poll()
{
  connectController();
  ros::Rate loop_rate(loopRate);
  while (ros::ok())
  {
    try
    {
      // 热插拔检测逻辑
      if (!serialConnect && shouldAttemptReconnect())
      {
        ROS_INFO("Attempting to reconnect serial port...");
        if (!attemptReconnect())
        {
          ROS_DEBUG("Reconnection failed, will retry later");
        }
      }
      
      // 检查串口连接状态
      if (serialConnect && !isSerialConnected())
      {
        handleSerialDisconnection();
      }
      
      if (serialConnect)
      {
        updateSerialSendData();
        ser.write(serialSendBuf, SERIAL_BUF_SIZE);
        size_t num = ser.available();
        if(num > 0)
        {
          ser.read(serialReceiveBuf, num);
          if ((serialReceiveBuf[0] == SERIAL_HEAD0) && (serialReceiveBuf[1] == SERIAL_RECEIVE_HEAD1))
          {
            errorReceiveCount = 0;
            switch(serialReceiveBuf[2])
            {
              case 1:
                if(serialReceiveBuf[7] == 1)
                  r_Data1.driveSpeed = 0;
                else
                  r_Data1.driveSpeed = BitCovTo16(serialReceiveBuf,3);
                r_Data1.steerAngle = BitCovTo16(serialReceiveBuf,5);
                r_Data1.emergencyStop = serialReceiveBuf[7];
                r_Data1.bumper = serialReceiveBuf[8];
                r_Data1.buttonStop = serialReceiveBuf[9];
                r_Data1.buttonStart = serialReceiveBuf[10];
                r_Data1.leftForkTipAvoid = serialReceiveBuf[11];
                r_Data1.rightForkTipAvoid = serialReceiveBuf[12];
                r_Data1.forkUpLimitStatus = serialReceiveBuf[13];
                r_Data1.forkDownLimitStatus = serialReceiveBuf[14];
                r_Data1.cargoStatus = serialReceiveBuf[15];
                r_Data1.chargeStatus = serialReceiveBuf[16];
                r_Data1.isAuto = serialReceiveBuf[17];
                break;
              case 2:
                r_Data2.leftObstacleLidarTrigger1 = serialReceiveBuf[3];
                r_Data2.leftObstacleLidarTrigger2 = serialReceiveBuf[4];
                r_Data2.leftObstacleLidarTrigger3 = serialReceiveBuf[5];
                r_Data2.rightObstacleLidarTrigger1 = serialReceiveBuf[6];
                r_Data2.rightObstacleLidarTrigger2 = serialReceiveBuf[7];
                r_Data2.rightObstacleLidarTrigger3 = serialReceiveBuf[8];
                r_Data2.driveMotorErrorCode = serialReceiveBuf[9];
                r_Data2.steeringMotorErrorCode = serialReceiveBuf[10];
                r_Data2.liftMotorErrorCode = serialReceiveBuf[12];
                r_Data2.hardwareStatus = serialReceiveBuf[13];
                r_Data2.batteryPctg = serialReceiveBuf[14];
                r_Data2.batteryVal = serialReceiveBuf[15];
                r_Data2.batteryCurrent = BitCovTo16(serialReceiveBuf,16) * (-0.1);
                break;
              case 3:
                r_Data3.leftMiddleUltrasonicRadarValue = BitCovTo16(serialReceiveBuf,3);
                r_Data3.rightMiddleUltrasonicRadarValue = BitCovTo16(serialReceiveBuf,5);
                r_Data3.dirveMotorEncoder = BitCovTo32(serialReceiveBuf,7);
                r_Data3.leftUpUltrasonicRadarValue = BitCovTo16(serialReceiveBuf,11);
                r_Data3.leftDownUltrasonicRadarValue = BitCovTo16(serialReceiveBuf,13);
                r_Data3.rightUpUltrasonicRadarValue = BitCovTo16(serialReceiveBuf,15);
                r_Data3.rightDownUltrasonicRadarValue = BitCovTo16(serialReceiveBuf,17);
                r_Data3.wingsStatus = serialReceiveBuf[19]; 
                break;
              default:
                break;
            }
          }
          else
          {
            errorReceiveCount++;
            if(errorReceiveCount > 200)
            {
              errorReceiveCount = 0;
              std::cout << "receive error data over 5 seconds,then will reset the UART..." << std::endl;
              handleSerialDisconnection();
            }
            //for (size_t i = 0; i < SERIAL_BUF_SIZE; i++)
            //{
            //  std::cout << "receive data" << i << ": " << std::to_string(serialReceiveBuf[i]) << std::endl;
            //}
          }
        }
        else
        {
          errorReceiveCount++;
          if(errorReceiveCount > 200)
          {
            errorReceiveCount = 0;
            std::cout << "no receive data over 5 seconds,then will reset the UART..." << std::endl;
            handleSerialDisconnection();
          }
        }
      }
      s_Data.ack++;
      if(s_Data.ack > SERIAL_ACK_MAX)
        s_Data.ack = SERIAL_ACK_MIN;
      warnLightProcess();
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return true;
}

void ForkliftDrive::initHotPlug()
{
  hotPlugEnabled = true;
  reconnectInterval = 2000; // 2秒重连间隔
  maxReconnectAttempts = -1; // 无限重连
  currentReconnectAttempts = 0;
  lastReconnectAttempt = std::chrono::steady_clock::now();
  //ROS_INFO("Hot-plug functionality initialized");
}

bool ForkliftDrive::isSerialConnected()
{
  if (!ser.isOpen())
  {
    serialConnect = false;
    return false;
  }
  
  // 尝试检查串口是否真正可用
  try
  {
    // 检查串口是否可写入（简单的连接性测试）
    if (ser.available() >= 0) // available()方法会在串口断开时抛出异常
    {
      return true;
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("Serial connection check failed: " << e.what());
    serialConnect = false;
    return false;
  }
  
  return false;
}

bool ForkliftDrive::shouldAttemptReconnect()
{
  if (!hotPlugEnabled || serialConnect)
    return false;
    
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReconnectAttempt);
  
  return elapsed.count() >= reconnectInterval;
}


bool ForkliftDrive::attemptReconnect()
{
  lastReconnectAttempt = std::chrono::steady_clock::now();
  
  ROS_INFO_STREAM("Attempting to reconnect to configured port: " << port);
  
  try
  {
    // 确保串口已关闭
    if (ser.isOpen())
    {
      ser.close();
    }
    
    // 尝试重新连接配置的串口
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(timeout);
    ser.open();
    
    if (ser.isOpen())
    {
      ROS_INFO_STREAM("Serial port reconnected successfully: " << port);
      serialConnect = true;
      currentReconnectAttempts = 0;
      errorReceiveCount = 0;
      ser.flushInput();
      return true;
    }
  }
  catch (const serial::IOException& e)
  {
    ROS_DEBUG_STREAM("Reconnection attempt failed for port " << port << ": " << e.what());
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("Unexpected error during reconnection to port " << port << ": " << e.what());
  }
  
  serialConnect = false;
  currentReconnectAttempts++;
  
  // 检查是否达到最大重连次数限制
  if (maxReconnectAttempts > 0 && currentReconnectAttempts >= maxReconnectAttempts)
  {
    ROS_WARN_STREAM("Maximum reconnection attempts (" << maxReconnectAttempts << ") reached for port " << port);
  }
  
  return false;
}

void ForkliftDrive::handleSerialDisconnection()
{
  if (serialConnect)
  {
    ROS_WARN("Serial connection lost, enabling hot-plug detection");
    serialConnect = false;
    // 安全关闭串口
    try
    {
      if (ser.isOpen())
      {
        ser.close();
      }
    }
    catch (const std::exception& e)
    {
      ROS_DEBUG_STREAM("Error closing serial port: " << e.what());
    }
    // 重置错误计数
    errorReceiveCount = 0;
  }
}

}