/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Mercury model on this example : All models using Protocol 2.0
// This example is tested with two Mercury PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Mercury PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include "../../include/mercury_sdk/mercury_sdk.h" // Uses Mercury SDK library

// Control table address
#define ADDR_MCY_TORQUE_ENABLE 0x30 // Mercury Control table register addresses
#define ADDR_MCY_GOAL_POSITION 0x4e
#define ADDR_MCY_PRESENT_POSITION 0x5a

// Data Byte Length
#define LEN_MCY_GOAL_POSITION 4
#define LEN_MCY_PRESENT_POSITION 4

// Default setting
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyACM1" // Check which port is being used on your controller
                                  // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE 1                  // Value for enabling the torque
#define TORQUE_DISABLE 0                 // Value for disabling the torque
#define MCY_MINIMUM_POSITION_VALUE -2000 // Mercury will rotate between this value
#define MCY_MAXIMUM_POSITION_VALUE 2000  // and this value (note that the Mercury would not move when the position value is out of movable range. Check e-manual about the range of the Mercury you use.)
#define MCY_MOVING_STATUS_THRESHOLD 20   // Mercury moving status threshold
    
#define ESC_ASCII_VALUE 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  mercury::PortHandler *portHandler = mercury::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  mercury::PacketHandler *packetHandler = mercury::PacketHandler::getPacketHandler();

  // Initialize GroupSyncWrite instance
  mercury::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MCY_GOAL_POSITION, LEN_MCY_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  mercury::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);

  struct mcy_servo
  {
    uint8_t id;
    int32_t mcy_present_position;
  };

  const int acknowledge_response_delay_ms = 1000; 
  // This should be chosen with the longest acknowledge response time in mind.

  const int rom_write_delay_ms = 1e06;

  std::vector<mcy_servo> mcy_servos{{1, 0}, {2, 0}, {3, 0}};
   
  int index = 0;
  int mcy_comm_result = COMM_TX_FAIL;                                                   // Communication result
  bool mcy_addparam_result = false;                                                    // addParam result
  bool mcy_getdata_result = false;                                                     // GetParam result
  int mcy_goal_position[2] = {MCY_MINIMUM_POSITION_VALUE, MCY_MAXIMUM_POSITION_VALUE}; // Goal position

  uint8_t mcy_error = 0; // Mercury error
  uint8_t param_goal_position[4];

  // Open port
  if (portHandler->openPort())
  { 
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

 // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    // This is a rom write, so add a delay of 1s
    usleep(rom_write_delay_ms);

    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  printf("**** Synchronising Mercury servos ****\n");
  for (mcy_servo servo : mcy_servos)
  {

    printf("Synchronising Mercury#%d\n", servo.id);
    usleep(acknowledge_response_delay_ms);

    mcy_comm_result = packetHandler->synchronise(portHandler, servo.id, &mcy_error);
    if (mcy_comm_result != COMM_SUCCESS)
    { 
      printf("Mercury#%d: has failed to synchronise. Error: %s\n", servo.id, packetHandler->getTxRxResult(mcy_comm_result));
    }
    else
    {
      printf("Mercury#%d is synchronised\n", servo.id);
    } 
  }

  printf("**** All Mercury servos are synchronised ****\n");
 
  printf("**** Connecting Mercury servos ****\n");

  for (mcy_servo servo : mcy_servos)
  {
    usleep(acknowledge_response_delay_ms);
    mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, servo.id, ADDR_MCY_TORQUE_ENABLE, TORQUE_ENABLE, &mcy_error);
    if (mcy_comm_result != COMM_SUCCESS)
    {
      printf("Mercury#%d: %s\n", servo.id, packetHandler->getTxRxResult(mcy_comm_result));
    }
    else if (mcy_error != 0)
    {
      printf("Mercury#%d torque enabled failed: %s\n", servo.id, packetHandler->getRxPacketError(mcy_error));
    }
    else
    {
      printf("Mercury#%d has been successfully connected \n", servo.id);
    }
  }
 
  printf("**** All Mercury servos are connected ****\n");

  printf("Press any key to continue! (or press ESC to quit!)\n");
  getch();

  for (mcy_servo servo : mcy_servos)
  {
    usleep(acknowledge_response_delay_ms);
    // Add parameter storage for Mercury#1 present position value
    mcy_addparam_result = groupSyncRead.addParam(servo.id);
    if (mcy_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", servo.id);
    }
  }

  int iteration = 0;
  while (1)
  {
#if defined(__linux__)
    usleep(100000);
#endif

    if (kbhit()) 
    {
      int ch = getch();
      if (ch == ESC_ASCII_VALUE)
        break;
    }

    iteration++;

    printf("Iteration:%03d\t\n", iteration);

    // Allocate goal position value into byte array
    param_goal_position[0] = MCY_LOBYTE(MCY_LOWORD(mcy_goal_position[index]));
    param_goal_position[1] = MCY_HIBYTE(MCY_LOWORD(mcy_goal_position[index]));
    param_goal_position[2] = MCY_LOBYTE(MCY_HIWORD(mcy_goal_position[index]));
    param_goal_position[3] = MCY_HIBYTE(MCY_HIWORD(mcy_goal_position[index]));

    for (mcy_servo servo : mcy_servos)
    {
      // Add Mercury goal position value to the Syncwrite storage
      mcy_addparam_result = groupSyncWrite.addParam(servo.id, param_goal_position);
      if (mcy_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", servo.id);
        return 0;
      }
    }

    usleep(acknowledge_response_delay_ms);
    // Syncwrite goal position
    mcy_comm_result = groupSyncWrite.txPacket();
    if (mcy_comm_result != COMM_SUCCESS)
      printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
#if defined(__linux__)
      usleep(100000);
#endif

      // Syncread present position
      mcy_comm_result = groupSyncRead.txRxPacket();
      if (mcy_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
      }
      else
      {
        for (mcy_servo servo : mcy_servos)
        {
          if (groupSyncRead.getError(servo.id, &mcy_error))
            printf("[ID:%03d] %s\n", servo.id, packetHandler->getRxPacketError(mcy_error));
        }
      }

      for (mcy_servo& servo : mcy_servos)
      {
        usleep(acknowledge_response_delay_ms);

        // Check if groupsyncread data of Mercury#1 is available
        mcy_getdata_result = groupSyncRead.isAvailable(servo.id, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);
        if (mcy_getdata_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupSyncRead isAvailable failed \n", servo.id);
        }
        else 
        {
          servo.mcy_present_position = groupSyncRead.getData(servo.id, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);
          printf("[ID:%03d] GoalPos1:%03d  PresPos:%03d ", (int)servo.id, mcy_goal_position[index], servo.mcy_present_position);
        }
      }

      printf("\t\n");

    } while (std::any_of(mcy_servos.begin(), mcy_servos.end(), [mcy_goal_position, index](mcy_servo servo) 
          {return (abs(mcy_goal_position[index] - servo.mcy_present_position) > MCY_MOVING_STATUS_THRESHOLD);}));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

  for (mcy_servo servo : mcy_servos)
  {
    usleep(acknowledge_response_delay_ms);
    mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, servo.id, ADDR_MCY_TORQUE_ENABLE, TORQUE_DISABLE, &mcy_error);
    if (mcy_comm_result != COMM_SUCCESS)
    {
      printf("Mercury#%d: %s\n", servo.id, packetHandler->getTxRxResult(mcy_comm_result));
    }
    else if (mcy_error != 0)
    {
      printf("Mercury#%d torque disable failed: %s\n", servo.id, packetHandler->getRxPacketError(mcy_error));
    }
    else
    { 
      printf("Mercury#%d has been successfully disconnected \n", servo.id);
    }
  }

  // Close port
  portHandler->closePort();

  return 0;
}