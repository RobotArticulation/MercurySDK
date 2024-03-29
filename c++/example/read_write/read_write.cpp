/*******************************************************************************
* Copyright (C) 2021 <Robot Articulation/code@robotarticulation.com> 
*
* Source files modified to support the Mercury range of digital servo motors from Robot Articulation
*******************************************************************************/

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

/* Original author: Zerom, Leon (RyuWoon Jung) */

//
// *********     Read and Write Example      *********
//
// This example is tested with a Mercury M65, and a USB2Mercury. 
// Ensure that the Mercury servo is in position control mode prior to running this example.
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

#include "mercury_sdk/mercury_sdk.h"                                  // Uses Mercury SDK library
#include "mercury_sdk/synchronisation_helper.h"

#define ADDR_MCY_TORQUE_ENABLE           0x30                // Mercury Control table register addresses
#define ADDR_MCY_GOAL_POSITION           0x4e
#define ADDR_MCY_PRESENT_POSITION        0x5a
#define ADDR_MCY_HARDWARE_STATUS         0x6b

// Default setting
#define MCY_ID                          1                   // Mercury ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define MCY_MINIMUM_POSITION_VALUE      -1000                 // Mercury will rotate between this value
#define MCY_MAXIMUM_POSITION_VALUE      1000                // and this value (note that the Mercury would not move when the position value is out of movable range. Check e-manual about the range of the Mercury you use.)
#define MCY_MOVING_STATUS_THRESHOLD     0                  // Mercury moving status threshold

#define ESC_ASCII_VALUE                 0x1b

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

  int index = 0;
  std::vector<uint8_t> mcy_servos = {MCY_ID};
  int mcy_comm_result = COMM_TX_FAIL;             // Communication result
  int mcy_goal_position[2] = {MCY_MINIMUM_POSITION_VALUE, MCY_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t mcy_error = 0;                          // Mercury error
  uint32_t mcy_present_position = 0;              // Present position

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
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  do_synchronisation(&mcy_servos, packetHandler, portHandler, &mcy_error);

  // Enable Mercury Torque
  mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_MCY_TORQUE_ENABLE, TORQUE_ENABLE, &mcy_error);
  if (mcy_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
  }
  else if (mcy_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(mcy_error));
  }
  else
  {
    printf("Mercury has been successfully connected \n");
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write goal position
    mcy_comm_result = packetHandler->write4ByteTxRx(portHandler, MCY_ID, ADDR_MCY_GOAL_POSITION, mcy_goal_position[index], &mcy_error);
    if (mcy_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
    }
    else if (mcy_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(mcy_error));
    }

    do
    {
  #if defined(__linux__)
        usleep(10000);
  #endif

      // Read present position
      mcy_comm_result = packetHandler->read4ByteTxRx(portHandler, MCY_ID, ADDR_MCY_PRESENT_POSITION, &mcy_present_position, &mcy_error);
      if (mcy_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
      }
      else if (mcy_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(mcy_error));
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", MCY_ID, mcy_goal_position[index], mcy_present_position);

    }while((labs(mcy_goal_position[index] - mcy_present_position) > MCY_MOVING_STATUS_THRESHOLD));

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

  // Disable Mercury Torque
  mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_MCY_TORQUE_ENABLE, TORQUE_DISABLE, &mcy_error);
  if (mcy_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
  }
  else if (mcy_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(mcy_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}