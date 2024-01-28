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

//
// *********     Clear Multiturn Example      *********
//
// This example is tested with a Mercury M1, and a USB2Mercury
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

#include "mercury_sdk.h"                                   // Uses Mercury SDK library

// Control table address
#define ADDR_OPERATING_MODE             0x06                  // Control table address is different in Mercury model
#define ADDR_TORQUE_ENABLE              0x30
#define ADDR_GOAL_POSITION              0x4e
#define ADDR_PRESENT_POSITION           0x5a

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Mercury

// Default setting
#define MCY_ID                          1                   // Mercury ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM1"      // Check which port is being used on your controller
                                                            // eg: Windows: "COM1"   Linux: "/dev/ttyACM0" 

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define MAX_POSITION_VALUE              -1
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Mercury moving status threshold
#define EXT_POSITION_CONTROL_MODE       3                   // Value for extended position control mode (operating mode)

#define ESC_ASCII_VALUE                 0x1b
#define SPACE_ASCII_VALUE               0x20

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

void msecSleep(int waitTime)
{
#if defined(__linux__) || defined(__APPLE__)
  usleep(waitTime * 1000);
#elif defined(_WIN32) || defined(_WIN64)
  _sleep(waitTime);
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

  int mcy_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t mcy_error = 0;                          // Mercury error
  int32_t mcy_present_position = 0;               // Present position

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

  uint8_t operating_mode = 0;
  mcy_comm_result = packetHandler->readTxRx(portHandler, MCY_ID, ADDR_OPERATING_MODE, 0x01, &operating_mode, &mcy_error);
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
    if (operating_mode != EXT_POSITION_CONTROL_MODE)
    {
      mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_TORQUE_ENABLE, 0, &mcy_error);
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
        printf("Control_enable has been disabled \n");
      }


      // Set operating mode to extended position control mode
      mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &mcy_error);
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
        // This is a rom write, so add a delay of 1s
        usleep(1e06);
        printf("Operating mode changed to extended position control mode. \n");
      }
    }
  }
  
  // Enable Mercury Torque
   mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &mcy_error);
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
    printf("  Press SPACE key to clear multi-turn information! (or press ESC to quit!)\n");

    // Write goal position
    mcy_comm_result = packetHandler->write4ByteTxRx(portHandler, MCY_ID, ADDR_GOAL_POSITION, MAX_POSITION_VALUE, &mcy_error);
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
      // Read present position
      mcy_comm_result = packetHandler->read4ByteTxRx(portHandler, MCY_ID, ADDR_PRESENT_POSITION, (uint32_t*)&mcy_present_position, &mcy_error);
      if (mcy_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
      }
      else if (mcy_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(mcy_error));
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d \n", MCY_ID, MAX_POSITION_VALUE, mcy_present_position);

      usleep(10000);

    }while((abs(MAX_POSITION_VALUE - mcy_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // we've reached the goal position

    char c = getch();
    if (c == SPACE_ASCII_VALUE)
    {
        printf("\n  Clearing Multi-Turn Information... \n");

        //printf("[ID:%03d] GoalPos:%03d New PresPos:%03d \n", MCY_ID, MAX_POSITION_VALUE, (mcy_present_position - 8192) % 16384);

        // Clear Multi-Turn Information
        mcy_comm_result = packetHandler->clearMultiTurn(portHandler, MCY_ID, &mcy_error);
        if (mcy_comm_result != COMM_SUCCESS)
        {
          printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
        }
        else if (mcy_error != 0)
        {
          printf("%s\n", packetHandler->getRxPacketError(mcy_error));
        }

        printf("[ID:%03d] GoalPos:%03d New PresPos:%03d \n", MCY_ID, MAX_POSITION_VALUE, (mcy_present_position - 8192) % 16384);

        // Write the present position to the goal position. This will move servo to same position after x number of revolutions
        mcy_comm_result = packetHandler->write4ByteTxRx(portHandler, MCY_ID, ADDR_GOAL_POSITION, mcy_present_position, &mcy_error);
        if (mcy_comm_result != COMM_SUCCESS)
        {
          printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
        }
        else if (mcy_error != 0)
        {
          printf("%s\n", packetHandler->getRxPacketError(mcy_error));
        }

        msecSleep(300);
    }
    else if (c == ESC_ASCII_VALUE)
      break;

    do
    {
      // Read present position
      mcy_comm_result = packetHandler->read4ByteTxRx(portHandler, MCY_ID, ADDR_PRESENT_POSITION, (uint32_t*)&mcy_present_position, &mcy_error);
      if (mcy_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
      }
      else if (mcy_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(mcy_error));
      }
      usleep(1000);

    }while((abs(MAX_POSITION_VALUE - mcy_present_position) > DXL_MOVING_STATUS_THRESHOLD));
 }

  // Disable Mercury Torque
  mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &mcy_error);
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
