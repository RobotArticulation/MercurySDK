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
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "../../include/mercury_sdk/mercury_sdk.h"                                 // Uses Mercury SDK library

// Control table address
#define ADDR_MCY_TORQUE_ENABLE           0x30                // Mercury Control table register addresses
#define ADDR_MCY_GOAL_POSITION           0x4e
#define ADDR_MCY_PRESENT_POSITION        0x56

// Data Byte Length
#define LEN_MCY_GOAL_POSITION           4
#define LEN_MCY_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Mercury

// Default setting
#define MCY1_ID                         1                   // Mercury#1 ID: 1
#define MCY2_ID                         2                   // Mercury#2 ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define MCY_MINIMUM_POSITION_VALUE      -2000             // Mercury will rotate between this value
#define MCY_MAXIMUM_POSITION_VALUE      2000              // and this value (note that the Mercury would not move when the position value is out of movable range. Check e-manual about the range of the Mercury you use.)
#define MCY_MOVING_STATUS_THRESHOLD     20                  // Mercury moving status threshold

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

  // Initialize GroupSyncWrite instance
  mercury::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MCY_GOAL_POSITION, LEN_MCY_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  mercury::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);

  int index = 0;
  int mcy_comm_result = COMM_TX_FAIL;               // Communication result
  bool mcy_addparam_result = false;                 // addParam result
  bool mcy_getdata_result = false;                  // GetParam result
  int mcy_goal_position[2] = {MCY_MINIMUM_POSITION_VALUE, MCY_MAXIMUM_POSITION_VALUE};  // Goal position

  uint8_t mcy_error = 0;                            // Mercury error
  uint8_t param_goal_position[4];
  int32_t mcy1_present_position = 0, mcy2_present_position = 0;                         // Present position

  bool mcy2_present = false;

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

  // Enable Mercury#1 Torque
  mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY1_ID, ADDR_MCY_TORQUE_ENABLE, TORQUE_ENABLE, &mcy_error);
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
    printf("Mercury#%d has been successfully connected \n", MCY1_ID);
  }

  if (mcy2_present) {
    // Enable Mercury#2 Torque
    mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY2_ID, ADDR_MCY_TORQUE_ENABLE, TORQUE_ENABLE, &mcy_error);
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
      printf("Mercury#%d has been successfully connected \n", MCY2_ID);
    }
  }

  // Add parameter storage for Mercury#1 present position value
  mcy_addparam_result = groupSyncRead.addParam(MCY1_ID);
  if (mcy_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", MCY1_ID);
    return 0;
  }

  if (mcy2_present) {
    // Add parameter storage for Mercury#2 present position value
    mcy_addparam_result = groupSyncRead.addParam(MCY2_ID);
    if (mcy_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", MCY2_ID);
      return 0;
    }
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Allocate goal position value into byte array
    param_goal_position[0] = MCY_LOBYTE(MCY_LOWORD(mcy_goal_position[index]));
    param_goal_position[1] = MCY_HIBYTE(MCY_LOWORD(mcy_goal_position[index]));
    param_goal_position[2] = MCY_LOBYTE(MCY_HIWORD(mcy_goal_position[index]));
    param_goal_position[3] = MCY_HIBYTE(MCY_HIWORD(mcy_goal_position[index]));

    // Add Mercury#1 goal position value to the Syncwrite storage
    mcy_addparam_result = groupSyncWrite.addParam(MCY1_ID, param_goal_position);
    if (mcy_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", MCY1_ID);
      return 0;
    }

    if (mcy2_present) {
      // Add Mercury#2 goal position value to the Syncwrite parameter storage
      mcy_addparam_result = groupSyncWrite.addParam(MCY2_ID, param_goal_position);
      if (mcy_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", MCY2_ID);
        return 0;
      }
    }

    // Syncwrite goal position
    mcy_comm_result = groupSyncWrite.txPacket();
    if (mcy_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
      // Syncread present position
      mcy_comm_result = groupSyncRead.txRxPacket();
      if (mcy_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
      }
      else if (groupSyncRead.getError(MCY1_ID, &mcy_error))
      {
        printf("[ID:%03d] %s\n", MCY1_ID, packetHandler->getRxPacketError(mcy_error));
      }
      else if (mcy2_present && groupSyncRead.getError(MCY2_ID, &mcy_error))
      {
        printf("[ID:%03d] %s\n", MCY2_ID, packetHandler->getRxPacketError(mcy_error));
      }

      // Check if groupsyncread data of Mercury#1 is available
      mcy_getdata_result = groupSyncRead.isAvailable(MCY1_ID, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);
      if (mcy_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", MCY1_ID);
        return 0;
      }

      if (mcy2_present) {
        // Check if groupsyncread data of Mercury#2 is available
        mcy_getdata_result = groupSyncRead.isAvailable(MCY2_ID, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);
        if (mcy_getdata_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", MCY2_ID);
          return 0;
        }
      }

      // Get Mercury#1 present position value
      mcy1_present_position = groupSyncRead.getData(MCY1_ID, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);

      if (mcy2_present) {
      // Get Mercury#2 present position value
        mcy2_present_position = groupSyncRead.getData(MCY2_ID, ADDR_MCY_PRESENT_POSITION, LEN_MCY_PRESENT_POSITION);
        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", MCY1_ID, mcy_goal_position[index], mcy1_present_position, MCY2_ID, mcy_goal_position[index], mcy2_present_position);
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d]\n", MCY1_ID, mcy_goal_position[index], mcy1_present_position);

    }while((abs(mcy_goal_position[index] - mcy1_present_position) > MCY_MOVING_STATUS_THRESHOLD) || (mcy2_present && (abs(mcy_goal_position[index] - mcy2_present_position) > MCY_MOVING_STATUS_THRESHOLD)));

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

  // Disable Mercury#1 Torque
  mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY1_ID, ADDR_MCY_TORQUE_ENABLE, TORQUE_DISABLE, &mcy_error);
  if (mcy_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
  }
  else if (mcy_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(mcy_error));
  }

  if (mcy2_present) {
    // Disable Mercury#2 Torque
    mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY2_ID, ADDR_MCY_TORQUE_ENABLE, TORQUE_DISABLE, &mcy_error);
    if (mcy_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
    }
    else if (mcy_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(mcy_error));
    }
  }

  // Close port
  portHandler->closePort();

  return 0;
}
