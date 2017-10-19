/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Protocol Combined Example      *********
//
//
// Available Mercury model on this example : All models using Protocol 1.0 and 2.0
// This example is tested with a Mercury MX-28, a Mercury PRO 54-200 and an USB2MERCURY
// Be sure that properties of Mercury MX and PRO are already set as %% MX - ID : 1 / Baudnum : 34 (Baudrate : 57600) , PRO - ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

// Be aware that:
// This example configures two different control tables (especially, if it uses Mercury and Mercury PRO). It may modify critical Mercury parameter on the control table, if Mercurys have wrong ID.
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

#include "mercury_sdk.h"                                  // Uses Mercury SDK library

// Control table address for Mercury MX
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Mercury model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Control table address for Mercury PRO
#define ADDR_PRO_TORQUE_ENABLE          562
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Mercury
#define PROTOCOL_VERSION2               2.0

// Default setting
#define DXL1_ID                         1                   // Mercury#1 ID: 1
#define DXL2_ID                         2                   // Mercury#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL1_MINIMUM_POSITION_VALUE     100                 // Mercury will rotate between this value
#define DXL1_MAXIMUM_POSITION_VALUE     4000                // and this value (note that the Mercury would not move when the position value is out of movable range. Check e-manual about the range of the Mercury you use.)
#define DXL2_MINIMUM_POSITION_VALUE     -150000
#define DXL2_MAXIMUM_POSITION_VALUE     150000
#define DXL1_MOVING_STATUS_THRESHOLD    10                  // Mercury MX moving status threshold
#define DXL2_MOVING_STATUS_THRESHOLD    20                  // Mercury PRO moving status threshold

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
  mercury::PacketHandler *packetHandler1 = mercury::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
  mercury::PacketHandler *packetHandler2 = mercury::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;       // Communication result
  int dxl1_goal_position[2] = {DXL1_MINIMUM_POSITION_VALUE, DXL1_MAXIMUM_POSITION_VALUE};     // Goal position of Mercury MX
  int dxl2_goal_position[2] = {DXL2_MINIMUM_POSITION_VALUE, DXL2_MAXIMUM_POSITION_VALUE};     // Goal position of Mercury PRO

  uint8_t dxl_error = 0;                    // Mercury error
  uint16_t dxl1_present_position = 0;       // Present position of Mercury MX
  int32_t dxl2_present_position = 0;        // Present position of Mercury PRO

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

  // Enable Mercury#1 torque
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler1->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler1->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Mercury#%d has been successfully connected \n", DXL1_ID);
  }
  // Enable Mercury#2 torque
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler2->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler2->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Mercury#%d has been successfully connected \n", DXL2_ID);
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write Mercury#1 goal position
    dxl_comm_result = packetHandler1->write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_position[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler1->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler1->getRxPacketError(dxl_error));
    }

    // Write Mercury#2 goal position
    dxl_comm_result = packetHandler2->write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler2->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler2->getRxPacketError(dxl_error));
    }

    do
    {
      // Read Mercury#1 present position
      dxl_comm_result = packetHandler1->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl1_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler1->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler1->getRxPacketError(dxl_error));
      }

      // Read Mercury#2 present position
      dxl_comm_result = packetHandler2->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler2->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler2->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d [ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl1_goal_position[index], dxl1_present_position, DXL2_ID, dxl2_goal_position[index], dxl2_present_position);

    }while((abs(dxl1_goal_position[index] - dxl1_present_position) > DXL1_MOVING_STATUS_THRESHOLD) || (abs(dxl2_goal_position[index] - dxl2_present_position) > DXL2_MOVING_STATUS_THRESHOLD));

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
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler1->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler1->getRxPacketError(dxl_error));
  }

  // Disable Mercury#2 Torque
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler2->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler2->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
