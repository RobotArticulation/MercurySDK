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
// *********     Action Example      *********
//
// This example is tested with a Mercury M65, and a USB2Mercury
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

#include "mercury_sdk.h"                                  // Uses Mercury SDK library

#define ADDR_MX_TORQUE_ENABLE           0x30                // Mercury Control table register addresses

// Default setting
#define MCY_ID                          1                   // Mercury ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) 
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__)
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

  int mcy_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t mcy_error = 0;                          // Mercury error

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

  // Enable Mercury Torque
  mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &mcy_error);
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

  mcy_comm_result = packetHandler->action(portHandler, MCY_ID);
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
