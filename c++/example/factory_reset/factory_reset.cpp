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
// *********     Factory Reset Example      *********
//
// This example was tested with a Mercury M65, and a USB2Mercury
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <stdio.h>

#include "mercury_sdk.h"                                  // Uses Mercury SDK library

// Control table address
#define ADDR_PRO_BAUDRATE               4                   // Control table address is different in Mercury model

// Default setting
#define MCY_ID                          1                   // Mercury ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define FACTORYRST_DEFAULTBAUDRATE      1000000             // Mercury baudrate set by factoryreset
#define NEW_BAUDNUM                     3                   // New baudnum to recover Mercury baudrate as it was
#define OPERATION_MODE                  0x01                // 0xFF : reset all values
                                                            // 0x01 : reset all values except ID
                                                            // 0x02 : reset all values except ID and baudrate

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
  uint8_t mcy_baudnum_read;                       // Read baudnum

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

  // Read present baudrate of the controller
  printf("Now the controller baudrate is : %d\n", portHandler->getBaudRate());

  // Try factoryreset
  printf("[ID:%03d] Try factoryreset : ", MCY_ID);
  mcy_comm_result = packetHandler->factoryReset(portHandler, MCY_ID, OPERATION_MODE, &mcy_error);
  if (mcy_comm_result != COMM_SUCCESS)
  {
    printf("Aborted\n");
    printf("%s\n", packetHandler->getTxRxResult(mcy_comm_result));
    return 0;
  }
  else if (mcy_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(mcy_error));
  }
  
  // Wait for reset
  printf("Wait for reset...\n");
  msecSleep(2000);

  printf("[ID:%03d] factoryReset Success!\n", MCY_ID);

  // Set controller baudrate to Mercury default baudrate
  if (portHandler->setBaudRate(FACTORYRST_DEFAULTBAUDRATE))
  {
    printf("Succeed to change the controller baudrate to : %d\n", FACTORYRST_DEFAULTBAUDRATE);
  }
  else
  {
    printf("Failed to change the controller baudrate\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Read Mercury baudnum
  mcy_comm_result = packetHandler->read1ByteTxRx(portHandler, MCY_ID, ADDR_PRO_BAUDRATE, &mcy_baudnum_read, &mcy_error);
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
    printf("[ID:%03d] DXL baudnum is now : %d\n", MCY_ID, mcy_baudnum_read);
  }

  // Write new baudnum
  mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, MCY_ID, ADDR_PRO_BAUDRATE, NEW_BAUDNUM, &mcy_error);
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
    printf("[ID:%03d] Set Mercury baudnum to : %d\n", MCY_ID, NEW_BAUDNUM);
  }

  // Set port baudrate to BAUDRATE
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeed to change the controller baudrate to : %d\n", BAUDRATE);
  }
  else
  {
    printf("Failed to change the controller baudrate\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  msecSleep(200);

  // Read Mercury baudnum
  mcy_comm_result = packetHandler->read1ByteTxRx(portHandler, MCY_ID, ADDR_PRO_BAUDRATE, &mcy_baudnum_read, &mcy_error);
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
    printf("[ID:%03d] Mercury Baudnum is now : %d\n", MCY_ID, mcy_baudnum_read);
  }

  // Close port
  portHandler->closePort();

  return 0;
}
