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

////////////////////////////////////////////////////////////////////////////////
/// @file The file for Dynamixel Sync Write
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef MERCURY_SDK_INCLUDE_MERCURY_SDK_GROUPSYNCWRITE_H_
#define MERCURY_SDK_INCLUDE_MERCURY_SDK_GROUPSYNCWRITE_H_


#include "port_handler.h"
#include "packet_handler.h"
#include "group_handler.h"

namespace mercury
{

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for writing multiple Dynamixel data from same address with same length at once
////////////////////////////////////////////////////////////////////////////////
class WINDECLSPEC GroupSyncWrite : public GroupHandler
{
private:
    uint16_t start_address_;
    uint16_t data_length_;

    void makeParam();

public:
  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that Initializes instance for Sync Write
  /// @param port PortHandler instance
  /// @param ph PacketHandler instance
  /// @param start_address Address of the data for write
  /// @param data_length Length of the data for write
  ////////////////////////////////////////////////////////////////////////////////
  GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that calls clearParam function to clear the parameter list for Sync Write
  ////////////////////////////////////////////////////////////////////////////////
  ~GroupSyncWrite() { clearParam(); }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that adds id, start_address, data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data Data for write
  /// @return false
  /// @return   when the ID exists already in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    addParam    (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that removes id from the Sync Write list
  /// @param id Dynamixel ID
  ////////////////////////////////////////////////////////////////////////////////
  void    removeParam (uint8_t id);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that changes the data for write in id -> start_address -> data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data for replacement
  /// @return false
  /// @return   when the ID doesn't exist in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    changeParam (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that clears the Sync Write list
  ////////////////////////////////////////////////////////////////////////////////
  void    clearParam();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that transmits the Sync Write instruction packet which might be constructed by GroupSyncWrite::addParam function
  /// @return COMM_NOT_AVAILABLE
  /// @return   when the list for Sync Write is empty
  /// @return or the other communication results which come from PacketHandler::syncWriteTxOnly
  ////////////////////////////////////////////////////////////////////////////////
  int     txPacket();
};

}


#endif /* MERCURY_SDK_INCLUDE_MERCURY_SDK_GROUPSYNCWRITE_H_ */
