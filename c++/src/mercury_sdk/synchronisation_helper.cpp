#include <mercury_sdk.h>
#include <synchronisation_helper.h>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <algorithm>

#define ADDR_MCY_TORQUE_ENABLE 0x30
#define ACKNOWLEDGE_RESPONSE_DELAY_MS 1000

bool do_synchronisation(std::vector<mcy_servo> *mcy_servos, mercury::PacketHandler *packetHandler, mercury::PortHandler *portHandler, uint8_t *mcy_comm_result)
{
  /*
   * Check for any servos that have not been synced.
  */
  std::vector<uint8_t> unsynchronised_servos = {};
  std::for_each(mcy_servos->begin(), mcy_servos->end(), [&](mcy_servo servo){

    if (!is_synchronised(servo.id, packetHandler, portHandler, mcy_comm_result))
    {
      unsynchronised_servos.push_back(servo.id);
    }
    else if (*mcy_comm_result != COMM_SUCCESS)
    {
      printf("Mercury#%d: %s\n", servo.id, packetHandler->getTxRxResult(*mcy_comm_result));
    }
  });

  if (unsynchronised_servos.size() == 0)
  {
    printf ("**** All Mercury servos are synchronised ****\n");
    return true;
  }
  else if (*mcy_comm_result != COMM_SUCCESS)
  {
    return false;
  }
  
  /*
   * Start the synchronisation of the servos that haven't been synced.
  */
  if (std::all_of(unsynchronised_servos.begin(), unsynchronised_servos.end(), [=](uint8_t id) {
        return start_synchronisation(id, packetHandler, portHandler, mcy_comm_result);
    }) == false)
  {
    return false; // error occured.
  }

 /*
  * Now monitor the servos being synced...
 */
  while (!std::all_of(unsynchronised_servos.begin(), unsynchronised_servos.end(), [&](uint8_t id) {
      if (!is_synchronised(id, packetHandler, portHandler, mcy_comm_result))
      {
        return false;
      }

      auto it = std::find(unsynchronised_servos.begin(), unsynchronised_servos.end(), id); 
      if (it != unsynchronised_servos.end()) 
      {
          unsynchronised_servos.erase(it); 
      }

      return true;
      
    }))
  {
    if (*mcy_comm_result != COMM_SUCCESS)
      return false;

    printf ("Synchronising...\n");
    usleep(1e06);
  }

  return true;
}

bool start_synchronisation(uint8_t id, mercury::PacketHandler *packetHandler, mercury::PortHandler *portHandler, uint8_t *mcy_comm_result)
{
  bool success = false;

  usleep(ACKNOWLEDGE_RESPONSE_DELAY_MS);
  uint8_t mcy_error = 0;
  *mcy_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_MCY_TORQUE_ENABLE, 0x02, &mcy_error);
  if (*mcy_comm_result != COMM_SUCCESS)
  {
    printf("Mercury#%d: %s\n", id, packetHandler->getTxRxResult(*mcy_comm_result));
  }
  else if (mcy_error != 0)
  {
    printf("Mercury#%d synchronise start failed: %s\n", id, packetHandler->getRxPacketError(mcy_error));
  }
  else
  {
    printf("Starting sychronisation of Mercury#%d\n", id);
    success = true;
  }

  return success;
}

bool is_synchronised(uint8_t id, mercury::PacketHandler *packetHandler, mercury::PortHandler *portHandler, uint8_t *mcy_comm_result)
{
  bool success = false;

  usleep(ACKNOWLEDGE_RESPONSE_DELAY_MS);
  uint8_t mcy_error = 0;
  uint8_t data;
  *mcy_comm_result = packetHandler->read1ByteTxRx(portHandler, id, 0x6b, &data, &mcy_error);
  if (*mcy_comm_result != COMM_SUCCESS)
  {
    printf("Mercury#%d: %s\n", id, packetHandler->getTxRxResult(*mcy_comm_result));
  }
  else if (mcy_error != 0)
  {
    printf("Mercury#%d torque enabled failed: %s\n", id, packetHandler->getRxPacketError(mcy_error));
  }
  else
  {
    success = (data & 0x02) == 0;
  }

  return success;
}