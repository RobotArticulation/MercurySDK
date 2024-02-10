#ifndef MERCURY_SDK_INCLUDE_MERCURY_SDK_SYNCHRONISATION_HELPER_H_
#define MERCURY_SDK_INCLUDE_MERCURY_SDK_SYNCHRONISATION_HELPER_H_

#include <vector>

bool do_synchronisation(std::vector<mcy_servo> *mcy_servos, mercury::PacketHandler *packetHandler, mercury::PortHandler *portHandler, uint8_t *mcy_comm_result);
bool start_synchronisation(uint8_t id, mercury::PacketHandler *packetHandler, mercury::PortHandler *portHandler, uint8_t *mcy_comm_result);
bool is_synchronised(uint8_t id, mercury::PacketHandler *packetHandler, mercury::PortHandler *portHandler, uint8_t *mcy_comm_result);

#endif /* MERCURY_SDK_INCLUDE_MERCURY_SDK_SYNCHRONISATION_HELPER_H_ */