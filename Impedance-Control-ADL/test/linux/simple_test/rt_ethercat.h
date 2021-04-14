/** @file rt_ethercat.h
 *  @brief Interface to EtherCAT hardware
 */
#ifndef _rt_ethercat
#define _rt_ethercat

#include <stdint.h>
#include "rt_tiboard_data.h"



void rt_ethercat_init(char *ifname);

void rt_ethercat_get_data_3DOF(TiBoardData* data);

void rt_ethercat_set_command_3DOF(TiBoardCommand* command);

void rt_ethercat_run();

void rt_ethercat_stop();


#endif
