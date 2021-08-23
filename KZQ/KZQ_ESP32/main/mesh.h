#ifndef _MESH_H
#define _MESH_H

#include "mdf_common.h"
#include "mwifi.h"
void wifi_mesh_init();
extern uint8_t mode;
extern uint8_t mesh_channel;
extern uint8_t mesh_id[10];
extern char ssid[100];
extern char pwd[100];
enum{
    jdq,
    hw,
};
#define device_kind jdq

#endif