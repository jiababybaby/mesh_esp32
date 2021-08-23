/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _GATTS_H
#define _GATTS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void BLE_INIT(void);
void BLE_DEINIT(void);
/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    IDX_CHAR_D,
    IDX_CHAR_VAL_D,
    IDX_CHAR_CFG_D,

    HRS_IDX_NB,
};
#endif