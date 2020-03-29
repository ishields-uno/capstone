/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    HRS_IDX_NB,
};

/*  
    NAME:               ble_init
    AURTHOR(S):         Isaac Shields
    CALLED BY:          app_main() in main.c
    PURPOSE:            Initializes ble stack.
    CALLING CONVENTION: ble_init();
    CONDITIONS AT EXIT: 
    DATE STARTED:       3/29/2020
    UPDATE HISTORY:     See Git logs.
    NOTES:              
*/
void ble_init();
