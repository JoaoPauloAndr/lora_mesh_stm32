#ifndef AT_CMD_PROCESSING_H
#define AT_CMD_PROCESSING_H

#include <string.h>
#include <stdlib.h>

typedef enum
{
    Send_Cmd,				//AT+SEND=DEVICE_ID,MSG
		CSend_Cmd, 		//AT+CSEND=DEVICE_ID,MSG
    SetID_Cmd,			//AT+SETID=DEVICE_ID
    SetRoute_Cmd,	//AT+SETRT=DESTINY_ID,RELAY_ID
    Input_Error 
}input_type;


void proccess_cmd(input_type* in, char* cmd2proccess, size_t cmdLen, char* msg, int* destiny);

#endif