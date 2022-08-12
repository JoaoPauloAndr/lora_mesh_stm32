#ifndef AT_CMD_PROCESSING_H
#define AT_CMD_PROCESSING_H

#include <string.h>
#include <stdlib.h>

enum input_type
{
    Send_Cmd,
		CSend_Cmd,
    Input_Error 
};


void proccess_cmd(char* cmd2proccess, size_t cmdLen, char* msg, int* destiny, int *ack);

#endif