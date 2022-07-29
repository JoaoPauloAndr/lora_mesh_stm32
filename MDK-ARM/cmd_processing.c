#include "Cmd_Processing.h"

void setDefaultDataFormat(char* cmd2proccess, size_t cmdLen, char* msg, int* destiny)
{
	strncpy(msg, cmd2proccess, cmdLen);
	*destiny = 0;
}

int getIndex(char* str, char key)
{
    char *e;
    e = strchr(str, key);
    return (int)(e - str);
}

enum input_type checkCmdType(char* command)
{
	int send = strncmp("SEND", command, 4);
	if(send == 0)
	{
		return Send_Cmd;
	}	
	else
	{
		return Input_Error;
	}	
}	

void proccess_cmd(char* cmd2proccess, size_t cmdLen, char* msg, int* destiny)
{
	char at_cmd[3];
	strncpy(at_cmd, cmd2proccess, 3);
	int check = strncmp("AT+", at_cmd, 3);
	
	if(cmdLen < 4 || check != 0)
	{
		setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
		return;
	}
	
	int equalsIndex = getIndex(cmd2proccess, '=');
	if(equalsIndex > cmdLen && equalsIndex < 3)
	{
		setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
		return;
	}
	
	int command_len = equalsIndex-3; 
  char command[equalsIndex-3];
  strncpy(command, cmd2proccess+3, command_len);
	command[command_len] = '\0';
	enum input_type num_code = Input_Error;
	num_code = checkCmdType(command);
	if(num_code == Input_Error)
	{
		setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
		return;
	}	
	
	strncat(msg, "funciona", 8);
	*destiny = 1;
}	