#include "Cmd_Processing.h"
#include<stdint.h>
//#include<stdio.h>

uint8_t isDigit(char ch)
{
    if (ch >= '0' && ch <= '9')
        return 1;

    return 0;
}

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

void proccess_cmd(char* cmd2proccess, size_t cmdLen, char* msg, int* destiny, int *ack)
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
	if(equalsIndex > cmdLen && equalsIndex < 3 && (cmdLen - equalsIndex) > 3)
	{
		setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
		return;
	}
	
	uint16_t command_len = equalsIndex-3; 
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
	
	//CSEND
	if(num_code != CSend_Cmd)
	{
		*ack = 0;
	}	
	
	//Aqui sera tratado o comando SEND
	//AT+SEND=1,ola
	//copiar conteudo apos '='
	uint16_t contentLen = cmdLen - equalsIndex - 1;
	char content[contentLen];
	strncat(content, cmd2proccess+equalsIndex+1, contentLen);
	//verificar virgula
	int commaIndex = getIndex(cmd2proccess, ',');
	if(commaIndex > cmdLen && commaIndex < equalsIndex+1)
	{
		setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
		return;
	}
	//verificar o destino
	*destiny = 0;
	uint8_t digits = 0;
	for(uint16_t i = equalsIndex+1; i < commaIndex; i++)
	{
		char c = cmd2proccess[i];
		if(!isDigit(c))
		{
			setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
			return;
		}	
		*destiny = *destiny * 10 + (c - '0');
		digits++;
	}
	strncpy(msg, cmd2proccess+commaIndex+1, cmdLen-digits+1);
}	