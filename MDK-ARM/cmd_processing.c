#include "Cmd_Processing.h"
#include<stdint.h>
#include<stdio.h>

uint8_t isDigit(char ch)
{
    if (ch >= '0' && ch <= '9')
        return 1;

    return 0;
}

void setDefaultDataFormat(char* cmd2proccess, size_t cmdLen, char* msg, int* destiny)
{
	strncpy(msg, cmd2proccess, cmdLen);
}

int getIndex(char* str, char key)
{
    char *e;
    e = strchr(str, key);
    return (int)(e - str);
}

input_type checkCmdType(char* command)
{
	int send = strncmp("SEND", command, 4);
	int csend = strncmp("CSEND", command, 5);
	int setID = strncmp("SETID", command, 5);
	int setRT = strncmp("SETRT", command, 5);
	if(send == 0)
	{
		return Send_Cmd;
	}	
	else if(csend == 0)
	{
		return CSend_Cmd;
	}
  else if(setID == 0)
	{
		return SetID_Cmd;
	}
	else if(setRT == 0)
	{
		return SetRoute_Cmd;
	}
	else
	{
		return Input_Error;
	}	
}

void proccess_send(char* cmd2proccess, size_t cmdLen, int equalsIndex, char* msg, int* destiny)
{
	//Aqui sera tratado o comando SEND
	//AT+SEND=1,ola
	//copiar conteudo apos '='
	uint16_t contentLen = cmdLen - equalsIndex - 1;
	char content[contentLen];
	strncpy(content, cmd2proccess+equalsIndex+1, contentLen);
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

void proccess_setID(char* cmd2proccess, size_t cmdLen, int equalsIndex, char* msg, int* destiny)
{
	//copiar conteudo apos '='
	uint16_t contentLen = cmdLen - equalsIndex - 1;
	char content[contentLen];
	strncpy(content, cmd2proccess+equalsIndex+1, contentLen);
	int aux = 0;
	for(uint16_t i = 0; i < contentLen; i++)
	{
		char c = content[i];
		if(!isDigit(c))
		{
			setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
			return;
		}	
		aux = aux * 10 + (c - '0');
	}
	if(aux < 256)
	{
		*destiny = aux;
	}	
}

//void proccess_setRT(char* cmd2proccess, size_t cmdLen, int equalsIndex, char* msg, int* destiny)
//{

//}	

void proccess_cmd(input_type* in, char* cmd2proccess, size_t cmdLen, char* msg, int* destiny)
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
	*in = checkCmdType(command);
	switch(*in)
	{
		case CSend_Cmd:
		case Send_Cmd:
			proccess_send(cmd2proccess, cmdLen, equalsIndex, msg, destiny);
			break;
		case SetID_Cmd:
			proccess_setID(cmd2proccess, cmdLen, equalsIndex, msg, destiny);
			break;
		case SetRoute_Cmd:
			//set routes
			break;
		default:
			setDefaultDataFormat(cmd2proccess, cmdLen, msg, destiny);
			break;
	}
}	