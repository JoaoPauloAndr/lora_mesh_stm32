/*!
 * \file      subghz_phy_app.c
 *
 * \brief     Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
/**
  ******************************************************************************
  *
  *          Portions COPYRIGHT 2020 STMicroelectronics
  *
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "app_version.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include<string.h>
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "Cmd_Processing.h"
#include "Queue.h"
#include "stm32_systime.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
typedef enum
{
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE           (0xFFFFFF) /* continuous Rx */
#define TX_TIMEOUT_VALUE           400
/* PING string*/
#define PING "PING"
/* PONG string*/
#define PONG "PONG"
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE     255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN                200
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH        83333
/* LED blink Period*/
#define LED_PERIOD_MS                 200
#define DEVICE_ID													 0
#define NET_ID																 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim2;

/*Ping Pong FSM states */
static States_t State = RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/* Led Timers objects*/
static UTIL_TIMER_Object_t timerLed;
/* device state. Master: true, Slave: false*/
bool isMaster = true;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
   take for the devices to sync when started simultaneously*/
static int32_t random_delay;

//Timers
static UTIL_TIMER_Object_t timerRxPolling;
static UTIL_TIMER_Object_t timerTxPolling;
SysTime_t currentTime;
SysTime_t timeDiff;
SysTime_t lastMsgTime;

//Buffers para Rx
uint8_t aRxBuffer[RXBUFFERSIZE] = {0};
volatile uint16_t uartRxIdx = 0;
bool receivedRx = false;
Queue queue;
uint8_t AddressTable[N_MAX_NODES];
uint8_t DeviceID = DEVICE_ID;

/*****TESTE*****/
//RECEPTOR
//uint8_t msgs_received = 0;
//TRANSMISSOR
//uint8_t msgs_sent = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  Function executed on when led timer elapses
  * @param  context ptr of LED context
  */
static void OnledEvent(void *context);

/**
  * @brief PingPong state machine implementation
  */
static void PingPong_Process(void);

/**
  * @brief state machine implementation
  */
static void StateMachineProccess(void);

/**
  * @brief Polling para verificar o tempo decorrido desde o inicio da mensagem recebida por UART
  */
static void OnUART_RxPollingTimer(void *context);

/**
  * @brief Polling para verificar se há mensagens a serem enviadas
  */
static void TxPollingTimer(void *context);

/**
  * @brief Processar mensagens recebidas por UART
  */
static void ProccessCmd(void);

//funcao para Buzzer
void Buzz_Master(void);
void Buzz_Slave(void);
void BuzzDeInit(void);

/* USER CODE END PFP */
void CMD_GetChar(uint8_t *rxChar, uint16_t size, uint8_t error)
{
	if(RXBUFFERSIZE - strlen((char*)aRxBuffer) >= strlen((char*)rxChar))
	{
		aRxBuffer[uartRxIdx++] = *rxChar;
	}
	receivedRx = true;
	lastMsgTime = SysTimeGet();
}

static void OnUART_RxPollingTimer(void *context)
{
	if(receivedRx)
	{
		currentTime = SysTimeGet();
		timeDiff = SysTimeSub(currentTime, lastMsgTime);
		if (timeDiff.SubSeconds > 5 && !QueueFull(&queue)) 
		{
			uartRxIdx = 0;
			receivedRx = false;
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Process_Messages), CFG_SEQ_Prio_0);
		}
	}	
}

static void ProccessCmd(void)
{
	int destiny = DeviceID ;
	int msglen;
	size_t rxBufferLen = strlen((char*)aRxBuffer);
	char *msg = (char*) calloc(rxBufferLen + 1, sizeof(char));
	input_type in = Input_Error;
	proccess_cmd(&in, (char*)aRxBuffer, rxBufferLen, msg, &destiny);
	uint8_t ack = 0;
	
	//Processar diferentes tipos de comando com switch
	switch(in)
	{
		case CSend_Cmd:
			ack = 1;
		case Send_Cmd:
			//Montar pacote no formato do protocolo e enfileirar
			msglen = strlen(msg);
			Payload payload = {.fields =  119, 33, msglen,		//header1, header2, size 
																		ack, DeviceID, destiny,  //control, origin, destiny
																		0																			//netId,
																	};
			//APP_LOG(TS_ON, VLEVEL_L, "Sending to destiny = %d\n\r", destiny);
			memcpy(payload.fields.payload, (uint8_t*)msg, msglen);
			Enqueue(&queue, payload);
			break;
		case SetID_Cmd:
			DeviceID = destiny;
			APP_LOG(TS_OFF, VLEVEL_L, "\n\rDEVICE_ID is now = %d\n\r", DeviceID);
			break;
		//fazer este caso
		case SetRoute_Cmd:
		default:
			APP_LOG(TS_ON, VLEVEL_L, "Error in input.\n\r");
			break;
	};	
	free(msg);
	memset(aRxBuffer, 0, rxBufferLen);
}	

static void TxPollingTimer(void *context)
{
	if(!QueueEmpty(&queue))
	{
		Payload transmission;
		Dequeue(&queue, &transmission);
		memcpy(BufferTx, transmission.bytes, sizeof(transmission.bytes));
		APP_LOG(TS_OFF, VLEVEL_L, "\n\r");
		APP_LOG(TS_ON, VLEVEL_L, "Transmitting message\n\r");
		Radio.Send(BufferTx, PAYLOAD_LEN);
	}		
}	

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
	
	UTIL_ADV_TRACE_StartRxProcess(CMD_GetChar);
	
  APP_LOG(TS_OFF, VLEVEL_M, "\n\rLORA MESH\n\r");
	APP_LOG(TS_OFF, VLEVEL_M, "DEVICE ID=%d\n\r", DeviceID);
	
	/* UART RX Timer*/
  UTIL_TIMER_Create(&timerRxPolling, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, OnUART_RxPollingTimer, NULL);
  UTIL_TIMER_SetPeriod(&timerRxPolling, 5);
  UTIL_TIMER_Start(&timerRxPolling);
	
	/* TX Timer*/
  UTIL_TIMER_Create(&timerTxPolling, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, TxPollingTimer, NULL);
  UTIL_TIMER_SetPeriod(&timerTxPolling, 20);
  UTIL_TIMER_Start(&timerTxPolling);

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  /* Radio Set frequency */
  Radio.SetChannel(RF_FREQUENCY);

  /* Radio configuration */
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, false, TX_TIMEOUT_VALUE); //LORA_IQ_INVERSION_ON

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, false, true); //LORA_IQ_INVERSION_ON

  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);

  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                    0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                    0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                    0, 0, false, true);

  Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);

#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
  /* LED initialization*/
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  /*calculate random delay for synchronization*/
  random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/
  /*fills tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  //APP_LOG(TS_ON, VLEVEL_L, "rand=%d\n\r", random_delay);
  /*starts reception*/
  Radio.Rx(3000 + random_delay); //3000
  /*register task to to be run in while(1) after Radio IT*/
  //UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_StateMachineProccess), UTIL_SEQ_RFU, StateMachineProccess);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Process_Messages), UTIL_SEQ_RFU, ProccessCmd);
	
	/*TESTE DE ROTEAMENTO*/
	//DEVICE ID = 0
	for(int i = 0; i < N_MAX_NODES; i++)
	{
		AddressTable[i] = 0xFF;
	}
	AddressTable[0] = 0;
	AddressTable[1] = 1;
	AddressTable[2] = 1;
	
	APP_LOG(TS_OFF, VLEVEL_M, "ROUTING TABLE = [");
	for(uint8_t i = 0; i < N_MAX_NODES-1; i++)
	{
		APP_LOG(TS_OFF, VLEVEL_M, "%d,", AddressTable[i]);
	}
	APP_LOG(TS_OFF, VLEVEL_M, "%d]\n\r", AddressTable[N_MAX_NODES-1]);
  /* USER CODE END SubghzApp_Init_2 */
}
extern void MX_TIM2_Init(void);
/* USER CODE BEGIN EF */
void Buzz_Master(void)
{
	MX_TIM2_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(10);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(10);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	BuzzDeInit();
}

void Buzz_Slave(void)
{
	MX_TIM2_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(20);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	BuzzDeInit();
}	

void BuzzDeInit(void)
{
	HAL_TIM_Base_DeInit(&htim2);	
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
}	
/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/

static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  /* Update the State of the FSM*/
  State = RX; //TX
	 /* Clear BufferTx*/
  memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_StateMachineProccess), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
  /* Record payload Signal to noise ratio in Lora*/
  SnrValue = LoraSnr_FskCfo;
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
#if ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, Cfo=%dkHz\n\r", rssi, LoraSnr_FskCfo);
  SnrValue = 0; /*not applicable in GFSK*/
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
  /* Update the State of the FSM*/
  State = RX;
  /* Clear BufferRx*/
  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    memcpy(BufferRx, payload, RxBufferSize);
  }
  /* Record Received Signal Strength*/
  RssiValue = rssi;
  /* Record payload content*/
  //APP_LOG(TS_ON, VLEVEL_L, "payload. size=%d \n\r", size); //VLEVEL_H
	//APP_LOG(TS_OFF, VLEVEL_L, "%s\n", (char*)BufferRx);
  //APP_LOG(TS_OFF, VLEVEL_L, "\n\r");
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_StateMachineProccess), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");
	/* Clear BufferTx*/
  memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);
  /* Update the State of the FSM*/
  State = RX;//TX_TIMEOUT;
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_StateMachineProccess), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout\n\r");
  /* Update the State of the FSM*/
  //State = RX_TIMEOUT;
	State = RX;
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_StateMachineProccess), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  //APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");
  /* Update the State of the FSM*/
  //State = RX_ERROR;
  State = RX;
	/* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_StateMachineProccess), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxError */
}
	
/* USER CODE BEGIN PrFD */

static void StateMachineProccess(void)
{
	Radio.Sleep();
	
	switch(State)
	{
		case RX:
			//			/*****TESTE - RECEPTOR*****/
			//				if(RxBufferSize > 0)
			//				{
			//						//converter para protocolo e verificar validade da mensagem
			//						//Buzz_Master();
			//						RxBufferSize = 0;
			//						Payload received;
			//						received.fields.header1 = BufferRx[0];
			//						received.fields.header2 = BufferRx[1];
			//						uint8_t origin = BufferRx[4];
			//						uint8_t destiny = BufferRx[5];
			//						uint8_t header1 = 119;
			//						uint8_t header2 = 33;
			//						if(received.fields.header1 == header1 && received.fields.header2 == header2)
			//						{
			//							if(destiny == DEVICE_ID)
			//							{
			//								//MENSAGEM RECEBIDA COM SUCESSO
			//								//char msg[245];
			//								APP_LOG(TS_ON, VLEVEL_L, "%d message(s) received\n\r", ++msgs_received);
			//								uint8_t msglen = BufferRx[2] + 1;
			//								char *msg = (char*)calloc(msglen, sizeof(char));
			//								memcpy(msg, (char*)BufferRx+8, msglen);
			//								APP_LOG(TS_ON, VLEVEL_L, "Device %d sent me: %s\n\r", origin,msg);
			//								free(msg);
			//							}
			//						}
			//				}	
			if(RxBufferSize > 0)
			{
				//Recebeu algo
				/*TODO:
				* Atestar validez da mensagem recebida (examinar headers)
				* Veriricar destinatário final (nível 0): 
				* Caso seja este nó, imprimir e verificar ack
				* Se for para outro nó, verificar destinatário nível 1:
				* Se for este nó, repassar.
				* Caso contrário, descartar a mensagem
				*/
					RxBufferSize = 0;
					Payload received;
					received.fields.header1 = BufferRx[0];
					received.fields.header2 = BufferRx[1];
					uint8_t origin0 = BufferRx[4];
					uint8_t destiny0 = BufferRx[5];
					uint8_t origin1 = BufferRx[6];
					uint8_t destiny1 = BufferRx[7];
					uint8_t header1 = 119;
					uint8_t header2 = 33;
					if(received.fields.header1 == header1 && received.fields.header2 == header2)
					{
						if(destiny0 == DeviceID)
						{
							uint8_t msglen = BufferRx[2] + 1;
							char *msg = (char*)calloc(msglen, sizeof(char));
							memcpy(msg, (char*)BufferRx+10, msglen);
							APP_LOG(TS_ON, VLEVEL_L, "Device %d sent me: %s\n\r", origin0,msg);
							free(msg);
							uint8_t ack = BufferRx[3];
							if(ack & 1)
							{
								Payload response = {
									.fields = 119, 33, 2,
									0, DeviceID, origin0,
									DeviceID, AddressTable[origin0], NET_ID
								};
								memcpy(response.fields.payload, "ok", 2);
								Enqueue(&queue, response);
								//memset(BufferTx, 0, PAYLOAD_LEN);
								//memcpy(BufferTx, response.bytes, PAYLOAD_LEN);
								//Radio.Send(BufferTx, PAYLOAD_LEN);
							}	
						}
						else if(destiny1 == DeviceID)
						{
							//Repassar mensagem
							memset(BufferTx, 0, PAYLOAD_LEN);
							BufferRx[6] = DeviceID; //origin1
							BufferRx[7] = AddressTable[destiny0]; //destiny1
							memcpy(BufferTx, BufferRx, PAYLOAD_LEN);
							APP_LOG(TS_ON, VLEVEL_L, "Retransmitting message passed by Device %d to Device %d\n\r", origin1,destiny0);
							Radio.Send(BufferTx, PAYLOAD_LEN);
						}		
					}	
//						if(destiny == DEVICE_ID)
//						{
//							if(ack & 1)
//							{
//								//HAL_Delay(1000);
//								Payload response = {
//									.fields = 119, 33, 2,
//									0, DEVICE_ID, origin, 0
//							};
//							memcpy(response.fields.payload, "ok", 2);
//							response.fields.crc = 16;
//							memset(BufferTx, 0, PAYLOAD_LEN);
//							memcpy(BufferTx, response.bytes, PAYLOAD_LEN);
//							Radio.Send(BufferTx, PAYLOAD_LEN);
//							}	
//						}
//						else
//						{
//							APP_LOG(TS_ON, VLEVEL_L, "Device %d is reaching for device %d\n\r", origin, destiny);
//						}	
//					APP_LOG(TS_ON, VLEVEL_L, "Listening...\n\r");
//					//Buzz_Slave();
//					Radio.Rx(RX_TIMEOUT_VALUE);
//			}
//			else //Continuar escutando
//			{
//				APP_LOG(TS_ON, VLEVEL_L, "Listening...\n\r");
//				//Buzz_Slave();
//				Radio.Rx(RX_TIMEOUT_VALUE);
//			}
			}		
			APP_LOG(TS_ON, VLEVEL_L, "Listening...\n\r");
			Radio.Rx(RX_TIMEOUT_VALUE);
			break;
		case TX:
			APP_LOG(TS_ON, VLEVEL_L, "\n\rPayload transmitted.\n\rBack to Listening...\n\r");
      Radio.Rx(RX_TIMEOUT_VALUE);
			break;
		case RX_TIMEOUT:
		case RX_ERROR:
			APP_LOG(TS_ON, VLEVEL_L, "Rx Timeout/Error...\n\r");
			//Buzz_Slave();
			//break;
		case TX_TIMEOUT:
			//APP_LOG(TS_ON, VLEVEL_L, "Transmission failed.\n\rTrying again.\n\r");
			//Buzz_Slave();
			State = RX;
			Radio.Rx(RX_TIMEOUT_VALUE);
			break;
		default:
			break;
	}

	/*****TESTE - TRANSMISSOR*****/
//	if(msgs_sent < 100)
//	{
//		Payload response = {
//		.fields = 119, 33, 32,
//		0, DEVICE_ID, 10, 0				//destiny = 10
//		};
//		memcpy(response.fields.payload, "teste teste teste teste teste ok", 32);
//		response.fields.crc = 16;
//		memset(BufferTx, 0, PAYLOAD_LEN);
//		memcpy(BufferTx, response.bytes, PAYLOAD_LEN);
//		Radio.Send(BufferTx, PAYLOAD_LEN);
//		msgs_sent++;
//		APP_LOG(TS_ON, VLEVEL_L, "%d message(s) sent\n\r", msgs_sent);
//		HAL_Delay(2000);
//	}
//	else
//	{
//		APP_LOG(TS_ON, VLEVEL_L, "TEST DONE!\n\r");
//		HAL_Delay(10000);
//	}	
}	

//static void PingPong_Process(void)
//{
//  Radio.Sleep();

//  switch (State)
//  {
//    case RX:

//      if (isMaster == true)
//      {
//        if (RxBufferSize > 0)
//        {
//          if (strncmp((const char *)BufferRx, PONG, sizeof(PONG) - 1) == 0)
//          {
//						Buzz_Master();
//            /* Add delay between RX and TX */
//            HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
//            /* master sends PING*/
//            APP_LOG(TS_ON, VLEVEL_L, "..."
//                    "PING"
//                    "\n\r");
//            APP_LOG(TS_ON, VLEVEL_L, "Master Tx start\n\r");
//						if(n_messages)
//						{
//							memcpy(BufferTx, (uint8_t*)messages[n_messages-1].msg, strlen(messages[n_messages-1].msg) +1);
//							APP_LOG(TS_ON, VLEVEL_L, "Transmitting: %s\n\r", (char*)BufferTx);
//							//free(messages[n_messages].msg);
//							n_messages--;
//						}
//						else
//						{
//							memcpy(BufferTx, PING, sizeof(PING) - 1);
//						}		
//            Radio.Send(BufferTx, PAYLOAD_LEN);
//          }
//          else if (strncmp((const char *)BufferRx, PING, sizeof(PING) - 1) == 0)
//          {
//            /* A master already exists then become a slave */
//            isMaster = false;
//            APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
//            Radio.Rx(RX_TIMEOUT_VALUE);
//          }
//          else /* valid reception but neither a PING or a PONG message */
//          {
//						//receber echo
//						APP_LOG(TS_ON, VLEVEL_L, "Recebido=%s \n\r", (char *)BufferRx);
//            /* Set device as master and start again */
//            isMaster = true;
//            APP_LOG(TS_ON, VLEVEL_L, "Master Rx start\n\r");
//            Radio.Rx(RX_TIMEOUT_VALUE);
//          }
//        }
//      }
//      else
//      {
//				//Slave
//        if (RxBufferSize > 0)
//        {
//          if (strncmp((const char *)BufferRx, PING, sizeof(PING) - 1) == 0)
//          {
//            //UTIL_TIMER_Stop(&timerLed);
//            /* switch off red led */
//            //BSP_LED_Off(LED_RED);
//            /* slave toggles green led */
//            //BSP_LED_Toggle(LED_GREEN);
//						Buzz_Slave();
//            /* Add delay between RX and TX */
//            HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
//            /*slave sends PONG*/
//            APP_LOG(TS_ON, VLEVEL_L, "..."
//                    "PONG"
//                    "\n\r");
//            APP_LOG(TS_ON, VLEVEL_L, "Slave  Tx start\n\r");
//            memcpy(BufferTx, PONG, sizeof(PONG) - 1);
//            Radio.Send(BufferTx, PAYLOAD_LEN);
//          }
//          else /* valid reception but not a PING as expected */
//          {
//						//Reenviar pacote
//						
//						//Buzz_Slave();
//            /* Add delay between RX and TX */
//            HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
//            /*slave sends PONG*/
//           // APP_LOG(TS_ON, VLEVEL_L, "..."
//           //         "PONG"
//           //         "\n\r");
//            APP_LOG(TS_ON, VLEVEL_L, "Slave  Tx start\n\r");
//            memcpy(BufferTx, BufferRx, sizeof(BufferRx) - 1);
//            Radio.Send(BufferTx, PAYLOAD_LEN);
//          }
//        }
//      }
//      break;
//    case TX:
//      APP_LOG(TS_ON, VLEVEL_L, "Rx start\n\r");
//      Radio.Rx(RX_TIMEOUT_VALUE);
//      break;
//    case RX_TIMEOUT:
//    case RX_ERROR:
//      if (isMaster == true)
//      {
//        /* Send the next PING frame */
//        /* Add delay between RX and TX*/
//        /* add random_delay to force sync between boards after some trials*/
//				Buzz_Master();
//        HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN + random_delay);
//        APP_LOG(TS_ON, VLEVEL_L, "Master Tx start\n\r");
//        /* master sends PING*/
//        if(n_messages)
//				{
//					memcpy(BufferTx, (uint8_t*)messages[n_messages-1].msg, strlen(messages[n_messages-1].msg) +1);
//					//strncpy(BufferTx, (uint8_t*)messages[n_messages-1].msg, strlen(messages[n_messages-1].msg) +1);
//					//strncpy(BufferTx, (uint8_t*)"Hello", 5);
//					APP_LOG(TS_ON, VLEVEL_L, "Transmitting: %s\n\r", messages[n_messages-1].msg); //(char*)BufferTx
//					//free(messages[n_messages].msg);
//					n_messages--;
//				}
//				else
//				{
//					memcpy(BufferTx, PING, sizeof(PING) - 1);
//				}
//        Radio.Send(BufferTx, PAYLOAD_LEN);
//      }
//      else
//      {
//				Buzz_Slave();
//        APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
//        Radio.Rx(RX_TIMEOUT_VALUE);
//      }
//      break;
//    case TX_TIMEOUT:
//			Buzz_Slave();
//      APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
//      Radio.Rx(RX_TIMEOUT_VALUE);
//      break;
//    default:
//      break;
//  }	
//}

static void OnledEvent(void *context)
{
  BSP_LED_Toggle(LED_GREEN);
  BSP_LED_Toggle(LED_RED);
  UTIL_TIMER_Start(&timerLed);
}

/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
