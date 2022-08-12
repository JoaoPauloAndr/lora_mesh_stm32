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
#define TX_TIMEOUT_VALUE           3000
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
//volatile uint8_t n_messages = 0;
volatile uint16_t uartRxIdx = 0;
bool receivedRx = false;
Msg2Send messages[10];
Queue queue;

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
		if (timeDiff.SubSeconds > 20 && !QueueFull(&queue)) 
		{
			uartRxIdx = 0;
			receivedRx = false;
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Process_Messages), CFG_SEQ_Prio_0);
		}
//		else
//		{
//			APP_LOG(TS_OFF, VLEVEL_M, "Message queue has reached its limit (%d)\r\n", N_RXMESSAGES);
//		}	
	}	
}

static void ProccessCmd(void)
{
	int destiny, ack = 1;
	size_t rxBufferLen = strlen((char*)aRxBuffer);
	char *msg = (char*) calloc(rxBufferLen + 1, sizeof(char));
	proccess_cmd((char*)aRxBuffer, rxBufferLen, msg, &destiny, &ack);
	
	//Montar pacote no formato do protocolo e enfileirar
	int msglen = strlen(msg);
	Payload payload = {.fields =  119, 33, msglen,		//header1, header2, size 
																ack, DEVICE_ID, destiny,  //control, origin, destiny
																0																			//netId,
															};
	memcpy(payload.fields.payload, (uint8_t*)msg, msglen);
	payload.fields.crc = 16;
	Enqueue(&queue, payload);
	free(msg);
	memset(aRxBuffer, 0, rxBufferLen);
	//printf("msg: %s\n\r", msg);
//	messages[n_messages].destiny = destiny;
//	messages[n_messages].msg = (char*) malloc((msglen + 1)*sizeof(char));
//	strncpy(messages[n_messages].msg, msg, msglen);
//	messages[n_messages].len = msglen;
	//printf("\n\rmsg: %s \n\r", messages[n_messages].msg);
	//n_messages++;
}	

static void TxPollingTimer(void *context)
{
	if(!QueueEmpty(&queue))
	{
		Payload transmission;
		Dequeue(&queue, &transmission);
		memcpy(BufferTx, transmission.bytes, sizeof(transmission.bytes));
		//APP_LOG(TS_ON, VLEVEL_L, "\n\rSize of BufferTx = %d\n\r", sizeof(BufferTx));															
		//memcpy(BufferTx, (uint8_t*)messages[n_messages-1].msg, strlen(messages[n_messages-1].msg) +1);
		APP_LOG(TS_ON, VLEVEL_L, "Transmitting message\n\r");
		Radio.Send(BufferTx, PAYLOAD_LEN);
	}		
}	

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
	
	UTIL_ADV_TRACE_StartRxProcess(CMD_GetChar);
	
  APP_LOG(TS_OFF, VLEVEL_M, "\n\rPING PONG\n\r");
  /* Print APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION= V%X.%X.%X\r\n",
          (uint8_t)(__APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB2_SHIFT));
  /* Led Timers*/
  UTIL_TIMER_Create(&timerLed, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnledEvent, NULL);
  UTIL_TIMER_SetPeriod(&timerLed, LED_PERIOD_MS);
  UTIL_TIMER_Start(&timerLed);
	
	/* UART RX Timer*/
  UTIL_TIMER_Create(&timerRxPolling, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, OnUART_RxPollingTimer, NULL);
  UTIL_TIMER_SetPeriod(&timerRxPolling, 500);
  UTIL_TIMER_Start(&timerRxPolling);
	
	/* TX Timer*/
  UTIL_TIMER_Create(&timerTxPolling, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, TxPollingTimer, NULL);
  UTIL_TIMER_SetPeriod(&timerTxPolling, 1000);
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
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

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

  APP_LOG(TS_ON, VLEVEL_L, "rand=%d\n\r", random_delay);
  /*starts reception*/
  Radio.Rx(3000 + random_delay);
  /*register task to to be run in while(1) after Radio IT*/
  //UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_StateMachineProccess), UTIL_SEQ_RFU, StateMachineProccess);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Process_Messages), UTIL_SEQ_RFU, ProccessCmd);
	
  /* USER CODE END SubghzApp_Init_2 */
}
extern void MX_TIM2_Init(void);
/* USER CODE BEGIN EF */
void Buzz_Master(void)
{
//	MX_TIM2_Init();
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//	HAL_Delay(100);
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//	BuzzDeInit();
	
	MX_TIM2_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	BuzzDeInit();
}

void Buzz_Slave(void)
{
MX_TIM2_Init();
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//	HAL_Delay(30);
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//	HAL_Delay(30);
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//	HAL_Delay(30);
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//	BuzzDeInit();
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(200);
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
	//Deletar mensagem
//	free(messages[n_messages-1].msg);
//	n_messages--;
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  /* Update the State of the FSM*/
  State = TX;
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
  APP_LOG(TS_ON, VLEVEL_L, "payload. size=%d \n\r", size); //VLEVEL_H
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
  /* Update the State of the FSM*/
  State = TX_TIMEOUT;
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
			//Tentar mudar para mandar a mensagem no final processamento dela (ProccerssCmd)
//			if(n_messages)
//			{
//				int msglen = messages[n_messages-1].len;
//				Payload payload = {.fields =  119, 33, msglen,																			//header1, header2, size 
//																		 1, DEVICE_ID, messages[n_messages-1].destiny,  //control, origin, destiny
//																			0																							//netId,
//																		};
//				memcpy(payload.fields.payload, (uint8_t*)messages[n_messages-1].msg, msglen+1);
//				payload.fields.crc = 16;
//				memcpy(BufferTx, payload.bytes, sizeof(payload.bytes));
//				APP_LOG(TS_ON, VLEVEL_L, "\n\rSize of BufferTx = %d\n\r", sizeof(BufferTx));															
//				//memcpy(BufferTx, (uint8_t*)messages[n_messages-1].msg, strlen(messages[n_messages-1].msg) +1);
//				APP_LOG(TS_ON, VLEVEL_L, "Transmitting message\n\r");
//				Radio.Send(BufferTx, PAYLOAD_LEN);
//			}
			if(RxBufferSize > 0)
			{
					//converter para protocolo e verificar validade da mensagem
					Buzz_Master();
					RxBufferSize = 0;
					Payload received;
					received.fields.header1 = BufferRx[0];
					received.fields.header2 = BufferRx[1];
					printf("header1 = %d\n\r", received.fields.header1);
					printf("header2 = %d\n\r", received.fields.header2);
					if(received.fields.header1 == (uint8_t)119 && received.fields.header2 == (uint8_t)33)
					{
						char msg[245];
						memcpy(msg, (char*)BufferRx+8, 245);
						APP_LOG(TS_ON, VLEVEL_L, "Received: %s\n\r", msg);
						//Testar Ack
						if(received.fields.control & 1)
						{
							HAL_Delay(1000);
							Payload response = {.fields = 119, 33, 2,
																							0, DEVICE_ID, received.fields.origin,
																							0
																						};
							memcpy(response.fields.payload, "ok", 2);
							response.fields.crc = 16;
							memcpy(BufferTx, response.bytes, sizeof(response.bytes));
							Radio.Send(BufferTx, PAYLOAD_LEN);															
						}	
					}
					
					//APP_LOG(TS_ON, VLEVEL_L, "Received message with error in header\n\r");					
					/* Add delay between RX and TX */
					HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
					Radio.Rx(RX_TIMEOUT_VALUE); //RX_TIMEOUT_VALUE
			}
			else //Continuar escutando
			{
				APP_LOG(TS_ON, VLEVEL_L, "Listening...\n\r");
				Buzz_Slave();
				Radio.Rx(RX_TIMEOUT_VALUE);
			}
			break;
		case TX:
			APP_LOG(TS_ON, VLEVEL_L, "\n\rPayload transmitted.\n\rBack to Listening...\n\r");
      Radio.Rx(RX_TIMEOUT_VALUE);
			break;
		case RX_TIMEOUT:
		case RX_ERROR:
			APP_LOG(TS_ON, VLEVEL_L, "Rx Timeout/Error...\n\r");
			//Buzz_Slave();
			State = RX;
			Radio.Rx(RX_TIMEOUT_VALUE);
			break;
		case TX_TIMEOUT:
			APP_LOG(TS_ON, VLEVEL_L, "Transmission failed.\n\rTrying again.\n\r");
			//Buzz_Slave();
			Radio.Rx(RX_TIMEOUT_VALUE);
			break;
		default:
			break;
	}	
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
