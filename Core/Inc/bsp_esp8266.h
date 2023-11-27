#ifndef __ESP8266_H_
#define __ESP8266_H_

#include "stm32f1xx.h"

#define ESP8266_BUF_SIZE   64

#define ACK_SUCCESS   1
#define ACK_DEFEAT    0

typedef enum
{
  TCP,
  UDP
} Conn_Type;

void ESP8266_Init(void);
uint8_t   ESP8266_Send_Cmd(char *cmd, char *ack, uint16_t waittime);
uint8_t   ESP8266_Send_Cmd2(char *cmd, char *ack, char *ack2, uint16_t waittime);
void ESP8266_Send_Data(uint8_t *data, uint8_t length);

void ESP8266_Reset(void);
void ESP8266_Set_Echo_Off(void);
uint8_t   ESP8266_Set_Stationmode(void);
uint8_t   ESP8266_Set_APmode(char *ap_ssid, char *ap_pwd, char chl, char ecn);
uint8_t   ESP8266_Connect_AP(const char *ssid, const char *passwd);
uint8_t   ESP8266_Set_Link_Mux(uint8_t mode);
uint8_t   ESP8266_Connect_TCP(const char *addr, const char *port);
uint8_t   ESP8266_Connect_UDP(const char *addr, const char *port);
uint8_t   ESP8266_Quit_Passthrough(void);
uint8_t   ESP8266_Start_Passthrough(void);
uint8_t   ESP8266_Passthrough_Request(Conn_Type type, const char *addr, char *port, void (*function)());

void Passthrough_Echo_Test(char *request);

#endif
