
#ifndef _NRF905_DRIVER_H_
#define _NRF905_DRIVER_H_







void Delay_us(int delay);



void Nrf905Init(char ch);
void PowerDownMode(void);
void PowerUpMode(void);
void TransmitMode(void);
void ReceiveMode(void);
void TransmitPacket(unsigned short dByte);
unsigned short ReceivePacket(void);
void Nrf905RegCom (unsigned short RegCom, unsigned short RegValue);
char GetTxStatus();
void sTxStatus();
void rTxStatus();
void TransmitMultiPacket(unsigned char *dByte, char num);
char ReceiveMultiPacket(unsigned char *data, char num);

#endif