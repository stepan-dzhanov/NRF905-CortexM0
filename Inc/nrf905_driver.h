
#ifndef _NRF905_DRIVER_H_
#define _NRF905_DRIVER_H_









void Nrf905Init(void);
void PowerDownMode(void);
void PowerUpMode(void);
void TransmitMode(void);
void ReceiveMode(void);
void TransmitPacket(unsigned short dByte);
unsigned short ReceivePacket(void);
void Nrf905RegCom (unsigned short RegCom, unsigned short RegValue);

#endif