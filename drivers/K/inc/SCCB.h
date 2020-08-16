/*
  ******************************************************************************
  * @file    SCCB.h
  * @author  董城彬移植山外
  * @version V2.5
  * @date    2017.1.16
  * @brief   河北工程大学创新实验室
  * @note    模拟SCCB库文件
  ******************************************************************************
*/
#ifndef __CH_LIB_SCCB_H_
#define __CH_LIB_SCCB_H_

#ifdef __cplusplus
	extern "C" {
#endif
#include <stdint.h>	 


#define ADDR_OV7725   0x42
#define ADDR_OV7620   0x42

#define DEV_ADR  ADDR_OV7725             /*设备地址定义*/

#define SCCB_DELAY()    SCCB_delay(400)


extern void SCCB_GPIO_init(void);
extern int SCCB_WriteByte( uint16_t WriteAddress , uint8_t SendByte);
extern int SCCB_ReadByte(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress);


#ifdef __cplusplus
}
#endif



#endif
