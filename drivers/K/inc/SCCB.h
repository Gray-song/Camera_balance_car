/*
  ******************************************************************************
  * @file    SCCB.h
  * @author  ���Ǳ���ֲɽ��
  * @version V2.5
  * @date    2017.1.16
  * @brief   �ӱ����̴�ѧ����ʵ����
  * @note    ģ��SCCB���ļ�
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

#define DEV_ADR  ADDR_OV7725             /*�豸��ַ����*/

#define SCCB_DELAY()    SCCB_delay(400)


extern void SCCB_GPIO_init(void);
extern int SCCB_WriteByte( uint16_t WriteAddress , uint8_t SendByte);
extern int SCCB_ReadByte(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress);


#ifdef __cplusplus
}
#endif



#endif
