/*
  ******************************************************************************
  * @file    SCCB.c
  * @author  ���Ǳ���ֲɽ��
  * @version V2.5
  * @date    2017.1.16
  * @brief   �ӱ����̴�ѧ����ʵ����
  * @note    ģ��SCCB���ļ�
  ******************************************************************************
*/
#include "common.h"
#include "gpio.h"
#include "SCCB.h"


static GPIO_Type * const GPIO_InstanceTable[] = GPIO_BASES;




#define SCL_DDR_OUT()   do {GPIO_PinConfig(HW_GPIOB, 2, kOutput);}while(0)
#define SCL_DDR_IN()    do {GPIO_PinConfig(HW_GPIOB, 2, kInput);}while(0)
#define SCL_H()         do {GPIO_WriteBit(HW_GPIOB, 2, 1);}while(0)
#define SCL_L()         do {GPIO_WriteBit(HW_GPIOB, 2, 0);}while(0)

#define SDA_DDR_OUT()   do {GPIO_PinConfig(HW_GPIOB, 3, kOutput);}while(0)
#define SDA_DDR_IN()    do {GPIO_PinConfig(HW_GPIOB, 3, kInput);}while(0)
//#define SDA_IN()         GPIO_ReadBit(HW_GPIOB,3)
#define SDA_H()         do {GPIO_WriteBit(HW_GPIOB, 3, 1);}while(0)
#define SDA_L()         do {GPIO_WriteBit(HW_GPIOB, 3, 0);}while(0)

static uint8_t SDA_IN()
{
    return GPIO_ReadBit(HW_GPIOB, 3);
}
static void SCCB_delay(uint16_t i);

/*!
 *  @brief      SCCB�ӳٺ���
 *  @param      time    ��ʱʱ��
 *  @since      v5.0
 */
static void SCCB_delay(volatile uint16_t time)
{
    while(time)
    {
        time--;
    }
}

/*!
 *  @brief      SCCB�ܽ�����
 *  @since      v5.0
 */
 void SCCB_GPIO_init(void)
{
    GPIO_QuickInit(HW_GPIOB, 2, kGPIO_Mode_OPP); //SCL
    GPIO_QuickInit(HW_GPIOB, 3, kGPIO_Mode_OPP);//SDA
    GPIO_WriteBit(HW_GPIOB , 2, 1);
    GPIO_WriteBit(HW_GPIOB, 3, 1);
    PORT_PinPullConfig(HW_GPIOB,2, kPullUp);
    PORT_PinPullConfig(HW_GPIOB,3, kPullUp);
}

/*!
 *  @brief      SCCB��ʼ�ź�
 *  @since      v5.0
 */
static uint8_t SCCB_Start(void)
{
    SDA_H();
    SCL_H();
    SCCB_DELAY();

    SDA_DDR_IN();
    if(!SDA_IN())
    {
        SDA_DDR_OUT();
        return 0;   /* SDA��Ϊ�͵�ƽ������æ,�˳� */
    }
    SDA_DDR_OUT();
    SDA_L();

    SCCB_DELAY();
    SCL_L();

    if(SDA_IN())
    {
        SDA_DDR_OUT();
        return 0;   /* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
    }
    //SDA_DDR_OUT();
    //SDA_L();
    //SCCB_delay();
    return 1;
}

/*!
 *  @brief      SCCBֹͣ�ź�
 *  @since      v5.0
 */
static void SCCB_Stop(void)
{
    SCL_L();
    //SCCB_DELAY();
    SDA_L();
    SCCB_DELAY();
    SCL_H();
    SCCB_DELAY();
    SDA_H();
    SCCB_DELAY();
}

/*!
 *  @brief      SCCBӦ���ź�
 *  @since      v5.0
 */
static void SCCB_Ack(void)
{
    SCL_L();
    SCCB_DELAY();
    SDA_L();
    SCCB_DELAY();
    SCL_H();
    SCCB_DELAY();
    SCL_L();
    SCCB_DELAY();
}

/*!
 *  @brief      SCCB��Ӧ���ź�
 *  @since      v5.0
 */
static void SCCB_NoAck(void)
{
    SCL_L();
    SCCB_DELAY();
    SDA_H();
    SCCB_DELAY();
    SCL_H();
    SCCB_DELAY();
    SCL_L();
    SCCB_DELAY();
}

/*!
 *  @brief      SCCB �ȴ�Ӧ��
 *  @return     Ӧ������0��ʾ��Ӧ��1��ʾ��Ӧ��
 *  @since      v5.0
 */
static int SCCB_WaitAck(void)
{
    SCL_L();
    //SDA_H();
    SDA_DDR_IN();

    SCCB_DELAY();
    SCL_H();

    SCCB_DELAY();

    if(SDA_IN())           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
    {
        SDA_DDR_OUT();
        SCL_L();
        return 0;
    }
    SDA_DDR_OUT();
    SCL_L();
    return 1;
}

/*!
 *  @brief      SCCB ���͵�����
 *  @param      SendByte    ��Ҫ���͵�����
 *  @since      v5.0
 */
static void SCCB_SendByte(uint8_t SendByte)
{
    uint8_t i = 8;
    while(i--)
    {
        if(SendByte & 0x80)     //SDA �������
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        SendByte <<= 1;
        SCCB_DELAY();
        SCL_H();                //SCL ���ߣ��ɼ��ź�
        SCCB_DELAY();
        SCL_L();                //SCL ʱ��������
        //SCCB_DELAY();
    }
    //SCL_L();
}

/*!
 *  @brief      ����SCCB���ߵ�����
 *  @return     ���յ�������
 *  @since      v5.0
 */
static int SCCB_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t ReceiveByte = 0;

    //SDA_H();
    //SCCB_DELAY();
    SDA_DDR_IN();

    while(i--)
    {
        ReceiveByte <<= 1;
        SCL_L();
        SCCB_DELAY();
        SCL_H();
        SCCB_DELAY();

        if(SDA_IN())
        {
            ReceiveByte |= 0x01;
        }
    }
    SDA_DDR_OUT();
    SCL_L();
    return ReceiveByte;
}

/*****************************************************************************************
* ��������SCCB_WriteByte
* ����  ��дһ�ֽ�����
* ����  ��- WriteAddress: ��д���ַ    - SendByte: ��д������  - DeviceAddress: ��������
* ���  ������Ϊ:=1�ɹ�д��,=0ʧ��
* ע��  ����
*****************************************************************************************/
static int SCCB_WriteByte_one( uint16_t WriteAddress , uint8_t SendByte );


int SCCB_WriteByte( uint16_t WriteAddress , uint8_t SendByte )            //���ǵ���sccb�Ĺܽ�ģ�⣬�Ƚ�����ʧ�ܣ���˶��Լ���
{
    uint8_t i = 0;
    while( 0 == SCCB_WriteByte_one ( WriteAddress, SendByte ) )
    {
        i++;
        if(i == 20)
        {
            return 0 ;
        }
    }
    return 1;
}

int SCCB_WriteByte_one( uint16_t WriteAddress , uint8_t SendByte )
{
    if(!SCCB_Start())
    {
        return 0;
    }
    SCCB_SendByte( DEV_ADR );                    /* ������ַ */
    if( !SCCB_WaitAck() )
    {
        SCCB_Stop();
        return 0;
    }
    SCCB_SendByte((uint8_t)(WriteAddress & 0x00FF));   /* ���õ���ʼ��ַ */
    SCCB_WaitAck();
    SCCB_SendByte(SendByte);
    SCCB_WaitAck();
    SCCB_Stop();
    return 1;
}




/******************************************************************************************************************
 * ��������SCCB_ReadByte
 * ����  ����ȡһ������
 * ����  ��- pBuffer: ��Ŷ�������  - length: ����������    - ReadAddress: ��������ַ        - DeviceAddress: ��������
 * ���  ������Ϊ:=1�ɹ�����,=0ʧ��
 * ע��  ����
 **********************************************************************************************************************/
static int SCCB_ReadByte_one(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress);

int SCCB_ReadByte(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress)
{
    uint8_t i = 0;
    while( 0 == SCCB_ReadByte_one(pBuffer, length, ReadAddress) )
    {
        i++;
        if(i == 30)
        {
            return 0 ;
        }
    }
    return 1;
}

int SCCB_ReadByte_one(uint8_t *pBuffer,   uint16_t length,   uint8_t ReadAddress)
{
    if(!SCCB_Start())
    {
        return 0;
    }
    SCCB_SendByte( DEV_ADR );         /* ������ַ */
    if( !SCCB_WaitAck() )
    {
        SCCB_Stop();
        return 0;
    }
    SCCB_SendByte( ReadAddress );           /* ���õ���ʼ��ַ */
    SCCB_WaitAck();
    SCCB_Stop();

    if(!SCCB_Start())
    {
        return 0;
    }
    SCCB_SendByte( DEV_ADR + 1 );               /* ������ַ */

    if(!SCCB_WaitAck())
    {
        SCCB_Stop();
        return 0;
    }
    while(length)
    {
        *pBuffer = SCCB_ReceiveByte();
        if(length == 1)
        {
            SCCB_NoAck();
        }
        else
        {
            SCCB_Ack();
        }
        pBuffer++;
        length--;
    }
    SCCB_Stop();
    return 1;
}
