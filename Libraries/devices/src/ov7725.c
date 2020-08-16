/*----------------------------------------------------------------------------------------------------
																			逐飞小钻风摄像头


  * @file    SCCB.C
  * @author  
  * @version V1.0
  * @date    2017.1.16
  * @brief   河北工程大学创新实验室
	* @pin		 SCL:B02  
						 SDA:B03
						 PCLK: C07
						 VSYNC:C16
						 HREF:C17
		         D0-D7:C08-C15														
  * @note    逐飞小钻风库文件

----------------------------------------------------------------------------------------------------*/

//#include <stdio.h>
//#include <string.h>
//#include <ctype.h>
//#include <stdlib.h>
//#include <stdarg.h>
//#include "uCOS_II.H"
//#include "os_cpu.h"
//#include "os_cfg.h"
#include "gpio.h"
#include "common.h"
//#include "systick.h"
#include "uart.h"
#include "dma.h"
#include "ov7725.h"
#include "i2c.h"
//#include "chgui.h"
//#include  <OLED.h>
#include "SCCB.h"
//#include <string.h>




#define OV7725_DEBUG		1
#if ( OV7725_DEBUG == 1 )
#include <stdio.h>
#define OV7725_TRACE	printf
#else
#define OV7725_TRACE(...)
#endif

char i;
//!< register defination 
struct ov7725_reg
{
    uint8_t addr;
    uint8_t val;
};

struct 
{
    uint32_t i2c_instance;
    uint8_t  addr;
    uint32_t h_size;
    uint32_t v_size;
}h_ov7725;

//uint8_t gImageBuffer[OV7725_SIZE];   //
//uint8_t image_dec[OV7725_H][OV7725_W];  //图像解压后的数组
static uint8_t ov7725_addr[] = {0x21};
static const struct ov7725_reg reg_tbl[] =
{

    {OV7725_COM4         , 0X40},
    {OV7725_CLKRC        , 0X00},
    {OV7725_COM2         , 0x03},
    {OV7725_COM3         , 0xD0},
    {OV7725_COM7         , 0x40},
    {OV7725_HSTART       , 0x3F},
    {OV7725_HSIZE        , 0x50},
    {OV7725_VSTRT        , 0x03},
    {OV7725_VSIZE        , 0x78},
    {OV7725_HREF         , 0x00},
    {OV7725_SCAL0        , 0x0A},
    {OV7725_AWB_Ctrl0    , 0xE0},
    {OV7725_DSPAuto      , 0xff},
    {OV7725_DSP_Ctrl2    , 0x0C},
    {OV7725_DSP_Ctrl3    , 0x00},
    {OV7725_DSP_Ctrl4    , 0x00},
    {OV7725_EXHCH        , 0x00},
    {OV7725_GAM1         , 0x0c},
    {OV7725_GAM2         , 0x16},
    {OV7725_GAM3         , 0x2a},
    {OV7725_GAM4         , 0x4e},
    {OV7725_GAM5         , 0x61},
    {OV7725_GAM6         , 0x6f},
    {OV7725_GAM7         , 0x7b},
    {OV7725_GAM8         , 0x86},
    {OV7725_GAM9         , 0x8e},
    {OV7725_GAM10        , 0x97},
    {OV7725_GAM11        , 0xa4},
    {OV7725_GAM12        , 0xaf},
    {OV7725_GAM13        , 0xc5},
    {OV7725_GAM14        , 0xd7},
    {OV7725_GAM15        , 0xe8},
    {OV7725_SLOP         , 0x20},
    {OV7725_LC_RADI      , 0x00},
    {OV7725_LC_COEF      , 0x13},
    {OV7725_LC_XC        , 0x08},
    {OV7725_LC_COEFB     , 0x14},
    {OV7725_LC_COEFR     , 0x17},
    {OV7725_LC_CTR       , 0x05},
    {OV7725_BDBase       , 0x99},
    {OV7725_BDMStep      , 0x03},
    {OV7725_SDE          , 0x04},
    {OV7725_BRIGHT       , 0x00},
    {OV7725_CNST         , 70},
    {OV7725_SIGN         , 0x06},
    {OV7725_UVADJ0       , 0x11},
    {OV7725_UVADJ1       , 0x02},
};

int ov7725_probe(uint8_t i2c_instance)
{
    int i,j;
    int r;
    uint8_t dummy;
    for(i = 0; i < ARRAY_SIZE(ov7725_addr); i++)
    {
        if(!SCCB_ReadSingleRegister(i2c_instance, ov7725_addr[i], OV7725_VER, &dummy))
        {
            /* found device */
            OV7725_TRACE("device found addr:0x%X\r\n", ov7725_addr[i]);
            /* reset */
            SCCB_WriteSingleRegister(i2c_instance, ov7725_addr[i], OV7725_COM7, 0x80);
            /* inject default register value */
            for(j = 0; j < ARRAY_SIZE(reg_tbl); j++)
            {
                DelayMs(1);
                r = SCCB_WriteSingleRegister(i2c_instance, ov7725_addr[i], reg_tbl[j].addr, reg_tbl[j].val);
                if(r)
                {
                    OV7725_TRACE("device[addr:0x%X]regiser[addr:0x%X] write error!\r\n", ov7725_addr[i], reg_tbl[j].addr);
                }
            }
//            h_ov7725.addr = ov7725_addr[i];
//            h_ov7725.i2c_instance = i2c_instance;
//            h_ov7725.h_size = 80;
//            h_ov7725.v_size = 60;
            return 0;
        }
    }
    OV7725_TRACE("no sccb device found!\r\n");
    return 1;
}

int SCCB_Init(uint32_t I2C_MAP)
{
    int r;
    uint32_t instance;
    instance = I2C_QuickInit(I2C_MAP, 50*1000);
    r = ov7725_probe(instance);
    if(r)
    {
        return 1;
    }
//    r = (ov7725_set_image_size(IMAGE_SIZE));
//    if(r)
//    {
//        printf("OV7725 set im age error\r\n");
//        return 1;
//    }
    return 0;
}


//int ov7725_set_image_size(ov7725_size size)
//{
//    switch(size)
//    {
//        case H_80_W_60:
//            h_ov7725.h_size = 80;
//            h_ov7725.v_size = 60;
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x14);
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x1E);
//            break;
//        case H_120_W_160:
//            h_ov7725.h_size = 160;
//            h_ov7725.v_size = 120;
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x28);
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x3C);  
//            break;
//        case H_180_W_240:
//            h_ov7725.h_size = 240;
//            h_ov7725.v_size = 180;
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x3C);
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x5A);  
//            break;
//        case H_240_W_320:
//            h_ov7725.h_size = 320;
//            h_ov7725.v_size = 240;
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x50);
//            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x78);  
//            break;
//        default:
//            OV7725_TRACE("wrong param in func:ov7725_set_image_size\r\n");
//            break;
//    }
//    return 0;
//}

void Image_Decompression(uint8_t *data1,uint8_t *data2)
{
    uint8_t  temp[2] = {0,1};					//显示为解压的图像可将此处改为{0,1},0代表黑，1代表白
    uint16_t lenth = OV7725_SIZE;
    uint8_t  i = 8;
        
    while(lenth--)
    {
        i = 8;
        while(i--)
        {
            *data2++ = temp[(*data1 >> i) & 0x01];
        }
        data1++;
    }
}
void write_reg(void )
{
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_COM4         , 0xC1);  
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_CLKRC        , 0x01);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_COM2         , 0x03);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_COM3         , 0xD0);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_COM7         , 0x40);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_HSTART       , 0x3F);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_HSIZE        , 0x50);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_VSTRT        , 0x03);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_VSIZE        , 0x78);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_HREF         , 0x00);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_SCAL0        , 0x0A);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_AWB_Ctrl0    , 0xE0);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_DSPAuto      , 0xff);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_DSP_Ctrl2    , 0x0C);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_DSP_Ctrl3    , 0x00);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_DSP_Ctrl4    , 0x00);
      
        if(OV7725_W == 80)              SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_HOutSize    , 0x14);
        else if(OV7725_W == 160)        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_HOutSize    , 0x28);
        else if(OV7725_W == 240)        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_HOutSize    , 0x3c);
        else if(OV7725_W == 320)        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_HOutSize    , 0x50);
        
        if(OV7725_H == 60)              SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_VOutSize    , 0x1E);
        else if(OV7725_H == 120)        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_VOutSize    , 0x3c);
        else if(OV7725_H == 180)        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_VOutSize    , 0x5a);
        else if(OV7725_H == 240)        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,  OV7725_DEV_ADD, OV7725_VOutSize    , 0x78);
		
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_REG28        , 0x01);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_EXHCH        , 0x10);
				SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_EXHCL        , 0x1F);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM1         , 0x0c);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM2         , 0x16);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM3         , 0x2a);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM4         , 0x4e);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM5         , 0x61);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM6         , 0x6f);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM7         , 0x7b);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM8         , 0x86);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM9         , 0x8e);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM10        , 0x97);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM11        , 0xa4);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM12        , 0xaf);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM13        , 0xc5);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM14        , 0xd7);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_GAM15        , 0xe8);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_SLOP         , 0x20);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_LC_RADI      , 0x00);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_LC_COEF      , 0x13);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_LC_XC        , 0x08);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_LC_COEFB     , 0x14);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_LC_COEFR     , 0x17);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_LC_CTR       , 0x05);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_BDBase       , 0x99);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_BDMStep      , 0x03);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_SDE          , 0x04);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_BRIGHT       , 0x00);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_SIGN         , 0x06);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_UVADJ0       , 0x11);
        SCCB_WriteSingleRegister(BOARD_OV7725_I2C,OV7725_DEV_ADD, OV7725_UVADJ1       , 0x02);
	
	
}

void SendImage_PC(uint32_t UART_instance , uint8_t *str)
{ 
		uint32_t count=0;
		for(count=0 ; count<OV7725_SIZE ; count++)
		{
			UART_WriteByte(UART_instance , *str++);
		}	
}
void _7725_DMA(void )
	{
		DMA_InitTypeDef DMA_InitStruct1 = {0};
    //DMA??
    DMA_InitStruct1.chl = BOARD_OV7725_DMA; //通道
    DMA_InitStruct1.chlTriggerSource = PORTC_DMAREQ; //触发源
    DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;
    DMA_InitStruct1.minorLoopByteCnt = 1;
    //DMA_InitStruct1.majorLoopCnt = ((OV7620_W/8));
		DMA_InitStruct1.majorLoopCnt = OV7725_SIZE;
    
    DMA_InitStruct1.sAddr = (uint32_t)&PTC->PDIR+BOARD_OV7725_DATA_OFFSET/8;
    DMA_InitStruct1.sLastAddrAdj = 0;
    DMA_InitStruct1.sAddrOffset = 0;
    DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.sMod = kDMA_ModuloDisable;
    
    DMA_InitStruct1.dAddr = NULL;
    DMA_InitStruct1.dLastAddrAdj = 0;
    DMA_InitStruct1.dAddrOffset = 1;
    DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.dMod = kDMA_ModuloDisable;

    /* initialize DMA moudle */
    DMA_Init(&DMA_InitStruct1);
	
}

void GPIO_PORT(void ){    /*行场pclk */

    GPIO_QuickInit(BOARD_OV7725_PCLK_PORT, BOARD_OV7725_PCLK_PIN, kGPIO_Mode_IPD);
    GPIO_QuickInit(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_Mode_IPD);
    /* install callback */
    GPIO_CallbackInstall(BOARD_OV7725_VSYNC_PORT, OV_ISR);
    
    GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, true);
//    GPIO_ITDMAConfig(BOARD_OV7620_PCLK_PORT, BOARD_OV7620_PCLK_PIN, kGPIO_DMA_RisingEdge, true);
    //GPIO_ITDMAConfig(BOARD_OV7620_VSYNC_PORT, BOARD_OV7620_VSYNC_PIN, kGPIO_IT_RisingEdge, true);
    GPIO_ITDMAConfig(BOARD_OV7725_PCLK_PORT, BOARD_OV7725_PCLK_PIN, kGPIO_DMA_FallingEdge, true);
    /* 数据端口初始化 */
    for(i=0;i<8;i++)
    {
        GPIO_QuickInit(BOARD_OV7725_DATA_PORT, BOARD_OV7725_DATA_OFFSET+i, kGPIO_Mode_IFT);
    }
	
	
}

void SXT_7725_Init(void)
{
	 if(SCCB_Init(BOARD_OV7725_I2C))
        printf("no ov7725device found!\r\n");
    else
			{
				printf("OV7725 setup complete\r\n");
        write_reg( ) ;  //ID号确认无误后  配置寄存器
			}	
 		GPIO_PORT( );
		_7725_DMA( );//摄像头配置完成
			
}
//void OV_ISR(uint32_t index)
//{
////    static uint32_t v_counter;
//    /* 场中断 */
//    if(index & (1 << BOARD_OV7725_VSYNC_PIN))
//    {
//        switch(status)
//        {
//            case kIdle:     /* 开始捕捉*/
//                DMA_SetMajorLoopCounter(BOARD_OV7725_DMA, OV7725_SIZE);
//                DMA_SetDestAddress(BOARD_OV7725_DMA, (uint32_t)gImageBuffer);  //目标地址
//                DMA_EnableRequest(BOARD_OV7725_DMA); 
//                status = kInCapture;
//                break;
//            case kInCapture:  /*捕捉完毕，关闭中断*/
//                GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, false);
//                PORTC->ISFR = 0xFFFFFFFF;  //清楚中断标志位
//                status = kComplete;
//                break;
//        }
//    }
//}
//void Tu_Xiang(void)
//{
//        if(status == kComplete)
//        {
////			      UART_WriteByte(HW_UART3, 0x00);
////						UART_WriteByte(HW_UART3, 0xff);
////						UART_WriteByte(HW_UART3, 0x01);
////						UART_WriteByte(HW_UART3, 0x01);	
////						SendImage_PC(HW_UART3 , gImageBuffer);             //向上位机发送数据
//						//Image_Decompression(gImageBuffer,image_dec[0]);  //解压
//					
//							show_60x80_7725(gImageBuffer);                 //OLED显示
//            /* 开启中断，开始下一次捕捉 */
//            status = kIdle;
//            GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, true);
//				}
//				
//					
//			}

