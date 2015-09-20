/*
  
  u8g_arm.c
  

  u8g utility procedures for LPC11xx

  Universal 8bit Graphics Library
  
  Copyright (c) 2013, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

 
  The following delay procedures must be implemented for u8glib. This is done in this file:

  void u8g_Delay(uint16_t val)		Delay by "val" milliseconds
  void u8g_MicroDelay(void)		Delay be one microsecond
  void u8g_10MicroDelay(void)	Delay by 10 microseconds
  
  Additional requirements:
  
      SysTick must be enabled, but SysTick IRQ is not required. Any LOAD values are fine,
      it is prefered to have at least 1ms
      Example:
        SysTick->LOAD = (SystemCoreClock/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = 7;   // enable, generate interrupt (SysTick_Handler), do not divide by 2
*/

#include <stdlib.h>
#include "u8g_arm.h"
#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "spi_master.h"
#include "boards.h"
#include "app_error.h"
#include "app_trace.h"


/*========================================================================*/
/*
  The following delay procedures must be implemented for u8glib

  void u8g_Delay(uint16_t val)		Delay by "val" milliseconds
  void u8g_MicroDelay(void)		Delay be one microsecond
  void u8g_10MicroDelay(void)	Delay by 10 microseconds

*/

void u8g_Delay(uint16_t val)
{
  nrf_delay_ms((uint32_t)val);
}

void u8g_MicroDelay(void)
{
  nrf_delay_us(1);
}

void u8g_10MicroDelay(void)
{
  nrf_delay_us(10);
}




/*========================================================================*/
/* u8glib com procedure */

static spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
#define OLED_DC 6
#define OLED_RESET 7
#define OLED_SPI SPI_MASTER_0

uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_STOP:
      break;
    
    case U8G_COM_MSG_INIT:
      //Configure SPI master.
      spi_config.SPI_Pin_SCK  = SPIM0_SCK_PIN;
      spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
      spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
      spi_config.SPI_Pin_SS   = SPIM0_SS_PIN;
      spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;


      if ( arg_val <= U8G_SPI_CLK_CYCLE_50NS )
      {
        spi_config.SPI_Freq      = SPI_FREQUENCY_FREQUENCY_M8;
      }
      else if ( arg_val <= U8G_SPI_CLK_CYCLE_300NS )
      {
        spi_config.SPI_Freq      = SPI_FREQUENCY_FREQUENCY_M2;
      }
      else if ( arg_val <= U8G_SPI_CLK_CYCLE_400NS )
      {
        spi_config.SPI_Freq      = SPI_FREQUENCY_FREQUENCY_M1;
      }
      else
      {
        spi_config.SPI_Freq      = SPI_FREQUENCY_FREQUENCY_K500;
      }

      //Initialize SPI master.
      uint32_t err_code = spi_master_open(OLED_SPI, &spi_config);
      APP_ERROR_CHECK(err_code);

      nrf_gpio_cfg_output(OLED_RESET);
      nrf_gpio_cfg_output(OLED_DC);
      //set_gpio_mode(u8g_pin_cs, 1, 0);		/* output, no pullup */

      u8g_MicroDelay();      
      break;
    
    case U8G_COM_MSG_ADDRESS:  /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g_10MicroDelay();
      nrf_gpio_pin_write(OLED_DC, arg_val);
      u8g_10MicroDelay();
     break;

    case U8G_COM_MSG_CHIP_SELECT:
#if 0
      if ( arg_val == 0 )
      {
        /* disable */
	uint8_t i;
	/* this delay is required to avoid that the display is switched off too early --> DOGS102 with LPC1114 */
	for( i = 0; i < 5; i++ )
	  u8g_10MicroDelay();
	set_gpio_level(u8g_pin_cs, 1);
      }
      else
      {
        /* enable */
	set_gpio_level(u8g_pin_cs, 0);
      }
      u8g_MicroDelay();
#endif
      break;
      
    case U8G_COM_MSG_RESET:
      nrf_gpio_pin_write(OLED_RESET, arg_val);
      u8g_10MicroDelay();
      break;
      
    case U8G_COM_MSG_WRITE_BYTE:
      {
        uint32_t err_code = spi_master_send_recv(OLED_SPI, &arg_val, 1, NULL, 1);
        APP_ERROR_CHECK(err_code);
        u8g_MicroDelay();
        while(spi_master_get_state(OLED_SPI) == SPI_MASTER_STATE_BUSY);
      }
      break;
    
    case U8G_COM_MSG_WRITE_SEQ:
    case U8G_COM_MSG_WRITE_SEQ_P:
      {
        uint32_t err_code = spi_master_send_recv(OLED_SPI, arg_ptr, arg_val, NULL, arg_val);
        APP_ERROR_CHECK(err_code);
        u8g_MicroDelay();
        while(spi_master_get_state(OLED_SPI) == SPI_MASTER_STATE_BUSY);
      }
      break;
  }
  return 1;
}

