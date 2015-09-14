/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

// LEDs definitions for BLE400
#define LEDS_NUMBER    5

#define LED_START      18
#define LED_1          18
#define LED_2          19
#define LED_3          20
#define LED_4          21
#define LED_5          22
#define LED_STOP       22

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 , LED_5 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4
#define BSP_LED_4      LED_5

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#define BSP_LED_3_MASK (1<<BSP_LED_3)
#define BSP_LED_4_MASK (1<<BSP_LED_4)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK | BSP_LED_4_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  ~LEDS_MASK

#define BUTTONS_NUMBER 2

#define BUTTON_START   16
#define BUTTON_1       16
#define BUTTON_2       17
#define BUTTON_STOP    17
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)

#define BUTTONS_MASK   0x001E0000

#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

#define SPIS_MISO_PIN  28    // SPI MISO signal. 
#define SPIS_CSN_PIN   12    // SPI CSN signal. 
#define SPIS_MOSI_PIN  25    // SPI MOSI signal. 
#define SPIS_SCK_PIN   29    // SPI SCK signal. 

#define SPIM0_SCK_PIN       4     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      1     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      3     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_SS_PIN        2     /**< SPI Slave Select GPIO pin number. */

#define SPIM1_SCK_PIN       15     /**< SPI clock GPIO pin number. */
#define SPIM1_MOSI_PIN      12     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM1_MISO_PIN      14     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM1_SS_PIN        13     /**< SPI Slave Select GPIO pin number. */


#endif // CUSTOM_BOARD_H
