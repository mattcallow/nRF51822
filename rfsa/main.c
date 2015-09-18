#include <stdio.h>
#include <math.h>
#include "nrf.h"
#include "micro_esb.h"
#include "uesb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"
#include "app_uart.h"
#include "spi_master.h"
#include "app_error.h"
#include "app_trace.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"
#include "nordic_common.h"
#include "boards.h"
#include "Adafruit_SSD1306.h"

static uesb_payload_t rx_payload;
#define UART_IRQ_PRIORITY                       APP_IRQ_PRIORITY_LOW

#define     RX_BUF_SIZE     256   /**< Size of desired RX buffer, must be a power of 2 or ZERO (No FIFO). */
#define     TX_BUF_SIZE     256   /**< Size of desired TX buffer, must be a power of 2 or ZERO (No FIFO) */

typedef struct {
  uint32_t id;
} my_event_t;

/**< Maximum size of scheduler events. */
#define SCHED_MAX_EVENT_DATA_SIZE        MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(my_event_t))          

/**< Maximum number of events in the scheduler queue. */
#define SCHED_QUEUE_SIZE                 8

#define TIMER_RTC_PRESCALER 0

/**
*@breif UART configuration structure
*/
static const app_uart_comm_params_t comm_params =
{
  .rx_pin_no  = RX_PIN_NUMBER,
  .tx_pin_no  = TX_PIN_NUMBER,
  .rts_pin_no = RTS_PIN_NUMBER,
  .cts_pin_no = CTS_PIN_NUMBER,
  //Below values are defined in ser_config.h common for application and connectivity
  .flow_control = APP_UART_FLOW_CONTROL_ENABLED, 
  .use_parity   = false,
  .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
};

#define ENDL "\r\n"


void uesb_event_handler()
{
  static uint32_t rf_interrupts;

  uesb_get_clear_interrupts(&rf_interrupts);

  if(rf_interrupts & UESB_INT_TX_SUCCESS_MSK)
  {   
  }

  if(rf_interrupts & UESB_INT_TX_FAILED_MSK)
  {
  }

  if(rf_interrupts & UESB_INT_RX_DR_MSK)
  {
    uesb_read_rx_payload(&rx_payload);
    NRF_GPIO->OUTCLR = 0xFF << 8;
    NRF_GPIO->OUTSET = rx_payload.data[1] << 8;
  }
}

void test_handler(void *p_event_data, uint16_t event_size)
{
  app_trace_log("test_handler\r\n");
}

void wait_timer_handler(void *p_context)
{
  static int i=0;
  //app_trace_log("timer count %d\r\n", i++);
  LEDS_INVERT(BSP_LED_0_MASK);
  if (i%64 == 0) ssd1306_clearDisplay();
  ssd1306_drawPixel(2*(i%64), i%64, WHITE);
  ssd1306_display();
  i++;
}

/**@brief   Function for handling UART interrupts.
*
* @details This function will receive a single character from the UART and append it to a string.
*          The string will be be sent over BLE when the last character received was a 'new line'
*          i.e '\n' (hex 0x0D) or if the string has reached a length of @ref NUS_MAX_DATA_LENGTH.
*/

void uart_evt_callback(app_uart_evt_t * uart_evt)
{
  switch (uart_evt->evt_type)
  {
  case APP_UART_DATA: 
    //Data is ready on the UART         
    break;

  case APP_UART_DATA_READY:
    //Data is ready on the UART FIFO    
    break;

  case APP_UART_TX_EMPTY:
    //Data has been successfully transmitted on the UART
    break;

  default:
    break;
  }

}

#define OLED_DC 6
#define OLED_RESET 7
#define OLED_SPI SPI_MASTER_0


#ifdef DEBUG
#define DBG_DELAY 200
#define DBGP(mask) do { LEDS_ON(mask); nrf_delay_ms(DBG_DELAY); } while (0);
#else
#define DBGP(mask)
#endif
int main()
{
  uint32_t err_code;
  // nrf_gpio_range_cfg_output(8, 31);
  LEDS_CONFIGURE(LEDS_MASK);
  LEDS_OFF(LEDS_MASK);
  DBGP(BSP_LED_0_MASK);

  nrf_drv_clock_init(NULL);
  nrf_drv_clock_hfclk_request();
  nrf_drv_clock_lfclk_request();



  // init the UART
  APP_UART_FIFO_INIT(&comm_params,
    RX_BUF_SIZE,
    TX_BUF_SIZE,
    uart_evt_callback,
    UART_IRQ_PRIORITY,
    err_code);
  APP_ERROR_CHECK(err_code);

  do
  {
    app_trace_log("HF clock %d LF clock %d\r\n", nrf_clock_hf_is_running(), nrf_clock_lf_is_running());
  } while(!nrf_clock_lf_is_running());
  app_trace_log("HF clock %d LF clock %d\r\n", nrf_clock_hf_is_running(), nrf_clock_lf_is_running());
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
  APP_TIMER_APPSH_INIT(TIMER_RTC_PRESCALER, 8, 8, true);


  DBGP(BSP_LED_1_MASK);
  /*
  app_trace_log("Set up radio\r\n");
  // Setup the radio
  uesb_config_t uesb_config       = UESB_DEFAULT_CONFIG;
  uesb_config.rf_channel          = 5;
  uesb_config.crc                 = UESB_CRC_16BIT;
  uesb_config.dynamic_ack_enabled = 0;
  uesb_config.payload_length      = 8;
  uesb_config.protocol            = UESB_PROTOCOL_ESB_DPL;
  uesb_config.bitrate             = UESB_BITRATE_2MBPS;
  uesb_config.mode                = UESB_MODE_PRX;
  uesb_config.event_handler       = uesb_event_handler;

  uesb_init(&uesb_config);

  */


  DBGP(BSP_LED_2_MASK); 
  app_timer_id_t wait_timer_id;
  err_code = app_timer_create(&wait_timer_id, APP_TIMER_MODE_REPEATED , wait_timer_handler);
  app_trace_log("%ld: wait_timer_id is %ld\r\n", err_code, wait_timer_id);
  APP_ERROR_CHECK(err_code);
  uint32_t ticks = APP_TIMER_TICKS(40, TIMER_RTC_PRESCALER);
  err_code = app_timer_start(wait_timer_id, ticks, NULL);
  app_trace_log("%ld: ticks=%ld\r\n", err_code, ticks);
  err_code = app_sched_event_put(NULL, 0, test_handler);
  app_trace_log("%ld: event scheduled\r\n", err_code);
  // err_code = app_sched_event_put(NULL, 0, test_handler);
  // app_trace_log("%ld: event scheduled\r\n", err_code);
  //unsigned int i=0;
  LEDS_OFF(LEDS_MASK);
  ssd1306_init(OLED_SPI, OLED_DC, OLED_RESET );
  ssd1306_begin(SSD1306_SWITCHCAPVCC, true);
  ssd1306_display();
  for(;;)
  {
    app_sched_execute();
    //app_trace_log("main loop count %d\r\n", i++);
    //nrf_delay_ms(20);
    __WFI();
  }

  return 0;
}

// vim: ts=2 sw=2 syn=off autoindent expandtab
