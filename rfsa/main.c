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
#include "u8g_arm.h"

static uesb_payload_t rx_payload;
#define UART_IRQ_PRIORITY                       APP_IRQ_PRIORITY_LOW

#define     RX_BUF_SIZE     256   /**< Size of desired RX buffer, must be a power of 2 or ZERO (No FIFO). */
#define     TX_BUF_SIZE     256   /**< Size of desired TX buffer, must be a power of 2 or ZERO (No FIFO) */

typedef enum {
 MEVT_RF_READY=1, 
 MEVT_RF_RSSI,
} event_id_t;

typedef struct {
  event_id_t id;
  uint32_t argv;
  void * argp;
} my_event_t;


#define MAX_SAMPLES 100

uint8_t samples[MAX_SAMPLES];

// graphics/display structure
u8g_t u8g;

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

void display_timer_handler(void *p_context)
{
  static int i=0;
  //app_trace_log("timer count %d\r\n", i++);
  LEDS_INVERT(BSP_LED_0_MASK);
  u8g_FirstPage(&u8g);
  do
  {
    //u8g_SetFont(&u8g, u8g_font_unifont);
    //u8g_DrawStr(&u8g,  i, 12, "Hello World!");
    for (int x=0;x<MAX_SAMPLES;x++) 
    {
      uint8_t h = samples[x];
      u8g_DrawVLine(&u8g, x, h, 64-h);
    }
  } while ( u8g_NextPage(&u8g) );

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

void radio_event_handler(void * p_event_data, uint16_t event_size)
{
  LEDS_OFF(BSP_LED_1_MASK);
  my_event_t *p_evt = (my_event_t *)p_event_data;
  switch (p_evt->id)
  {
  case MEVT_RF_READY:
    //app_trace_log("Radio Ready" ENDL);
    NRF_RADIO->TASKS_START=1;
    NRF_RADIO->TASKS_RSSISTART=1;
    break;
  case MEVT_RF_RSSI:
    //app_trace_log("Radio RSSI ready -%d" ENDL, (int)(NRF_RADIO->RSSISAMPLE));
    break;

  }
}

void RADIO_IRQHandler()
{
  LEDS_ON(BSP_LED_1_MASK);
  static my_event_t evt;
  if(NRF_RADIO->EVENTS_READY && (NRF_RADIO->INTENSET & RADIO_INTENSET_READY_Msk))
  {
      NRF_RADIO->EVENTS_READY = 0;
      evt.id = MEVT_RF_READY;
      app_sched_event_put(&evt, sizeof(evt), radio_event_handler);
  }
  NVIC_ClearPendingIRQ(RADIO_IRQn);
}

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
  app_trace_log("Set up radio state=%lu" ENDL, NRF_RADIO->STATE);
  //NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk | RADIO_INTENSET_RSSIEND_Msk;
  //NVIC_SetPriority(RADIO_IRQn, 0x01);
  //NVIC_EnableIRQ(RADIO_IRQn);
  NRF_RADIO->TASKS_RXEN=1;
  /*
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
  app_timer_id_t display_timer_id;
  err_code = app_timer_create(&display_timer_id, APP_TIMER_MODE_REPEATED , display_timer_handler);
  app_trace_log("%ld: display_timer_id is %ld\r\n", err_code, display_timer_id);
  APP_ERROR_CHECK(err_code);
  uint32_t ticks = APP_TIMER_TICKS(500, TIMER_RTC_PRESCALER);
  err_code = app_timer_start(display_timer_id, ticks, NULL);
  app_trace_log("%ld: ticks=%ld\r\n", err_code, ticks);

  LEDS_OFF(LEDS_MASK);

  u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_hw_spi, u8g_com_hw_spi_fn);
  u8g_SetDefaultForegroundColor(&u8g);
  uint32_t freq = 0;
  for(;;)
  {
    app_sched_execute();
    //__WFI();
    //app_trace_log("radio state=%lu" ENDL, NRF_RADIO->STATE);
    if(NRF_RADIO->EVENTS_READY == 1)
    {
      NRF_RADIO->EVENTS_READY  = 0;
      NRF_RADIO->TASKS_START = 1;
      NRF_RADIO->TASKS_RSSISTART = 1;
    }
    if(NRF_RADIO->EVENTS_RSSIEND == 1)
    {
      NRF_RADIO->EVENTS_RSSIEND = 0;
      //app_trace_log("radio RSSI for %lu=%lu" ENDL, freq,NRF_RADIO->RSSISAMPLE);
      samples[freq] = NRF_RADIO->RSSISAMPLE/2;
      NRF_RADIO->TASKS_DISABLE=1;
    }
    if(NRF_RADIO->EVENTS_DISABLED == 1)
    {
      NRF_RADIO->EVENTS_DISABLED = 0;
      freq++;
      if (freq > MAX_SAMPLES) freq=0;
      NRF_RADIO->FREQUENCY = freq;
      NRF_RADIO->TASKS_RXEN=1;
    }
  }

  return 0;
}


// vim: ts=2 sw=2 autoindent expandtab
