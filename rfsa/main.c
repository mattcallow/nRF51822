#include <stdio.h>
#include <math.h>
#include "nrf.h"
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
#define MAX_CYCLES 100
// IIR COEFF used by the IIR filter to average the samples
#define IIR_COEFF 0.9

typedef struct {
  uint8_t avg;
  uint8_t max;
} sample_t;
sample_t samples[MAX_SAMPLES];

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

void display_timer_handler(void *p_context)
{
  static int i=0;
  //app_trace_log("timer count %d\r\n", i++);
  LEDS_INVERT(BSP_LED_0_MASK);
  #define X_OFFSET 14
  #define X_AXIS_POS 54
  u8g_FirstPage(&u8g);
  do
  {
    u8g_SetFont(&u8g, u8g_font_5x7);
    for (int i=0;i<MAX_SAMPLES;i++) 
    {
      uint8_t x = i + X_OFFSET;
      uint8_t h = samples[i].avg;
      u8g_DrawVLine(&u8g, x, h, 64-h);
      u8g_DrawPixel(&u8g, x, 64-samples[i].max);
    }
    const char *x_label_1 = "2.4";
    const char *x_label_2 = "2.5";
    u8g_DrawStr(&u8g,  X_OFFSET - MIN(0, u8g_GetStrWidth(&u8g, x_label_1)), X_AXIS_POS, x_label_1);
    u8g_DrawStr(&u8g,  X_OFFSET+MAX_SAMPLES-MIN(0, u8g_GetStrWidth(&u8g, x_label_2)), X_AXIS_POS, x_label_2);
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

#ifdef DEBUG
#define DBG_DELAY 200
#define DBGP(mask) do { LEDS_ON(mask); nrf_delay_ms(DBG_DELAY); } while (0);
#else
#define DBGP(mask)
#endif

int main()
{
  uint32_t err_code;
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
  NRF_RADIO->TASKS_RXEN=1;

  DBGP(BSP_LED_2_MASK); 
  app_timer_id_t display_timer_id;
  err_code = app_timer_create(&display_timer_id, APP_TIMER_MODE_REPEATED , display_timer_handler);
  app_trace_log("%ld: display_timer_id is %ld\r\n", err_code, display_timer_id);
  APP_ERROR_CHECK(err_code);
  uint32_t ticks = APP_TIMER_TICKS(200, TIMER_RTC_PRESCALER);
  err_code = app_timer_start(display_timer_id, ticks, NULL);
  app_trace_log("%ld: ticks=%ld\r\n", err_code, ticks);

  LEDS_OFF(LEDS_MASK);

  u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_hw_spi, u8g_com_hw_spi_fn);
  u8g_SetDefaultForegroundColor(&u8g);
  uint32_t freq = 0;
  uint32_t cycle=0;
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
      uint32_t reading = NRF_RADIO->RSSISAMPLE/2;
      samples[freq].avg=samples[freq].avg*IIR_COEFF+(reading*(1.0-IIR_COEFF));
      if (cycle == 0 || reading > samples[freq].max)
      {
        samples[freq].max = reading;
      }
      NRF_RADIO->TASKS_DISABLE=1;
    }
    if(NRF_RADIO->EVENTS_DISABLED == 1)
    {
      NRF_RADIO->EVENTS_DISABLED = 0;
      freq++;
      if (freq > MAX_SAMPLES) 
      {
        freq=0;
        cycle++;
        if (cycle > MAX_CYCLES) cycle=0;
      }
      NRF_RADIO->FREQUENCY = freq;
      NRF_RADIO->TASKS_RXEN=1;
    }
  }
  return 0;
}


// vim: ts=2 sw=2 autoindent expandtab
