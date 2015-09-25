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
#define MAX_CYCLES 25
// IIR COEFF used by the IIR filter to average the samples
#define IIR_COEFF 0.5
#define MIN_RSSI -80.0
#define MAX_RSSI -50.0

typedef struct {
  float mean;
  float max;
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
  #define Y_OFFSET 20
  #define X_AXIS_POS 60
  #define _Y(y) (63-(y))
  u8g_FirstPage(&u8g);
  do
  {
    u8g_SetFont(&u8g, u8g_font_5x7);
    u8g_DrawFrame(&u8g, 0, 0, 127, 63);
    for (int i=0;i<MAX_SAMPLES;i++) 
    {
      uint8_t x = i + X_OFFSET;
      uint8_t y = Y_OFFSET+(uint8_t)(-MIN_RSSI+samples[i].mean);
      u8g_DrawLine(&u8g, x, _Y(Y_OFFSET), x, _Y(y));
      y = Y_OFFSET +(uint8_t)(-MIN_RSSI+samples[i].max);
      u8g_DrawPixel(&u8g, x, _Y(y));
      if (i%10==0)
      {
        u8g_DrawLine(&u8g, x, _Y(Y_OFFSET), x, _Y(Y_OFFSET-3));
      }
    }
    const char *x_label_1 = "2.4";
    const char *x_label_2 = "2.5";
    const char *x_label_3 = "GHz";
    u8g_DrawStr(&u8g,  X_OFFSET - u8g_GetStrWidth(&u8g, x_label_1)/2, X_AXIS_POS, x_label_1);
    u8g_DrawStr(&u8g,  X_OFFSET + MAX_SAMPLES- u8g_GetStrWidth(&u8g, x_label_2)/2, X_AXIS_POS, x_label_2);
    u8g_DrawStr(&u8g,  X_OFFSET + (MAX_SAMPLES- u8g_GetStrWidth(&u8g, x_label_3))/2, X_AXIS_POS, x_label_3);
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
  uint32_t ticks = APP_TIMER_TICKS(350, TIMER_RTC_PRESCALER);
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
      float reading = -1.0 * (float)(NRF_RADIO->RSSISAMPLE); // range 0-127 = 0 to -128dbm. Valid range 50 to 80 = -50 to -80dbm
      if (reading < MIN_RSSI) reading=MIN_RSSI;
      else if (reading > MAX_RSSI) reading=MAX_RSSI;
      samples[freq].mean   = ((1.0-IIR_COEFF)*samples[freq].mean)   + (IIR_COEFF*reading);
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
        //app_trace_log("cycle=%ld" ENDL, cycle);
      }
      NRF_RADIO->FREQUENCY = freq;
      NRF_RADIO->TASKS_RXEN=1;
    }
  }
  return 0;
}


// vim: ts=2 sw=2 autoindent expandtab
