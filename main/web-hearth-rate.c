#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <esp_http_server.h>
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_system.h"
#include "esp_smartconfig.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/inet.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "u8g2_esp32_hal.h"
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/spi_master.h>
#include <esp_adc/adc_continuous.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <lwip/api.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <u8g2.h>

// i2c
#define PIN_SDA 21
#define PIN_SCL 22

// adc
#define ADC2_CHAN0 ADC_CHANNEL_0 // pin VP
#define ADC_ATTEN ADC_ATTEN_DB_12

static EventGroupHandle_t s_wifi_event_group;
static adc_oneshot_unit_handle_t adc_handle;
static gptimer_handle_t gptimer_handle;
static TaskHandle_t task1_handle;
static TaskHandle_t task2_handle;
static QueueHandle_t queue_raw_handle;
static u8g2_t u8g2;
static const char *TAG = "Prot";

static EventGroupHandle_t s_wifi_event_group;

static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;

uint8_t reads = 0;
uint8_t beats = 0;
uint16_t BPMa[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool BPMb = 0;
int BPMm = 0;
uint8_t s_retry_num = 0;

volatile int rate[10];
volatile unsigned long samplecounter = 0;
volatile unsigned long lastbeatime = 0;
long runningTotal;
volatile int P = 512;
volatile int T = 2000;
volatile int amp = 0;
volatile bool fb = true;
volatile bool sb = false;
int adc_raw = 0;
volatile int IBI = 600;
volatile bool Pulse = false;
volatile bool QS = false;
volatile bool unr = false;
volatile int pulse_threshool = 2000;
httpd_handle_t mySocketHD;
int mySocketFD;
const static char *sse_format = "data:%d\r\n\r\n";

const static char html_str[] = "<!DOCTYPE html>"
                               "<html lang='en'>"
                               "<head>"
                               "<meta charset='UTF-8'>"
                               "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                               "<title>ESP-IDF SSE Demo</title>"
                               "</head>"
                               "<body>"
                               "<div id='target'>SSE Test</div> "
                               "</body>"
                               "<script>"
                               "var source = new EventSource('/sse');"
                               "source.addEventListener('message', function(e) {"
                               "document.getElementById('target').innerHTML = e.data;"
                               "}, false);"
                               "</script>"
                               "</html>";
const static char sse_resp[] = "HTTP/1.1 200 OK\r\n"
                               "Cache-Control: no-store\r\n"
                               "Content-Type: text/event-stream\r\n"
                               "\r\n"
                               "retry: 20000\r\n"
                               "\r\n";

static esp_err_t hello_get_handler(httpd_req_t *req)
{
  char *buf;
  size_t buf_len;

  /* Get header value string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (buf_len > 1)
  {
    buf = malloc(buf_len);
    /* Copy null terminated value string into buffer */
    if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found header => Host: %s", buf);
    }
    free(buf);
  }
  if (0 == strcmp(req->uri, "/"))
  {
    ESP_LOGW(TAG, "/");
    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    /* Send response with custom headers and body set as the
     * string passed in user context*/
    httpd_resp_send(req, html_str, strlen(html_str));
  }
  else if (0 == strcmp(req->uri, "/favicon.ico"))
  {
    ESP_LOGW(TAG, "favicon");
    httpd_resp_send(req, html_str, 0);
  }
  else if (0 == strcmp(req->uri, "/sse"))
  {
    ESP_LOGW(TAG, "sse");
    mySocketHD = req->handle;
    mySocketFD = httpd_req_to_sockfd(req);
    // char buf[50];
    // snprintf(buf, sizeof(buf), "data:%d\r\n", cnt++);
    httpd_socket_send(mySocketHD, mySocketFD, sse_resp, sizeof(sse_resp), 0);
  }

  /* After sending the HTTP response the old HTTP request
   * headers are lost. Check if HTTP request headers can be read now. */
  if (httpd_req_get_hdr_value_len(req, "Host") == 0)
  {
    ESP_LOGI(TAG, "Request headers lost");
  }
  return ESP_OK;
}

static const httpd_uri_t main = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = hello_get_handler,
};

static const httpd_uri_t ico = {
    .uri = "/favicon.ico",
    .method = HTTP_GET,
    .handler = hello_get_handler,
};
static const httpd_uri_t sse = {
    .uri = "/sse",
    .method = HTTP_GET,
    .handler = hello_get_handler,
};

httpd_handle_t start_webserver(void)
{
  ESP_LOGI("CU", "SUS1");
  /* Generate default configuration */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK)
  {
    /* Register URI handlers */
    httpd_register_uri_handler(server, &main);
    httpd_register_uri_handler(server, &ico);
    httpd_register_uri_handler(server, &sse);
  }
  ESP_LOGI("CU", "SUS");
  /* If server failed to start, handle will be NULL */
  return server;
}
// core 0
static void task2()
{

  for (;;)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC2_CHAN0, &adc_raw));
    samplecounter += 10;                 // keep track of the time in mS with this variable
    int N = samplecounter - lastbeatime; // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
    if (adc_raw < pulse_threshool && N > (IBI / 5) * 3)
    { // avoid dichrotic noise by waiting 3/5 of last IBI
      if (adc_raw < T)
      {              // T is the trough
        T = adc_raw; // keep track of lowest point in pulse wave
      }
    }

    if (adc_raw > P)
    {              // thresh condition helps avoid noise
      P = adc_raw; // P is the peak
    } // keep track of highest point in pulse wave

    //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
    // signal surges up in value every time there is a pulse
    if ((adc_raw > pulse_threshool) && (Pulse == false) && (N > (IBI / 5) * 3))
    {
      Pulse = true;                      // set the Pulse flag when we think there is a pulse
      IBI = samplecounter - lastbeatime; // measure time between beats in mS
      lastbeatime = samplecounter;       // keep track of time for next pulse

      if (sb)
      {             // if this is the second beat, if secondBeat == TRUE
        sb = false; // clear secondBeat flag
        for (int i = 0; i <= 9; i++)
        { // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;
        }
      }

      if (fb)
      {             // if it's the first time we found a beat, if firstBeat == TRUE
        fb = false; // clear firstBeat flag
        sb = true;  // set the second beat flag
        unr = true; // IBI value is unreliable so discard it
      }
      if (!unr)
      {
        // keep a running total of the last 10 IBI values
        runningTotal = 0; // clear the runningTotal variable

        for (int i = 0; i <= 8; i++)
        {                          // shift data in the rate array
          rate[i] = rate[i + 1];   // and drop the oldest IBI value
          runningTotal += rate[i]; // add up the 9 oldest IBI values
        }

        rate[9] = IBI;               // add the latest IBI to the rate array
        runningTotal += rate[9];     // add the latest IBI to runningTotal
        runningTotal /= 10;          // average the last 10 IBI values
        BPMm = 60000 / runningTotal; // how many beats can fit into a minute? that's BPM!
        QS = true;                   // set Quantified Self flag
        // QS FLAG IS NOT CLEARED INSIDE THIS ISR
      }
    }
    if (!unr)
    {
      if (adc_raw < pulse_threshool && Pulse == true)
      {                                // when the values are going down, the beat is over
        Pulse = false;                 // reset the Pulse flag so we can do it again
        amp = P - T;                   // get amplitude of the pulse wave
        pulse_threshool = amp / 2 + T; // set thresh at 50% of the amplitude
        P = pulse_threshool;           // reset these for next time
        T = pulse_threshool;
      }

      if (N > 2500)
      {                              // if 2.5 seconds go by without a beat
        pulse_threshool = 2000;      // set thresh default
        P = 512;                     // set P default
        T = 2000;                    // set T default
        lastbeatime = samplecounter; // bring the lastbeatime up to date
        fb = true;                   // set these to avoid noise
        sb = false;                  // when we get the heartbeat back
      }
    }
    unr = false;
    xQueueSend(queue_raw_handle, &adc_raw, (TickType_t)0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// core 1
static void task3()
{
  char sse_buf[50];
  while (1)
  {
    if (mySocketFD > 0)
    {
      snprintf(sse_buf, sizeof(sse_buf), sse_format, BPMm);
      httpd_socket_send(mySocketHD, mySocketFD, sse_buf, strlen(sse_buf), 0);
    }
    ESP_LOGI(TAG, "T %d", T);
    ESP_LOGI(TAG, "P %d", P);
    ESP_LOGI(TAG, "treash %d", pulse_threshool);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

static void task1()
{
  char BPM[12];
  uint8_t posy = 0;
  uint8_t new_read = 0;
  int raw;
  for (;;)
  {
    posy++;
    snprintf(BPM, sizeof(BPM), "BPM:%0*d", 3, BPMm);
    // keep all lines
    if (posy >= 128)
    {
      posy = 0;
      u8g2_ClearBuffer(&u8g2);
    }

    // draw BPM
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_DrawStr(&u8g2, 0, 9, BPM);

    // draw line
    //(int) (adc_raw[i]*0.16) - 336
    if (queue_raw_handle != 0)
    {
      uint8_t mw = uxQueueMessagesWaiting(queue_raw_handle);
      for (uint8_t i = 0; i < mw; i++)
      {
        xQueueReceive(queue_raw_handle, &raw, (TickType_t)10);
        new_read = (int)(raw * 0.16) - 336;
        u8g2_DrawLine(&u8g2, posy, 64, posy, 64 - new_read);
        posy++;
      }
    }

    u8g2_SendBuffer(&u8g2);

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

static void smartconfig_example_task()
{
  EventBits_t uxBits;
  ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
  smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
  while (1)
  {
    uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
    if (uxBits & CONNECTED_BIT)
    {
      ESP_LOGI(TAG, "WiFi Connected to ap");
    }
    if (uxBits & ESPTOUCH_DONE_BIT)
    {
      ESP_LOGI(TAG, "smartconfig over");
      esp_smartconfig_stop();
      start_webserver();
      vTaskDelete(NULL);
    }
  }
}

static void smartconfig_event_handler(void *arg, esp_event_base_t event_base,
                                      int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    xTaskCreatePinnedToCore(smartconfig_example_task,
                            "smartconfig",
                            4096,
                            NULL,
                            3,
                            NULL,
                            1);
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    esp_wifi_connect();
    xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
  }
  else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE)
  {
    ESP_LOGI(TAG, "Scan done");
  }
  else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL)
  {
    ESP_LOGI(TAG, "Found channel");
  }
  else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
  {
    ESP_LOGI(TAG, "Got SSID and password");

    smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
    wifi_config_t wifi_config;
    uint8_t ssid[33] = {0};
    uint8_t password[65] = {0};
    uint8_t rvd_data[33] = {0};

    bzero(&wifi_config, sizeof(wifi_config_t));
    memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
    memcpy(ssid, evt->ssid, sizeof(evt->ssid));
    memcpy(password, evt->password, sizeof(evt->password));
    ESP_LOGI(TAG, "SSID:%s", ssid);
    ESP_LOGI(TAG, "PASSWORD:%s", password);
    if (evt->type == SC_TYPE_ESPTOUCH_V2)
    {
      ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
      ESP_LOGI(TAG, "RVD_DATA:");
      for (int i = 0; i < 33; i++)
      {
        printf("%02x ", rvd_data[i]);
      }
      printf("\n");
    }

    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();
  }
  else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
  {
    xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
  }
}

static void initialise_wifi(void)
{
  // init
  ESP_ERROR_CHECK(esp_netif_init());
  s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  // config
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // event handler
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &smartconfig_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &smartconfig_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &smartconfig_event_handler, NULL));

  // start
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void setup_timer()
{
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000 * 10, // 10KHz, 1 tick = 0.1ms
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_handle));
  ESP_ERROR_CHECK(gptimer_enable(gptimer_handle));
  ESP_ERROR_CHECK(gptimer_start(gptimer_handle));

  ESP_LOGI(TAG, "Timer setup done");
  // Retrieve the timestamp at any time
}

void setup_adc()
{
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN,
  };

  adc_oneshot_unit_init_cfg_t init_config2 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc_handle));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC2_CHAN0, &config));

  ESP_LOGI(TAG, "ADC setup done");
}

void setup_display()
{

  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
  u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  // a structure which will contain all the data for one display
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(
      &u8g2, U8G2_R0,
      // u8x8_byte_sw_i2c,
      u8g2_esp32_i2c_byte_cb,
      u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in
                           // sleep mode after this,
  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);

  queue_raw_handle = xQueueCreate(10, sizeof(int));

  if (queue_raw_handle == 0)
  {
    ESP_LOGI(TAG, "Failed to create queue raw");
    return;
  }

  xTaskCreatePinnedToCore(task1,
                          "refresh",
                          4096,
                          NULL,
                          1,
                          &task1_handle,
                          1);

  ESP_LOGI(TAG, "Display setup done");
}

void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());

  initialise_wifi();
  setup_display();
  setup_adc();
  setup_timer();

  xTaskCreatePinnedToCore(task3,
                          "bea",
                          4096,
                          NULL,
                          1,
                          NULL,
                          1);

  // core 0
  xTaskCreatePinnedToCore(task2,
                          "beatrate",
                          4096,
                          NULL,
                          1,
                          &task2_handle,
                          0);
}