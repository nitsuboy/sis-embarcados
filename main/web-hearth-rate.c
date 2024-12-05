#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <driver/gptimer.h>
#include <stdio.h>
#include <string.h>
#include <u8g2.h>
#include <esp_adc/adc_continuous.h>

#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif_net_stack.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#if IP_NAPT
#include "lwip/lwip_napt.h"
#endif
#include "lwip/err.h"
#include "lwip/sys.h"

//i2c
#define PIN_SDA                             21
#define PIN_SCL                             22

//adc
#define ADC2_CHAN0                          ADC_CHANNEL_0 // pin 24,D4
#define ADC_ATTEN                           ADC_ATTEN_DB_12
#define PULSE_THRESHOLD                     2400

//wifi softap and station
#define ESP_WIFI_STA_SSID                   "prototipo"
#define ESP_WIFI_STA_PASSWD                 "mypassword"
#define ESP_MAXIMUM_RETRY                   10
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD   WIFI_AUTH_WPA2_PSK

#define ESP_WIFI_AP_SSID                    "prototipo"
#define ESP_WIFI_AP_PASSWD                  "mypassword"
#define ESP_WIFI_CHANNEL                    0
#define MAX_STA_CONN                        1

#define WIFI_CONNECTED_BIT                  BIT0
#define WIFI_FAIL_BIT                       BIT1

static u8g2_t                           u8g2;
static EventGroupHandle_t s_wifi_event_group;
static adc_oneshot_unit_handle_t adc2_handle;
static gptimer_handle_t              gptimer;
static TaskHandle_t             task1_handle;
static TaskHandle_t             task2_handle;
static TaskHandle_t             task3_handle;
static TaskHandle_t             task4_handle;

uint8_t reads = 0;
uint8_t beats = 0;
uint8_t BPMn = 0;
uint8_t s_retry_num = 0;
static int adc_raw[10];
static const char* TAG = "prot";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "Station "MACSTR" joined, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "Station "MACSTR" left, AID=%d, reason:%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Station started");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_netif_t *wifi_init_softap(void)
{
    esp_netif_t *esp_netif_ap = esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = ESP_WIFI_AP_SSID,
            .ssid_len = strlen(ESP_WIFI_AP_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_AP_PASSWD,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(ESP_WIFI_AP_PASSWD) == 0) {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_AP_SSID, ESP_WIFI_AP_PASSWD, ESP_WIFI_CHANNEL);

    return esp_netif_ap;
}

esp_netif_t *wifi_init_sta(void)
{
    esp_netif_t *esp_netif_sta = esp_netif_create_default_wifi_sta();

    wifi_config_t wifi_sta_config = {
        .sta = {
            .ssid = ESP_WIFI_STA_SSID,
            .password = ESP_WIFI_STA_PASSWD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .failure_retry_cnt = ESP_MAXIMUM_RETRY,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
            * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config) );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    return esp_netif_sta;
}

void task1() 
{
  char BPM[12];
  uint8_t posy      = 0;
  uint8_t last_read = 0;
  uint8_t new_read  = 0;
  for(;;)
  {
    posy = (posy + 1) % 128;
    snprintf(BPM, sizeof(BPM), "BPM:%0*d",3, BPMn);
    new_read = (int) (adc_raw[0]*0.16) - 336;
    
    //keep all lines
    if(posy >=128){
      u8g2_ClearBuffer(&u8g2);
    }

    if(posy == 0 || posy >=128){
      u8g2_ClearBuffer(&u8g2);
    }

    //(int) (adc_raw[i]*0.16) - 336
    ESP_LOGI(TAG,"display out + %d",reads);
    
    //draw BPM
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_DrawStr(&u8g2, 0, 9, BPM);
    
    //draw line
    u8g2_DrawLine(&u8g2, posy,64 - last_read, posy + 1, 64 - new_read);

    u8g2_SendBuffer(&u8g2);
    last_read = new_read;
    
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void task2() 
{
  for(;;)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_CHAN0, &adc_raw[0]));
    if(adc_raw[0]>2500)
    {
      beats++;
    }
    ESP_LOGI(TAG, "adc read %d",beats);
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

void task3()
{
  int ftstp = 0;
  int ststp = 0;
  for(;;)
  {
    BPMn = (beats/ftstp - ststp) * 60;
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

void task4()
{
  uint64_t count = 0;
  uint64_t countd;

  for(;;)
  {
    ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &countd));
    BPMn = (int)(beats/(countd - count)) * 600000;
    if(countd - count > 600000)
    {
      ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
      count = 0;
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }

}

void setup_wifi()
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  //Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* Initialize event group */
  s_wifi_event_group = xEventGroupCreate();

  /* Register Event handler */
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                  ESP_EVENT_ANY_ID,
                  &wifi_event_handler,
                  NULL,
                  NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                  IP_EVENT_STA_GOT_IP,
                  &wifi_event_handler,
                  NULL,
                  NULL));

  /*Initialize WiFi */
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

  /* Initialize AP */
  ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
  esp_netif_t *esp_netif_ap = wifi_init_softap();

  /* Initialize STA */
  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  esp_netif_t *esp_netif_sta = wifi_init_sta();

  /* Start WiFi */
  ESP_ERROR_CHECK(esp_wifi_start() );

  /*
    * Wait until either the connection is established (WIFI_CONNECTED_BIT) or
    * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT).
    * The bits are set by event_handler() (see above)
    */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned,
    * hence we can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
      ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                ESP_WIFI_STA_SSID, ESP_WIFI_STA_PASSWD);
  } else if (bits & WIFI_FAIL_BIT) {
      ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                ESP_WIFI_STA_SSID, ESP_WIFI_STA_PASSWD);
  } else {
      ESP_LOGE(TAG, "UNEXPECTED EVENT");
      return;
  }

  /* Set sta as the default interface */
  esp_netif_set_default_netif(esp_netif_sta);

  /* Enable napt on the AP netif */
  if (esp_netif_napt_enable(esp_netif_ap) != ESP_OK) {
    ESP_LOGE(TAG, "NAPT not enabled on the netif: %p", esp_netif_ap);
  }
}

void setup_timer() 
{
  gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1 * 1000*10, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  ESP_ERROR_CHECK(gptimer_start(gptimer));

  ESP_LOGI(TAG, "Timer setup done");
  // Retrieve the timestamp at any time
}

void setup_adc2() 
{
  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN,
  };

  adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_CHAN0, &config));

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
      u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&u8g2);  // send init sequence to the display, display is in
                            // sleep mode after this,
  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0);  // wake up display
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);

  ESP_LOGI(TAG, "Display setup done");
}

void app_main(void) {

  //setup_wifi();
  setup_display();
  setup_adc2();
  setup_timer();

  // core 1
  xTaskCreatePinnedToCore(task1,
                "refresh",
                4096,
                NULL,
                1,
                &task1_handle,
                1);
  
  xTaskCreatePinnedToCore(task4,
              "timer",
              4096,
              NULL,
              1,
              &task4_handle,
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