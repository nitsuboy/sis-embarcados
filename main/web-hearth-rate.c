#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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

#define PIN_SDA 21                                // pin 21
#define PIN_SCL 22                                // pin 22
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0 // pin 24
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

u8g2_t u8g2;
adc_oneshot_unit_handle_t adc2_handle;
uint8_t reads= 0;
static int adc_raw[10];
static const char* TAG = "prot";

void progressbar(int x, int y, int w, int h, float value) {
    u8g2_DrawFrame(&u8g2,x, y, w,  h);
    u8g2_DrawBox(&u8g2,x + 2, y + 2, (w - 4) * value,  h - 3);
}

void task1() {
    uint8_t teste = 0;
    char buffer[6];
    uint8_t pas = 0;
    uint8_t att = 0;
    for(;;) {
      if (pas == 0) {
        pas = (int) (adc_raw[0]*0.16) - 336;
      }
      
      teste = (teste + 1) % 128;
      snprintf(buffer, sizeof(buffer), "%0*d",3, pas);
      if(teste == 0 || teste >=128){
        u8g2_ClearBuffer(&u8g2);
      }
      //
      ESP_LOGI(TAG,"display out + %d",reads);
      u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
      u8g2_DrawStr(&u8g2, 0, 9, buffer);
      for(int i = 0;i < reads;i++){
        att = (int) (adc_raw[i]*0.16) - 336;
        u8g2_DrawLine(&u8g2, teste + i,64 - pas, teste+1+ i, 64 - att);
        pas = att;
      }
      teste = (teste + reads);
      if(teste >=128){
        u8g2_ClearBuffer(&u8g2);
      }
      
      //progressbar(0,26,100,6,(float) teste * 0.1f);
      u8g2_SendBuffer(&u8g2);
      reads = 0;

      vTaskDelay(100/portTICK_PERIOD_MS);

    }
}

void task2() {
  for(;;) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &adc_raw[reads]));
    reads++;
    ESP_LOGI(TAG, "adc read");
    vTaskDelay(50/portTICK_PERIOD_MS);

  }
}

void setup_adc2() {
  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
  };

  adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN0, &config));
}

void setup_display() {
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

  ESP_LOGI(TAG, "All done!");
}

void setup_softap() {

}

void app_main(void) {
  // nvs to save keys

  //esp_err_t ret = nvs_flash_init();
  //if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
  //  ESP_ERROR_CHECK(nvs_flash_erase());
  //  ret = nvs_flash_init();
  //}
  //ESP_ERROR_CHECK(ret);

  setup_display();
  setup_adc2();

  xTaskCreatePinnedToCore(task1,
                "refresh",
                4096,
                NULL,
                1,
                NULL,
                1);
  
  xTaskCreatePinnedToCore(task2,
              "beatrate",
              4096,
              NULL,
              1,
              NULL,
              0);
}