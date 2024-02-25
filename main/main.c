#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>

static const char *TAG = "Main";

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks (400)
#define BDC_MCPWM_GPIO_A0              5
#define BDC_MCPWM_GPIO_B0              16
#define BDC_MCPWM_GPIO_A1              17
#define BDC_MCPWM_GPIO_B1              21

#define BTN_FORWARD 22
#define BTN_BACKWARD 4

#define EN_A 14
#define EN_B 12

#define STACK_SIZE 4096

uint8_t btn_level_forward = 0;         
uint8_t prev_btn_level_forward = 0;
uint8_t btn_level_backward = 0;         
uint8_t prev_btn_level_backward = 0;
uint16_t adc_value = 0;
static int adc_raw[2][10];

esp_err_t create_tasks(void);
void mcpwm_motor_control(void *args);
// void adc_line_follower(void *args);

void app_main(void)
{
    // setup
    create_tasks();
}

void mcpwm_motor_control (void *args)
{
    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A0,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B0,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor = NULL;

    bdc_motor_config_t motor_config_2 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A1,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B1,
    };

    bdc_motor_mcpwm_config_t mcpwm_config_2 = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor_2 = NULL;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_2, &mcpwm_config_2, &motor_2));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor_2));

    gpio_set_direction(BTN_FORWARD, GPIO_MODE_INPUT);
    gpio_set_direction(BTN_BACKWARD, GPIO_MODE_INPUT);

    gpio_set_direction(EN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_B, GPIO_MODE_OUTPUT);

    gpio_set_level(EN_A, 1);
    gpio_set_level(EN_B, 1);

    ESP_ERROR_CHECK(bdc_motor_set_speed(motor, 250));
    ESP_ERROR_CHECK(bdc_motor_set_speed(motor_2, 250));

    while(1){
        btn_level_forward = gpio_get_level(BTN_FORWARD);
        btn_level_backward = gpio_get_level(BTN_BACKWARD);

      if (btn_level_forward == 1 && prev_btn_level_forward == 0 && btn_level_backward == 0) 
      {
        //ESP_LOGI(TAG, "Forward motor");
        ESP_ERROR_CHECK(bdc_motor_forward(motor));
        ESP_ERROR_CHECK(bdc_motor_forward(motor_2));
        prev_btn_level_forward = 1;
      } 

      if (btn_level_backward == 1 && prev_btn_level_backward == 0 && btn_level_forward == 0) {
        //ESP_LOGI(TAG, "Backward motor");
        ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_2));
        prev_btn_level_backward = 1;
      } 
      
      if(btn_level_backward == 0 && btn_level_forward == 0) {
        //ESP_LOGI(TAG, "Stop motor");
        prev_btn_level_backward = 0;
        prev_btn_level_forward = 0;
        ESP_ERROR_CHECK(bdc_motor_brake(motor));
        ESP_ERROR_CHECK(bdc_motor_brake(motor_2));
      }

      // if (&adc_raw[0][4] == 4095) {
      //   ESP_LOGI(TAG, "Para la derecha");
      //   ESP_ERROR_CHECK(bdc_motor_forward(motor));
      //   ESP_ERROR_CHECK(bdc_motor_brake(motor_2));
      // } else if (&adc_raw[0][3] == 4095) {
      //   ESP_LOGI(TAG, "Para la derecha");
      //   ESP_ERROR_CHECK(bdc_motor_forward(motor));
      //   ESP_ERROR_CHECK(bdc_motor_brake(motor_2));
      // } else if (&adc_raw[0][2] == 4095) {
      //   ESP_LOGI(TAG, "Para el centro");
      //   ESP_ERROR_CHECK(bdc_motor_forward(motor));
      //   ESP_ERROR_CHECK(bdc_motor_forward(motor_2));
      // } else if (&adc_raw[0][1] == 4095) {
      //   ESP_LOGI(TAG, "Para la izquierda");
      //   ESP_ERROR_CHECK(bdc_motor_forward(motor_2));
      //   ESP_ERROR_CHECK(bdc_motor_brake(motor));
      // } else if (&adc_raw[0][0] == 4095) {
      //   ESP_LOGI(TAG, "Para la izquierda");
      //   ESP_ERROR_CHECK(bdc_motor_forward(motor_2));
      //   ESP_ERROR_CHECK(bdc_motor_brake(motor));
      // }

    }
}

void adc_line_follower(void *args)
{

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_11,
    };

    // 5 CANALES
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));


  while(1) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw[0][0]);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &adc_raw[0][1]));
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_1, adc_raw[0][1]);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[0][2]));
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_2, adc_raw[0][2]);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &adc_raw[0][3]));
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_3, adc_raw[0][3]);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[0][4]));
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_4, adc_raw[0][4]);
  }
}

esp_err_t create_tasks(void)
{
    xTaskCreate(mcpwm_motor_control,
                "mcpwm_motor_control",
                STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL);
    xTaskCreate(adc_line_follower,
                "adc_line_follower",
                STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL);

    return ESP_OK;
}