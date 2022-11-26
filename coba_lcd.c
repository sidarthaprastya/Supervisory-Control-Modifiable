#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "i2c-lcd1602.h"


void display_task(void *pvParam)
{
    char lcd_buffer[17];

    smbus_info_t *smbus_info = smbus_malloc();
    i2c_lcd1602_info_t *lcd_info = i2c_lcd1602_malloc();

    smbus_init(smbus_info, I2C_NUM_0, 0x27);
    smbus_set_timeout(smbus_info, 20 / portTICK_PERIOD_MS);

    i2c_lcd1602_init(lcd_info, smbus_info, true, 2, 16, 16);
    i2c_lcd1602_reset(lcd_info);
    i2c_lcd1602_set_backlight(lcd_info, true);

    TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // switch (mode)
        // {
        // case OPERATING_MODE:
        // {
            i2c_lcd1602_move_cursor(lcd_info, 0, 0);
            i2c_lcd1602_write_string(lcd_info, "OPERATING ");
            i2c_lcd1602_move_cursor(lcd_info, 0, 1);
            sprintf(lcd_buffer, "%16f", setpoint);
            i2c_lcd1602_write_string(lcd_info, lcd_buffer);
        //     break;
        // }
        // case SET_P_MODE:
        // {
        //     i2c_lcd1602_move_cursor(lcd_info, 0, 0);
        //     i2c_lcd1602_write_string(lcd_info, "SETTING KP");
        //     i2c_lcd1602_move_cursor(lcd_info, 0, 1);
        //     sprintf(lcd_buffer, "%16f", kp);
        //     i2c_lcd1602_write_string(lcd_info, lcd_buffer);
        //     break;
        // }
        // case SET_I_MODE:
        // {
        //     i2c_lcd1602_move_cursor(lcd_info, 0, 0);
        //     i2c_lcd1602_write_string(lcd_info, "SETTING KI");
        //     i2c_lcd1602_move_cursor(lcd_info, 0, 1);
        //     sprintf(lcd_buffer, "%16f", ki);
        //     i2c_lcd1602_write_string(lcd_info, lcd_buffer);
        //     break;
        // }
        // case SET_D_MODE:
        // {
        //     i2c_lcd1602_move_cursor(lcd_info, 0, 0);
        //     i2c_lcd1602_write_string(lcd_info, "SETTING KD");
        //     i2c_lcd1602_move_cursor(lcd_info, 0, 1);
        //     sprintf(lcd_buffer, "%16f", kd);
        //     i2c_lcd1602_write_string(lcd_info, lcd_buffer);
        //     break;
        // }

        // default:
        //     break;
        // }

        xTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

void app_main()
{
    // gpio_config_t io_conf;
    i2c_config_t i2c_conf;
    // esp_adc_cal_characteristics_t adc_conf;

    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    // gpio_config(&io_conf);

    // esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_10Bit, 0, &adc_conf);
    // adc2_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN_11db);

    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = GPIO_NUM_21;
    i2c_conf.scl_io_num = GPIO_NUM_18;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = 100000;
    i2c_conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, ESP_INTR_FLAG_SHARED);

    // xTaskCreatePinnedToCore(pid_task, "PID Task", 2048, NULL, 2, NULL, 0);
    // xTaskCreatePinnedToCore(control_task, "Control Task", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(display_task, "Display Task", 2048, NULL, 3, NULL, 1);
}