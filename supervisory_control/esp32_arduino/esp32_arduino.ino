#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "fsm.h"

#define BUTTON_MODE   15
#define BUTTON_SHIFT  2
#define BUTTON_UP     4
#define POTENTIO      5

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// BUTTON INPUT SIGNAL
int button_left = 0, button_right = 0, button_mode = 0;

// DEBOUNCE SIGNAL
int count_deb_left = 0, count_deb_right = 0, count_deb_mode = 0;
int state_deb_left = 0, state_deb_right = 0, state_deb_mode = 0;
int deb_left = 0, deb_right = 0, deb_mode = 0;

// EDGE-DETECTOR SIGNAL
int state_edge_left = 0, state_edge_right = 0, state_edge_mode = 0;
int edge_left = 0, edge_right = 0, edge_mode = 0;

// HOLD-COUNTER SIGNAL
int count_hold_left = 0, count_hold_right = 0, count_hold_mode = 0;
int lim_alarm_left = 0, lim_alarm_right = 0, lim_alarm_mode = 0;
int out_hold_left = 0, out_hold_right = 0, out_hold_mode = 0;
int state_hold_left = 0, state_hold_right = 0, state_hold_mode = 0;

// FSM MODE SIGNAL
int state_mode = MODE_START;

// PARAMETER
float Kp = 1.0, Ki = 0.0, Kd = 0.0;

// PID VAR
float integral = 0;
float last_err = 0;

// SETPOINT (POTENTIO)
float setpoint = 0.0;

// COMMUNICATION WITH MOTOR
int next_valid = 0;
float motor_out = 0.0, pid_out = 0.0;

void pid(float input, float *output, float setpoint, float kp, float ki, float kd, float time){
  
    float error = setpoint - input;
    integral += error * time;
    float deriv = (last_err - error)/time;
    last_err = error;
    *output = kp * error + ki * integral + kd * deriv;
}


void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);
  pinMode(BUTTON_MODE, INPUT);
  pinMode(BUTTON_SHIFT, INPUT);
  pinMode(BUTTON_UP, INPUT);
  pinMode(POTENTIO, INPUT);
  
  // Task untuk button pada core 0
  xTaskCreatePinnedToCore(button_task, "Button Task", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(display_task, "Display Task", 2048, NULL, 1, NULL, 0);
    // Task untuk kontrol PID pada core 1
  xTaskCreatePinnedToCore(main_control, "Main Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(main_config, "Main Config Task", 2048, NULL, 1, NULL, 1);
}

void display_task(void *pvParam){
  while(1){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    switch(state_mode){
      case MODE_START:
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print("SETPOINT: ");
        break;

      case MODE_P:
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print("P: ");
        break;

      case MODE_I:
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print("I: ");
        break;
        
      case MODE_D:
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print("D: ");
        break;
    }
    vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
  }
}

void button_task(void *pvParam){
  while(1){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // timer_get_counter_time_sec(0, 0, &current_time_sec);
      //  if (gpio_get_level(GPIO_INPUT_PB) == 0 && (current_time_sec - last_time_sec > DELAY_S)) {
        //    button = 1;
        //    //start_program = 1;
    //      timer_get_counter_time_sec(0, 0, &last_time_sec);
      //  }

        if(digitalRead(BUTTON_SHIFT)==0){
            button_left = 1;
        }
        else{
            button_left = 0;
        }

        if(digitalRead(BUTTON_UP == 0)){
            button_right = 1;
        }
        else{
            button_right = 0;
        }

        if(digitalRead(BUTTON_MODE) == 0){
            button_mode = 1;
        }
        else{
            button_mode = 0;
        }

    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void main_control(void *pvParam){
  while(1){
    TickType_t xLastWakeTime1 = xTaskGetTickCount();

        // Membaca data dari desktop agar bisa lanjut
        while(Serial.available()>0){
          motor_out = Serial.read();
        }
//        if(scanf("%f", &motor_out) > 0){
//            next_valid = 1;
//        }

        // Dijalankan setelah pembacaan berhasil
        if(next_valid == 1 ){
            pid(motor_out, &pid_out, setpoint, Kp, Ki, Kd, 0.01);

            // Mengirimkan sel dan output_pid ke desktop
            Serial.println(pid_out);
//            Serial.print(
//            printf("%d;%.8f\r\n", sel, output_pid);
            
            next_valid = 0;
        }
        vTaskDelayUntil(&xLastWakeTime1, 30/portTICK_PERIOD_MS);
  }
}

void main_config(void *pvParam){
  while(1){
    TickType_t xLastWakeTime1 = xTaskGetTickCount();

    debounce(button_mode, &count_deb_mode, &state_deb_mode, &deb_mode);
    edge_detect(button_mode, deb_mode, &state_edge_mode, &edge_mode);
    hold_counter(edge_mode, MODE, 100, &count_hold_mode, &state_hold_mode, &out_hold_mode, &lim_alarm_mode);

    fsm_mode(lim_alarm_mode, lim_alarm_mode, &state_mode);
    
    vTaskDelayUntil(&xLastWakeTime1, 50/portTICK_PERIOD_MS);
  }
}


void loop() {
  // put your main code here, to run repeatedly:

}
