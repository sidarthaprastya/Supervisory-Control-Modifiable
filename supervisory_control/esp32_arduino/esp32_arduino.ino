#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#include "fsm.h"

#define PIN_BUTTON_MODE   15
#define PIN_BUTTON_SHIFT  2
#define PIN_BUTTON_ADD    5
#define PIN_POTENTIO      4

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// BUTTON INPUT SIGNAL
int button_mode = 0, button_shift = 0, button_add = 0;

// DEBOUNCER SIGNAL
int count_deb_shift = 0, count_deb_add = 0, count_deb_mode = 0;
int state_deb_shift = STATE_DETECT, state_deb_add = STATE_DETECT, state_deb_mode = STATE_DETECT;
int deb_shift = 0, deb_add = 0, deb_mode = 0;

// FSM_MODE SIGNAL
int state_mode = MODE_START;
int counter_mode = 0;

// FSM_NUM_SHIFT SIGNAL
int state_num_shift = BLOCK_1;

// PARAMETER
float Kp, Ki, Kd;

// NUM_ADD SIGNAL
int num_block_p[4] = {0, 0, 0, 0};
int num_block_i[4] = {0, 0, 0, 0};
int num_block_d[4] = {0, 0, 0, 0};

// PID VAR
float integral = 0;
float last_err = 0;

// SETPOINT (POTENTIO)
float setpoint = 0.0;

// COMMUNICATION WITH MOTOR
int next_valid = 0;
float motor_out = 0.0, pid_out = 0.0;
String buff;
int available = 0;

void pid(float input, float *output, float setpoint, float kp, float ki, float kd, float time){
  
    float error = setpoint - input;
    integral += error * time;
    float deriv = (last_err - error)/time;
    last_err = error;
    *output = kp * error + ki * integral + kd * deriv;

}

void read_eeprom(){
  Kp = EEPROM.read(0);
  Ki = EEPROM.read(1);
  Kd = EEPROM.read(2);

  for(int z=0; z<4; z++){
    num_block_p[z] = EEPROM.read(3+z);
  }
  for(int z=0; z<4; z++){
    num_block_i[z] = EEPROM.read(7+z);
  }
  for(int z=0; z<4; z++){
    num_block_d[z] = EEPROM.read(11+z);
  }
}

void write_eeprom(){
  EEPROM.writeFloat(0, Kp);
  EEPROM.writeFloat(1, Ki);
  EEPROM.writeFloat(2, Kd);

  for(int z=0; z<4; z++){
    EEPROM.write(3+z, num_block_p[z]);
  }

  for(int z=0; z<4; z++){
    EEPROM.write(7+z, num_block_i[z]);
  }

  for(int z=0; z<4; z++){
    EEPROM.write(11+z, num_block_d[z]);
  }
  EEPROM.commit();
}

void setup() {
  EEPROM.begin(64);
  read_eeprom();
  param_assign(num_block_p, num_block_i, num_block_d, &Kp, &Ki, &Kd);
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);
  pinMode(PIN_BUTTON_MODE, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SHIFT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_ADD, INPUT_PULLUP);
  pinMode(PIN_POTENTIO, INPUT);
  
  // Task untuk button dan display pada core 0
  xTaskCreatePinnedToCore(button_task, "Button Task", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(display_task, "Display Task", 2048, NULL, 1, NULL, 0);
    // Task untuk kontrol PID pada core 1
  xTaskCreatePinnedToCore(main_control, "Main Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(main_config, "Main Config Task", 2048, NULL, 2, NULL, 0);
}

void animate_shift(int mode, int shift_block, int num_block[4]){
    int y = 3;
    int dot_pos;
    switch(mode){
        case MODE_P:
            dot_pos = 3;
            break;
        
        case MODE_I:
            dot_pos = 2;
            break;
        
        case MODE_D:
            dot_pos = 2;
            break;
        
        default:
            break;
    }
    for(int x = 0; x < 5; x++){
        lcd.setCursor(6 + x, 0);
        if(x == dot_pos){
            lcd.print(".");
        }
        else{
            lcd.print(num_block[y]);
            if(y == shift_block){
                lcd.setCursor(6 + x, 1);
                lcd.print("^");
            }
            y--;
        }
    }
}

void display_task(void *pvParam){
  while(1){
    TickType_t xLastWakeTime = xTaskGetTickCount();

    switch(state_mode){
      case MODE_START:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("SET: ");
        lcd.setCursor(6,0);
        lcd.print(setpoint);
        break;

      case MODE_P:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("P: ");
        animate_shift(MODE_P, state_num_shift, num_block_p);
        break;

      case MODE_I:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("I: ");
        animate_shift(MODE_I, state_num_shift, num_block_i);
        break;
        
      case MODE_D:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("D: ");
        animate_shift(MODE_D, state_num_shift, num_block_d);
        break;
      
      default:
        break;
    }
    
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void button_task(void *pvParam){
  while(1){
    TickType_t xLastWakeTime = xTaskGetTickCount();
        if(digitalRead(PIN_BUTTON_SHIFT)==0){
            button_shift = READ_SHIFT;
        }
        else{
            button_shift = 0;
        }

        if(digitalRead(PIN_BUTTON_ADD) == 0){
            button_add = READ_ADD;
        }
        else{
            button_add = 0;
        }

        if(digitalRead(PIN_BUTTON_MODE) == 0){
            button_mode = READ_MODE;
        }
        else{
            button_mode = 0;
        }

        setpoint = map(analogRead(PIN_POTENTIO), 200, 4095, 0, 100);

    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

void main_control(void *pvParam){
  while(1){
    TickType_t xLastWakeTime1 = xTaskGetTickCount();
      
      while(Serial.available()>0){
        buff = Serial.readStringUntil(';');

        available = 1;
        Serial.flush();
      }
      if (available == 1){
        motor_out = buff.toFloat();
        // Dijalankan setelah pembacaan berhasil
        pid(motor_out, &pid_out, setpoint, Kp, Ki, Kd, 0.01);
        if(state_mode != MODE_START){
          setpoint = 0;
          pid_out = 0;
        }
        // Mengirimkan output_pid ke desktop
        Serial.print(setpoint);
        Serial.print(";");
        Serial.println(pid_out);
        
        available = 0;
      }
      
       vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

void main_config(void *pvParam){
  while(1){
    TickType_t xLastWakeTime1 = xTaskGetTickCount();

    debouncer_fsm(button_mode, 20, 20, START_FLAG, &count_deb_mode, &state_deb_mode, &deb_mode);
    debouncer_fsm(button_shift, 10, 2, READ_SHIFT, &count_deb_shift, &state_deb_shift, &deb_shift);
    debouncer_fsm(button_add, 10, 2, READ_ADD, &count_deb_add, &state_deb_add, &deb_add);

    fsm_mode(deb_mode, &state_mode);
    if(state_mode != MODE_START){
      fsm_num_shift(deb_shift, &state_num_shift);
      switch(state_mode){
        case MODE_P:
          num_add(deb_add, state_num_shift, num_block_p);
          break;
        
        case MODE_I:
          num_add(deb_add, state_num_shift, num_block_i);
          break;
        
        case MODE_D:
          num_add(deb_add, state_num_shift, num_block_d);
          break;
        
        default:
          break;
      }
    }

    if(deb_mode == START_FLAG){
      param_assign(num_block_p, num_block_i, num_block_d, &Kp, &Ki, &Kd);
      write_eeprom();
      
    }

    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}


void loop() {
  // put your main code here, to run repeatedly:

}
