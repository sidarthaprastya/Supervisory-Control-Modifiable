#include <SoftwareSerial.h>

SoftwareSerial portOne(10,11);
float x[2] = {0.0, 0.0};
float y[2] = {0.0, 0.0};
int next = 0;

void motor(float *input, float *output){
  output[1] = output[0];
  output[0] = 0.00413 * input[0] + 0.00413 * input[1] + 0.992 * output[1];
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  portOne.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  portOne.listen();
  
  while(portOne.available() > 0){
    float input = portOne.parseFloat(SKIP_ALL, '\n');

    x[1] = x[0];
    x[0] = input;
    motor(x, y);
    next = 1;
  }
  
  if(next == 1){
    portOne.println(y[0]);
  }
  
}
