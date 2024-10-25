/******************************************
457 Mechatronics Lab
Servo Lab
James DeMarco

Program to control a servo using an Arduino Nano, stepdown voltage converter, 9V battery
and MG 996R servo motor

*****************************************/

# include <Servo.h>
Servo servo0;
Servo servo1;
Servo servo2;

int ypr[3] = {43, 24, 86};

int servo0Value = map(ypr[0], -90, 90, 0, 180);
int servo1Value = map(ypr[1], -90, 90, 0, 180);
int servo2Value = map(ypr[2], -90, 90, 180, 0);

int pos =0;


void setup() {
  servo0.attach(9);                        // attach object myservo to pin 9
  servo1.attach(6);
  servo2.attach(5);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { 
    servo0.write(servo0Value);
    servo1.write(servo1Value);
    servo2.write(servo2Value);


    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    servo0.write(servo0Value);
    servo1.write(servo1Value);
    servo2.write(servo2Value);

    delay(15);
  }
}
