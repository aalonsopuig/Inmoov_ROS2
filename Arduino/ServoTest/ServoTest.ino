#include <Servo.h>
Servo myServo;

void setup() {
  myServo.attach(2);  

}

void loop() {
  // put your main code here, to run repeatedly:
  myServo.write(90);
  delay(1000);
  myServo.write(0);
  delay(1000);
}
