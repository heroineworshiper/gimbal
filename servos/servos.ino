// Camera controller for Arduino
// CAM yaw pitch
// CAM 66 124

// CAM 110 125

#include <Servo.h>

Servo yaw;
Servo pitch;

// 40 - 90
int yaw_angle = 66;
// 110 - 130
int pitch_angle = 124;

void setup() {
  Serial.begin(115200);
  Serial.println("Welcome to cam controller");
  Serial.setTimeout(1000);

  yaw.attach(2);
  pitch.attach(3);
  yaw.write(yaw_angle);
  pitch.write(pitch_angle);
}

void loop() {
  if(Serial.available())
  {
     String s = Serial.readStringUntil(' ');

     // servo command of the form "CAM 1 2\n" Important to have linefeed.
     if(s.equals("CAM"))
     {
        yaw_angle = Serial.parseInt();
        pitch_angle = Serial.parseInt();
        yaw.write(yaw_angle);
        pitch.write(pitch_angle);
// get the LF
		while(Serial.available())
		{
			Serial.read();
		}
        
        Serial.print("Got CAM ");
        Serial.print(yaw_angle);
        Serial.print(" ");
        Serial.print(pitch_angle);
        Serial.println();
     }
	 else
	 {
	 }
  }
  
  // put your main code here, to run repeatedly:
  int stick1 = analogRead(0);
  int stick2 = analogRead(1);

// joystick position of the form "JOY 123 456\n"
//  Serial.print("JOY: ");
//  Serial.print(stick1, HEX);
//  Serial.print(" ");
//  Serial.print(stick2, HEX);
//  Serial.println();

}
