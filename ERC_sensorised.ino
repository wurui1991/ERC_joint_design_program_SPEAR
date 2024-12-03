/*
 Pin 3 -> potentiometer middle pin
 Please refer to the pinout in the design guide
*/

float Angle, Angle0, RawAngle;

void setup() {
  Serial.begin(115200);
  RawAngle = analogRead(A3)-512;
  // calibration equation determined from five potentiometers (Alps Alpine RK09K1130A70)
  Angle0 = 0.2097*RawAngle + 2.243E-6*RawAngle*RawAngle + 1.095E-7*RawAngle*RawAngle*RawAngle; 
}

void loop() {
  RawAngle = analogRead(A3)-512;
  // calibration equation determined from five potentiometers (Alps Alpine RK09K1130A70)
  Angle = 0.2097*RawAngle + 2.243E-6*RawAngle*RawAngle + 1.095E-7*RawAngle*RawAngle*RawAngle - Angle0;

// Output values
  Serial.print(Angle*2); // bending angle = pin angle * 2
  Serial.print(" ");
  Serial.println();  // Sends a newline character
}