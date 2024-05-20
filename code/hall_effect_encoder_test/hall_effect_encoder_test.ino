#include <LiquidCrystal_I2C.h>
#include <wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);
  Serial.println("begin!");
  lcd.init();
  lcd.backlight();
}

void loop() {
  Wire.begin();
  delay(250);
  lcd.clear();
  int angle =map(analogRead(A1),  969, 0, 0, 360)-261;
  String output;
  if (angle < 0) {
    angle += 360;
  }else if (angle >360){
    angle +=360;
  }
  if (angle<100){
    output = "0"+(String)(angle);
  }else{
    output=(String)(angle);
  }
  Serial.println(output+" degrees");































}
