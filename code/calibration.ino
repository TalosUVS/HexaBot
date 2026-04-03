#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Ρυθμίσεις για MG996R
#define SERVOMIN  150 
#define SERVOMAX  600 

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  Serial.println("--- MG996R Control Ready ---");
  Serial.println("Grapse moires (0-180) kai pata Enter:");

  Wire.begin(6, 7); // Pins για ESP32-C6
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);
}

void loop() {
  // Ελέγχει αν έχεις γράψει κάτι στο Serial Monitor
  if (Serial.available() > 0) {
    // Διαβάζει τον αριθμό που έγραψες
    int moires = Serial.parseInt();

    // Αν ο αριθμός είναι έγκυρος (0-180)
    if (moires >= 0 && moires <= 180) {
      int pulse = map(moires, 0, 180, SERVOMIN, SERVOMAX);
      
      Serial.print("Kinisi stis: ");
      Serial.print(moires);
      Serial.println(" moires.");

      // Στέλνει την εντολή στο κανάλι 0
      pwm.setPWM(0, 0, pulse);
    } 
    // Καθαρισμός του buffer για την επόμενη είσοδο
    while(Serial.available() > 0) { Serial.read(); }
  }
}