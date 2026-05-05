#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver driver1;

int offset[3] = {90, 90, 90}; //coxia, femur and tibia in that order
int pins[3] = {0,1,2}; /////////////////////////////////////////////////////////pins (coxa, femur, tibia);
int driverNumber = 2; //will be taken as input from skeleton but is temporarily 1 (default). other options is 2


bool isKilled = false;

void emergencyStop() {
    // Στελνω το σημα 4096 σε ολα τα ντραιβερς για να τα κανει κιλ
    for (int i = 0; i < 16; i++) {
        driver1.setPWM(i, 4096, 0);   //το αντικείμενο μας ειναι απο την κλάση
    }
    isKilled = true;
    Serial.println("!!! PROGRAM KILLING !!!");  //Μηνυμα στον σειριακο
}

class move{
  private:
    // Μήκη των μελών σε cm
    const float L1 = 9.0;
    const float L2 = 15.0;

    // Ρυθμίσεις Servo (MG996R)
    const int SERVO_MIN = 150; 
    const int SERVO_MAX = 600;
    int offset[3];
    int driverNumber;


    int femurPin, tibiaPin, coxaPin;
    float targetDistance;

  public:
    move(int *pins, int *offset, int driverNumber);
    
    void raiseLeg(float x, float y);
    void walk();
};

move::move(int * pins, int *offset, int driverNumber) {
  // Αρχικοποίηση pins
  this->coxaPin = pins[0];
  this->femurPin = pins[1];
  this->tibiaPin = pins[2];

  //Αρχικοποίηση offset ανά pin

  for(int i = 0; i<3; i++){
    this->offset[i] = offset[i];
  }


  // Αρχικοποίηση του πρώτου driver



  //set initial position to 90 degrees
  // int n = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
  // driver1.setPWM(0, 0, n);


}



// Η βασική συνάρτηση Inverse Kinematics για 2 αρθρωσεις 

void move::raiseLeg(float x, float y) {
    
  targetDistance = sqrt(pow(x, 2) + pow(y, 2));

  // Έλεγχος ορίων (Constraints)

  if (targetDistance > (L1 + L2)) 
    targetDistance = L1 + L2;

  if (targetDistance < abs(L2 - L1)) 
    targetDistance = abs(L2 - L1);

  // Νόμος των Συνημιτόνων για τη γωνία του Γονάτου (Tibia)
  // cos(B) = (L1^2 + L2^2 - D^2) / (2 * L1 * L2)

  float cosB = (pow(L1, 2) + pow(L2, 2) - pow(targetDistance, 2)) / (2 * L1 * L2);
  float angleB_rad = acos(cosB);
  float angleB_deg = angleB_rad * 180.0 / PI;

  //Νόμος των Συνημιτόνων για τη γωνία του Μηρού (Femur)
  // Βρίσκουμε την εσωτερική γωνία του τριγώνου
  float cosA = (pow(L1, 2) + pow(targetDistance, 2) - pow(L2, 2)) / (2 * L1 * targetDistance);
  float angleA_rad = acos(cosA);
  float angleA_deg = angleA_rad * 180.0 / PI;
  
  //Μετατροπή των μοιρών σε PWM values
  //Εδω προσθετω και τα αντιστοιχα Offset 
  int pwmFemur = map(angleA_deg, 0, 180, SERVO_MIN, SERVO_MAX);////////////////////////////////////////////
  int pwmTibia = map(angleB_deg, 0, 180, SERVO_MIN, SERVO_MAX);////////////////////////////////////////////

  //Εντολή στον Driver
  
  driver1.setPWM(tibiaPin, 0, pwmTibia);
  delay(300);
  driver1.setPWM(femurPin, 0, pwmFemur);

  Serial.print("Target: "); Serial.print(targetDistance);
  Serial.print("cm -> Femur: "); Serial.print(angleA_deg);
  Serial.print(" deg, Tibia: "); Serial.println(angleB_deg);
}

move *leg = new move(pins, offset, 1);

void move::walk(){ //Βασική κίνηση περπατήματος 

  this->raiseLeg(10,8);
  Serial.print("1");
  delay(500);

  int coxaAngle = map(offset[0] + 20, 0, 180, SERVO_MIN, SERVO_MAX);
  driver1.setPWM(coxaPin, 0,  coxaAngle);
  Serial.print("2");
  delay(500);


  this->raiseLeg(16,2);
  Serial.print("3");
  delay(500);

  coxaAngle = map(offset[0] - 20, 0, 180, SERVO_MIN, SERVO_MAX);
  driver1.setPWM(coxaPin, 0, coxaAngle);
  Serial.print("4");
}

void setup(){
  //const int offset[3] = {90, 90, 90}; //coxia, femur and tibia in that order
  //const int pins[3] = {0,1,2}; /////////////////////////////////////////////////////////pins (coxa, femur, tibia);
  //const int driverNumber = 2; //will be taken as input from skeleton but is temporarily 1 (default). other options is 2
  Wire.begin(6, 7);
  Serial.begin(9600);
  driver1 = Adafruit_PWMServoDriver(0x40);    //I think the program doesn't run with this outside setup right now, but it is worth testing it
  driver1.begin();
  //50hz is mg996r(this servo)'s frequency
  driver1.setPWMFreq(50);
  // Αρχικοποίηση του πρώτου driver
}


//#define class move leg = new move(pins, offset, driverNumber);

void loop() {

  // Έλεγχος πληκτρολογίου ΑΝ ΠΑΤΗΘΕΙ ΤΟ S ΣΤΑΜΑΤΑΕΙ
  if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
          emergencyStop();
      }
  }

  //αν δεν εχει πατηθεί το κουμπί
  if (!isKilled) {
      leg->walk();
      delay(300);
  } else {
      // Αν ενεργοποιηθεί, ο κώδικας σταματά εδώ
      delay(100);
  }
}
