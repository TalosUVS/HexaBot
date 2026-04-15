#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define COXA_ANGLE = 10;    //temporary safe value so that legs don't hit each other. Adjust depending on lab results

#define L1 9.0;         //Leg part lengths
#define L2 15.0;

#define SERVO_MIN = 150;  //angle adjustment variables for servo safety
#define SERVO_MAX 600;

#define LEGPINS 3;      //number used for skeleton initialization (servos/leg)

#define FRONT 1;
#define BACK 0;

#define GROUPA = 1;
#define GROUPB = 2;

int offsetA[9] = {90, 90, 90, 90, 90, 90, 90, 90, 90};  //leg1: coxia, femur and tibia, leg2: coxia, femur and tibia... in that order      GROUP 1
int offsetB[9] = {90, 90, 90, 90, 90, 90, 90, 90, 90};  //GROUP 2
int pinsA[9] = {0,1,2,3,4,5,6,7,8};   //pins in the same order as above GROUP 1
int pinsB[9] = {0,1,2,3,4,5,6,7,8};   //GROUP 2


Adafruit_PWMServoDriver driver1, driver2;

driver1 = Adafruit_PWMServoDriver(0x40);    //temporarily here, might move to setup if it doesn't work
driver2 = Adafruit_PWMServoDriver(0x41);

bool isKilled = false;

void emergencyStop() {
    // Στελνω το σημα 4096 σε ολα τα ντραιβερς για να τα κανει κιλ
    for (int i = 0; i < 16; i++) {
      driver1.setPWM(i, 4096, 0);   //το αντικείμενο μας ειναι απο την κλάση
      driver2.setPWM(i, 4096, 0);
    }
    isKilled = true;
    Serial.println("!!! PROGRAM KILLING !!!");  //Μηνυμα στον σειριακο
}

class leg{
  private:
    int offset[3];
    Adafruit_PWMServoDriver driver;

    int femurPin, tibiaPin, coxaPin;
    float targetDistance;

  public:
    leg(int *pins, int *offset, Adafruit_PWMServoDriver driver);
    
    void raiseLeg(float x, float y);
    void walkleg();
    void coxaMove(int dir);
};

leg::leg(int * pins, int *offset, Adafruit_PWMServoDriver driver){
  // Αρχικοποίηση pins
  this->coxaPin = pins[0];
  this->femurPin = pins[1];
  this->tibiaPin = pins[2];

  //Αρχικοποίηση offset ανά pin

  for(int i = 0; i<3; i++){
    this->offset[i] = offset[i];
  }

  this->driver = driver;  

  //set initial position to 90 degrees
  // int n = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
  // driver.setPWM(0, 0, n);
}

void leg::coxaMove(int dir){
  if (dir == FRONT){
    int coxaAngle = map(offset[0] + COXA_ANGLE, 0, 180, SERVO_MIN, SERVO_MAX);
    driver.setPWM(coxaPin, 0,  coxaAngle);
  }else if(dir == BACK){
    int coxaAngle = map(offset[0] - COXA_ANGLE, 0, 180, SERVO_MIN, SERVO_MAX);
    driver.setPWM(coxaPin, 0,  coxaAngle);
  }
}

// Η βασική συνάρτηση Inverse Kinematics για 2 αρθρωσεις 

void leg::raiseLeg(float x, float y) {
    
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
  int pwmFemur = map(angleA_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  int pwmTibia = map(angleB_deg, 0, 180, SERVO_MIN, SERVO_MAX);

  //Εντολή στον Driver
  
  driver.setPWM(tibiaPin, 0, pwmTibia);
  delay(300);
  driver.setPWM(femurPin, 0, pwmFemur);

  Serial.print("Target: "); Serial.print(targetDistance);       //debugging print
  Serial.print("cm -> Femur: "); Serial.print(angleA_deg);
  Serial.print(" deg, Tibia: "); Serial.println(angleB_deg);
}


void leg::walkLeg(){ //Βασική κίνηση περπατήματος 

  this->raiseLeg(10,8);
  Serial.print("1");
  delay(500);

  int coxaAngle = map(offset[0] + COXA_ANGLE, 0, 180, SERVO_MIN, SERVO_MAX);
  driver.setPWM(coxaPin, 0,  coxaAngle);
  Serial.print("2");
  delay(500);


  this->raiseLeg(16,2);
  Serial.print("3");
  delay(500);

  coxaAngle = map(offset[0] - COXA_ANGLE, 0, 180, SERVO_MIN, SERVO_MAX);
  driver.setPWM(coxaPin, 0, coxaAngle);
  Serial.print("4");
}

class Hexabot{
  private:
    leg *legA1, *legA2;   //leg layout (looks like hexabot if looked at from the top)
    leg *legB1, *legB2;
    leg *legC1, *legC2;

    public:
      Hexabot();
      void hexaWalk());
};

Hexabot:Hexabot(){
  legA1 = new leg(&pinsA[0], &offsetA[0], driver1);    //initialize GroupA
  legB2 = new leg(&pinsA[LEGPINS], &offsetA[LEGPINS], driver1);
  legC1 = new leg(&pinsA[2*LEGPINS], &offsetA[2*LEGPINS], driver1);

  legA2 = new leg(&pinsB[0], &offsetB[0], driver2);    //initialize GroupB
  legB1 = new leg(&pinsB[LEGPINS], &offsetB[LEGPINS], driver2);
  legC2 = new leg(&pinsB[2*LEGPINS], &offsetB[2*LEGPINS], driver2);
}

void Hexabot:hexaWalk(){
  
  legA2->raiseLeg(16,2);  //lower groupB
  legB1->raiseLeg(16,2);
  legC2->raiseLeg(16,2);
  delay(250);
  legA1->raiseLeg(10,8);  //raise groupA
  legB2->raiseLeg(10,8);
  legC1->raiseLeg(10,8);
  Serial.print("1");
  delay(250);

  legA1->coxaMove(FRONT); //send groupA front, groupB back
  legB2->coxaMove(FRONT);
  legC1->coxaMove(FRONT);
  legA2->coxaMove(BACK);
  legB1->coxaMove(BACK);
  legC2->coxaMove(BACK);
  Serial.print("2");
  delay(500);

  legA1->raiseLeg(16,2);  //lower groupA
  legB2->raiseLeg(16,2);
  legC1->raiseLeg(16,2);
  delay(250);
  legA2->raiseLeg(10,8);  //raise groupB
  legB1->raiseLeg(10,8);
  legC2->raiseLeg(10,8);
  Serial.print("3");
  delay(250);

  legA1->coxaMove(BACK);  //send group A back, groupB front
  legB2->coxaMove(BACK);
  legC1->coxaMove(BACK);
  legA2->coxaMove(FRONT);
  legB1->coxaMove(FRONT);
  legC2->coxaMove(FRONT);
  Serial.print("4");
  delay(500);
}

void setup(){

  Wire.begin(6, 7);
  Serial.begin(9600);


  driver1.begin();
  driver2.begin();

  //50hz is mg996r(this servo)'s frequency
  driver1.setPWMFreq(50);
  driver2.setPWMFreq(50);
  // Αρχικοποίηση του πρώτου driver
}

Hexabot *hexa = new Hexabot();

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
      hexa->hexaWalk();
  } else {
      // Αν ενεργοποιηθεί, ο κώδικας σταματά εδώ
      delay(100);
  }
}