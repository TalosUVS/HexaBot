#include <Servo.h>
#include <math.h>
#define KNEE_LENGTH 9
#define FOOT_LENGTH 15

class Leg
{
  private:
    Servo hip, knee, ankle;
    int HipPin, KneePin, AnklePin;
    int hipOffset = 30, kneeOffset = 50, ankleOffset = 30;
  
  public:
    Leg(int, int, int);
    void move(float, float);

};


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

Leg :: Leg(int HipPin, int kneePin, int AnklePin)
{
  this.HipPin = HipPin;     //pins on board for each
  this.KneePin = KneePin;
  this.AnklePin = AnklePin;
}

void Leg :: move(int x, int z){
  float dist;     //distance
  dist = sqrt(x*x + z*z);
  dist = constrain(dist, 6.0, 24);

  //Tibia angle

  float D = (dist * dist - KNEE_LENGTH * KNEE_LENGTH - FOOT_LENGTH * FOOT_LENGTH) / (2 * KNEE_LENGTH * FOOT_LENGTH);
  D = constrain(D, -1.0, 1.0);
  float radianAnkleAngle = acos(D);
  float ankleAngle = radianAnkleAngle * 180.0 / PI;      //knee angle adjusted to degrees from radians

  //Femur angle

  float radiankneeAngle = atan2(z, x) - atan2(FOOT_LENGTH * sin(radianKneeAngle), FOOT_LENGTH + KNEE_LENGTH * cos(radianKneeAngle));
  float kneeAngle = radianKneeAngle * 180.0 / PI;

  //write to servos
  knee.write(kneeOffset + KneeAngle);
  ankle.write(ankleOffset - tibiaDeg);
  


}


