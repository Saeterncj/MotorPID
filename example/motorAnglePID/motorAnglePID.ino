#include <QuadratureEncoder.h>
#include <MX1508.h>
#include <MotorPID.h>

#define COUNTS_PER_REV 2527.08  // This includes gear ratio at 210.59 and 12 encoder cts/ motor rev.
#define DEGREES_PER_REV 360
#define MILLISEC_2_SEC 1000
#define ENCODERPIN_A 2
#define ENCODERPIN_B 3
#define IN3_RIGHT_PIN   9
#define IN4_RIGHT_PIN   10
#define OUTPUT_MAX    255
#define OUTPUT_MIN    -255

#define KP_ANGLE  2.025
#define KI_ANGLE  0.0
#define KD_ANGLE  4

Encoders myEncoder(2,3);
MX1508 myMotorDriver(9,10,FAST_DECAY,2);

double desiredAngle = -100, actualAngle;
int pwmAngle;
motorPID anglePID(&desiredAngle, &actualAngle, &pwmAngle, OUTPUT_MIN, OUTPUT_MAX, KP_ANGLE, KI_ANGLE, KD_ANGLE);

int findMotorDeadBand() {
  myEncoder.setEncoderCount(0L);
  int pwmVal = 1;
  long start = millis();
  while (myEncoder.getEncoderCount() == 0) {
    myMotorDriver.motorGo(pwmVal);
    if (millis() - start > 40) {
      start = millis();
      pwmVal++;
    }
  }
  Serial.print("The value for Deadband is: ");
  Serial.println(pwmVal);
  return pwmVal;
}

void setup() {
  Serial.begin(115200);
  int deadband = findMotorDeadBand();
  anglePID.setDeadBandOffset(deadband, false);
  delay(1000);
}

void loop() {
  static unsigned long lastMilli = 0;
  static bool cwDirection = true; // assume initial direction(positive pwm) is clockwise
  static int incremental = 1;

  // Desired Speed goes ramps up to 100rpm then ramps down to -100rpm
  if(millis()-lastMilli > 10){ // every 50 millisecond
      if (cwDirection && incremental++ > 100 ) {  
        cwDirection = false;
        desiredAngle = incremental;
      } else if (!cwDirection && incremental-- < -100) {
        cwDirection =  true;
        desiredAngle = incremental;
      } 
      //desiredAngle = incremental;
      lastMilli = millis();
  }
  pidMotorAngle();
}
void pidMotorAngle(){
  static unsigned long lastMilli = 0;
  unsigned long dt = millis()-lastMilli;
  if( dt > 50){
    // calculate Angle every 15 millisecond
    long currentEncoderCount = myEncoder.getEncoderCount();
    actualAngle = ((double)currentEncoderCount / COUNTS_PER_REV) * DEGREES_PER_REV; // in cts/sec 1000*60/dt/2527.08 
    anglePID.compute();
    Serial.print(desiredAngle);
    Serial.print(" , ");
    Serial.println(actualAngle);
    lastMilli = millis();
  }
  myMotorDriver.motorGo(pwmAngle);
}