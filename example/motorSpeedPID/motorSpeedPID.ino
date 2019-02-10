#include <QuadratureEncoder.h>
#include <MX1508.h>
#include <MotorPID.h>

//#define EI_ARDUINO_INTERRUPTED_PIN 
//#include <EnableInterrupt.h>
#define COUNTS_PER_REV 2527.08  // This includes gear ratio at 210.59 and 12 encoder cts/ motor rev.
#define DEGREES_PER_REV 360
#define MILLISEC_2_SEC 1000
#define SEC_2_MIN     60
#define ENCODERPIN_A 2
#define ENCODERPIN_B 3
#define IN3_RIGHT_PIN   9
#define IN4_RIGHT_PIN   10
#define OUTPUT_MAX    255
#define OUTPUT_MIN    -255
// 0.585 , 0.171, 0
#define KP_SPEED  0.585
#define KI_SPEED  0.171
#define KD_SPEED  0.0


Encoders myEncoder(2,3);
MX1508 myMotorDriver(9,10,FAST_DECAY,2);

double desiredSpeed = 0, actualSpeed;
int pwmSpeed;
motorPID speedPID(&desiredSpeed, &actualSpeed, &pwmSpeed, OUTPUT_MIN, OUTPUT_MAX, KP_SPEED, KI_SPEED, KD_SPEED);

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
  speedPID.setDeadBandOffset(deadband, true);
  delay(1000);
}


void loop() {  
  static unsigned long lastMilli = 0;
  static bool cwDirection = true; // assume initial direction(positive pwm) is clockwise
  static int incremental = 1;

  // Desired Speed goes ramps up to 100rpm then ramps down to -100rpm
  if(millis()-lastMilli > 20){ // every 50 millisecond
      if (cwDirection && incremental++ > 100 ) {  
        cwDirection = false;
        //desiredSpeed = incremental;
      } else if (!cwDirection && incremental-- < -100) {
        cwDirection =  true;
        //desiredSpeed = incremental;
      } 
      desiredSpeed = incremental;
      lastMilli = millis();
  }
  pidMotorSpeed();
}


void pidMotorSpeed(){
  static unsigned long lastMilli = 0;
  static long lastEncoderCount = 0;
  unsigned long dt = millis()-lastMilli;
  if( dt > 50){
    // calculate speed every 50 millisecond
    long currentEncoderCount = myEncoder.getEncoderCount();
    long diff = currentEncoderCount - lastEncoderCount;
    lastEncoderCount = currentEncoderCount; 
    actualSpeed = (double)diff*MILLISEC_2_SEC*SEC_2_MIN/dt/COUNTS_PER_REV   ; // in cts/sec 1000*60/dt/2527.08 
    speedPID.compute();
//    Serial.println(pwmSpeed);
//    Serial.print(" , ");    
    Serial.print(desiredSpeed);
    Serial.print(" , ");
    Serial.println(actualSpeed);
    lastMilli = millis();
    myMotorDriver.motorGo(pwmSpeed);
  }
}