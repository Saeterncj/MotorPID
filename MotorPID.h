#ifndef motorPID_h
#define motorPID_h

#include "Arduino.h"

class motorPID{
  private:
    double _Kp, _Ki, _Kd;
    double *_setPoint, *_measuredFeedback;
    double  _integralError = 0; 
    double _prevMeasuredFeedback, _prevError;
    int _deadBandOffset = 0;
    int *_pwmOutput;
    int _samplingTime = 0; 
    int _outputMin, _outputMax;
    bool _isSpeedPID = true;
    unsigned long _lastMilli = 0;
    
  public:
    motorPID(double *setPoint, double *measuredFeedback,int *pwmOutput, int outputMin, int outputMax, double Kp, double Ki, double Kd );
    void setDeadBandOffset(int deadBandOffset, bool isSpeedPID);
    void compute();
    void changeGains(double kp, double ki, double kd ); 
    void changeSamplingTime(int samplingTime);
};


#endif
