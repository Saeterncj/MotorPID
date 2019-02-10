#include "MotorPID.h"
   

motorPID:: motorPID(double *setPoint, double *measuredFeedback,int *pwmOutput, int outputMin, int outputMax, double Kp, double Ki, double Kd ){
  _setPoint = setPoint;
  _measuredFeedback = measuredFeedback;
  _pwmOutput = pwmOutput;
  _outputMin = outputMin;
  _outputMax = outputMax;
  changeGains(Kp,Ki,Kd);
}

void motorPID::changeGains(double kp, double ki, double kd ){
  _Kp = kp;
  _Ki = ki;
  _Kd = kd;
}

void motorPID::changeSamplingTime(int samplingTime){
  _samplingTime = samplingTime;
}

void motorPID::setDeadBandOffset(int deadband, bool isSpeedPID){
  _deadBandOffset = deadband;
  _isSpeedPID = isSpeedPID;
}
// PID ouput= Kp*errorP + Ki*errorI + Ki*errorD
void motorPID::compute(){
  unsigned long _dT = millis()- _lastMilli;
  if( _dT >= _samplingTime){
    double _porportionalError = *_setPoint - *_measuredFeedback;
    //to prevent derivative kick
    double _derivativeError = (_prevMeasuredFeedback - *_measuredFeedback);
    _prevMeasuredFeedback = *_measuredFeedback;
    
    // prevent integral wind up using clamping method!
    if(*_pwmOutput < _outputMax && *_pwmOutput > _outputMin){
//      _integralError += (_porportionalError + _prevError)/2;
        _integralError +=_porportionalError;
    }
    _prevError = _porportionalError;

    *_pwmOutput = _Kp*_porportionalError + _Ki*_integralError + _Kd*_derivativeError;
    
    // Allow user to define to use Deadband or not

    // Deadband needs to  be changed based on speed controller or positional controller
    if(_isSpeedPID){
      if(*_setPoint > 0 ){
        *_pwmOutput += _deadBandOffset;
      }else {
        *_pwmOutput -= _deadBandOffset;
      }
    }else{
      if(_porportionalError > 0 ){
        *_pwmOutput += _deadBandOffset;
      }else {
        *_pwmOutput -= _deadBandOffset;
      }
    }
    
    
    
    
    // Saturation
    if(*_pwmOutput > _outputMax) *_pwmOutput = _outputMax;
    else if(*_pwmOutput < _outputMin) *_pwmOutput = _outputMin;
    _lastMilli = millis();
  }
}
