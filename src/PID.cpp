#include <PID.h>

void initPID(PID &pid,double Kp, double Ki, double Kd, double limitIntegral, double minOutput=2, double maxOutput=12){
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.limitIntegral = limitIntegral;
    pid.minOutput = minOutput;
    pid.maxOutput = maxOutput;
    pid.error = 0;
    pid.prevError = 0;
    pid.firstRun = true;
}

void resetPID(PID &pid){
    pid.prevError = 0;
    pid.error = 0;
    pid.firstRun = true;
}

void setPIDmax(PID &pid, double maxOutput) {
    pid.maxOutput = maxOutput;
}

void setPIDmin(PID &pid, double minOutput){
    pid.minOutput = minOutput;
}

double calculatePID(PID &pid, double destination, double current){
    double totalGain = 0;
    double proportionalGain = 0;
    double integralGain = 0;
    double derivativeGain = 0;

    pid.error = destination - current;
     
    proportionalGain = pid.Kp * pid.error;
    
    if (pid.firstRun) {
       pid.prevError = pid.error;
       pid.firstRun = false;
    }

    if(pid.error < pid.limitIntegral){
        pid.integral = pid.integral + pid.error;
    }
    else{
        pid.integral = 0;
    }
    integralGain = pid.Ki * pid.integral;
    
    derivativeGain = (pid.error - pid.prevError) * pid.Kd;
    pid.prevError = pid.error;

    totalGain = proportionalGain + integralGain + derivativeGain;
    
    if (totalGain > pid.maxOutput) {
        totalGain = pid.maxOutput;
    }
    if (totalGain < pid.minOutput) {
        totalGain = pid.minOutput;
    }    
    return totalGain;
}