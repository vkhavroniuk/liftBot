#include <PID.h>

void initPID(PID &pid,double Kp, double Ki, double Kd, double limitIntegral, double minOutput=2, double maxOutput=12){
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.limitIntegral = limitIntegral;
    pid.minOutput = minOutput;
    pid.maxOutput = maxOutput;
    pid.prevError = 0;
    pid.firstRun = true;
}

void resetPID(PID &pid){
    pid.prevError = 0;
    pid.firstRun = true;
}

void setPIDmax(PID &pid, double maxOutput) {
    pid.maxOutput = maxOutput;
}

void setPIDmin(PID &pid, double minOutput){
    pid.minOutput = minOutput;
}

double calculatePID(PID &pid, double destination, double current){
    double error;
    double totalGain = 0;
    double proportionalGain = 0;
    double integralGain = 0;
    double derivativeGain = 0;

    error = destination - current;
    proportionalGain = pid.Kp * error;
    
    if (pid.firstRun) {
       pid.prevError = error;
       pid.firstRun = false;
    }

    if(error < pid.limitIntegral){
        pid.integral = pid.integral + error;
    }
    else{
        pid.integral = 0;
    }
    integralGain = pid.Ki * pid.integral;
    
    derivativeGain = (error - pid.prevError) * pid.Kd;
    pid.prevError = error;

    totalGain = proportionalGain + integralGain + derivativeGain;
    
    if (totalGain > pid.maxOutput) {
        totalGain = pid.maxOutput;
    }
    if (totalGain < pid.minOutput) {
        totalGain = pid.minOutput;
    }    
    return totalGain;
}