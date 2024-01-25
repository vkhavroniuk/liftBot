struct PID{
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double prevError;
    double limitIntegral;
    double maxOutput;
    double minOutput;
    double firstRun;
};

void initPID(PID &pid, double Kp, double Ki, double Kd, double limitIntegral, double minOutput, double maxOutput);
void setPIDmax(PID &pid, double maxOutput);
void setPIDmin(PID &pid, double minOutput);
double calculatePID(PID &pid, double destination, double current);
void resetPID(PID &pid);