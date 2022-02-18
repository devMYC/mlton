struct _GC_PIDController {
    double Kp;
    double Ki;
    double Kd;
    double setpoint;
    double prevErr;
    double prevI;
    double prevD;
    double prevOutput;
};

GC_PIDController
new_controller(double Kp, double Ki, double Kd, double setpoint)
{
    GC_PIDController ctrlr = (GC_PIDController)malloc(sizeof(struct _GC_PIDController)); 
    ctrlr->Kp = Kp;
    ctrlr->Ki = Ki;
    ctrlr->Kd = Kd;
    ctrlr->setpoint = setpoint;
    ctrlr->prevErr = 0.0;
    ctrlr->prevI = 0.0;
    ctrlr->prevD = 0.0;
    return ctrlr;
}

double
pid_update(GC_state s, double measurement)
{
    GC_PIDController ctrlr = s->pidController;
    double dt = (double)(s->pidStatistics.bytesAllocated >> 10); // kB
    double err = ctrlr->setpoint - measurement;
    double P = err * ctrlr->Kp;
    double I = 0.5 * ctrlr->Ki * dt * (err + ctrlr->prevErr) + ctrlr->prevI;
    double D = 2 * ctrlr->Kd / dt * (err - ctrlr->prevErr) - ctrlr->prevD;
    ctrlr->prevErr = err;
    ctrlr->prevI = I;
    ctrlr->prevD = D;
    double output = P + I + D;
    ctrlr->prevOutput = output;
    return output;
}

double
pid_timediff(struct timespec *start, struct timespec *end)
{
    static const uintmax_t BILLION = 1000000000;
    return (BILLION * (end->tv_sec - start->tv_sec) + end->tv_nsec - start->tv_nsec)
            / (double)BILLION;
}

double
pid_getoutput(GC_PIDController ctrlr)
{
    return ctrlr->prevOutput;
}

