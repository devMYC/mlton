struct _GC_PIDController {
    double Kp;
    double Ki;
    double Kd;
    double setpoint;
    double prevErr[2];
    double prevI;
    double prevD;
    double prevOut;
};

GC_PIDController
new_controller(double Kp, double Ki, double Kd, double setpoint)
{
    GC_PIDController ctrlr = (GC_PIDController)malloc(sizeof(struct _GC_PIDController)); 
    ctrlr->Kp = Kp;
    ctrlr->Ki = Ki;
    ctrlr->Kd = Kd;
    ctrlr->setpoint = setpoint;
    ctrlr->prevErr[0] = 0.0;
    ctrlr->prevErr[1] = 0.0;
    ctrlr->prevI = 0.0;
    ctrlr->prevD = 0.0;
    ctrlr->prevOut = 0.0;
    return ctrlr;
}

double
pid_update(GC_state s, double measurement)
{
    GC_PIDController ctrlr = s->pidController;
    double dt = (double)s->pidStatistics.bytesAllocated / (1 << 10);
    if (s->controls.messages)
    {
        fprintf(stderr, "[GC: dt=%f ", dt);
        fprintf(stderr, "recentGCOverheads=[");
        for (int i = 0; i < PID_STATS_WIN_SIZE; i++)
            fprintf(stderr, "%f%s",
                    s->pidStatistics.recentGCOverheads[i],
                    i+1 == PID_STATS_WIN_SIZE ? "" : ", ");
        fprintf(stderr, "]]\n");
    }
    double err = ctrlr->setpoint - measurement;
    double P = ctrlr->Kp * err;
    double I = ctrlr->Ki*dt*0.5 * (err + ctrlr->prevErr[0]) + ctrlr->prevI;
    double D = ctrlr->Kd*2.0/dt * (err - ctrlr->prevErr[0]) - ctrlr->prevD;
    // double D = ctrlr->Kd * (err - ctrlr->prevErr[0]) / dt;
    ctrlr->prevErr[0] = err;
    ctrlr->prevI = I;
    ctrlr->prevD = D;
    return P + I + D;
}

double
pid_update2(GC_state s, double measurement)
{
    GC_PIDController ctrlr = s->pidController;
    const double dt = (double)s->pidStatistics.bytesAllocated / (1 << 10);
    if (s->controls.messages)
    {
        fprintf(stderr, "[GC: dt=%f ", dt);
        fprintf(stderr, "recentGCOverheads=[");
        for (int i = 0; i < PID_STATS_WIN_SIZE; i++)
            fprintf(stderr, "%f%s",
                    s->pidStatistics.recentGCOverheads[i],
                    i+1 == PID_STATS_WIN_SIZE ? "" : ", ");
        fprintf(stderr, "]]\n");
    }
    const double err = ctrlr->setpoint - measurement;
    const double a0 = ctrlr->Kp + ctrlr->Ki*dt*0.5 + ctrlr->Kd/dt;
    const double a1 = ctrlr->Ki*dt*0.5 - ctrlr->Kp - 2.0*ctrlr->Kd/dt;
    const double a2 = ctrlr->Kd / dt;
    // const double b0 = 1.0;
    // const double b1 = -1.0;
    // const double b2 = 0.0;
    // const out = 1/b0 * (a0*err + a1*ctrlr->prevErr[1] + a2*ctrlr->prevErr[0] - b1*ctrlr->prevOut);
    const double out = a0*err + a1*ctrlr->prevErr[1] + a2*ctrlr->prevErr[0] + ctrlr->prevOut;
    ctrlr->prevErr[0] = ctrlr->prevErr[1];
    ctrlr->prevErr[1] = err;
    ctrlr->prevOut = out;
    return out;
}

static const uintmax_t BILLION = 1000000000;

double
pid_timediff(struct timespec *start, struct timespec *end)
{
    return (BILLION * (end->tv_sec - start->tv_sec) + end->tv_nsec - start->tv_nsec)
            / (double)BILLION;
}

void
pid_accgctime(struct timespec *acc, struct timespec *start, struct timespec *end)
{
    double diff = pid_timediff(start, end);
    uintmax_t sec = diff;
    uintmax_t nsec = acc->tv_nsec + (uintmax_t)((diff - sec) * BILLION);
    acc->tv_sec += sec;
    if (nsec < BILLION)
        acc->tv_nsec = nsec;
    else {
        sec = nsec / BILLION;
        acc->tv_sec += sec;
        acc->tv_nsec = nsec - BILLION * sec;
    }
}

double
pid_measure(GC_state s, struct timespec *now)
{
    struct timespec *acc = &s->pidStatistics.gcTimeAcc;
    double gcsec = acc->tv_sec + (double)acc->tv_nsec / BILLION;
    return gcsec / pid_timediff(&s->pidStatistics.lastMajorGC, now);
}

