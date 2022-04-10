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

void
pid_set_kp(GC_state s, double kp)
{
    s->pidController->Kp = kp;
}

void
pid_set_ki(GC_state s, double ki)
{
    s->pidController->Ki = ki;
}

void
pid_set_kd(GC_state s, double kd)
{
    s->pidController->Kd = kd;
}

void
pid_set_setpoint(GC_state s, double sp)
{
    s->pidController->setpoint = sp;
}

double
pid_update(GC_state s, const double measurement, const double dt)
{
    static const double integratorBound = 5.0;
    static bool first = TRUE;
    GC_PIDController ctrlr = s->pidController;
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
    const double err = measurement - ctrlr->setpoint;
    if (first)
    {
        ctrlr->prevErr[0] = err;
        first = FALSE;
    }
    const double P = ctrlr->Kp * err;
    double I = 0.5*ctrlr->Ki*dt * (err + ctrlr->prevErr[0]) + ctrlr->prevI;
    const double D = 2.0*ctrlr->Kd/dt * (err - ctrlr->prevErr[0]) - ctrlr->prevD;
    // const double D = ctrlr->Kd * (err - ctrlr->prevErr[0]) / dt;

    if (I > integratorBound) I = integratorBound;
    else if (I < -integratorBound) I = -integratorBound;

    ctrlr->prevI = I;
    ctrlr->prevD = D;
    ctrlr->prevErr[0] = err;
    double out = P + I + D;
    return out;
}

double
pid_update2(GC_state s,
            const double measurement,
            const double dt,
            const double minOut,
            const double maxOut)
{
    static bool first = TRUE;
    GC_PIDController ctrlr = s->pidController;
    if (s->controls.messages)
    {
        fprintf(stderr, "[GC: Kp=%f, Ki=%f, Kd=%f]\n", ctrlr->Kp, ctrlr->Ki, ctrlr->Kd);
        fprintf(stderr, "[GC: dt=%f ", dt);
        fprintf(stderr, "recentGCOverheads=[");
        for (int i = 0; i < PID_STATS_WIN_SIZE; i++)
            fprintf(stderr, "%f%s",
                    s->pidStatistics.recentGCOverheads[i],
                    i+1 == PID_STATS_WIN_SIZE ? "" : ", ");
        fprintf(stderr, "]]\n");
    }
    const double err = measurement - ctrlr->setpoint;
    if (first)
    {
        ctrlr->prevErr[0] = ctrlr->prevErr[1] = err;
        first = FALSE;
    }
    const double b0 = ctrlr->Kp + ctrlr->Ki*dt*0.5 + ctrlr->Kd/dt;
    const double b1 = ctrlr->Ki*dt*0.5 - ctrlr->Kp - 2.0*ctrlr->Kd/dt;
    const double b2 = ctrlr->Kd / dt;
    // const double a0 = 1.0;
    // const double a1 = -1.0;
    // const double a2 = 0.0;

    // u[n] = b0*e[n] + b1*e[n-1] + b2*e[n-2] + u[n-1]
    double out = b0*err + b1*ctrlr->prevErr[1] + b2*ctrlr->prevErr[0] + ctrlr->prevOut;
    if (s->controls.messages)
        fprintf(stderr, "--- err=%f, out=%f\n", err, out);

    if (out < minOut) out = minOut;
    else if (out > maxOut) out = maxOut;

    ctrlr->prevOut = out;
    ctrlr->prevErr[0] = ctrlr->prevErr[1];
    ctrlr->prevErr[1] = err;
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

double
pid_overhead_median(GC_state s)
{
    double window[PID_STATS_WIN_SIZE];
    memcpy(window, s->pidStatistics.recentGCOverheads, PID_STATS_WIN_SIZE*sizeof(double));
    // insertion sort is used since the window size is relatively small
    for (int i = 1; i < PID_STATS_WIN_SIZE; i++)
    {
        int j = i;
        while (j > 0 && window[j-1] > window[j])
        {
            double temp = window[j];
            window[j] = window[j-1];
            window[j-1] = temp;
            j--;
        }
    }
    return window[PID_STATS_WIN_SIZE / 2];
}

