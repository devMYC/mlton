#if (defined (MLTON_GC_INTERNAL_TYPES))

typedef struct _GC_PIDController *GC_PIDController;

#endif

#if (defined (MLTON_GC_INTERNAL_FUNCS))

static GC_PIDController new_controller(double Kp, double Ki, double Kd, double setpoint);
static void pid_set_kp(GC_state s, double kp);
static void pid_set_ki(GC_state s, double ki);
static void pid_set_kd(GC_state s, double kd);
static void pid_set_setpoint(GC_state s, double sp);
static double pid_update(GC_state s, double measurement);
static double pid_update2(GC_state s, double measurement);
static double pid_timediff(struct timespec *start, struct timespec *end);
static void pid_accgctime(struct timespec *acc, struct timespec *start, struct timespec *end);
static double pid_measure(GC_state s, struct timespec *end);
static double pid_overhead_median(GC_state s);

#endif

