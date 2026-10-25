#include "foc/foc_pi.h"

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

void foc_pi_init(foc_pi_t *pi, float kp, float ki, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integ = 0.0f;
}

void foc_pi_reset(foc_pi_t *pi)
{
    pi->integ = 0.0f;
}

void foc_pi_set_limits(foc_pi_t *pi, float out_min, float out_max)
{
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integ = clampf(pi->integ, out_min, out_max);
}

float foc_pi_step(foc_pi_t *pi, float err, float ff, float dt)
{
    float unsat = pi->kp * err + pi->integ + ff;
    float out = clampf(unsat, pi->out_min, pi->out_max);

    /* conditional integration: freeze the integrator only when it would
     * drive the output further into saturation */
    int sat_hi = (unsat > pi->out_max) && (err > 0.0f);
    int sat_lo = (unsat < pi->out_min) && (err < 0.0f);
    if (!sat_hi && !sat_lo) {
        pi->integ += pi->ki * err * dt;
        pi->integ = clampf(pi->integ, pi->out_min, pi->out_max);
    }
    return out;
}
