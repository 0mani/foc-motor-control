/*
 * Flat C API over the FOC library, for the Python simulator (ctypes).
 * Only scalars and float arrays cross the boundary, so the Python side
 * does not need to mirror any C struct layout.
 */
#include <stdlib.h>

#include "foc/foc_ctrl.h"

typedef struct {
    foc_ctrl_t ctrl;
} focsim_t;

void *focsim_create(float rs, float ls, float flux, int pole_pairs,
                    float inertia, float friction, float ts_current,
                    int speed_div, float bw_current_hz, float bw_speed_hz,
                    float iq_max)
{
    foc_motor_t m = {
        .rs = rs, .ls = ls, .flux = flux, .pole_pairs = (uint8_t)pole_pairs,
        .inertia = inertia, .friction = friction,
    };
    foc_config_t cfg;
    if (foc_tune(&m, ts_current, (uint16_t)speed_div, bw_current_hz,
                 bw_speed_hz, iq_max, &cfg) != 0) {
        return NULL;
    }
    focsim_t *h = (focsim_t *)calloc(1, sizeof(*h));
    if (h != NULL) {
        foc_ctrl_init(&h->ctrl, &cfg);
    }
    return h;
}

void focsim_destroy(void *h)
{
    free(h);
}

void focsim_set_mode(void *h, int mode)
{
    foc_ctrl_set_mode(&((focsim_t *)h)->ctrl, (foc_mode_t)mode);
}

void focsim_set_speed(void *h, float omega_m_ref)
{
    foc_ctrl_set_speed(&((focsim_t *)h)->ctrl, omega_m_ref);
}

void focsim_set_current(void *h, float id_ref, float iq_ref)
{
    foc_ctrl_set_current(&((focsim_t *)h)->ctrl, id_ref, iq_ref);
}

void focsim_set_slew(void *h, float slew)
{
    ((focsim_t *)h)->ctrl.cfg.speed_slew = slew;
}

void focsim_set_decouple(void *h, int on)
{
    ((focsim_t *)h)->ctrl.cfg.decouple = (uint8_t)(on != 0);
}

/* Override the auto-tuned gains (used by the Python tuning study). */
void focsim_set_gains(void *h, float kp_i, float ki_i, float kp_w, float ki_w)
{
    foc_ctrl_t *c = &((focsim_t *)h)->ctrl;
    c->cfg.kp_current = kp_i;
    c->cfg.ki_current = ki_i;
    c->cfg.kp_speed = kp_w;
    c->cfg.ki_speed = ki_w;
    c->pi_id.kp = kp_i;
    c->pi_id.ki = ki_i;
    c->pi_iq.kp = kp_i;
    c->pi_iq.ki = ki_i;
    c->pi_speed.kp = kp_w;
    c->pi_speed.ki = ki_w;
}

/* out[4] = {kp_current, ki_current, kp_speed, ki_speed} */
void focsim_get_gains(void *h, float *out)
{
    const foc_config_t *cfg = &((focsim_t *)h)->ctrl.cfg;
    out[0] = cfg->kp_current;
    out[1] = cfg->ki_current;
    out[2] = cfg->kp_speed;
    out[3] = cfg->ki_speed;
}

/* out[9] = {duty_a, duty_b, duty_c, id, iq, vd, vq, iq_ref, v_limited} */
void focsim_step(void *h, float ia, float ib, float theta_e, float omega_m,
                 float vdc, float *out)
{
    foc_input_t in = {ia, ib, theta_e, omega_m, vdc};
    foc_output_t o;
    foc_ctrl_step(&((focsim_t *)h)->ctrl, &in, &o);
    out[0] = o.duty.a;
    out[1] = o.duty.b;
    out[2] = o.duty.c;
    out[3] = o.i_dq.d;
    out[4] = o.i_dq.q;
    out[5] = o.v_dq.d;
    out[6] = o.v_dq.q;
    out[7] = o.iq_ref;
    out[8] = (float)o.v_limited;
}
