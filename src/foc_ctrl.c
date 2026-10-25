#include "foc/foc_ctrl.h"

#include <math.h>
#include <string.h>

/* Keep a small margin below the SVPWM linear limit (vdc/sqrt3) so the
 * current loops always have some headroom left for disturbance rejection. */
#define FOC_VLIM_FACTOR 0.96f

float foc_torque_constant(const foc_motor_t *m)
{
    return 1.5f * (float)m->pole_pairs * m->flux;
}

int foc_tune(const foc_motor_t *motor, float ts_current, uint16_t speed_div,
             float bw_current_hz, float bw_speed_hz, float iq_max,
             foc_config_t *cfg)
{
    if (motor == NULL || cfg == NULL || ts_current <= 0.0f || speed_div == 0u) {
        return -1;
    }
    float ts_speed = ts_current * (float)speed_div;

    /* time-scale separation rules: sampling >> current loop >> speed loop */
    if (bw_current_hz * ts_current > 0.1f) {
        return -1;
    }
    if (bw_speed_hz * 5.0f > bw_current_hz || bw_speed_hz * ts_speed > 0.1f) {
        return -1;
    }

    float wc = FOC_TWO_PI * bw_current_hz;
    float ws = FOC_TWO_PI * bw_speed_hz;
    float kt = foc_torque_constant(motor);
    if (kt <= 0.0f) {
        return -1;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->motor = *motor;
    cfg->ts_current = ts_current;
    cfg->speed_div = speed_div;

    /* pole/zero cancellation of the R-L plant */
    cfg->kp_current = motor->ls * wc;
    cfg->ki_current = motor->rs * wc;

    /* symmetric optimum with a = 2 on the J*s plant */
    cfg->kp_speed = motor->inertia * ws / kt;
    cfg->ki_speed = cfg->kp_speed * ws / 4.0f;

    cfg->iq_max = iq_max;
    cfg->speed_slew = 0.0f;
    cfg->decouple = 1u;
    return 0;
}

void foc_ctrl_init(foc_ctrl_t *c, const foc_config_t *cfg)
{
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    foc_pi_init(&c->pi_id, cfg->kp_current, cfg->ki_current, 0.0f, 0.0f);
    foc_pi_init(&c->pi_iq, cfg->kp_current, cfg->ki_current, 0.0f, 0.0f);
    foc_pi_init(&c->pi_speed, cfg->kp_speed, cfg->ki_speed,
                -cfg->iq_max, cfg->iq_max);
    c->mode = FOC_MODE_DISABLED;
}

void foc_ctrl_set_mode(foc_ctrl_t *c, foc_mode_t mode)
{
    if (mode != c->mode) {
        foc_pi_reset(&c->pi_id);
        foc_pi_reset(&c->pi_iq);
        foc_pi_reset(&c->pi_speed);
        c->speed_count = 0u;
        c->mode = mode;
    }
}

void foc_ctrl_set_speed(foc_ctrl_t *c, float omega_m_ref)
{
    c->speed_ref = omega_m_ref;
}

void foc_ctrl_set_current(foc_ctrl_t *c, float id_ref, float iq_ref)
{
    c->id_ref = id_ref;
    c->iq_ref = iq_ref;
}

static float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static void speed_loop(foc_ctrl_t *c, float omega_m)
{
    float ts_speed = c->cfg.ts_current * (float)c->cfg.speed_div;

    if (c->cfg.speed_slew > 0.0f) {
        float step = c->cfg.speed_slew * ts_speed;
        float diff = c->speed_ref - c->speed_ref_slewed;
        c->speed_ref_slewed += clampf(diff, -step, step);
    } else {
        c->speed_ref_slewed = c->speed_ref;
    }

    float err = c->speed_ref_slewed - omega_m;
    c->iq_ref = foc_pi_step(&c->pi_speed, err, 0.0f, ts_speed);
    c->id_ref = 0.0f; /* maximum torque per amp for a surface PMSM */
}

void foc_ctrl_step(foc_ctrl_t *c, const foc_input_t *in, foc_output_t *out)
{
    memset(out, 0, sizeof(*out));

    if (c->mode == FOC_MODE_DISABLED || in->vdc <= 0.0f) {
        out->duty.a = 0.5f;
        out->duty.b = 0.5f;
        out->duty.c = 0.5f;
        return;
    }

    if (c->mode == FOC_MODE_SPEED) {
        if (c->speed_count == 0u) {
            speed_loop(c, in->omega_m);
        }
        c->speed_count++;
        if (c->speed_count >= c->cfg.speed_div) {
            c->speed_count = 0u;
        }
    }

    float iq_ref = clampf(c->iq_ref, -c->cfg.iq_max, c->cfg.iq_max);
    float id_ref = c->id_ref;

    foc_sincos_t sc = foc_sincos(in->theta_e);
    foc_dq_t i_dq = foc_park(foc_clarke(in->ia, in->ib), sc);

    /* cross-coupling and back-EMF feed-forward */
    float vd_ff = 0.0f;
    float vq_ff = 0.0f;
    if (c->cfg.decouple) {
        const foc_motor_t *m = &c->cfg.motor;
        float we = (float)m->pole_pairs * in->omega_m;
        vd_ff = -we * m->ls * i_dq.q;
        vq_ff = we * (m->ls * i_dq.d + m->flux);
    }

    /* d axis gets priority on the voltage circle, q uses what is left */
    float vmax = FOC_VLIM_FACTOR * in->vdc * FOC_INV_SQRT3;
    foc_pi_set_limits(&c->pi_id, -vmax, vmax);
    float ts = c->cfg.ts_current;

    foc_dq_t v_dq;
    v_dq.d = foc_pi_step(&c->pi_id, id_ref - i_dq.d, vd_ff, ts);

    float q_room = sqrtf(fmaxf(vmax * vmax - v_dq.d * v_dq.d, 0.0f));
    foc_pi_set_limits(&c->pi_iq, -q_room, q_room);
    float vq_unsat = c->pi_iq.kp * (iq_ref - i_dq.q) + c->pi_iq.integ + vq_ff;
    v_dq.q = foc_pi_step(&c->pi_iq, iq_ref - i_dq.q, vq_ff, ts);

    out->duty = foc_svpwm(foc_inv_park(v_dq, sc), in->vdc);
    out->i_dq = i_dq;
    out->v_dq = v_dq;
    out->iq_ref = iq_ref;
    out->v_limited = (fabsf(vq_unsat) > q_room) ? 1u : 0u;
}
