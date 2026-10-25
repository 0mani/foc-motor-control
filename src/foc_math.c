#include "foc/foc_math.h"

#include <math.h>

foc_ab_t foc_clarke(float ia, float ib)
{
    foc_ab_t ab;
    ab.alpha = ia;
    ab.beta = (ia + 2.0f * ib) * FOC_INV_SQRT3;
    return ab;
}

foc_dq_t foc_park(foc_ab_t ab, foc_sincos_t sc)
{
    foc_dq_t dq;
    dq.d = ab.alpha * sc.cos + ab.beta * sc.sin;
    dq.q = -ab.alpha * sc.sin + ab.beta * sc.cos;
    return dq;
}

foc_ab_t foc_inv_park(foc_dq_t dq, foc_sincos_t sc)
{
    foc_ab_t ab;
    ab.alpha = dq.d * sc.cos - dq.q * sc.sin;
    ab.beta = dq.d * sc.sin + dq.q * sc.cos;
    return ab;
}

foc_sincos_t foc_sincos(float theta_e)
{
    foc_sincos_t sc;
    sc.sin = sinf(theta_e);
    sc.cos = cosf(theta_e);
    return sc;
}

float foc_wrap_2pi(float theta)
{
    theta = fmodf(theta, FOC_TWO_PI);
    if (theta < 0.0f) {
        theta += FOC_TWO_PI;
    }
    return theta;
}

int foc_limit_circle(foc_dq_t *v, float vmax)
{
    float mag2 = v->d * v->d + v->q * v->q;
    if (mag2 <= vmax * vmax) {
        return 0;
    }
    /* d axis first, then whatever magnitude is left goes to q */
    if (v->d > vmax) {
        v->d = vmax;
    } else if (v->d < -vmax) {
        v->d = -vmax;
    }
    float q_room = sqrtf(fmaxf(vmax * vmax - v->d * v->d, 0.0f));
    if (v->q > q_room) {
        v->q = q_room;
    } else if (v->q < -q_room) {
        v->q = -q_room;
    }
    return 1;
}

static float clamp01(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

foc_duty_t foc_svpwm(foc_ab_t v_ab, float vdc)
{
    /* inverse Clarke to phase voltages */
    float va = v_ab.alpha;
    float vb = -0.5f * v_ab.alpha + FOC_SQRT3_BY_2 * v_ab.beta;
    float vc = -0.5f * v_ab.alpha - FOC_SQRT3_BY_2 * v_ab.beta;

    /* min/max zero-sequence injection centres the vector in the hexagon */
    float vmax = fmaxf(va, fmaxf(vb, vc));
    float vmin = fminf(va, fminf(vb, vc));
    float v0 = 0.5f * (vmax + vmin);

    float inv_vdc = 1.0f / vdc;
    foc_duty_t d;
    d.a = clamp01(0.5f + (va - v0) * inv_vdc);
    d.b = clamp01(0.5f + (vb - v0) * inv_vdc);
    d.c = clamp01(0.5f + (vc - v0) * inv_vdc);
    return d;
}
