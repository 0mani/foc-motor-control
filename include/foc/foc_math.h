/**
 * @file foc_math.h
 * @brief Coordinate transforms and space-vector PWM for field-oriented control.
 *
 * All functions are pure, allocation-free and use single-precision float,
 * so they map directly onto the FPU of Cortex-M4F/M7, ESP32 and similar MCUs.
 * Transforms are amplitude-invariant (Clarke scaled by 2/3).
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_PI          3.14159265358979f
#define FOC_TWO_PI      6.28318530717959f
#define FOC_SQRT3       1.73205080756888f
#define FOC_INV_SQRT3   0.57735026918963f
#define FOC_SQRT3_BY_2  0.86602540378444f

/** Two-axis stationary-frame quantity (alpha/beta). */
typedef struct {
    float alpha;
    float beta;
} foc_ab_t;

/** Two-axis rotating-frame quantity (d/q). */
typedef struct {
    float d;
    float q;
} foc_dq_t;

/** Pre-computed sine/cosine of the electrical angle. */
typedef struct {
    float sin;
    float cos;
} foc_sincos_t;

/** Phase duty cycles, each in [0, 1]. */
typedef struct {
    float a;
    float b;
    float c;
} foc_duty_t;

/** Clarke transform from two measured phase currents (ia + ib + ic = 0). */
foc_ab_t foc_clarke(float ia, float ib);

/** Park transform: stationary frame -> rotor frame. */
foc_dq_t foc_park(foc_ab_t ab, foc_sincos_t sc);

/** Inverse Park transform: rotor frame -> stationary frame. */
foc_ab_t foc_inv_park(foc_dq_t dq, foc_sincos_t sc);

/** sin/cos of an electrical angle in radians (any range). */
foc_sincos_t foc_sincos(float theta_e);

/** Wrap an angle to [0, 2*pi). */
float foc_wrap_2pi(float theta);

/**
 * Limit a d/q voltage vector to a circle of radius vmax.
 * The d axis has priority so flux control is kept under saturation.
 * @return 1 if the vector was limited, 0 otherwise.
 */
int foc_limit_circle(foc_dq_t *v, float vmax);

/**
 * Space-vector PWM using min/max zero-sequence injection.
 * Equivalent to classic 7-segment SVPWM, with no sector lookup.
 * Linear range: |v_ab| <= vdc / sqrt(3).
 *
 * @param v_ab  stator voltage reference in the alpha/beta frame [V]
 * @param vdc   DC-bus voltage [V], must be > 0
 * @return      duty cycles in [0, 1], clamped if over-modulated
 */
foc_duty_t foc_svpwm(foc_ab_t v_ab, float vdc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
