/**
 * @file foc_pi.h
 * @brief Discrete PI controller with output clamping and anti-windup.
 */
#ifndef FOC_PI_H
#define FOC_PI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;       /**< proportional gain */
    float ki;       /**< integral gain [1/s] (continuous-time) */
    float out_min;  /**< lower output limit */
    float out_max;  /**< upper output limit */
    float integ;    /**< integrator state */
} foc_pi_t;

/** Initialise gains and limits, and clear the integrator. */
void foc_pi_init(foc_pi_t *pi, float kp, float ki, float out_min, float out_max);

/** Clear the integrator (e.g. on enable or fault recovery). */
void foc_pi_reset(foc_pi_t *pi);

/**
 * One controller step.
 *
 * Anti-windup uses conditional integration: the integrator stops growing
 * when the output is saturated and the error would push it further.
 *
 * @param err  reference minus measurement
 * @param ff   feed-forward term added to the output before limiting
 * @param dt   sample time [s]
 * @return     limited controller output
 */
float foc_pi_step(foc_pi_t *pi, float err, float ff, float dt);

/** Change limits at run time; the integrator is clamped into the new range. */
void foc_pi_set_limits(foc_pi_t *pi, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PI_H */
