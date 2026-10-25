/**
 * @file foc_ctrl.h
 * @brief Cascaded FOC controller: speed loop -> dq current loops -> SVPWM.
 *
 * Call foc_ctrl_step() from the PWM/ADC interrupt once per PWM period.
 * The speed loop runs every `speed_div` current-loop periods.
 *
 * Typical ISR integration:
 * @code
 *   void ADC_IRQHandler(void) {
 *       foc_input_t in = {
 *           .ia = adc_to_amps(ADC->JDR1), .ib = adc_to_amps(ADC->JDR2),
 *           .theta_e = encoder_elec_angle(), .omega_m = encoder_speed(),
 *           .vdc = vbus_volts(),
 *       };
 *       foc_output_t out;
 *       foc_ctrl_step(&g_foc, &in, &out);
 *       pwm_set_duty(out.duty.a, out.duty.b, out.duty.c);
 *   }
 * @endcode
 */
#ifndef FOC_CTRL_H
#define FOC_CTRL_H

#include <stdint.h>
#include "foc_math.h"
#include "foc_pi.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Electrical and mechanical motor parameters (SI units). */
typedef struct {
    float rs;          /**< phase resistance [ohm] */
    float ls;          /**< phase inductance [H] (Ld = Lq assumed) */
    float flux;        /**< PM flux linkage [V*s/rad] */
    uint8_t pole_pairs;/**< number of pole pairs */
    float inertia;     /**< rotor + load inertia [kg*m^2] */
    float friction;    /**< viscous friction [N*m*s/rad] */
} foc_motor_t;

typedef enum {
    FOC_MODE_DISABLED = 0, /**< outputs 50% duty on all phases (zero voltage) */
    FOC_MODE_TORQUE,       /**< iq reference set directly */
    FOC_MODE_SPEED,        /**< speed loop generates iq reference */
} foc_mode_t;

/** Controller configuration. Fill by hand or with foc_tune(). */
typedef struct {
    foc_motor_t motor;
    float ts_current;      /**< current-loop sample time [s] */
    uint16_t speed_div;    /**< speed loop runs every N current periods */
    float kp_current;      /**< [V/A] */
    float ki_current;      /**< [V/(A*s)] */
    float kp_speed;        /**< [A/(rad/s)] */
    float ki_speed;        /**< [A/rad] */
    float iq_max;          /**< current limit [A] */
    float speed_slew;      /**< speed reference slew limit [rad/s^2], 0 = off */
    uint8_t decouple;      /**< 1 = add dq cross-coupling and back-EMF feed-forward */
} foc_config_t;

/** Per-sample measurements. */
typedef struct {
    float ia;       /**< phase A current [A] */
    float ib;       /**< phase B current [A] */
    float theta_e;  /**< electrical angle [rad] */
    float omega_m;  /**< mechanical speed [rad/s] */
    float vdc;      /**< DC-bus voltage [V] */
} foc_input_t;

/** Per-sample outputs and diagnostics. */
typedef struct {
    foc_duty_t duty;    /**< PWM duty cycles in [0, 1] */
    foc_dq_t i_dq;      /**< measured dq currents [A] */
    foc_dq_t v_dq;      /**< applied dq voltages [V] */
    float iq_ref;       /**< current reference from the speed loop [A] */
    uint8_t v_limited;  /**< 1 if the voltage vector hit the SVPWM limit */
} foc_output_t;

/** Controller state. Treat as opaque. */
typedef struct {
    foc_config_t cfg;
    foc_mode_t mode;
    foc_pi_t pi_id;
    foc_pi_t pi_iq;
    foc_pi_t pi_speed;
    float speed_ref;       /**< target speed [rad/s] */
    float speed_ref_slewed;
    float id_ref;
    float iq_ref;
    uint16_t speed_count;
} foc_ctrl_t;

/**
 * Model-based auto-tuning (pole/zero cancellation + symmetric optimum).
 *
 * Current loop: Kp = L*wc, Ki = R*wc, so the closed loop is first order
 * with bandwidth wc. Speed loop: Kp = J*ws/Kt, Ki = Kp*ws/a^2 with a = 2,
 * which gives about 37 deg of phase margin with a fast integrator.
 *
 * @param motor          motor parameters
 * @param ts_current     current-loop sample time [s]
 * @param speed_div      speed-loop decimation
 * @param bw_current_hz  current-loop bandwidth [Hz]; keep below 1/(10*Ts)
 * @param bw_speed_hz    speed-loop bandwidth [Hz]; keep below bw_current/10
 * @param iq_max         current limit [A]
 * @param cfg            filled configuration
 * @return 0 on success, -1 if the bandwidths violate the separation rules
 */
int foc_tune(const foc_motor_t *motor, float ts_current, uint16_t speed_div,
             float bw_current_hz, float bw_speed_hz, float iq_max,
             foc_config_t *cfg);

/** Initialise the controller; starts disabled. */
void foc_ctrl_init(foc_ctrl_t *c, const foc_config_t *cfg);

/** Change mode; integrators are cleared on every mode change. */
void foc_ctrl_set_mode(foc_ctrl_t *c, foc_mode_t mode);

/** Set the speed reference [rad/s] used in FOC_MODE_SPEED. */
void foc_ctrl_set_speed(foc_ctrl_t *c, float omega_m_ref);

/** Set dq current references [A] used in FOC_MODE_TORQUE. */
void foc_ctrl_set_current(foc_ctrl_t *c, float id_ref, float iq_ref);

/** Run one current-loop period. */
void foc_ctrl_step(foc_ctrl_t *c, const foc_input_t *in, foc_output_t *out);

/** Torque constant Kt = 1.5 * p * flux [N*m/A]. */
float foc_torque_constant(const foc_motor_t *m);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CTRL_H */
