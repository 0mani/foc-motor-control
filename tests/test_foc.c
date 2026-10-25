/* Host unit tests for the FOC library. No framework needed: build and run. */
#include <math.h>
#include <stdio.h>

#include "foc/foc_ctrl.h"
#include "foc/foc_math.h"
#include "foc/foc_pi.h"

static int g_failures;
static int g_checks;

#define CHECK_NEAR(actual, expected, tol)                                       \
    do {                                                                        \
        g_checks++;                                                             \
        double a_ = (double)(actual), e_ = (double)(expected);                  \
        if (fabs(a_ - e_) > (tol)) {                                            \
            printf("  FAIL %s:%d  %s = %.6f, expected %.6f\n", __FILE__,        \
                   __LINE__, #actual, a_, e_);                                  \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

#define CHECK(cond)                                                             \
    do {                                                                        \
        g_checks++;                                                             \
        if (!(cond)) {                                                          \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

static void test_clarke_park_balanced(void)
{
    /* balanced currents with amplitude 2 A and phase angle theta should
     * map to d = 2, q = 0 when Park uses the same angle */
    for (int k = 0; k < 360; k += 15) {
        float th = (float)k * FOC_PI / 180.0f;
        float ia = 2.0f * cosf(th);
        float ib = 2.0f * cosf(th - 2.0f * FOC_PI / 3.0f);
        foc_dq_t dq = foc_park(foc_clarke(ia, ib), foc_sincos(th));
        CHECK_NEAR(dq.d, 2.0f, 1e-4);
        CHECK_NEAR(dq.q, 0.0f, 1e-4);
    }
}

static void test_park_round_trip(void)
{
    foc_ab_t ab = {1.3f, -0.7f};
    foc_sincos_t sc = foc_sincos(1.1f);
    foc_ab_t back = foc_inv_park(foc_park(ab, sc), sc);
    CHECK_NEAR(back.alpha, ab.alpha, 1e-5);
    CHECK_NEAR(back.beta, ab.beta, 1e-5);
}

static void test_wrap(void)
{
    CHECK_NEAR(foc_wrap_2pi(-0.5f), FOC_TWO_PI - 0.5f, 1e-5);
    CHECK_NEAR(foc_wrap_2pi(7.0f), 7.0f - FOC_TWO_PI, 1e-5);
    CHECK_NEAR(foc_wrap_2pi(0.25f), 0.25f, 1e-6);
}

static void test_limit_circle(void)
{
    foc_dq_t v = {3.0f, 4.0f};
    CHECK(foc_limit_circle(&v, 10.0f) == 0);
    CHECK_NEAR(v.d, 3.0f, 1e-6);

    v.d = 3.0f;
    v.q = 4.0f;
    CHECK(foc_limit_circle(&v, 4.0f) == 1);
    CHECK_NEAR(v.d, 3.0f, 1e-6); /* d keeps priority */
    CHECK_NEAR(sqrtf(v.d * v.d + v.q * v.q), 4.0f, 1e-5);
}

static void test_svpwm_reproduces_voltage(void)
{
    /* the line-to-neutral voltage seen by a star-connected motor is the
     * pole voltage minus the common mode; it must match the reference */
    const float vdc = 24.0f;
    const float mag = 0.95f * vdc * FOC_INV_SQRT3;
    for (int k = 0; k < 360; k += 7) {
        float th = (float)k * FOC_PI / 180.0f;
        foc_ab_t ref = {mag * cosf(th), mag * sinf(th)};
        foc_duty_t d = foc_svpwm(ref, vdc);

        CHECK(d.a >= 0.0f && d.a <= 1.0f);
        CHECK(d.b >= 0.0f && d.b <= 1.0f);
        CHECK(d.c >= 0.0f && d.c <= 1.0f);

        float cm = (d.a + d.b + d.c) / 3.0f;
        float va = (d.a - cm) * vdc;
        float vb = (d.b - cm) * vdc;
        foc_ab_t got = foc_clarke(va, vb);
        CHECK_NEAR(got.alpha, ref.alpha, 1e-3);
        CHECK_NEAR(got.beta, ref.beta, 1e-3);
    }
}

static void test_svpwm_zero_vector(void)
{
    foc_ab_t zero = {0.0f, 0.0f};
    foc_duty_t d = foc_svpwm(zero, 12.0f);
    CHECK_NEAR(d.a, 0.5f, 1e-6);
    CHECK_NEAR(d.b, 0.5f, 1e-6);
    CHECK_NEAR(d.c, 0.5f, 1e-6);
}

static void test_pi_anti_windup(void)
{
    foc_pi_t pi;
    foc_pi_init(&pi, 1.0f, 100.0f, -1.0f, 1.0f);

    /* drive hard into saturation for a long time */
    for (int i = 0; i < 1000; i++) {
        float out = foc_pi_step(&pi, 10.0f, 0.0f, 1e-3f);
        CHECK(out <= 1.0f);
    }
    CHECK(pi.integ <= 1.0f);

    /* on error reversal the output must leave saturation immediately */
    float out = foc_pi_step(&pi, -2.0f, 0.0f, 1e-3f);
    CHECK(out < 0.0f);
}

static void test_pi_steady_state(void)
{
    /* integral action removes constant error on a first-order plant */
    foc_pi_t pi;
    foc_pi_init(&pi, 0.5f, 50.0f, -10.0f, 10.0f);
    float y = 0.0f;
    const float dt = 1e-3f;
    for (int i = 0; i < 5000; i++) {
        float u = foc_pi_step(&pi, 1.0f - y, 0.0f, dt);
        y += dt * (-y + u) / 0.05f;
    }
    CHECK_NEAR(y, 1.0f, 1e-3);
}

static void test_tune(void)
{
    foc_motor_t m = {
        .rs = 0.5f, .ls = 0.5e-3f, .flux = 0.008f, .pole_pairs = 4,
        .inertia = 2e-5f, .friction = 1e-5f,
    };
    foc_config_t cfg;
    CHECK(foc_tune(&m, 50e-6f, 10, 1000.0f, 50.0f, 10.0f, &cfg) == 0);
    CHECK_NEAR(cfg.kp_current, 0.5e-3f * FOC_TWO_PI * 1000.0f, 1e-4);
    CHECK_NEAR(cfg.ki_current, 0.5f * FOC_TWO_PI * 1000.0f, 1e-1);
    CHECK_NEAR(foc_torque_constant(&m), 0.048f, 1e-6);

    /* current loop too fast for 20 kHz sampling */
    CHECK(foc_tune(&m, 50e-6f, 10, 5000.0f, 50.0f, 10.0f, &cfg) == -1);
    /* speed loop too close to the current loop */
    CHECK(foc_tune(&m, 50e-6f, 10, 1000.0f, 400.0f, 10.0f, &cfg) == -1);
}

static void test_ctrl_disabled_outputs_zero_voltage(void)
{
    foc_motor_t m = {0.5f, 0.5e-3f, 0.008f, 4, 2e-5f, 1e-5f};
    foc_config_t cfg;
    foc_tune(&m, 50e-6f, 10, 1000.0f, 50.0f, 10.0f, &cfg);
    foc_ctrl_t c;
    foc_ctrl_init(&c, &cfg);

    foc_input_t in = {1.0f, -0.5f, 0.3f, 10.0f, 24.0f};
    foc_output_t out;
    foc_ctrl_step(&c, &in, &out);
    CHECK_NEAR(out.duty.a, 0.5f, 1e-6);
    CHECK_NEAR(out.duty.b, 0.5f, 1e-6);
    CHECK_NEAR(out.duty.c, 0.5f, 1e-6);
}

static void test_ctrl_voltage_stays_in_limit(void)
{
    foc_motor_t m = {0.5f, 0.5e-3f, 0.008f, 4, 2e-5f, 1e-5f};
    foc_config_t cfg;
    foc_tune(&m, 50e-6f, 10, 1000.0f, 50.0f, 10.0f, &cfg);
    foc_ctrl_t c;
    foc_ctrl_init(&c, &cfg);
    foc_ctrl_set_mode(&c, FOC_MODE_TORQUE);
    foc_ctrl_set_current(&c, 0.0f, 50.0f); /* clamped to iq_max */

    foc_input_t in = {0.0f, 0.0f, 0.0f, 400.0f, 24.0f};
    foc_output_t out;
    for (int i = 0; i < 200; i++) {
        foc_ctrl_step(&c, &in, &out);
    }
    float vmag = sqrtf(out.v_dq.d * out.v_dq.d + out.v_dq.q * out.v_dq.q);
    CHECK(vmag <= 24.0f * FOC_INV_SQRT3 + 1e-3f);
    CHECK_NEAR(out.iq_ref, 10.0f, 1e-6);
    CHECK(out.v_limited == 1u);
}

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"clarke_park_balanced", test_clarke_park_balanced},
        {"park_round_trip", test_park_round_trip},
        {"wrap", test_wrap},
        {"limit_circle", test_limit_circle},
        {"svpwm_reproduces_voltage", test_svpwm_reproduces_voltage},
        {"svpwm_zero_vector", test_svpwm_zero_vector},
        {"pi_anti_windup", test_pi_anti_windup},
        {"pi_steady_state", test_pi_steady_state},
        {"tune", test_tune},
        {"ctrl_disabled_outputs_zero_voltage", test_ctrl_disabled_outputs_zero_voltage},
        {"ctrl_voltage_stays_in_limit", test_ctrl_voltage_stays_in_limit},
    };
    const int n = (int)(sizeof(tests) / sizeof(tests[0]));

    for (int i = 0; i < n; i++) {
        int before = g_failures;
        tests[i].fn();
        printf("%s %s\n", (g_failures == before) ? "[ OK ]" : "[FAIL]", tests[i].name);
    }
    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
