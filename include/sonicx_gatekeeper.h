#ifndef SONICX_GATEKEEPER_H
#define SONICX_GATEKEEPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float altitude;
    float velocity[3];
    float quaternion[4];
    float angular_velocity[3];
    float dynamic_pressure;
} SonicXTelemetryState;

typedef struct {
    float pitch_gimbal_deg;
    float yaw_gimbal_deg;
    int32_t execution_mode; // 0: Nominal (PPO), 1: Clamped, 2: PD Fallback, 3: Abort
} SonicXValidatedCommand;

// Evaluates safety invariants on target silicon in sub-microsecond execution time
int32_t sonicx_gatekeeper_evaluate(
    const SonicXTelemetryState* state,
    const float* raw_action,
    SonicXValidatedCommand* out_command
);

#ifdef __cplusplus
}
#endif

#endif // SONICX_GATEKEEPER_H