//
// Created by Robert Wang on 2022/8/22.
//

#include "PX4Accelerometer.hpp"

bool PX4Accelerometer::attack_enabled(const uint8_t &attack_type, const hrt_abstime &timestamp_sample) const {
    return _attack_flag_prev & attack_type && _attack_timestamp != 0 && timestamp_sample >= _attack_timestamp;
}

void PX4Accelerometer::applyAccelAttack(sensor_accel_s &accel) {
    // Attempt to Apply Attack on X-axis
    if (attack_enabled(sensor_attack::ATK_MASK_ACCEL, accel.timestamp_sample)) {
        _last_deviation[0] = _param_atk_acc_bias.get();
        _last_deviation[1] = 0.f;
        _last_deviation[2] = 0.f;

        accel.x += _last_deviation[0];
        accel.y += _last_deviation[1];
        accel.z += _last_deviation[2];

    } else {
        _last_deviation[0] = 0.f;
        _last_deviation[1] = 0.f;
        _last_deviation[2] = 0.f;
    }
}

void PX4Accelerometer::applyAccelAttack(sensor_accel_s &accel, sensor_accel_fifo_s &accel_fifo) {
    // Attempt to Apply Attack
    applyAccelAttack(accel);

    if (attack_enabled(sensor_attack::ATK_MASK_ACCEL, accel_fifo.timestamp_sample)) {
        // Also apply the deviation to FIFO samples
        const uint8_t N = accel_fifo.samples;
        for (int n = 0; n < N; n++) {
            accel_fifo.x[n] += static_cast<int16_t>(_last_deviation[0] / _scale);
            accel_fifo.y[n] += static_cast<int16_t>(_last_deviation[1] / _scale);
            accel_fifo.z[n] += static_cast<int16_t>(_last_deviation[2] / _scale);
        }
    }
}

