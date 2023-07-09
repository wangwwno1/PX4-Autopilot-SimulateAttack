//
// Created by Robert Wang on 2022/8/29.
//

#include "VehicleGPSPosition.hpp"

namespace sensors
{
    bool VehicleGPSPosition::attack_enabled(const uint8_t &attack_type) const {
        return _param_atk_apply_type.get() & attack_type &&
                _attack_timestamp != 0 && hrt_absolute_time() >= _attack_timestamp;
    }

    void VehicleGPSPosition::ConductVelocitySpoofing(sensor_gps_s &gps_position)
    {
        if (attack_enabled(sensor_attack::ATK_GPS_VEL)) {
            if (!_vel_deviation) {
                PX4_INFO("Initiate GPS Velocity Spoofing Attack");
                _vel_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_v_cls.get(), &_vel_atk_params));

                const float time_to_max_deviation_s = (*_vel_deviation).time_to_max_deviation();
                if (PX4_ISFINITE(time_to_max_deviation_s)) {
                    PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                } else {
                    PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                }

            }

            if (_vel_deviation) {
                const Vector3f deviation = (*_vel_deviation).calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_velocity_spoofing(gps_position, deviation);
            }

        } else if (_vel_deviation) {
            _vel_deviation.reset();
            PX4_INFO("GPS Velocity Spoofing Stopped");
        }
    }

    void VehicleGPSPosition::ConductPositionSpoofing(sensor_gps_s &gps_position) {
        if (attack_enabled(sensor_attack::ATK_GPS_POS)) {
            if (!_pos_deviation) {
                PX4_INFO("Initiate GPS Position Spoofing Attack");
                _pos_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_p_cls.get(), &_pos_atk_params));

                const float time_to_max_deviation_s = (*_pos_deviation).time_to_max_deviation();
                if (PX4_ISFINITE(time_to_max_deviation_s)) {
                    PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                } else {
                    PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                }

            }

            if (_pos_deviation) {
                const Vector3f deviation = (*_pos_deviation).calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_position_spoofing(gps_position, deviation);
            }

        } else if (_pos_deviation) {
            _pos_deviation.reset();
            PX4_INFO("GPS Position Spoofing Stopped");
        }

    }

}  // sensors