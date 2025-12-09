/* esp32-firmware
 * Copyright (C) 2022 Matthias Bolte <matthias@tinkerforge.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#pragma once

#include "module.h"
#include "config.h"


#include "tinkerforgeIMU.h"
#include "TimeProvider.h"
#include "AttitudeEstimator.h"
#include "PID.h"
#include "MotorMixer.h"
#include "MotorOutput.h"

struct RemoteControl {
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
    float throttle   = 0.0f;
};

class FlightControl final : public IModule
{
public:
    FlightControl(){};
    void pre_setup() override;
    void setup() override;
    void register_urls() override;
    void loop() override;

    ConfigRoot state;
    ConfigRoot config;
    ConfigRoot config_update; 

    void onRemoteControlData(float roll, float pitch, float yaw, float throttle);
    void update();
    float stickToThrust(float stick);

    float m_lastTime = 0.0f;
    float* m_gyro;
    float* m_linearAccel;
    Quaternion m_quaternion;
    RemoteControl remoteControlData;

    TimeProvider timeProvider;
    TinkerforgeIMU tinkerforgeIMU;
    AttitudeEstimator attitudeEstimator;
    MotorMixer motorMixer;
    MotorOutput motorOutput;

    PID pidRollAngle;
    PID pidPitchAngle;

    PID pidRollRate;
    PID pidPitchRate;
    PID pidYawRate;


    PID pidVz;   // Vz-Regler: vz_setpoint vs m_vz → thrustOffset
    PID pidAlt;  // Höhen-Regler: alt_setpoint vs m_altitude → vz_setpoint

    // Hover-Schub als Konstante
    float m_kAttQuat    = 1.0f; // Quaternion-Attitude-Gain

    // Höhe / Vertikalgeschwindigkeit
    float m_vz              = 0.0f;  // m/s
    float m_altitude        = 0.0f;  // m
    float m_altitudeSetpoint = 0.0f; // m
    bool  m_altHoldActive   = false;
};
