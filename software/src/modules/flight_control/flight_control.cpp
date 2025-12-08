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

#include "flight_control.h"

#include <Arduino.h>

#include "event_log_prefix.h"
#include "module_dependencies.h"

#include "bindings/hal_common.h"
#include "bindings/errors.h"

extern TF_HAL hal;

constexpr float deg2rad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

static void onIMUData(float lastGyro[3], float lastAccel[3], float lastEuler[3], void *context_data){
    
    //FlightControl *self = (FlightControl *)context_data;
    FlightControl* self = static_cast<FlightControl*>(context_data);
    self->m_gyro = lastGyro;
    self->m_accel = lastAccel;
    self->m_euler = lastEuler;

    self->update();
    
    //logger.printfln("FC lastGyro %f", lastGyro[0]);

}

void FlightControl::onRemoteControlData(float roll, float pitch, float yaw, float throttle) {
    logger.printfln("FC throttle %f", throttle);
    remoteControlData.roll = roll;
    remoteControlData.pitch = pitch;
    remoteControlData.yaw = yaw;
    remoteControlData.throttle = throttle;
}

void FlightControl::update() {
    const float now = timeProvider.now();


    float dt = 0.01; //now - m_lastTime;
    m_lastTime = now;

    attitudeEstimator.update(m_euler);
    const Attitude& att = attitudeEstimator.getAttitude();
    //logger.printfln("%f, %f, %f", att.roll, att.pitch, att.yaw);


    // ---------- ANGLE-MODE (wie vorher) ----------
    constexpr float PI_F = 3.14159265358979323846f;
    auto deg2rad = [](float d){ return d * PI_F / 180.0f; };

    constexpr float MAX_ANGLE_DEG = 30.0f;
    const float maxAngleRad = deg2rad(MAX_ANGLE_DEG);

    const float rollAngleSetpoint  = remoteControlData.roll  * maxAngleRad;
    const float pitchAngleSetpoint = remoteControlData.pitch * maxAngleRad;

    dt = 0.01f;
    // Outer Loop: Winkel-PID -> Rate-Sollwerte
    const float rollRateSetpointFromAngle =  pidRollAngle.update(rollAngleSetpoint,  att.roll,  dt);
    const float pitchRateSetpointFromAngle = pidPitchAngle.update(pitchAngleSetpoint, att.pitch, dt);

    // Yaw weiter als Rate-Mode
    constexpr float MAX_YAW_RATE_DEG = 180.0f;
    const float maxYawRateRad = deg2rad(MAX_YAW_RATE_DEG);
    const float yawRateSetpoint = remoteControlData.yaw * maxYawRateRad;


    // -------------------------------
    // INNER LOOP: Rate-PIDs
    // -------------------------------
    const float tauRoll  = pidRollRate.update( rollRateSetpointFromAngle, m_gyro[0], dt );
    const float tauPitch = pidPitchRate.update( pitchRateSetpointFromAngle, m_gyro[1], dt );
    const float tauYaw   = pidYawRate.update( yawRateSetpoint, m_gyro[2], dt );

    // 4. Thrust aus Throttle-Stick
    //float thrust = stickToThrust(0.2);


    // -------------------------------
    // HÖHENREGELUNG mit linearer Beschleunigung
    // -------------------------------
    const float ax = m_accel[0];
    const float ay = m_accel[1];
    const float az = m_accel[2];

    const float sphi = std::sin(att.roll);
    const float cphi = std::cos(att.roll);
    const float sthe = std::sin(att.pitch);
    const float cthe = std::cos(att.pitch);

    // Rotation Body -> Welt, Z-Komponente
    // a_z_world_linear = -sin(theta)*ax + sin(phi)*cos(theta)*ay + cos(phi)*cos(theta)*az
    float a_z_world_linear = -sthe * ax + sphi * cthe * ay + cphi * cthe * az;

    // KEIN g abziehen – linear_accel ist bereits ohne Gravitation!

    // Vertikalgeschwindigkeit integrieren
    m_vz += a_z_world_linear * dt;  // [m/s]

    // Throttle-Stick -> gewünschte Vertikalgeschwindigkeit
    constexpr float MAX_VZ = 2000.0f; // m/s
    float vz_setpoint = remoteControlData.throttle * MAX_VZ;

    // PID auf Vertikalgeschwindigkeit -> Thrust-Offset
    float thrustOffset = pidVz.update(vz_setpoint, m_vz, dt); // z.B. ±0.3

    // Basis-Hover-Schub + Offset
    float thrust = m_hoverThrust + thrustOffset;
    thrust = clamp(thrust, 0.0f, 1.0f);








    // 5. Motoren mixen
    float motors[4];
    motorMixer.mix(thrust, tauRoll, tauPitch, tauYaw, motors);

    logger.printfln("FL: %f, FR: %f, RR: %f, RL: %f", motors[0], motors[1], motors[2], motors[3]);

}

float FlightControl::stickToThrust(float stick) {
        // Stick [-1,1] -> [0,1], mit kleinem Minimum (0.1)
        const float norm = (stick * 0.5f) + 0.5f; // [-1,1] → [0,1]
        const float thrust = 0.1f + 0.9f * norm;
        return clamp(thrust, 0.0f, 1.0f);
    }

void FlightControl::pre_setup()
{   

    state = Config::Object({
        {"motor", Config::Float((float)0)}
    });

    config = Config::Object({
        {"setAltitude", Config::Float((float)0)}
    });

    config_update = config;
}

void FlightControl::setup()
{
    uint16_t interval = 100;

    logger.printfln("FlightControl module initializing...");
    tinkerforgeIMU = TinkerforgeIMU();
    timeProvider = TimeProvider();
    attitudeEstimator = AttitudeEstimator(0.98f);
    motorMixer = MotorMixer();
    motorOutput = MotorOutput(&hal);

    pidRollAngle = PID(4.0f, 0.0f, 0.0f, 0.5f, 1);
    pidPitchAngle = PID(4.0f, 0.0f, 0.0f, 0.5f, 1);

    pidRollRate = PID(0.1f, 0.01f, 0.001f, 10.0f, 0.5f);
    pidPitchRate = PID(0.1f, 0.01f, 0.001f, 10.0f, 0.5f);
    pidYawRate = PID(0.1f, 0.01f, 0.000f, 10.0f, 0.5f);

    pidVz = PID(1.0f, 0.2f, 0.0f, 2.0f, 0.3f);

    int r = tinkerforgeIMU.start(&hal, interval, &onIMUData, this);


    logger.printfln("FlightControl module initialized: %d", r);
    initialized = true;
}

void FlightControl::register_urls()
{
    api.addState("flight_control/config", &config, {}, {}, true);
    api.addState("flight_control/state", &state, {}, {}, true);

    api.addCommand("flight_control/config_update", &config_update, {}, [this](String &/*errmsg*/) {
        /*
            Received from FrontEnd
        */       
        float throttle = config_update.get("setAltitude")->asFloat();
        onRemoteControlData((float)0, (float)0, (float)0, throttle);

    }, false);
}

void FlightControl::loop()
{
}
