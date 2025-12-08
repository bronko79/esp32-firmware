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

static void onIMUData(float lastGyro[3], Quaternion quaternion, void *context_data){
    
    //FlightControl *self = (FlightControl *)context_data;
    FlightControl* self = static_cast<FlightControl*>(context_data);
    self->m_gyro = lastGyro;
    self->m_quaternion = quaternion;

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
    float dt = now - m_lastTime;
    m_lastTime = now;

    m_quaternion.normalize(); 

    float roll_e, pitch_e, yaw_e;
    m_quaternion.toEuler(roll_e, pitch_e, yaw_e);    

    constexpr float MAX_TILT_DEG = 30.0f;
    const float maxTiltRad = deg2rad(MAX_TILT_DEG);

    float roll_sp  = remoteControlData.roll  * maxTiltRad;
    float pitch_sp = remoteControlData.pitch * maxTiltRad;

    // Yaw im Quaternion-Angle-Controller NICHT ändern (Heading hold nicht gewünscht),
    // deshalb yaw_sp = yaw_e (aktueller Yaw)
    float yaw_sp = yaw_e;

    Quaternion q_des = Quaternion::fromEuler(roll_sp, pitch_sp, yaw_sp);

    // 5. Fehler-Quaternion: q_err = q_des * conj(q_current)
    Quaternion q_err = q_des * m_quaternion.conjugate();
    q_err.normalize();

   // Sicherstellen, dass wir den "kleinen" Winkel nehmen:
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    // 6. Quaternion-Fehler → Rate-Sollwerte (Body-Frame)
    // Für kleine Fehler gilt: omega_sp ≈ 2 * Kp * (qx,qy,qz)
    float rollRateSetpoint  = 2.0f * m_kAttQuat * q_err.x;
    float pitchRateSetpoint = 2.0f * m_kAttQuat * q_err.y;

    // Yaw: weiter klassisch über Stick als Rate-Command
    constexpr float MAX_YAW_RATE_DEG = 180.0f;
    const float maxYawRateRad = deg2rad(MAX_YAW_RATE_DEG);
    float yawRateSetpoint = remoteControlData.yaw * maxYawRateRad;
    
    // Optional: Begrenzen der Setpoints
    constexpr float MAX_RATE_DEG = 400.0f;
    const float maxRateRad = deg2rad(MAX_RATE_DEG);
    rollRateSetpoint  = clamp(rollRateSetpoint,  -maxRateRad, maxRateRad);
    pitchRateSetpoint = clamp(pitchRateSetpoint, -maxRateRad, maxRateRad);

    // 7. Innerer Rate-Loop (wie gehabt)
    float tauRoll  = pidRollRate.update( rollRateSetpoint,  m_gyro[0], dt );
    float tauPitch = pidPitchRate.update( pitchRateSetpoint, m_gyro[1], dt );
    float tauYaw   = pidYawRate.update( yawRateSetpoint,      m_gyro[2], dt );

    // 8. Schub (hier z.B. erstmal linear aus Throttle, z.B. ohne Höhenregelung)
    float thrust = 0.1f + 0.9f * (remoteControlData.throttle * 0.5f + 0.5f); // [-1,1]→[0,1]
    thrust = clamp(thrust, 0.0f, 1.0f);

    // 9. Motoren mischen & ausgeben
    float motors[4];
    motorMixer.mix(thrust, tauRoll, tauPitch, tauYaw, motors);
    //m_motorOutput.writeMotors(motors);    
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

    pidRollRate = PID(0.1f, 0.01f, 0.001f, 10.0f, 0.5f);
    pidPitchRate = PID(0.1f, 0.01f, 0.001f, 10.0f, 0.5f);
    pidYawRate = PID(0.1f, 0.01f, 0.000f, 10.0f, 0.5f);

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
