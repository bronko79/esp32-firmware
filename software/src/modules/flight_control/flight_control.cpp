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

static void onIMUData(float lastGyro[3], float linearAccData[3], Quaternion quaternion, void *context_data){
    
    //FlightControl *self = (FlightControl *)context_data;
    FlightControl* self = static_cast<FlightControl*>(context_data);
    self->m_gyro = lastGyro;
    self->m_quaternion = quaternion;
    self->m_linearAccel = linearAccData;

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
    // 0) Zeitdelta
    const float now = timeProvider.now();
    float dt = now - m_lastTime;
    m_lastTime = now;

    if (dt <= 0.0f) {
        return;
    }
    if (dt > 0.05f) { // Ausreißer begrenzen
        dt = 0.05f;
    }

    // 1) Orientierung aus Quaternion
    m_quaternion.normalize();

    float roll_e, pitch_e, yaw_e;
    m_quaternion.toEuler(roll_e, pitch_e, yaw_e); // rad

    // 2) Quaternion-Attitude-Control (Roll/Pitch), Yaw als Rate vom Stick

    // Ziel-Tilt aus RC-Sticks (Roll/Pitch in rad)
    constexpr float MAX_TILT_DEG = 30.0f;
    const float maxTiltRad = deg2rad(MAX_TILT_DEG);

    float roll_sp  = remoteControlData.roll  * maxTiltRad;
    float pitch_sp = remoteControlData.pitch * maxTiltRad;

    // Yaw-Sollwinkel: hier aktueller Yaw (kein Heading-Hold)
    float yaw_sp = yaw_e;

    // Ziel-Quaternion
    Quaternion q_des = Quaternion::fromEuler(roll_sp, pitch_sp, yaw_sp);

    // Fehler-Quaternion: q_err = q_des * conj(q_current)
    Quaternion q_err = q_des * m_quaternion.conjugate();
    q_err.normalize();

    // „kleine“ Rotation wählen
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    // Quaternion-Fehler → Rate-Sollwerte
    // Für kleine Fehler: omega_sp ≈ 2 * Kp * (qx,qy,qz)
    float rollRateSetpoint  = 2.0f * m_kAttQuat * q_err.x;
    float pitchRateSetpoint = 2.0f * m_kAttQuat * q_err.y;

    // Yaw als Rate-Command direkt vom Stick
    constexpr float MAX_YAW_RATE_DEG = 180.0f;
    const float maxYawRateRad = deg2rad(MAX_YAW_RATE_DEG);
    float yawRateSetpoint = remoteControlData.yaw * maxYawRateRad;

    // Rate-Setpoints begrenzen
    constexpr float MAX_RATE_DEG = 400.0f;
    const float maxRateRad = deg2rad(MAX_RATE_DEG);
    rollRateSetpoint  = clamp(rollRateSetpoint,  -maxRateRad, maxRateRad);
    pitchRateSetpoint = clamp(pitchRateSetpoint, -maxRateRad, maxRateRad);

    // 3) Innerer Rate-Loop (Gyro in rad/s)
    float tauRoll  = pidRollRate.update( rollRateSetpoint,  m_gyro[0], dt );
    float tauPitch = pidPitchRate.update( pitchRateSetpoint, m_gyro[1], dt );
    float tauYaw   = pidYawRate.update( yawRateSetpoint,      m_gyro[2], dt );

    // 4) Höhe und Vertikalgeschwindigkeit aus linearer Beschleunigung

    // lineare Beschleunigung im Body-Frame (ohne g!)
    const float ax = m_linearAccel[0];
    const float ay = m_linearAccel[1];
    const float az = m_linearAccel[2];

    const float sphi = std::sin(roll_e);
    const float cphi = std::cos(roll_e);
    const float sthe = std::sin(pitch_e);
    const float cthe = std::cos(pitch_e);

    // Body -> Welt, Z-Komponente (linear, ohne g)
    // a_z_world_linear = -sin(theta)*ax + sin(phi)*cos(theta)*ay + cos(phi)*cos(theta)*az
    float a_z_world_linear = -sthe * ax + sphi * cthe * ay + cphi * cthe * az;

    // Vertikalgeschwindigkeit integrieren
    m_vz += a_z_world_linear * dt;       // [m/s]
    // Höhe integrieren
    m_altitude += m_vz * dt;            // [m]

    // 5) Throttle-Logik: Stick ≠ 0 → steigen/sinken, Stick ≈ 0 → Höhe halten

    float throttleStick = remoteControlData.throttle; // [-1,1]
    constexpr float THROTTLE_DEADBAND = 0.05f;        // Bereich um 0 für Alt-Hold
    constexpr float MAX_VZ = 2.0f;                    // m/s (limit für vertikale Geschwindigkeit)

    float vz_setpoint = 0.0f;

    if (std::fabs(throttleStick) > THROTTLE_DEADBAND) {
        // Manuelle Steig-/Sinkrate
        // Stick [-1,1] → [-MAX_VZ, +MAX_VZ]
        vz_setpoint = throttleStick * MAX_VZ;

        // Altitude-Hold ist inaktiv, Zielhöhe wird beim nächsten Zentrieren neu gesetzt
        m_altHoldActive = false;
    } else {
        // Stick im Deadband → Altitude-Hold
        if (!m_altHoldActive) {
            // Beim Übergang in den Deadband: aktuelle Höhe einfrieren
            m_altitudeSetpoint = m_altitude;
            m_altHoldActive = true;
        }

        // Höhenregler: aus Höhe → gewünschte Vertikalgeschwindigkeit
        vz_setpoint = pidAlt.update(m_altitudeSetpoint, m_altitude, dt);
        // Optional: vz_setpoint begrenzen
        if (vz_setpoint >  MAX_VZ) vz_setpoint =  MAX_VZ;
        if (vz_setpoint < -MAX_VZ) vz_setpoint = -MAX_VZ;
    }

    // 6) Vz-Regler: gewünschte vs. Ist-Vertikalgeschwindigkeit → thrustOffset
    float thrustOffset = pidVz.update(vz_setpoint, m_vz, dt);

    // Basis-Schub = fester Hover + Vz-Offset
    float thrust = thrustOffset;

    // 7) Tilt-Kompensation: bei Roll/Pitch mehr Schub, damit vertikaler Lift stimmt
    float tiltCos = std::cos(roll_e) * std::cos(pitch_e);
    const float MIN_TILTCOS = 0.5f; // max Faktor 2 Kompensation
    if (tiltCos < MIN_TILTCOS) {
        tiltCos = MIN_TILTCOS;
    }

    thrust = thrust / tiltCos;

    // 8) Schub begrenzen und an Mixer geben
    thrust = clamp(thrust, 0.0f, 1.0f);

    float motors[4];
    motorMixer.mix(thrust, tauRoll, tauPitch, tauYaw, motors);
    
    //motorOutput.writeMotors(motors);

    logger.printfln("Th: %f, Ro: %f, Pi: %f, Ya: %f ", thrust, tauRoll, tauPitch, tauYaw);
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

    pidRollRate = PID(4.0f, 0.01f, 0.001f, 10.0f, 1.0f);
    pidPitchRate = PID(4.0f, 0.01f, 0.001f, 10.0f, 1.0f);
    pidYawRate = PID(4.0f, 0.01f, 0.000f, 10.0f, 1.0f);

    // Vz-Regler (geschätzte Werte)
    pidVz  = PID(1.0f, 0.2f, 0.0f,  2.0f, 0.3f);  // output = thrustOffset

    // Höhen-Regler (langsam, sanft)
    pidAlt = PID(1.0f, 0.0f, 0.3f,  5.0f, 2.0f);  // output = vz_setpoint [m/s]

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
