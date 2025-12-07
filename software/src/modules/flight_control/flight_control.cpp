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

static void onIMUData(const float lastGyro[3], const float lastAccel[3], void *context_data){
    
    FlightControl *self = (FlightControl *)context_data;
    logger.printfln("FC lastGyro %f", lastGyro[0]);
}

void FlightControl::onRemoteControlData(float roll, float pitch, float yaw, float throttle) {
    logger.printfln("FC throttle %f", throttle);
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
    uint16_t interval = 1000;

    logger.printfln("FlightControl module initializing...");
    tinkerforgeIMU = TinkerforgeIMU();
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

/*
void FlightControl::mixer()
{
    float motor_out[4];

    float altitudeAccelerationToZero = flightControlSetPoints.altitudeAcceleration - flightControlCurrentValues.altitudeAcceleration;
    float rollToZero = flightControlSetPoints.roll - flightControlCurrentValues.roll;
    float pitchToZero = flightControlSetPoints.pitch - flightControlCurrentValues.pitch;
    float yawToZero = flightControlSetPoints.yaw - flightControlCurrentValues.yaw;
    //logger.printfln("Flight Control received update.");

    

    float base = 0; //flightControlSettings.idleThrottle + altitudeAccelerationToZero;
    float newRoll = rollToZero;
    float newPitch = pitchToZero;
    float newYaw = yawToZero;

    motor_out[0] = base + newPitch + newRoll - newYaw; // FL
    motor_out[1] = base + newPitch - newRoll + newYaw; // FR
    motor_out[2] = base - newPitch - newRoll - newYaw; // RR
    motor_out[3] = base - newPitch + newRoll + newYaw; // RL

    state.get("motor")->updateFloat(motor_out[0]);

}
*/