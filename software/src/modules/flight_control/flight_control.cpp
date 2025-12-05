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
//TF_DistanceIRV2 distance_ir_v2;
TF_IMUV3 imu;

void static quaternion_handler(TF_IMUV3 *device, int16_t w, int16_t x, int16_t y, int16_t z, void *user_data) {
    
    FlightControl *flightControl = (FlightControl *)user_data;                            
    //logger.printfln("X: %f", x / (float)16383);
}

void FlightControl::pre_setup()
{   
    config = Config::Object({
        {"setAltitude", Config::Float((float)0)}
    });

    config_update = config;
}

void FlightControl::setup()
{
    logger.printfln("FlightControl module initializing...");

    tf_imu_v3_create(&imu, NULL, &hal);
    tf_imu_v3_register_quaternion_callback(&imu, quaternion_handler, this);
    tf_imu_v3_set_quaternion_callback_configuration(&imu, 1000, false);

    initialized = true;
    logger.printfln("FlightControl module initialized");
}

void FlightControl::register_urls()
{
    api.addState("flight_control/config", &config, {}, {}, true);

    api.addCommand("flight_control/config_update", &config_update, {}, [this](String &/*errmsg*/) {
        /*
            Received from FrontEnd
        */
        logger.printfln("Flight Control received update.");
        float receivedSetAltitude = config_update.get("setAltitude")->asFloat();
        //logger.printfln("Flight Control received update: %f", receivedSetAltitude);
    }, false);
}

void FlightControl::loop()
{
}
