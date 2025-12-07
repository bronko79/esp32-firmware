#pragma once

#include "bindings/hal_common.h"
#include "bindings/bricklet_imu_v3.h"

#define M_PI 3.14159265358979323846
 

typedef void (*IMUUserCallback)(float lastGyro[3], float lastAccel[3], void *context_data);

class TinkerforgeIMU {

public:
    TinkerforgeIMU();
    IMUUserCallback m_userCallback = nullptr;
    void *m_context_data;

    int start(TF_HAL *hal, uint32_t period, IMUUserCallback callback, void *context_data);

    
private:    
    TF_IMUV3 imu;
    

};