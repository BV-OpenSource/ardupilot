#pragma once

#include "AP_Proximity_Backend_Serial.h"

#if HAL_PROXIMITY_ENABLED
#define NRMR72_TIMEOUT_MS            300                               // requests timeout after 0.3 seconds
#define NRMR72_BUFFER_SIZE  14

class AP_Proximity_NanoradarMR72 : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:
    class Message {
        public:
            static const uint16_t HEADER = 0xAAAA;
            static const uint16_t END = 0x5555;
            static const uint16_t TYPE_SENSOR_STATUS = 0x060A;
            static const uint16_t TYPE_TARGET_STATUS = 0x070B;
            static const uint16_t TYPE_TARGET_READING = 0x070C;
    };
    
    // check and process replies from sensor
    bool read_sensor_data();
    void update_sector_data(float angle_deg, float distance_m);

    // reply related variables
    uint8_t buffer[NRMR72_BUFFER_SIZE]; // buffer where to store data from serial
    uint8_t buffer_count;

    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
};

#endif // HAL_PROXIMITY_ENABLED
