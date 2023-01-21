/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Proximity_NanoradarMR72.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// update the state of the sensor
void AP_Proximity_NanoradarMR72::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > NRMR72_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_NanoradarMR72::distance_max() const
{
    return 40.0f;
}
float AP_Proximity_NanoradarMR72::distance_min() const
{
    return 0.20f;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_NanoradarMR72::read_sensor_data()
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t message_count = 0;
    int16_t nbytes = _uart->available();

    bool partial_header_found = false;

    while (nbytes-- > 0) {
        char c = _uart->read();
        if (static_cast<uint8_t>(c) == ((Message::HEADER >> 8*(1-partial_header_found)) & 0x0F)) {
            if (partial_header_found) {
                buffer[0] = Message::HEADER >> 8;
                buffer_count = 1;
            } 
            partial_header_found = !partial_header_found;
        } else if (partial_header_found) {
            partial_header_found = false;
        }

        buffer[buffer_count++] = c;

        // we should always read 14 bytes 
        if ((buffer_count == NRMR72_BUFFER_SIZE) && 
            (UINT16_VALUE(buffer[0], buffer[1]) == Message::HEADER) && // << This might not be needed
            (UINT16_VALUE(buffer[NRMR72_BUFFER_SIZE-2], buffer[NRMR72_BUFFER_SIZE-1]) == Message::END)){

            buffer_count = 0;

            // check the message type received
            switch(UINT16_VALUE(buffer[3], buffer[2])) {
                case Message::TYPE_TARGET_READING: { //0xAA 0xAA MsgIdL MsgIdH Id AzH RgH RgL AzL [RC Rsv VrelH] VrelL Rcs 0x55 0x55
                    const float azimuth = correct_angle_for_orientation(UINT16_VALUE(buffer[5], buffer[8])*0.01f - 90.0f); // (deg)
                    const float range   = UINT16_VALUE(buffer[6], buffer[7])*0.01f; // (m)
                    // For future usage, if needed
                    // uint8_t id = buffer[4];
                    // float rcs = buffer[11]*0.5f - 50.0f;
                    // uint8_t roll_count = buffer[9] >> 6;
                    // float rel_velocity = UINT16_VALUE(buffer[9] & 0x07, buffer[10]) * 0.05 - 35; /// (m/s)
                    update_sector_data(azimuth, range);
                    message_count++;
                    break;
                }
                case Message::TYPE_TARGET_STATUS: { //0xAA 0xAA MsgIdL MsgIdH NrTgta [Rsv RC] Rsv Rsv Rsv Rsv Rsv Rsv 0x55 0x55
                    if (buffer[4] == 0) { // No targets detected
                        update_sector_data(0,   distance_max() + 1.0f);
                        update_sector_data(45,  distance_max() + 1.0f);
                        update_sector_data(315, distance_max() + 1.0f);
                        message_count++;
                    }
                }
                case Message::TYPE_SENSOR_STATUS:
                default:
                    break;
            }
        }
    }
    return (message_count > 0);
}

// process reply
void AP_Proximity_NanoradarMR72::update_sector_data(float angle_deg, float distance_m)
{
    // Get location on 3-D boundary based on angle to the object
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
    if (!ignore_reading(angle_deg, distance_m, false)) {
        frontend.boundary.set_face_attributes(face, angle_deg, distance_m, state.instance);
        // update OA database
        database_push(angle_deg, distance_m);
    } else {
        frontend.boundary.reset_face(face, state.instance);
    }
    _last_distance_received_ms = AP_HAL::millis();
}

#endif // HAL_PROXIMITY_ENABLED
