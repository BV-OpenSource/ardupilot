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

#include "AP_Proximity_AWR1843_CAN.h"
#include <GCS_MAVLink/GCS.h>


#if HAL_PROXIMITY_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#ifndef AP_PROXIMITY_AWR1843_CAN_ENABLED
#define AP_PROXIMITY_AWR1843_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 0)
#endif

#if AP_PROXIMITY_AWR1843_CAN_ENABLED

/*
  constructor
 */
AP_Proximity_AWR1843_CAN::AP_Proximity_AWR1843_CAN(AP_Proximity& _frontend, AP_Proximity::Proximity_State& _state, AP_Proximity_Params& _params) :
    CANSensor("AWR1843"),
    AP_Proximity_Backend(_frontend, _state, _params)
{
    register_driver(AP_CANManager::Driver_Type_AWR1843);

    parser_state = ParserState::READ_MAGIC_WORD;
    tlv_type = TLVType::MSG_NULL;

    buffer_read = nullptr;
    buffer_bytes_used = 0;
    buffer_read_alloc_size = 0;
}

// update the state of the sensor
void AP_Proximity_AWR1843_CAN::update(void)
{
    // if (_uart == nullptr) {
    //     return;
    // }

    // process incoming messages
    // read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > NRMR72_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_AWR1843_CAN::distance_max() const
{
    return 100.0f;
}
float AP_Proximity_AWR1843_CAN::distance_min() const
{
    return 0.50f;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_AWR1843_CAN::read_sensor_data()
{
    // if (_uart == nullptr) {
    //     return false;
    // }

    uint16_t message_count = 0;
    int16_t nbytes = 20; //_uart->available();

    bool partial_header_found = false;

    while (nbytes-- > 0) {
        char c = 'a'; //_uart->read();
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
                    // const float azimuth = correct_angle_for_orientation(UINT16_VALUE(buffer[5], buffer[8])*0.01f - 90.0f); // (deg)
                    // const float range   = UINT16_VALUE(buffer[6], buffer[7])*0.01f; // (m)
                    
                    // For future usage, if needed
                    // uint8_t id = buffer[4];
                    // float rcs = buffer[11]*0.5f - 50.0f;
                    // uint8_t roll_count = buffer[9] >> 6;
                    // float rel_velocity = UINT16_VALUE(buffer[9] & 0x07, buffer[10]) * 0.05 - 35; /// (m/s)
                    // update_sector_data(azimuth, range);
                    message_count++;
                    break;
                }
                case Message::TYPE_TARGET_STATUS: { //0xAA 0xAA MsgIdL MsgIdH NrTgta [Rsv RC] Rsv Rsv Rsv Rsv Rsv Rsv 0x55 0x55
                    if (buffer[4] == 0) { // No targets detected
                        // update_sector_data(0,   distance_max() + 1.0f);
                        // update_sector_data(45,  distance_max() + 1.0f);
                        // update_sector_data(315, distance_max() + 1.0f);
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
void AP_Proximity_AWR1843_CAN::update_sector_data(float azimuth_deg, float elevation_deg, float distance_m)
{
    // Get location on 3-D boundary based on angle to the object
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(elevation_deg, azimuth_deg);
    if (!ignore_reading(elevation_deg, azimuth_deg, distance_m, false)) {
        frontend.boundary.set_face_attributes(face, elevation_deg, azimuth_deg, distance_m, state.instance);
        // update OA database
        database_push(azimuth_deg, elevation_deg, distance_m);
    } else {
        frontend.boundary.reset_face(face, state.instance);
    }
    _last_distance_received_ms = AP_HAL::millis();
}

// Find Magic Word signaling the start of the frame
int16_t AP_Proximity_AWR1843_CAN::find_magic_word(std::vector<uint8_t> *data, uint8_t added_bytes) 
{
    const uint8_t magic_word_size = sizeof(magic_word);
    const uint16_t data_size = data->size();
    const uint16_t init_byte = std::max(data_size - added_bytes - magic_word_size, 0); // Avoid check bytes already checked. 
    // Repeats last check to cover the case vector size was smaller than magic_word_size in previous iteration.

    for (uint16_t i = init_byte; i <= data_size-magic_word_size; i++)
    {
        uint8_t j = 0;
        for (j = 0; j < magic_word_size; j++)
        {
            if(magic_word[j] != (*data)[i+j]) 
            {
                break;
            }
        }

        if (j == sizeof(magic_word)) 
        {
            return i;
        }
    }

    return -1;
}

void AP_Proximity_AWR1843_CAN::sync_buffer(std::vector<uint8_t> *data, int16_t n_pos) 
{
    if (n_pos < 0) // This should not be needed, but kept as protection
    {
        return;
    }

    data->erase(data->begin(), data->begin()+n_pos);
}

uint32_t AP_Proximity_AWR1843_CAN::uint32_from_bytes_LE(uint8_t *byte_array)
{
    return byte_array[0] + (byte_array[1] << 8) + (byte_array[2] << 16) + (byte_array[2] << 24); 
}

// handler for incoming frames.
void AP_Proximity_AWR1843_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    // uint8_t frame_bytes = (frame.isCanFDFrame() ? 64:8);
    uint8_t frame_bytes = frame.dlcToDataLength(frame.dlc);

    // gcs().send_text(MAV_SEVERITY_INFO, "DATA Len: %u", frame_bytes);

    buf_read.insert(buf_read.end(), frame.data, frame.data + frame_bytes);


    // if (buffer_read == nullptr && parser_state == ParserState::READ_HEADER) 
    // {
    //     buffer_read_alloc_size = frame_bytes > 2*sizeof(magic_word) ? frame_bytes : 2*sizeof(magic_word);
    //     buffer_read = (uint8_t*)calloc(buffer_read_alloc_size, sizeof(uint8_t));

    //     if(buffer_read == NULL) 
    //     {
    //         gcs().send_text(MAV_SEVERITY_ERROR, "AWR1843_CAN: Error allocating data frame memory!");
    //         buffer_read = nullptr;
    //         buffer_read_alloc_size = 0;
    //         return;
    //     }
    // }

    // if (buffer_read_alloc_size - buffer_bytes_used < frame_bytes) 
    // {
    //     // Need to realloc the buffer if there is no space allocated for the new frame;
    //     uint8_t *new_buffer_read;
    //     new_buffer_read = (uint8_t*)std::realloc(buffer_read, (frame_bytes + buffer_bytes_used)*sizeof(uint8_t));

    //     if(new_buffer_read == NULL) 
    //     {
    //         gcs().send_text(MAV_SEVERITY_ERROR, "AWR1843_CAN: Error reallocating data frame memory!");
    //         free(buffer_read);
    //         buffer_read = nullptr;
    //         buffer_bytes_used = 0;
    //         buffer_read_alloc_size = 0;
    //         return;
    //     }

    //     buffer_read = new_buffer_read;
    //     buffer_read_alloc_size = frame_bytes + buffer_bytes_used;
    // }

    // // Enough memory shall be allocated at this point
    // memcpy(buffer_read + buffer_bytes_used, frame.data, frame_bytes);

    // gcs().send_text(MAV_SEVERITY_INFO, "Proximity read S: %u [%u] %u %u %u %u %u %u %u %u", parser_state, buf_read.size(), (frame.data[0]), unsigned(frame.data[1]), unsigned(frame.data[2]), unsigned(frame.data[3]), unsigned(frame.data[4]), unsigned(frame.data[5]), unsigned(frame.data[6]), unsigned(frame.data[7]));

    // Using if statements instead of switch case to allow fall through within the same reading
    if (parser_state == ParserState::READ_MAGIC_WORD)
    {
        const int16_t offset_bytes = find_magic_word(&buf_read, frame_bytes);
        if (offset_bytes > 0) 
        { 
            sync_buffer(&buf_read, offset_bytes);

            // gcs().send_text(MAV_SEVERITY_INFO, "Magic Word found");
        } else if(offset_bytes < 0) {
            // Keep searching for Magic Word on the next frame if not found
            return;
        }
        
        num_tlvs_read = 0;
        num_detobj_read = 0;
        num_clus_read = 0;
        num_track_read = 0;
        last_pos_read = sizeof(magic_word);
        parser_state = ParserState::READ_HEADER;
    }

    if (parser_state == ParserState::READ_HEADER)
    {
        if (buf_read.size() >= last_pos_read + sizeof(MessageHeader)) 
        {

            memcpy(&packet_header, &buf_read[last_pos_read], sizeof(MessageHeader));

            // expected_bytes_total = uint32_from_bytes_LE(&buf_read[sizeof(magic_word)+4]); // 4-byte Version Number after magic byte
            // gcs().send_text(MAV_SEVERITY_INFO, "Expected payload %u %ld and %ld or %ld", packet_header.version.major, expected_bytes_total, uint32_from_bytes_LE(&buf_read[sizeof(magic_word)+4]), packet_header.totalPacketLen);
            
            parser_state = ParserState::CHECK_TLV_TYPE;
            last_pos_read += sizeof(MessageHeader);
            
            if(packet_header.numTLVs == 0) { // Nothing to process, but the sensor is alive
                parser_state = ParserState::READ_MAGIC_WORD;

                update_sector_data(0,  0, distance_max()+1);
                update_sector_data(45, 0, distance_max()+1);
                update_sector_data(90, 0, distance_max()+1);
                update_sector_data(315, 0, distance_max()+1);
                update_sector_data(270, 0, distance_max()+1);
            }
        }
    }

    while ( parser_state != ParserState::READ_MAGIC_WORD &&  
            parser_state != ParserState::READ_HEADER && 
            num_tlvs_read < packet_header.numTLVs) 
    {
        if (parser_state == ParserState::CHECK_TLV_TYPE)
        {
            if (buf_read.size() >= last_pos_read + sizeof(TLV))
            {
                memcpy(&tlv_read, &buf_read[last_pos_read], sizeof(TLV));

                // gcs().send_text(MAV_SEVERITY_INFO, "TLV Type: %lu", tlv_read.type);

                switch ((TLVType)tlv_read.type)
                {
                case TLVType::MSG_DETECTED_POINTS:
                    parser_state = ParserState::READ_DETECTED_POINTS;
                    break;
                case TLVType::MSG_CLUSTERS:
                    parser_state = ParserState::READ_CLUSTERS;
                    break;
                case TLVType::MSG_TRACKED_OBJ:
                    parser_state = ParserState::READ_TRACKED_OBJECTS;
                    break;
                default: // TLV type unknown. Reject remaining packet
                    parser_state = ParserState::READ_MAGIC_WORD;
                    break;
                }

                last_pos_read += sizeof(TLV);
            }
            else // Need to read more data. Wait for next frame
            {
                break;
            }
        }

        if (parser_state == ParserState::READ_DETECTED_POINTS)
        {
            while (buf_read.size() >= last_pos_read + sizeof(DetectedObj))
            {
                memcpy(&detected_obj_read, &buf_read[last_pos_read], sizeof(DetectedObj));

                // X - right
                // Y - front
                // Z - up

                float obj_x = (float)detected_obj_read.x / pow(2, tlv_read.xyzQFormat);
                float obj_y = (float)detected_obj_read.y / pow(2, tlv_read.xyzQFormat);
                float obj_z = (float)detected_obj_read.z / pow(2, tlv_read.xyzQFormat);
                // float obj_peakValue = detected_obj_read.peakVal / pow(2, tlv_read.xyzQFormat);
                // float obj_doppler = detected_obj_read.dopplerIdx / pow(2, tlv_read.xyzQFormat);

                float range = sqrt(obj_x*obj_x + obj_y*obj_y + obj_z*obj_z );

                float azimuth_deg = 0;
                if(detected_obj_read.x != 0) {
                    azimuth_deg = atan2(obj_x, obj_y) * 180.0f/M_PI;
                }
                
                float hor_range = sqrt(obj_x*obj_x + obj_y*obj_y);

                float elevation_deg = 0;
                if(detected_obj_read.z != 0) {
                    elevation_deg = atan2(obj_z, hor_range) * 180.0f/M_PI;
                }

                // float intensity = 10 * log10(obj_peakValue + 1);
                // float velocity = obj_doppler;
                
                // gcs().send_text(MAV_SEVERITY_INFO, "Object X:%f Y:%f Z:%f", obj_x, obj_y, obj_z);
                // gcs().send_text(MAV_SEVERITY_INFO, "Object R:%f A:%f E:%f", range, azimuth_deg, elevation_deg);

                azimuth_deg = correct_angle_for_orientation(azimuth_deg); // (deg)

                update_sector_data(azimuth_deg, elevation_deg, range);

                last_pos_read += sizeof(DetectedObj);
                num_detobj_read++;
            }

            if(num_detobj_read == tlv_read.numObjOut) 
            {
                num_tlvs_read++;
                // if (tlv_read.length % frame_bytes > 0) { // Skip some dummy bytes due to CAN communication
                //     last_pos_read += frame_bytes-(tlv_read.length % frame_bytes);
                // }

                if(num_tlvs_read >= packet_header.numTLVs) // Packet fully processed
                {
                    parser_state = ParserState::READ_MAGIC_WORD;
                }
                else // More TLV to process
                {
                    parser_state = ParserState::CHECK_TLV_TYPE;
                }

                num_detobj_read = 0;
            }
            else // Need to read more data. Wait for next frame
            {
                break;
            }
        }

        if (parser_state == ParserState::READ_CLUSTERS)
        {
            while (buf_read.size() >= last_pos_read + sizeof(Cluster))
            {
                memcpy(&cluster_read, &buf_read[last_pos_read], sizeof(Cluster));
                // gcs().send_text(MAV_SEVERITY_INFO, "Cluster X:%d Y:%d", cluster_read.xCenter, cluster_read.yCenter);
                // TODO: treat values

                last_pos_read += sizeof(Cluster);
                num_clus_read++;
            }

            if(num_clus_read == tlv_read.numObjOut) 
            {
                num_tlvs_read++;
                // if (tlv_read.length % frame_bytes > 0) { // Skip some dummy bytes due to CAN communication
                //     last_pos_read += frame_bytes-(tlv_read.length % frame_bytes);
                // }

                if(num_tlvs_read >= packet_header.numTLVs) // Packet fully processed
                {
                    parser_state = ParserState::READ_MAGIC_WORD;
                }
                else // More TLV do deal
                {
                    parser_state = ParserState::CHECK_TLV_TYPE;
                }

                num_clus_read = 0;
            }
            else // Need to read more data. Wait for next frame
            {
                break;
            }
        }

        if (parser_state == ParserState::READ_TRACKED_OBJECTS)
        {
            while (buf_read.size() >= last_pos_read + sizeof(TrackedObj))
            {
                memcpy(&tracked_obj_read, &buf_read[last_pos_read], sizeof(TrackedObj));
                // gcs().send_text(MAV_SEVERITY_INFO, "Tracked X:%d Y:%d", tracked_obj_read.x, tracked_obj_read.y);

                // TODO: treat values

                last_pos_read += sizeof(TrackedObj);
                num_track_read++;
            }

            if(num_track_read >= tlv_read.numObjOut) 
            {
                num_tlvs_read++;
                // if (tlv_read.length % frame_bytes > 0) { // Skip some dummy bytes due to CAN communication
                //     last_pos_read += frame_bytes-(tlv_read.length % frame_bytes);
                // }

                if(num_tlvs_read == packet_header.numTLVs) // Packet fully processed
                {
                    parser_state = ParserState::READ_MAGIC_WORD;
                }
                else // More TLV do deal
                {
                    parser_state = ParserState::CHECK_TLV_TYPE;
                }

                num_track_read = 0;
            }
            else // Need to read more data. Wait for next frame
            {
                break;
            }
        }
    }


    // if (parser_state == ParserState::READ_DETECTED_POINTS ||
    //     parser_state == ParserState::READ_CLUSTERS ||
    //     parser_state == ParserState::READ_TRACKED_OBJECTS)
    // {
    //     if(num_tlvs_read == packet_header.numTLVs) // Packet fully processed
    //     {
    //         parser_state = ParserState::READ_MAGIC_WORD;
    //     }
    //     else // More TLV do deal
    //     {
    //         parser_state = ParserState::CHECK_TLV_TYPE;
    //     }
    // }
    // switch (parser_state)
    // {
    // case ParserState::READ_MAGIC_WORD:
    //     {
    //         const int16_t offset_bytes = find_magic_word(&buf_read, frame_bytes);
    //         if (offset_bytes > 0) 
    //         { 
    //             sync_buffer(&buf_read, offset_bytes);

    //             gcs().send_text(MAV_SEVERITY_INFO, "Magic Word found");
    //         } else if(offset_bytes < 0) {
    //             // Keep searching for Magic Word on the next frame if not found
    //             break;
    //         }
            
    //         parser_state = ParserState::READ_HEADER;
    //         break; // NEED to CHECK this
    //     }
    // case ParserState::READ_HEADER:
    //     {
    //         if (buf_read.size() >= sizeof(magic_word) + sizeof(MessageHeader)) 
    //         {

    //             memcpy(&packet_header, &buf_read[sizeof(magic_word)], sizeof(packet_header));

    //             // expected_bytes_total = uint32_from_bytes_LE(&buf_read[sizeof(magic_word)+4]); // 4-byte Version Number after magic byte
    //             // gcs().send_text(MAV_SEVERITY_INFO, "Expected payload %u %ld and %ld or %ld", packet_header.version.major, expected_bytes_total, uint32_from_bytes_LE(&buf_read[sizeof(magic_word)+4]), packet_header.totalPacketLen);
    //             parser_state = ParserState::CHECK_TLV_TYPE;
    //         }
    //         break;
    //     }
    // case ParserState::CHECK_TLV_TYPE:
    //     {
    //         if (buf_read.size() >= sizeof(magic_word) + sizeof(MessageHeader) + sizeof(TLV)) 
    //         {
    //             memcpy(&tlv_read, &buf_read[sizeof(magic_word) + sizeof(MessageHeader)], sizeof(tlv_read));

    //             switch (tlv_read.type)
    //             {
    //             case TLVType::MSG_DETECTED_POINTS:
                    
    //                 break;
    //             case TLVType::MSG_CLUSTERS:
    //                 /* code */
    //                 break;
    //             case TLVType::MSG_TRACKED_OBJ:
    //                 /* code */
    //                 break;
    //             default:
    //                 break;
    //             }
    //         }
    //     }
    // default:
    //     break;
    // }


    // WITH_SEMAPHORE(_sem);
    // const uint16_t dist_cm = (frame.data[0]<<8) | frame.data[1];
    // _distance_sum += dist_cm * 0.01;
    // _distance_count++;
}

#endif // AP_PROXIMITY_AWR1843_CAN_ENABLED
#endif // HAL_PROXIMITY_ENABLED
