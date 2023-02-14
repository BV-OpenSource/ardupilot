#pragma once

#include "AP_Proximity_Backend_Serial.h"
#include <AP_CANManager/AP_CANSensor.h>

#if HAL_PROXIMITY_ENABLED
#define AWR1843_TIMEOUT_MS            500                               // requests timeout after 0.3 seconds

class AP_Proximity_AWR1843_CAN : public CANSensor, public AP_Proximity_Backend
{

public:
    AP_Proximity_AWR1843_CAN(AP_Proximity& _frontend, AP_Proximity::Proximity_State& _state, AP_Proximity_Params& _params);
    // using AP_Proximity_Backend::AP_Proximity_Backend;
    ~AP_Proximity_AWR1843_CAN(void) {}

    // update state
    void update(void) override;

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:
    uint8_t magic_word[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

    enum ParserState {
        READ_MAGIC_WORD = 0,
        READ_HEADER,
        READ_PAYLOAD,
        CHECK_TLV_TYPE,
        READ_DETECTED_POINTS,
        READ_CLUSTERS,
        READ_TRACKED_OBJECTS
    };

    enum TLVType {
        MSG_NULL = 0,
        MSG_DETECTED_POINTS,
        MSG_CLUSTERS,
        MSG_TRACKED_OBJ,
        MSG_PARKING_ASSIST
    };

    struct Version {
        uint8_t build;
        uint8_t bugfix;
        uint8_t minor;
        uint8_t major;
    };

    struct MessageHeader  {
        /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
        Version     version;

        /*! @brief   Total packet length including header in Bytes */
        uint32_t    totalPacketLen;

        /*! @brief   platform type */
        uint32_t    platform;

        /*! @brief   Frame number */
        uint32_t    frameNumber;

        /*! @brief   Time in CPU cycles when the message was created. For XWR16xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
        uint32_t    timeCpuCycles;
        
        /*! @brief   Number of detected objects */
        uint32_t    numDetectedObj;

        /*! @brief   Number of TLVs */
        uint32_t    numTLVs;

        /*! @brief   Sub-frame Number (not used with XWR14xx) */
        uint32_t    subFrameNumber;
    };

    struct TLV {
        uint32_t type;
        uint32_t length;
        uint16_t numObjOut;
        uint16_t xyzQFormat;
    };

    struct DetectedObj {
        // uint16_t   rangeIdx;     /*!< @brief Range index */
        uint16_t   dopplerIdx;   /*!< @brief Dopler index */
        uint16_t   peakVal;      /*!< @brief Peak value */
        int16_t  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
        int16_t  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
        int16_t  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
    };

    struct Cluster {   
        int16_t  xCenter;
        int16_t  yCenter;
        int16_t  xSize;
        int16_t  ySize;
    };

    struct TrackedObj {   
        int16_t  x;
        int16_t  y;
        int16_t  xd;
        int16_t  yd;
        int16_t  xSize;
        int16_t  ySize;
    };

    ParserState parser_state;
    std::vector<uint8_t> buf_read;

    // check and process replies from sensor
    void update_sector_data(float azimuth_deg, float elevation_deg, float distance_m);
    int16_t find_magic_word(std::vector<uint8_t> *data, uint8_t added_bytes);
    void sync_buffer(std::vector<uint8_t> *data, int16_t n_pos);

    // reply related variables
    MessageHeader packet_header;
    TLV tlv_read;
    DetectedObj detected_obj_read;
    Cluster cluster_read;
    TrackedObj tracked_obj_read;

    uint32_t num_tlvs_read;
    uint32_t num_detobj_read;
    uint32_t num_clus_read;
    uint32_t num_track_read;
    uint32_t last_pos_read;

    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
};

#endif // HAL_PROXIMITY_ENABLED
