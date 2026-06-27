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

/*
  support for Inertial Sense INS
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>

class AP_ExternalAHRS_InertialSense : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_InertialSense(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override;

protected:
    uint8_t num_gps_sensors(void) const override;

private:
    /*
     * Inertial Sense ISB protocol types, condensed from ISComm.h and base_port.h.
     * Only the subset needed by this driver is included.  Data-set payload types
     * (ins_3_t, etc.) are forward-declared here and fully defined in the .cpp.
     */

    // Forward declarations for ISB data-set payload types (defined in .cpp).
    struct dev_info_t;
    struct pimu_t;
    struct ins_3_t;
    struct gnss_pos_t;
    struct gnss_vel_t;
    struct gnss_rtk_misc_t;
    struct magnetometer_t;
    struct barometer_t;
    struct inl2_ned_sigma_t;
    struct sys_params_t;
    struct bit_t;

    // Opaque port handle (from base_port.h).
    using port_handle_t = void*;

    enum ProtocolType
    {
        _PTYPE_NONE                 = 0,
        _PTYPE_PARSE_ERROR          = 1,
        _PTYPE_INERTIAL_SENSE_ACK   = 2,
        _PTYPE_INERTIAL_SENSE_CMD   = 3,
        _PTYPE_INERTIAL_SENSE_DATA  = 4,
        _PTYPE_NMEA                 = 5,
        _PTYPE_UBLOX                = 6,
        _PTYPE_RTCM3                = 7,
        _PTYPE_SPARTN               = 8,
        _PTYPE_SONY                 = 9,
        _PTYPE_SEPTENTRIO_SBF       = 10,
        _PTYPE_SEPTENTRIO_REPLY     = 11,
        _PTYPE_FIRST_DATA           = _PTYPE_INERTIAL_SENSE_DATA,
        _PTYPE_LAST_DATA            = _PTYPE_SEPTENTRIO_REPLY,
        _PTYPE_SIZE                 = _PTYPE_LAST_DATA + 1,
    };

    enum IsbPacketFlags
    {
        PKT_TYPE_INVALID                        = 0,
        PKT_TYPE_ACK                            = 1,
        PKT_TYPE_NACK                           = 2,
        PKT_TYPE_GET_DATA                       = 3,
        PKT_TYPE_DATA                           = 4,
        PKT_TYPE_SET_DATA                       = 5,
        PKT_TYPE_STOP_BROADCASTS_ALL_PORTS      = 6,
        PKT_TYPE_STOP_DID_BROADCAST             = 7,
        PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT   = 8,
        PKT_TYPE_COUNT                          = 9,
        PKT_TYPE_MAX_COUNT                      = 16,
        PKT_TYPE_MASK                           = 0x0F,
        ISB_FLAGS_MASK                          = 0xF0,
        ISB_FLAGS_EXTENDED_PAYLOAD              = 0x10,
        ISB_FLAGS_PAYLOAD_W_OFFSET              = 0x20,
    };

    enum ProtocolMask
    {
        ENABLE_PROTOCOL_ISB     = (0x00000001 << _PTYPE_INERTIAL_SENSE_DATA),
        ENABLE_PROTOCOL_NMEA    = (0x00000001 << _PTYPE_NMEA),
        ENABLE_PROTOCOL_UBLOX   = (0x00000001 << _PTYPE_UBLOX),
        ENABLE_PROTOCOL_RTCM3   = (0x00000001 << _PTYPE_RTCM3),
        ENABLE_PROTOCOL_SPARTN  = (0x00000001 << _PTYPE_SPARTN),
        ENABLE_PROTOCOL_SONY    = (0x00000001 << _PTYPE_SONY),
        ENABLE_PROTOCOL_SBF     = (0x00000001 << _PTYPE_SEPTENTRIO_SBF),
        ENABLE_PROTOCOL_SEPT_REPLY = (0x00000001 << _PTYPE_SEPTENTRIO_REPLY),
    };

    static constexpr uint32_t MAX_DATASET_SIZE = 1024;
    static constexpr uint32_t PKT_BUF_SIZE     = 2048;
    static constexpr uint32_t DEFAULT_PROTO_MASK = ENABLE_PROTOCOL_ISB;

    struct PACKED packet_hdr_t
    {
        uint16_t    preamble;
        uint8_t     flags;
        uint8_t     id;
        uint16_t    payloadSize;
    };

    struct PACKED p_data_hdr_t
    {
        uint8_t     id;
        uint16_t    size;
        uint16_t    offset;
    };

    struct PACKED bufPtr_t
    {
        uint8_t     *ptr;
        uint32_t    size;
    };

    struct PACKED packet_t
    {
        union
        {
            struct
            {
                packet_hdr_t    hdr;
                uint16_t        offset;
            };
            struct
            {
                uint16_t        preamble;
                uint8_t         flags;
                p_data_hdr_t    dataHdr;
            };
        };
        bufPtr_t    data;
        uint16_t    hdrCksum;
        uint16_t    checksum;
        uint16_t    size;
        uint16_t    id;
    };

    struct PACKED packet_buf_t
    {
        packet_hdr_t    hdr;
        union
        {
            uint8_t     data;
            uint16_t    offset;
        }               payload;
    };

    struct PACKED p_data_t
    {
        p_data_hdr_t    hdr;
        uint8_t         *ptr;
    };

    struct PACKED p_data_buf_t
    {
        p_data_hdr_t    hdr;
        uint8_t         buf[MAX_DATASET_SIZE];
    };

    struct PACKED p_data_get_t
    {
        uint16_t    id;
        uint16_t    size;
        uint16_t    offset;
        uint16_t    period;
    };

    enum ParseErrorType {
        EPARSE_INVALID_PREAMBLE,
        EPARSE_INVALID_SIZE,
        EPARSE_INVALID_CHKSUM,
        EPARSE_INVALID_DATATYPE,
        EPARSE_MISSING_EOS_MARKER,
        EPARSE_INCOMPLETE_PACKET,
        EPARSE_INVALID_HEADER,
        EPARSE_INVALID_PAYLOAD,
        EPARSE_RXBUFFER_FLUSHED,
        EPARSE_STREAM_UNPARSABLE,
        NUM_EPARSE_ERRORS
    };

    struct is_comm_parser_t
    {
        int16_t     state;
        uint16_t    size;
        uint32_t    timeMs;
    };

    using pFnProcessPkt = ProtocolType(*)(void*);

    using pfnIsCommPortWrite = int(*)(port_handle_t port, const uint8_t* buf, int len);
    using pfnIsCommPortRead = int(*)(port_handle_t port, uint8_t* buf, int bufLen);
    using pfnIsCommIsbDataHandler = int(*)(void* ctx, p_data_t* data, port_handle_t port);
    using pfnIsCommGenMsgHandler = int(*)(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port);
    using pfnIsCommHandler = int(*)(void* ctx, ProtocolType ptype, packet_t *pkt, port_handle_t port);

    struct is_comm_buffer_t
    {
        uint8_t*    start;
        uint8_t*    end;
        uint32_t    size;
        uint8_t*    head;
        uint8_t*    tail;
        uint8_t*    scan;
        uint8_t*    scanPrior;
    };

    struct is_comm_callbacks_t
    {
        uint32_t                    protocolMask;
        void*                       context;
        pfnIsCommHandler            all;
        pfnIsCommIsbDataHandler     isbData;
        pfnIsCommGenMsgHandler      generic[_PTYPE_SIZE];
    };

    struct is_comm_instance_t
    {
        is_comm_buffer_t    rxBuf;
        uint32_t            txPktCount;
        uint32_t            rxPktCount;
        uint32_t            rxErrorCount;
        ParseErrorType      rxErrorType;
        uint32_t            rxErrorTypeCount[NUM_EPARSE_ERRORS];
        pFnProcessPkt       processPkt;
        is_comm_parser_t    parser;
        uint32_t            ackNeeded;
        packet_t            rxPkt;
        uint8_t             rxErrorState;
        is_comm_callbacks_t cb;
    };

    static void dev_info_populate_missing_hardware(dev_info_t *devInfo);
    static uint16_t isb_fletcher16(uint16_t cksum_init, const void* data, uint32_t size);
    static int is_comm_reset_buffer(is_comm_instance_t* c);
    static void is_comm_reset_parser(is_comm_instance_t* c);
    static void is_comm_to_isb_p_data(const is_comm_instance_t *comm, p_data_t *data);
    static ProtocolType report_parse_error(is_comm_instance_t* c, ParseErrorType errorType);
    static ProtocolType parse_error_reset_state(is_comm_instance_t* c, ParseErrorType errorType);
    static void valid_packet_reset(is_comm_instance_t* c, int pktSize);
    static void set_parser_start(is_comm_instance_t* c, pFnProcessPkt processPkt);
    static ProtocolType process_isb_pkt(void* v);
    static void is_comm_init(is_comm_instance_t* c, uint8_t *buf, int bufSize, pfnIsCommHandler pktHandler);
    static void is_comm_enable_protocol(is_comm_instance_t* comm, ProtocolType ptype);
    static pfnIsCommIsbDataHandler is_comm_register_isb_handler(is_comm_instance_t* comm, pfnIsCommIsbDataHandler cbHandler);
    static int is_comm_free(is_comm_instance_t* c);
    static void is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);
    static void memcpy_inc_update_checksum(uint8_t **dstBuf, const uint8_t* srcBuf, int len, uint16_t *checksum);
    static int is_comm_write_isb_precomp_to_buffer(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, packet_t *pkt);
    static int is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);
    static int is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, uint32_t did, uint32_t size, uint32_t offset, uint32_t periodMultiple);
    static ProtocolType is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs);
    static ProtocolType is_comm_parse(is_comm_instance_t* comm);
    static void parse_messages(is_comm_instance_t* comm, port_handle_t port);
    static void is_comm_buffer_parse_messages(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm);

    void initialize();
    void start();
    void update_thread();
    void read_fifo();
    bool write_buffer(int size);

    // True when EAHRS_OPTIONS selects "use device as a raw IMU": the on-device
    // EKF is disabled and we consume PIMU only (no INS solution messages).
    bool use_as_imu() const;

    // Partial write of a DID_FLASH_CONFIG field (SET_DATA with byte offset).
    bool write_flash_config_field(uint16_t offset, const void *data, uint16_t size);

    // Deferred GCS reporting. The read thread runs at PRIORITY_SPI and must not
    // call GCS_SEND_TEXT directly: send_textv() fans out into AP_Logger,
    // AP_Notify and the telemetry singletons, which are owned by the main loop.
    // Instead the thread queues text here and update() (main thread) drains it.
    void queue_gcs_text(uint8_t severity, const char *fmt, ...) FMT_PRINTF(3, 4);
    void send_pending_text();

    int stop_message_broadcasting();
    int enable_message_broadcasting();

    void handle_pimu_message(pimu_t* pimu);
    void handle_ins3_message(ins_3_t* ins);
    void handle_gnss_pos_message(gnss_pos_t* pos);
    void handle_gnss_vel_message(gnss_vel_t* vel);
    void handle_gnss_rtk_pos_misc_message(gnss_rtk_misc_t* misc);
    void handle_gnss2_pos_message(gnss_pos_t* pos);
    void handle_gnss2_vel_message(gnss_vel_t* vel);
    void handle_flash_config_message(const uint8_t *raw, uint16_t size);
    void handle_magnetometer_message(magnetometer_t* mag);
    void handle_barometer_message(barometer_t* bar);
    void handle_inl2_ned_sigma_message(inl2_ned_sigma_t *sigmas);
    void handle_sys_params_message(sys_params_t *sys);
    void handle_dev_info_message(dev_info_t *dev_info);
    void handle_bit_message(bit_t* bit);
    int parse_isb_data(void* ctx, p_data_t* data, port_handle_t port);

    // callback helper
    static AP_ExternalAHRS_InertialSense *instance;
    static int isb_data_handler(void* ctx, p_data_t* data, port_handle_t port) {
        return instance->parse_isb_data(ctx, data, port);
    }
    bool initialized = false;
    bool _healthy = false;
    AP_GPS_FixType _fix_type = AP_GPS_FixType::NONE;

    uint32_t baudrate;
    int8_t port_num = -1;
    uint8_t buffer[64];          // TX scratch (request/stop packets are small)

    // RX drain buffer. The IMU sample rate seen by the EKF equals the rate at
    // which complete PIMU packets are parsed, which is gated by how many bytes
    // we pull off SPI per poll. 64 B / 2 ms (~32 KB/s) throttled the whole
    // subscription set so PIMU only emerged at ~81 Hz. Drain a much larger
    // chunk per poll so SPI bandwidth is no longer the limiter.
    uint8_t rx_buffer[512];

    // PIMU broadcast period, as a multiple of the device's preintegrated-IMU
    // source rate (~290 Hz on the IMX5). This is the IMU feed for the EKF, so
    // we want every sample: period 1. (A previous value of 4 decimated it 4:1,
    // delivering only ~72 Hz to the EKF.)
    int imu_sample_duration = 1;

    // Latest IMU-sample temperature, sourced from the barometer message (PIMU
    // carries none). Defaults to a benign value until the first baro message.
    float _imu_temperature = 25.0f;

    // True once DID_GNSS2_* has been subscribed (only when a second GNSS is
    // present); prevents re-subscribing on repeated flash-config messages.
    bool gnss2_subscribed = false;

    // True once the device-side mode config (sysCfgBits/startupImuDtMs) has been
    // checked against the selected mode; prevents repeated writes.
    bool flash_config_checked = false;

    // Last time (ms) we re-requested DID_FLASH_CONFIG while waiting for the
    // one-shot GET response (see update_thread).
    uint32_t last_flash_cfg_req_ms = 0;

    // DID_BAROMETER liveness/values, reported periodically by the sys_params
    // diagnostic (which fires even when no baro arrives). count==0 => never
    // delivered; count>0 with kPa~0 => device sends an all-zero baro.
    uint32_t baro_rx_count = 0;
    float    last_baro_kpa = 0;
    float    last_baro_temp = 0;
    float    last_baro_msl = 0;

    // True once the first DID_MAGNETOMETER has been reported to the GCS (diagnostic).
    bool mag_reported = false;

    // DID_SYS_PARAMS / PIMU-dt diagnostics are emitted *periodically* (a few
    // times, ~5 s apart) rather than one-shot: dataflash logging can start ~10 s
    // after boot, well after a one-shot boot message would have fired, so a
    // single emission shows up live on the GCS but never lands in the .bin log.
    static constexpr uint8_t DIAG_REPORT_MAX = 6;       // ~30 s of coverage
    static constexpr uint32_t DIAG_REPORT_INTERVAL_MS = 5000;
    uint8_t  sys_params_reports = 0;
    uint32_t last_sys_params_report_ms = 0;
    uint8_t  pimu_reports = 0;
    uint32_t last_pimu_report_ms = 0;

    // Flash-config startup periods captured from DID_FLASH_CONFIG, re-emitted in
    // the periodic sys_params diagnostic so the flash-vs-runtime comparison
    // (startupNavDtMs vs navOutputPeriodMs) lands in the .bin log, not just live.
    uint32_t cfg_nav_dt_ms = 0;
    uint32_t cfg_imu_dt_ms = 0;

    // SPSC ring of pending GCS messages (producer: read thread, consumer: update()).
    struct pending_text_t {
        uint8_t severity;
        char text[64];
    };
    static constexpr uint8_t PENDING_TEXT_QUEUE_SIZE = 8;
    pending_text_t pending_text[PENDING_TEXT_QUEUE_SIZE];
    uint8_t pending_text_head = 0;
    uint8_t pending_text_tail = 0;

    uint32_t last_gps_pkt = 0;
    uint32_t last_filter_pkt = 0;    // INS_3 (on-device EKF solution); INS mode only
    uint32_t last_imu_pkt = 0;       // PIMU; used for health in IMU mode

    char _name[40] = "Inertial Sense";

    uint8_t comm_buf[2048];
    is_comm_instance_t comm;

    float vel_cov = 0;
    float pos_cov = 0;
    float hgt_cov = 0;

    uint8_t _num_gps_sensors = 1;

    AP_ExternalAHRS::gps_data_message_t gps_data_msg;
    AP_ExternalAHRS::gps_data_message_t gps2_data_msg;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    HAL_Semaphore sem;
};

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
