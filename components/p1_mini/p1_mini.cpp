//-------------------------------------------------------------------------------------
// ESPHome P1 Electricity Meter custom sensor
// Copyright 2025 Johnny Johansson
// Copyright 2022 Erik Björk
// Copyright 2020 Pär Svanström
// 
// History
//  0.1.0 2020-11-05:   Initial release
//  0.2.0 2022-04-13:   Major rewrite
//  0.3.0 2022-04-23:   Passthrough to secondary P1 device
//  0.4.0 2022-09-20:   Support binary format
//  0.5.0 ????-??-??:   Rewritten as an ESPHome "external component"
//  0.6.0 2025-01-04:   Introduced text sensors
//
// MIT License
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
// IN THE SOFTWARE.
//-------------------------------------------------------------------------------------

#include "esphome/core/log.h"
#include "p1_mini.h"

#include <ctime>

namespace esphome {
    namespace p1_mini {

        namespace {
            // Combine the three values defining a sensor into a single unsigned int for easier
            // handling and comparison
            inline uint32_t OBIS(uint32_t major, uint32_t minor, uint32_t micro)
            {
                return OBIS_CODE(major, minor, micro);
            }

            constexpr static uint32_t OBIS_ERROR{ 0xffffffff };

            inline uint32_t OBIS(char const *code)
            {
                uint32_t major{ 0 };
                uint32_t minor{ 0 };
                uint32_t micro{ 0 };

                char const *C{ code };
                while (std::isdigit(*C)) major = major * 10 + (*C++ - '0');
                if (*C++ == '\0') return OBIS_ERROR;
                while (std::isdigit(*C)) minor = minor * 10 + (*C++ - '0');
                if (*C++ == '\0') return OBIS_ERROR;
                while (std::isdigit(*C)) micro = micro * 10 + (*C++ - '0');
                if (*C++ != '\0') return OBIS_ERROR;
                return OBIS_CODE(major, minor, micro);
            }

            uint16_t crc16_ccitt_false(char *pData, int length) {
                int i;
                uint16_t wCrc = 0;
                while (length--) {
                    wCrc ^= *(unsigned char *)pData++;
                    for (i = 0; i < 8; i++)
                        wCrc = wCrc & 0x0001 ? (wCrc >> 1) ^ 0xA001 : wCrc >> 1;
                }
                return wCrc;
            }

            uint16_t crc16_x25(char *pData, int length) {
                int i;
                uint16_t wCrc = 0xffff;
                while (length--) {
                    wCrc ^= *(unsigned char *)pData++ << 0;
                    for (i = 0; i < 8; i++)
                        wCrc = wCrc & 0x0001 ? (wCrc >> 1) ^ 0x8408 : wCrc >> 1;
                }
                return wCrc ^ 0xffff;
            }

            constexpr static const char *TAG = "P1Mini";
        }


        P1MiniSensorBase::P1MiniSensorBase(std::string obis_code, double multiplier)
            : m_obis{ OBIS(obis_code.c_str()) }
            , m_multiplier{ multiplier }

        {
            if (m_obis == OBIS_ERROR) ESP_LOGE(TAG, "Not a valid OBIS code: '%s'", obis_code.c_str());
        }

        P1MiniTextSensorBase::P1MiniTextSensorBase(std::string identifier)
            : m_identifier{ identifier }
            , m_obis{ OBIS(identifier.c_str()) }
        {
            if (m_obis == OBIS_ERROR) ESP_LOGW(TAG, "Not a valid OBIS code: '%s'", identifier.c_str());
            //ESP_LOGI(TAG, "New text sensor: '%s'", identifier.c_str());
        }

        P1Mini::P1Mini(uint32_t min_period_ms, int buffer_size, bool secondary_p1, data_formats data_format)
            : m_error_recovery_time{ millis() }
            , m_message_buffer_size{ buffer_size }
            , m_min_period_ms{ min_period_ms }
            , m_secondary_p1{ secondary_p1 }
            , m_data_format{ data_format }
        {
            m_message_buffer = new char[m_message_buffer_size];
            m_dlms_message_buffer = new char[m_message_buffer_size];
            if ((m_message_buffer == nullptr) || (m_dlms_message_buffer == nullptr)) {
                ESP_LOGE(TAG, "Failed to allocate %d bytes for buffer.", m_message_buffer_size);
                static char dummy[2];
                static char dlms_dummy[2];
                m_message_buffer = dummy;
                m_message_buffer_size = 2;
                m_dlms_message_buffer = dlms_dummy;
            }
            else {
                m_message_buffer_UP.reset(m_message_buffer);
                m_dlms_message_buffer_UP.reset(m_dlms_message_buffer);
            }
            memset(m_counter_import, 0, sizeof(m_counter_import));
            memset(m_counter_import_previous, 0, sizeof(m_counter_import_previous));
            memset(m_counter_export, 0, sizeof(m_counter_export));
            memset(m_counter_export_previous, 0, sizeof(m_counter_export_previous));
            memset(m_dlms_struct_size, 0, sizeof(m_dlms_struct_size));
        }

        void P1Mini::setup() {
            //ESP_LOGD("P1Mini", "setup()");
        }

        void P1Mini::register_text_sensor(IP1MiniTextSensor *sensor)
        {
            // Sort long identifiers first in the vector
            auto iter{ m_text_sensors.begin() };
            while (iter != m_text_sensors.end() && sensor->Identifier().size() < (*iter)->Identifier().size()) ++iter;
            m_text_sensors.insert(iter, sensor);
            if (sensor->Obis() != OBIS_ERROR)
                m_obis_text_sensors.emplace(sensor->Obis(), sensor);
        }

        void P1Mini::loop() {
            unsigned long const loop_start_time{ millis() };
            switch (m_state) {
            case states::IDENTIFYING_MESSAGE:
                if (!available()) {
                    constexpr unsigned long max_wait_time_ms{ 60000 };
                    if (max_wait_time_ms < loop_start_time - m_identifying_message_time) {
                        ESP_LOGW(TAG, "No data received for %d seconds.", max_wait_time_ms / 1000);
                        ChangeState(states::ERROR_RECOVERY);
                    }
                    break;
                }
                {
                    char const read_byte{ GetByte() };
                    char expected_byte{ '/' };
                    switch(m_data_format)
                    {
                        case data_formats::ASCII:
                            expected_byte = '/';
                            ESP_LOGD(TAG, "ASCII data format");
                            break;
                        case data_formats::BINARY:
                            expected_byte = 0x7e;
                            ESP_LOGD(TAG, "BINARY data format");
                            break;
                    }
                    if (read_byte != expected_byte) {
                        ESP_LOGW(TAG, "Unexpected data (0x%02x). Resetting.", read_byte);
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    m_message_buffer[m_message_buffer_position++] = read_byte;
                    ChangeState(states::READING_MESSAGE);
                }
                // Not breaking here! The delay caused by exiting the loop function here can cause
                // the UART buffer to overflow, so instead, go directly into the READING_MESSAGE
                // part.
            case states::READING_MESSAGE:
                ++m_num_message_loops;
                while (available()) {
                    // While data is available, read it one byte at a time.
                    char const read_byte{ GetByte() };

                    m_message_buffer[m_message_buffer_position++] = read_byte;

                    // Find out where CRC will be positioned
                    if (m_data_format == data_formats::ASCII && read_byte == '!') {
                        // The exclamation mark indicates that the main message is complete
                        // and the CRC will come next.
                        m_crc_position = m_message_buffer_position;
                    }
                    else if (m_data_format == data_formats::BINARY && m_message_buffer_position == 3) {
                        if ((0xe0 & m_message_buffer[1]) != 0xa0) {
                            ESP_LOGW(TAG, "Unknown frame format (0x%02X). Resetting.", read_byte);
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
						
                        m_crc_position = ((0x7 & m_message_buffer[1]) << 8) + m_message_buffer[2] - 1;
						ESP_LOGD(TAG, "Frame size: %d", m_crc_position);
                    }

                    // If end of CRC is reached, start verifying CRC
                    if (m_crc_position > 0 && m_message_buffer_position > m_crc_position) {
                        if (m_data_format == data_formats::ASCII && read_byte == '\n') {
                            ChangeState(states::VERIFYING_CRC);
                            return;
                        }
                        else if (m_data_format == data_formats::BINARY && m_message_buffer_position == m_crc_position + 3) {
                            if (read_byte != 0x7e) {
                                ESP_LOGW(TAG, "Unexpected end. Resetting.");
                                ChangeState(states::ERROR_RECOVERY);
                                return;
                            }
                            ChangeState(states::VERIFYING_CRC);
                            return;
                        }
                    }
                    if (m_message_buffer_position == m_message_buffer_size) {
                        ESP_LOGW(TAG, "Message buffer overrun. Resetting.");
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }

                }
                {
                    constexpr unsigned long max_message_time_ms{ 10000 };
                    if (max_message_time_ms < loop_start_time - m_reading_message_time && m_reading_message_time < loop_start_time) {
                        ESP_LOGW(TAG, "Complete message not received within %d seconds. Resetting.", max_message_time_ms / 1000);
                        ChangeState(states::ERROR_RECOVERY);
                    }
                }
                break;
            case states::VERIFYING_CRC: {
                int crc_from_msg = -1;
                int crc = 0;

                if (m_data_format == data_formats::ASCII) {
                    crc_from_msg = (int)strtol(m_message_buffer + m_crc_position, NULL, 16);
                    crc = crc16_ccitt_false(m_message_buffer, m_crc_position);
                }
                else if (m_data_format == data_formats::BINARY) {
                    crc_from_msg = (m_message_buffer[m_crc_position + 1] << 8) + m_message_buffer[m_crc_position];
                    crc = crc16_x25(&m_message_buffer[1], m_crc_position - 1);
                }

                if (crc == crc_from_msg) {
                    ESP_LOGD(TAG, "CRC verification OK");
                    if (m_data_format == data_formats::ASCII) {
                        ChangeState(states::PROCESSING_ASCII);
                    }
                    else if (m_data_format == data_formats::BINARY) {
                        
                        for (int i = 9; i < m_crc_position; i++) {
                            m_dlms_message_buffer[m_dlms_message_buffer_position++] = m_message_buffer[i];
                            if (m_dlms_message_buffer_position == m_message_buffer_size) {
                                ESP_LOGW(TAG, "Dlms Message buffer overrun. Resetting.");
                                ChangeState(states::ERROR_RECOVERY);
                                return;
                            }
                        }
                        if ((m_message_buffer[1] & 8) != 0) {

                            ChangeState(states::IDENTIFYING_MESSAGE);
                        }
                        else {
                            #ifdef ESPHOME_LOG_HAS_DEBUG
                            ESP_LOGD(TAG, "Dlms Buffer:");
                            char hex_buffer[81];
                            hex_buffer[80] = '\0';
                            for (int i = 0; i * 40 < m_dlms_message_buffer_position; i++) {
                                int j;
                                for (j = 0; j + i * 40 < m_dlms_message_buffer_position && j < 40; j++) {
                                    sprintf(&hex_buffer[2 * j], "%02X", m_dlms_message_buffer[j + i * 40]);
                                }
                                if (j >= m_dlms_message_buffer_position) {
                                    hex_buffer[j] = '\0';
                                }
                                ESP_LOGD(TAG, "%s", hex_buffer);
                            }
                            #endif
                            ChangeState(states::PROCESSING_BINARY);
                        }
                    }
                    else {
                        ChangeState(states::ERROR_RECOVERY);
                    }
                    return;
                }

                // CRC verification failed
                ESP_LOGW(TAG, "CRC mismatch, calculated %04X != %04X. Message ignored.", crc, crc_from_msg);
                if (m_data_format == data_formats::ASCII) {
                    ESP_LOGD(TAG, "Buffer:\n%s (%d)", m_message_buffer, m_message_buffer_position);
                }
                else if (m_data_format == data_formats::BINARY) {
                    ESP_LOGD(TAG, "Buffer:");
                    char hex_buffer[81];
                    hex_buffer[80] = '\0';
                    for (int i = 0; i * 40 < m_message_buffer_position; i++) {
                        int j;
                        for (j = 0; j + i * 40 < m_message_buffer_position && j < 40; j++) {
                            sprintf(&hex_buffer[2 * j], "%02X", m_message_buffer[j + i * 40]);
                        }
                        if (j >= m_message_buffer_position) {
                            hex_buffer[j] = '\0';
                        }
                        ESP_LOGD(TAG, "%s", hex_buffer);
                    }
                }
                ChangeState(states::ERROR_RECOVERY);
                return;
            }
            case states::PROCESSING_ASCII:
                ++m_num_processing_loops;
                do {
                    while (*m_start_of_data == '\n' || *m_start_of_data == '\r') ++m_start_of_data;
                    char *end_of_line{ m_start_of_data };
                    while (*end_of_line != '\n' && *end_of_line != '\r' && *end_of_line != '\0' && *end_of_line != '!') ++end_of_line;
                    char const end_of_line_char{ *end_of_line };
                    *end_of_line = '\0';

                    if (end_of_line != m_start_of_data) {
                        int minor{ -1 }, major{ -1 }, micro{ -1 };
                        double value{ -1.0 };
                        if (sscanf(m_start_of_data, "1-0:%d.%d.%d(%lf", &major, &minor, &micro, &value) != 4) {
                            bool matched_text_sensor{ false };
                            for (IP1MiniTextSensor *text_sensor : m_text_sensors) {
                                if (strncmp(m_start_of_data, text_sensor->Identifier().c_str(), text_sensor->Identifier().size()) == 0) {
                                    matched_text_sensor = true;
                                    text_sensor->publish_val(m_start_of_data);
                                    break;
                                }
                                
                            }
                            if (!matched_text_sensor) ESP_LOGD(TAG, "No sensor matched line '%s'", m_start_of_data);
                        }
                        else {
                            uint32_t const obisCode{ OBIS(major, minor, micro) };
                            auto iter{ m_sensors.find(obisCode) };
                            if (iter != m_sensors.end()) iter->second->publish_val(value);
                            else {
                                ESP_LOGD(TAG, "No sensor matching: %d.%d.%d (0x%x)", major, minor, micro, obisCode);
                            }
                            switch(obis_code)
                            {
                                case OBIS_CODE(0, 2, 2):
                                    m_useComputedTariff = false;
                                    break;
                                case OBIS_CODE(1,8,1):
                                    m_counter_import[0] = value;
                                    break;
                                case OBIS_CODE(1,8,2):
                                    m_counter_import[1] = value;
                                    break;
                                case OBIS_CODE(1,8,3):
                                    m_counter_import[2] = value;
                                    break;
                                case OBIS_CODE(1,8,4):
                                    m_counter_import[3] = value;
                                    break;
                                case OBIS_CODE(2,8,1):
                                    m_counter_export[0] = value;
                                    break;
                                case OBIS_CODE(2,8,2):
                                    m_counter_export[1] = value;
                                    break;
                                case OBIS_CODE(2,8,3):
                                    m_counter_export[2] = value;
                                    break;
                                case OBIS_CODE(2,8,4):
                                    m_counter_export[3] = value;
                                    break;
                            }                            
                        }
                    }
                    *end_of_line = end_of_line_char;
                    if (end_of_line_char == '\0' || end_of_line_char == '!') {
                        ChangeState(states::WAITING);
                        return;
                    }
                    m_start_of_data = end_of_line + 1;
                } while (millis() - loop_start_time < 25);
                break;
            case states::PROCESSING_BINARY: {
                ++m_num_processing_loops;
                if (m_start_of_data == m_dlms_message_buffer) {
                    if ((!IsDlsmBufferSizeAvailable(8)) || m_start_of_data[0] != 0xe6 || m_start_of_data[1] != 0xe7 || m_start_of_data[2] != 0x00 || m_start_of_data[3] != 0x0f) {
                        ESP_LOGW(TAG, "Could not find frame identifier. Resetting.");
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    m_start_of_data += 8;
                    if (!IsDlsmBufferSizeAvailable(1)) {
                        ESP_LOGW(TAG, "Could not find datetime size. Resetting.");
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    int dateSize = *m_start_of_data;
                    if (dateSize > 0)
                    {
                        m_start_of_data++;
                        if ((!IsDlsmBufferSizeAvailable(dateSize))) {
                            ESP_LOGW(TAG, "Could not read date header. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        if (dateSize == 12) {
                            uint16_t year = m_start_of_data[0] << 8 | m_start_of_data[1];
                            uint8_t month = m_start_of_data[2];
                            uint8_t day = m_start_of_data[3];
                            uint8_t hour = m_start_of_data[5];
                            uint8_t minute = m_start_of_data[6];
                            uint8_t second = m_start_of_data[7];
                            short deviation = ((short)m_start_of_data[9] << 8 | m_start_of_data[10]) * -1;
                            bool dayLightSavingActive = ((m_start_of_data[11] & 0x80) != 0);

                            ESP_LOGD(TAG, "Time: %d-%02d-%02d %02d:%02d:%02d TZ:%f %s",  year, month, day, hour, minute, second, deviation / 60.0, dayLightSavingActive ? "DST" : "NO DST");

                            for (IP1MiniTextSensor *text_sensor : m_text_sensors) {
                                if (strncmp("metertime", text_sensor->Identifier().c_str(), text_sensor->Identifier().size()) == 0) {

                                    //tm timeinfo;
                                    //timeinfo.tm_year = year - 1900; // tm_year is year since 1900
                                    //timeinfo.tm_mon = month - 1;    // tm_mon ranges from 0 to 11
                                    //timeinfo.tm_mday = day;
                                    //timeinfo.tm_hour = hour;
                                    //timeinfo.tm_min = minute;
                                    //timeinfo.tm_sec = second;
                                    //timeinfo.tm_gmtoff = deviation;
                                    //timeinfo.tm_isdst = dayLightSavingActive ? 1 : 0;
                                    int timezone_hour = abs(deviation / 60);
                                    int timezone_minute = abs(deviation % 60);
                                    char buffer [80];
                                    //strftime(buffer,80,"%Y-%m-%dT%H:%M:%S+00:00",&timeinfo);

                                    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d", year, month, day, hour, minute, second, (deviation >= 0 ? '+' : '-'), timezone_hour, timezone_minute);

                                    text_sensor->publish_val(buffer);
                                    break;
                                }
                                
                            }
                        }
                        m_start_of_data += dateSize;
                    }
                    if (!IsDlsmBufferSizeAvailable(0)) {
                        ESP_LOGW(TAG, "Could not find data start. Resetting.");
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    memset(m_dlms_struct_size, 0, sizeof(m_dlms_struct_size));
                    memset(m_dlms_struct_offset, 0, sizeof(m_dlms_struct_offset));
                    m_dlms_struct_level = 0;
                    obis_code = OBIS_ERROR;
                    m_text_value[0] = '\0';
                    m_value = 0.0;
                    m_scalar = 0;
                    m_unit = 0;
                    m_value_kind = value_kind::UNKNOW;
                }
                do {
                    if (!IsDlsmBufferSizeAvailable(1)) {
                        ESP_LOGW(TAG, "no enough for reading data type. Resetting.");
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    bool newStruct = false;
                    uint8_t type = *m_start_of_data;
                   // ESP_LOGD(TAG,"offset %d", m_start_of_data - m_dlms_message_buffer);
                    switch (type) {
                    case 0x00:
                        m_start_of_data++;
                        break;
                    case 0x01: // array
                        m_start_of_data += 2;
                        break;
                    case 0x02: // struct
                        if (!IsDlsmBufferSizeAvailable(2)) {
                            ESP_LOGW(TAG, "no enough for struct data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        if (m_dlms_struct_size[m_dlms_struct_level] == 0)
                            m_dlms_struct_size[m_dlms_struct_level] = m_start_of_data[1];
                        else if (m_dlms_struct_level < 9)
                            m_dlms_struct_size[++m_dlms_struct_level] = m_start_of_data[1];
                        newStruct = true;
                        m_dlms_struct_offset[m_dlms_struct_level] = 0;
                        //ESP_LOGD(TAG, "[%d, %d] Struct Size: %d at level %d",m_num_processing_loops, loop, m_start_of_data[1], m_dlms_struct_level);
                        m_start_of_data += 2;
                        break;
                    case 0x05: {// DLMS_DATA_TYPE_INT32
                        if (!IsDlsmBufferSizeAvailable(5)) {
                            ESP_LOGW(TAG, "no enough for int32 data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        int32_t v = (*(m_start_of_data + 1) << 24 | *(m_start_of_data + 2) << 16 | *(m_start_of_data + 3) << 8 | *(m_start_of_data + 4));
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 1) {
                            m_value = v;
                            m_value_kind = value_kind::NUMBER;
                        }
                        break;
                    }
                    case 0x06: {// DLMS_DATA_TYPE_UINT32
                        if (!IsDlsmBufferSizeAvailable(5)) {
                            ESP_LOGW(TAG, "no enough for uint32 data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        uint32_t v = (*(m_start_of_data + 1) << 24 | *(m_start_of_data + 2) << 16 | *(m_start_of_data + 3) << 8 | *(m_start_of_data + 4));
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 1) {
                            m_value = v;
                            m_value_kind = value_kind::NUMBER;
                            switch(obis_code)
                            {
                                case OBIS_CODE(1,8,1):
                                    m_counter_import[0] = v;
                                    break;
                                case OBIS_CODE(1,8,2):
                                    m_counter_import[1] = v;
                                    break;
                                case OBIS_CODE(1,8,3):
                                    m_counter_import[2] = v;
                                    break;
                                case OBIS_CODE(1,8,4):
                                    m_counter_import[3] = v;
                                    break;
                                case OBIS_CODE(2,8,1):
                                    m_counter_export[0] = v;
                                    break;
                                case OBIS_CODE(2,8,2):
                                    m_counter_export[1] = v;
                                    break;
                                case OBIS_CODE(2,8,3):
                                    m_counter_export[2] = v;
                                    break;
                                case OBIS_CODE(2,8,4):
                                    m_counter_export[3] = v;
                                    break;
                            }
                        }
                        m_start_of_data += 1 + 4;
                        break;
                    }
                    case 0x09: {// octet
                        if (!IsDlsmBufferSizeAvailable(2)) {
                            ESP_LOGW(TAG, "no enough for octet stream data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        int len = *(m_start_of_data + 1);
                        if (!IsDlsmBufferSizeAvailable(2 + len)) {
                            ESP_LOGW(TAG, "no enough for octet stream data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 0) {
                            if (len == 0x06) {
                                int minor{ -1 }, major{ -1 }, micro{ -1 };
                                major = *(m_start_of_data + 4);
                                minor = *(m_start_of_data + 5);
                                micro = *(m_start_of_data + 6);

                                obis_code = OBIS(major, minor, micro);
                                if (obis_code == OBIS(0, 2, 2))
                                    m_useComputedTariff = false;
                            }
                        }
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 1)
                        {
                            if (len > sizeof(m_text_value) - 1)
                                len = sizeof(m_text_value) - 1;
                            memcpy(m_text_value, m_start_of_data + 2, len);
                            m_text_value[len] = '\0';
                            m_value_kind = value_kind::TEXT;
                        }
                        m_start_of_data += 2 + len;
                        break;
                    }
                    case 0x0a: {// string
                        if (!IsDlsmBufferSizeAvailable(2)) {
                            ESP_LOGW(TAG, "no enough for string data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        int len = *(m_start_of_data + 1);
                        if (!IsDlsmBufferSizeAvailable(2 + len)) {
                            ESP_LOGW(TAG, "no enough for string data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        m_start_of_data += 2 + len;
                        break;
                    }
                    case 0x0c: // datetime
                        if (!IsDlsmBufferSizeAvailable(13)) {
                            ESP_LOGW(TAG, "no enough for datetime data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        m_start_of_data += 13;
                        break;
                    case 0x0f: {// DLMS_DATA_TYPE_INT8
                        if (!IsDlsmBufferSizeAvailable(2)) {
                            ESP_LOGW(TAG, "no enough for int8 data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        int8_t v = *(m_start_of_data + 1);
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 1) {
                            m_value = v;
                            m_value_kind = value_kind::NUMBER;
                        }
                        if (m_dlms_struct_level == 2 && m_dlms_struct_offset[m_dlms_struct_level] == 0) {
                            m_scalar = v;
                        }
                        //ESP_LOGD(TAG, "Int8: %d %d %d", v, m_dlms_struct_level, m_dlms_struct_offset[m_dlms_struct_level]);
                        m_start_of_data += 2;
                        break;
                    }
                    case 0x10: {// DLMS_DATA_TYPE_INT16
                        if (!IsDlsmBufferSizeAvailable(3)) {
                            ESP_LOGW(TAG, "no enough for int16 data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        uint16_t v = (*(m_start_of_data + 1) << 8 | *(m_start_of_data + 2));
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 1) {
                            m_value = v;
                            m_value_kind = value_kind::NUMBER;
                        }
                        m_start_of_data += 3;
                        break;
                    }
                    case 0x11: {// DLMS_DATA_TYPE_UINT8
                        if (!IsDlsmBufferSizeAvailable(2)) {
                            ESP_LOGW(TAG, "no enough for uint8 data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        uint8_t v = *(m_start_of_data + 1);
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 1) {
                            m_value = v;
                            m_value_kind = value_kind::NUMBER;
                        }
                        m_start_of_data += 2;
                        break;
                    }                  
                    case 0x12: {// DLMS_DATA_TYPE_UINT16
                        if (!IsDlsmBufferSizeAvailable(3)) {
                            ESP_LOGW(TAG, "no enough for uint16 data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        int16_t v = (*(m_start_of_data + 1) << 8 | *(m_start_of_data + 2));
                        if (m_dlms_struct_level == 1 && m_dlms_struct_offset[m_dlms_struct_level] == 1) {
                            m_value = v;
                            m_value_kind = value_kind::NUMBER;
                        }
                        m_start_of_data += 3;
                        break;
                    }
                    case 0x16: // enum
                        if (!IsDlsmBufferSizeAvailable(2)) {
                            ESP_LOGW(TAG, "no enough for enum data. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        m_unit = *(m_start_of_data + 1);
                        m_start_of_data += 2;
                        break;
                    default:
                        ESP_LOGW(TAG, "Unsupported data type 0x%02x. Resetting.", type);
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    if (!newStruct) {

                        m_dlms_struct_offset[m_dlms_struct_level]++;

                        while (m_dlms_struct_size[m_dlms_struct_level] == 1)
                        {
                            m_dlms_struct_size[m_dlms_struct_level] = 0;
                            if (m_dlms_struct_level > 0) {
                                m_dlms_struct_level--;
                            }
                            else
                                break;
                        }

                        if (m_dlms_struct_level == 0) {
                            if (obis_code != OBIS_ERROR) {
                                int major = (obis_code >> 16) & 0xfff;
                                int minor = (obis_code >> 8) & 0xff;
                                int micro = (obis_code) & 0xff;
                                switch(m_value_kind)
                                {
                                    case value_kind::NUMBER: {
                                        double factor_value = m_value * pow(10, m_scalar);
                                        ESP_LOGD(TAG, "publish %d.%d.%d %f [%s]", major, minor, micro, factor_value, UnitToString(m_unit));
                                        auto iter{ m_sensors.find(obis_code) };
                                        if (iter != m_sensors.end()) iter->second->publish_val(factor_value);
                                        break;
                                    }
                                    case value_kind::TEXT: {
                                        ESP_LOGD(TAG, "publish %d.%d.%d %s", major, minor, micro, m_text_value);
                                        auto iter{ m_obis_text_sensors.find(obis_code) };
                                        if (iter != m_obis_text_sensors.end()) iter->second->publish_val(m_text_value);
                                        break;
                                    }
                                    default:
                                        break;
                                }
                            }
                            obis_code = OBIS_ERROR;
                            m_text_value[0] = '\0';
                            m_value = 0.0;
                            m_scalar = 0;
                            m_unit = 0;
                            m_value_kind = value_kind::UNKNOW;
                        }
                        if (m_dlms_struct_size[m_dlms_struct_level] > 0) {
                            m_dlms_struct_size[m_dlms_struct_level]--;
                        }
                    }
                    //ESP_LOGD(TAG, "[%d, %d] Type: %d, Struct Level: %d [ %d, %d, %d, %d, %d, %d, %d, %d, %d, %d ]", m_num_processing_loops, loop, type, m_dlms_struct_level, m_dlms_struct_size[0], m_dlms_struct_size[1], m_dlms_struct_size[2], m_dlms_struct_size[3], m_dlms_struct_size[4], m_dlms_struct_size[5], m_dlms_struct_size[6], m_dlms_struct_size[7], m_dlms_struct_size[8], m_dlms_struct_size[9]);
                    if (!IsDlsmBufferSizeAvailable(1)) {
                        m_message_buffer_position = m_dlms_message_buffer_position;
                        m_dlms_message_buffer_position = 0;
                        ChangeState(states::WAITING);
                        return;
                    }
                } while (millis() - loop_start_time < 25);
                //ESP_LOGD(TAG,"offset exiting %d", m_start_of_data - m_dlms_message_buffer);
                break;
            }
            case states::WAITING:
                if (m_display_time_stats) {
                    m_display_time_stats = false;
                    ESP_LOGD(TAG, "Cycle times: Identifying = %d ms, Message = %d ms (%d loops), Processing = %d ms (%d loops), (Total = %d ms). %d bytes in buffer",
                        m_reading_message_time - m_identifying_message_time,
                        m_processing_time - m_reading_message_time,
                        m_num_message_loops,
                        m_waiting_time - m_processing_time,
                        m_num_processing_loops,
                        m_waiting_time - m_identifying_message_time,
                        m_message_buffer_position
                    );
                    //ESP_LOGD(TAG,"Import: %d %d %d %d", m_counter_import[0], m_counter_import[1], m_counter_import[2], m_counter_import[3]);
                    //ESP_LOGD(TAG,"Export: %d %d %d %d", m_counter_export[0], m_counter_export[1], m_counter_export[2], m_counter_export[3]);
                    int tariff = -1;
                    for (int i = 0; i < 4; i++) {
                        if (((m_counter_import_previous[i] != m_counter_import[i]) && (m_counter_import_previous[i] != 0)) || ((m_counter_export_previous[i] != m_counter_export[i]) && (m_counter_export_previous[i] != 0)))
                            tariff = i;

                        m_counter_import_previous[i] = m_counter_import[i];
                        m_counter_export_previous[i] = m_counter_export[i];
                    }
                    if ((tariff != -1) && (m_useComputedTariff)) {
                        auto iter{ m_sensors.find(OBIS(0,2,2)) };
                        if (iter != m_sensors.end()) iter->second->publish_val(tariff + 1);
                    }
                }
                if (m_min_period_ms == 0 || m_min_period_ms < loop_start_time - m_identifying_message_time) {
                    ChangeState(states::IDENTIFYING_MESSAGE);
                }
                else if (available()) {
                    ESP_LOGE(TAG, "Data was received before beeing requested. If flow control via the RTS signal is not used, the minimum_period should be set to 0s in the yaml. Resetting.");
                    ChangeState(states::ERROR_RECOVERY);
                }
                break;
            case states::ERROR_RECOVERY:
                if (available()) {
                    int max_bytes_to_discard{ 200 };
                    do { AddByteToDiscardLog(GetByte()); } while (available() && max_bytes_to_discard-- != 0);
                }
                else if (500 < loop_start_time - m_error_recovery_time) {
                    ChangeState(states::WAITING);
                    FlushDiscardLog();
                }
                break;
            }
        }

        bool P1Mini::IsDlsmBufferSizeAvailable(int size)
        {
            return (m_start_of_data + size - 1 < m_dlms_message_buffer + m_dlms_message_buffer_position);
        }

        void P1Mini::ChangeState(enum states new_state)
        {
            unsigned long const current_time{ millis() };
            switch (new_state) {
            case states::IDENTIFYING_MESSAGE:
                m_identifying_message_time = current_time;
                m_crc_position = m_message_buffer_position = 0;
                m_num_message_loops = m_num_processing_loops = 0;
                for (auto T : m_ready_to_receive_triggers) T->trigger();
                break;
            case states::READING_MESSAGE:
                m_reading_message_time = current_time;
                break;
            case states::VERIFYING_CRC:
                m_verifying_crc_time = current_time;
                for (auto T : m_update_received_triggers) T->trigger();
                break;
            case states::PROCESSING_ASCII:
            case states::PROCESSING_BINARY:
                m_processing_time = current_time;
                m_start_of_data = m_dlms_message_buffer;
                break;
            case states::WAITING:
                if (m_state != states::ERROR_RECOVERY) m_display_time_stats = true;
                m_waiting_time = current_time;
                break;
            case states::ERROR_RECOVERY:
                m_dlms_message_buffer_position = 0;
                m_error_recovery_time = current_time;
                for (auto T : m_communication_error_triggers) T->trigger();
            }
            m_state = new_state;
        }

        void P1Mini::AddByteToDiscardLog(uint8_t byte)
        {
            constexpr char hex_chars[] = "0123456789abcdef";
            *m_discard_log_position++ = hex_chars[byte >> 4];
            *m_discard_log_position++ = hex_chars[byte & 0xf];
            if (m_discard_log_position == m_discard_log_end) FlushDiscardLog();
        }

        void P1Mini::FlushDiscardLog()
        {
            if (m_discard_log_position != m_discard_log_buffer) {
                *m_discard_log_position = '\0';
                ESP_LOGW(TAG, "Discarding: %s", m_discard_log_buffer);
                m_discard_log_position = m_discard_log_buffer;
            }
        }


        void P1Mini::dump_config() {
            ESP_LOGCONFIG(TAG, "P1 Mini component");
        }


        const char* P1Mini::UnitToString(int unit) {
            static const std::unordered_map<int, const char*> unitToStringMap = {
                {1, "a"}, {2, "mo"}, {3, "wk"}, {4, "d"}, {5, "h"}, {6, "min."}, {7, "s"}, {8, "°"},
                {9, "°C"}, {10, "currency"}, {11, "m"}, {12, "m/s"}, {13, "m^3"}, {14, "m^3"}, {15, "m^3/h"},
                {16, "m^3/h"}, {17, "m^3/d"}, {18, "m^3/d"}, {19, "l"}, {20, "kg"}, {21, "N"}, {22, "Nm"},
                {23, "Pa"}, {24, "bar"}, {25, "J"}, {26, "J/h"}, {27, "W"}, {28, "VA"}, {29, "var"},
                {30, "Wh"}, {31, "VAh"}, {32, "varh"}, {33, "A"}, {34, "C"}, {35, "V"}, {36, "V/m"},
                {37, "F"}, {38, "Ohm"}, {39, "Ohm*m^2/m"}, {40, "Wb"}, {41, "T"}, {42, "A/m"}, {43, "H"},
                {44, "Hz"}, {45, "1/(Wh)"}, {46, "1/(varh)"}, {47, "1/(VAh)"}, {48, "V^2h"}, {49, "A^2h"},
                {50, "kg/s"}, {51, "S"}, {52, "K"}, {53, "1/(V^2h)"}, {54, "1/(A^2h)"}, {55, "1/m^3"},
                {56, "%"}, {57, "Ah"}, {60, "Wh/m^3"}, {61, "J/m^3"}, {62, "Mol %"}, {63, "g/m^3"},
                {64, "Pa s"}, {253, ""}, {254, "other"}, {255, "count"}
            };

            auto it = unitToStringMap.find(unit);
            if (it != unitToStringMap.end()) {
                return it->second;
            } else {
                return "Unknown";
            }
}





    }  // namespace p1_mini
}  // namespace esphome