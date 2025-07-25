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

namespace esphome {
    namespace p1_mini {

        namespace {
            // Combine the three values defining a sensor into a single unsigned int for easier
            // handling and comparison
            inline uint32_t OBIS(uint32_t major, uint32_t minor, uint32_t micro)
            {
                return (major & 0xfff) << 16 | (minor & 0xff) << 8 | (micro & 0xff);
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
                return OBIS(major, minor, micro);
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


        P1MiniSensorBase::P1MiniSensorBase(std::string obis_code)
            : m_obis{ OBIS(obis_code.c_str()) }
        {
            if (m_obis == OBIS_ERROR) ESP_LOGE(TAG, "Not a valid OBIS code: '%s'", obis_code.c_str());
        }

        P1MiniTextSensorBase::P1MiniTextSensorBase(std::string identifier)
            : m_identifier{ identifier }
        {
            //ESP_LOGI(TAG, "New text sensor: '%s'", identifier.c_str());
        }

        P1Mini::P1Mini(uint32_t min_period_ms, int buffer_size)
            : m_error_recovery_time{ millis() }
            , m_message_buffer_size{ buffer_size }
            , m_min_period_ms{ min_period_ms }
        {
            m_message_buffer = new char[m_message_buffer_size];
            if (m_message_buffer == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate %d bytes for buffer.", m_message_buffer_size);
                static char dummy[2];
                m_message_buffer = dummy;
                m_message_buffer_size = 2;
            }
            else {
                m_message_buffer_UP.reset(m_message_buffer);
            }
        }

        void P1Mini::setup() {
            //ESP_LOGD("P1Mini", "setup()");
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
                    if (read_byte == '/') {
                        ESP_LOGD(TAG, "ASCII data format");
                        m_data_format = data_formats::ASCII;
                    }
                    else if (read_byte == 0x7e) {
                        ESP_LOGD(TAG, "BINARY data format");
                        m_data_format = data_formats::BINARY;
                    }
                    else {
                        ESP_LOGW(TAG, "Unknown data format (0x%02x). Resetting.", read_byte);
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
                        m_crc_position = ((0x1f & m_message_buffer[1]) << 8) + m_message_buffer[2] - 1;
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
                        ChangeState(states::PROCESSING_BINARY);
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
                        bool matched_sensor{ false };
                        bool is_regular_sensor{ sscanf(m_start_of_data, "1-0:%d.%d.%d(%lf", &major, &minor, &micro, &value) == 4 };
                        if (is_regular_sensor) {
                            auto iter{ m_sensors.find(OBIS(major, minor, micro)) };
                            if (iter != m_sensors.end()) {
                                matched_sensor = true;
                                iter->second->publish_val(value);
                            }
                        }
                        if (!matched_sensor) {
                            for (IP1MiniTextSensor *text_sensor : m_text_sensors) {
                                if (strncmp(m_start_of_data, text_sensor->Identifier().c_str(), text_sensor->Identifier().size()) == 0) {
                                    matched_sensor = true;
                                    text_sensor->publish_val(m_start_of_data);
                                    break;
                                }
                            }
                        }
                        if (!matched_sensor) {
                            if (is_regular_sensor)
                                ESP_LOGD(TAG, "No sensor matched line '%s' with obis code %d.%d.%d", m_start_of_data, major, minor, micro);
                            else
                                ESP_LOGD(TAG, "No sensor matched line '%s'", m_start_of_data);
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
                if (m_start_of_data == m_message_buffer) {
                    m_start_of_data += 3;
                    while (*m_start_of_data != 0x13 && m_start_of_data <= m_message_buffer + m_crc_position) ++m_start_of_data;
                    if (m_start_of_data > m_message_buffer + m_crc_position) {
                        ESP_LOGW(TAG, "Could not find control byte. Resetting.");
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    m_start_of_data += 6;
                }

                do {
                    uint8_t type = *m_start_of_data;
                    switch (type) {
                    case 0x00:
                        m_start_of_data++;
                        break;
                    case 0x01: // array
                        m_start_of_data += 2;
                        break;
                    case 0x02: // struct
                        m_start_of_data += 2;
                        break;
                    case 0x06: {// unsigned double long
                        uint32_t v = (*(m_start_of_data + 1) << 24 | *(m_start_of_data + 2) << 16 | *(m_start_of_data + 3) << 8 | *(m_start_of_data + 4));
                        float fv = v * 1.0 / 1000;
                        auto iter{ m_sensors.find(m_obis_code) };
                        if (iter != m_sensors.end()) iter->second->publish_val(fv);
                        m_start_of_data += 1 + 4;
                        break;
                    }
                    case 0x09: // octet
                        if (*(m_start_of_data + 1) == 0x06) {
                            int minor{ -1 }, major{ -1 }, micro{ -1 };
                            major = *(m_start_of_data + 4);
                            minor = *(m_start_of_data + 5);
                            micro = *(m_start_of_data + 6);

                            m_obis_code = OBIS(major, minor, micro);
                        }
                        m_start_of_data += 2 + (int)*(m_start_of_data + 1);
                        break;
                    case 0x0a: // string
                        m_start_of_data += 2 + (int)*(m_start_of_data + 1);
                        break;
                    case 0x0c: // datetime
                        m_start_of_data += 13;
                        break;
                    case 0x0f: // scalar
                        m_start_of_data += 2;
                        break;
                    case 0x10: {// unsigned long
                        uint16_t v = (*(m_start_of_data + 1) << 8 | *(m_start_of_data + 2));
                        float fv = v * 1.0 / 10;
                        auto iter{ m_sensors.find(m_obis_code) };
                        if (iter != m_sensors.end()) iter->second->publish_val(fv);
                        m_start_of_data += 3;
                        break;
                    }
                    case 0x12: {// signed long
                        int16_t v = (*(m_start_of_data + 1) << 8 | *(m_start_of_data + 2));
                        float fv = v * 1.0 / 10;
                        auto iter{ m_sensors.find(m_obis_code) };
                        if (iter != m_sensors.end()) iter->second->publish_val(fv);
                        m_start_of_data += 3;
                        break;
                    }
                    case 0x16: // enum
                        m_start_of_data += 2;
                        break;
                    default:
                        ESP_LOGW(TAG, "Unsupported data type 0x%02x. Resetting.", type);
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    if (m_start_of_data >= m_message_buffer + m_crc_position) {
                        ChangeState(states::WAITING);
                        return;
                    }
                } while (millis() - loop_start_time < 25);
                break;
            }
            case states::WAITING:
                if (m_display_time_stats) {
                    m_display_time_stats = false;
                    if (m_time_stats_as_info_next == ++m_time_stats_counter) {
                        m_time_stats_as_info_next <<= 1;
                        ESP_LOGI(TAG, "Cycle times: Identifying = %d ms, Message = %d ms (%d loops), Processing = %d ms (%d loops), (Total = %d ms). %d bytes in buffer",
                            m_reading_message_time - m_identifying_message_time,
                            m_processing_time - m_reading_message_time,
                            m_num_message_loops,
                            m_waiting_time - m_processing_time,
                            m_num_processing_loops,
                            m_waiting_time - m_identifying_message_time,
                            m_message_buffer_position
                        );
                    }
                    else
                        ESP_LOGD(TAG, "Cycle times: Identifying = %d ms, Message = %d ms (%d loops), Processing = %d ms (%d loops), (Total = %d ms). %d bytes in buffer",
                            m_reading_message_time - m_identifying_message_time,
                            m_processing_time - m_reading_message_time,
                            m_num_message_loops,
                            m_waiting_time - m_processing_time,
                            m_num_processing_loops,
                            m_waiting_time - m_identifying_message_time,
                            m_message_buffer_position
                    );
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

        void P1Mini::ChangeState(enum states new_state)
        {
            unsigned long const current_time{ millis() };
            switch (new_state) {
            case states::IDENTIFYING_MESSAGE:
                m_identifying_message_time = current_time;
                m_crc_position = m_message_buffer_position = 0;
                m_num_message_loops = m_num_processing_loops = 0;
                m_data_format = data_formats::UNKNOWN;
                m_secondary_p1 = m_secondary_rts != nullptr && m_secondary_rts->state;
                for (auto T : m_ready_to_receive_triggers) T->trigger();
                break;
            case states::READING_MESSAGE:
                m_reading_message_time = current_time;
                for (auto T : m_receiving_update_triggers) T->trigger();
                break;
            case states::VERIFYING_CRC:
                m_verifying_crc_time = current_time;
                for (auto T : m_update_received_triggers) T->trigger();
                break;
            case states::PROCESSING_ASCII:
            case states::PROCESSING_BINARY:
                m_processing_time = current_time;
                m_start_of_data = m_message_buffer;
                break;
            case states::WAITING:
                if (m_state != states::ERROR_RECOVERY) {
                    m_display_time_stats = true;
                    for (auto T : m_update_processed_triggers) T->trigger();
                }
                m_waiting_time = current_time;
                break;
            case states::ERROR_RECOVERY:
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

    }  // namespace p1_mini
}  // namespace esphome