//-------------------------------------------------------------------------------------
// ESPHome P1 Electricity Meter custom sensor
// Copyright 2022 Johnny Johansson, Erik Björk
// Copyright 2020 Pär Svanström
// 
// History
//  0.1.0 2020-11-05:   Initial release
//  0.2.0 2022-04-13:   Major rewrite
//  0.3.0 2022-04-23:   Passthrough to secondary P1 device
//  0.4.0 2022-09-20:   Support binary format
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

#include "esphome.h"

class P1Reader : public Component, public UARTDevice {
public:

    // Call from a lambda in the yaml file to set up each sensor.
    Sensor *AddSensor(int major, int minor, int micro)
    {
        m_sensor_list = new SensorListItem(m_sensor_list, OBIS(major, minor, micro));
        return m_sensor_list->GetSensor();
    }

    P1Reader(UARTComponent *parent,
        Number *update_period_number,
        esphome::gpio::GPIOSwitch *CTS_switch,
        esphome::gpio::GPIOSwitch *status_switch = nullptr,
        esphome::gpio::GPIOBinarySensor * secondary_RTS = nullptr)
        : UARTDevice(parent)
        , m_minimum_period_ms{ static_cast<unsigned long>(update_period_number->state * 1000.0f + 0.5f) }
        , m_CTS_switch{ CTS_switch }
        , m_status_switch{ status_switch }
        , m_update_period_number{ update_period_number }
        , m_secondary_RTS{ secondary_RTS }
    {}

private:
    unsigned long m_minimum_period_ms{ 0 };
    unsigned long m_reading_message_time;
    unsigned long m_verifying_crc_time;
    unsigned long m_processing_time;
    unsigned long m_resending_time;
    unsigned long m_waiting_time;
    unsigned long m_error_recovery_time;
    int m_num_message_loops;
    int m_num_processing_loops;
    bool m_display_time_stats{ false };

    // Store the message as it is being received:
    constexpr static int message_buffer_size{ 2048 };
    char m_message_buffer[message_buffer_size];
    int m_message_buffer_position{ 0 };
    int m_crc_position{ 0 };

    // Keeps track of the start of the data record while processing.
    char *m_start_of_data;

    // Keeps track of bytes sent when resending the message
    int m_bytes_resent;

    enum class states {
        READING_MESSAGE,
        VERIFYING_CRC,
        PROCESSING_ASCII,
        PROCESSING_BINARY,
        RESENDING, // To the optional secondary P1-port
        WAITING,
        ERROR_RECOVERY
    };
    enum states m_state { states::READING_MESSAGE };

    enum class data_formats {
        UNKNOWN,
        ASCII,
        BINARY
    };
    enum data_formats m_data_format{ data_formats::UNKNOWN };

    void ChangeState(enum states new_state)
    {
        unsigned long const current_time{ millis() };
        switch (new_state) {
        case states::READING_MESSAGE:
            m_reading_message_time = current_time;
            m_num_message_loops = m_num_processing_loops = 0;
            m_CTS_switch->turn_on();
            if (m_status_switch != nullptr) m_status_switch->turn_on();
            m_crc_position = m_message_buffer_position = 0;
            break;
        case states::VERIFYING_CRC:
            m_verifying_crc_time = current_time;
            break;
        case states::PROCESSING_ASCII:
            m_processing_time = current_time;
            m_CTS_switch->turn_off();
            m_start_of_data = m_message_buffer;
            break;
        case states::PROCESSING_BINARY:
            m_processing_time = current_time;
            m_CTS_switch->turn_off();
            m_start_of_data = m_message_buffer;
            break;
        case states::RESENDING:
            m_resending_time = current_time;
            if (!m_secondary_RTS->state) {
                ChangeState(states::WAITING);
                return;
            }
            m_bytes_resent = 0;
            break;
        case states::WAITING:
            if (m_state != states::ERROR_RECOVERY) m_display_time_stats = true;
            m_waiting_time = current_time;
            if (m_status_switch != nullptr) m_status_switch->turn_off();
            break;
        case states::ERROR_RECOVERY:
            m_error_recovery_time = current_time;
            m_CTS_switch->turn_off();
        }
        m_state = new_state;
    }

    // Combine the three values defining a sensor into a single unsigned int for easier
    // handling and comparison
    static uint32_t OBIS(uint32_t major, uint32_t minor, uint32_t micro)
    {
        return (major & 0xfff) << 16 | (minor & 0xff) << 8 | (micro & 0xff);
    }

    class SensorListItem {
        uint32_t const m_obisCode;
        Sensor m_sensor;
        SensorListItem *const m_next{ nullptr };
    public:
        SensorListItem(SensorListItem *next, uint32_t obisCode)
            : m_obisCode(obisCode)
            , m_next(next)
        {}

        Sensor *GetSensor() { return &m_sensor; }
        uint32_t GetCode() const { return m_obisCode; }
        SensorListItem *Next() const { return m_next; }
    };

    // Linked list of all sensors
    SensorListItem *m_sensor_list{ nullptr };

    esphome::gpio::GPIOSwitch *const m_CTS_switch;
    esphome::gpio::GPIOSwitch *const m_status_switch;
    Number const *const m_update_period_number{ nullptr };
    esphome::gpio::GPIOBinarySensor const * const m_secondary_RTS{ nullptr };

public:

    void setup() override
    {
        ChangeState(states::READING_MESSAGE);
    }

    void loop() override {
        unsigned long const loop_start_time{ millis() };
        m_minimum_period_ms = static_cast<unsigned long>(m_update_period_number->state * 1000.0f + 0.5f);
        switch (m_state) {
        case states::READING_MESSAGE:
            ++m_num_message_loops;
            while (available()) {
                // While data is available, read it one byte at a time.
                char const read_byte{ (char)read() };

                // First byte tells which data format
                if (m_message_buffer_position == 0) {
                    if (read_byte == '/') {
                        ESP_LOGD("p1reader", "ASCII data format");
                        m_data_format = data_formats::ASCII;
                    } else if (read_byte == 0x7e) {
                        ESP_LOGD("p1reader", "BINARY data format");
                        m_data_format = data_formats::BINARY;
                    } else {
                        ESP_LOGW("p1reader", "Unknown data format (0x%02X). Resetting.", read_byte);
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                }

                m_message_buffer[m_message_buffer_position++] = read_byte;
                if (m_message_buffer_position == message_buffer_size) {
                    ESP_LOGW("p1reader", "Message buffer overrun. Resetting.");
                    ChangeState(states::ERROR_RECOVERY);
                    return;
                }

                // Find out where CRC will be positioned
                if (m_data_format == data_formats::ASCII && read_byte == '!') {
                    // The exclamation mark indicates that the main message is complete
                    // and the CRC will come next.
                    m_crc_position = m_message_buffer_position;
                } else if (m_data_format == data_formats::BINARY && m_message_buffer_position == 3) {
                    if ((0xe0 & m_message_buffer[1]) != 0xa0) {
                        ESP_LOGW("p1reader", "Unknown frame format (0x%02X). Resetting.", read_byte);
                        ChangeState(states::ERROR_RECOVERY);
                        return;
                    }
                    m_crc_position = ((0x1f & m_message_buffer[1]) << 8) + m_message_buffer[2] - 1;
                }

                // If end of CRC is reached, start verifying CRC
                if (m_crc_position > 0 && m_message_buffer_position > m_crc_position) {
                    if (m_data_format == data_formats::ASCII && read_byte == '\n') {
                        ChangeState(states::VERIFYING_CRC);
                    } else if (m_data_format == data_formats::BINARY && m_message_buffer_position == m_crc_position + 3) {
                        if (read_byte != 0x7e) {
                            ESP_LOGW("p1reader", "Unexpected end. Resetting.");
                            ChangeState(states::ERROR_RECOVERY);
                            return;
                        }
                        ChangeState(states::VERIFYING_CRC);
                    }
                }
            }
            break;
        case states::VERIFYING_CRC: {
            int crc_from_msg = -1;
            int crc = 0;

            if (m_data_format == data_formats::ASCII) {
                crc_from_msg = (int) strtol(m_message_buffer + m_crc_position, NULL, 16);
                crc = crc16_ccitt_false(m_message_buffer, m_crc_position - 1);
            } else if (m_data_format == data_formats::BINARY) {
                crc_from_msg = (m_message_buffer[m_crc_position + 1] << 8) + m_message_buffer[m_crc_position];
                crc = crc16_x25(&m_message_buffer[1], m_crc_position - 1);
            }

            if (crc == crc_from_msg) {
                ESP_LOGD("p1reader", "CRC verification OK");
                if (m_data_format == data_formats::ASCII) {
                    ChangeState(states::PROCESSING_ASCII);
                } else if (m_data_format == data_formats::BINARY) {
                    ChangeState(states::PROCESSING_BINARY);
                } else {
                    ChangeState(states::ERROR_RECOVERY);
                }
                return;
            }

            // CRC verification failed
            ESP_LOGW("p1reader", "CRC mismatch, calculated %04X != %04X. Message ignored.", crc, crc_from_msg);
            if (m_data_format == data_formats::ASCII) {
                ESP_LOGD("p1reader", "Buffer:\n%s (%d)", m_message_buffer, m_message_buffer_position);
            } else if (m_data_format == data_formats::BINARY) {
                ESP_LOGD("p1reader", "Buffer:");
                char hex_buffer[81];
                hex_buffer[80] = '\0';
                for (int i = 0; i * 40 < m_message_buffer_position; i++) {
                    int j;
                    for (j = 0; j + i * 40 < m_message_buffer_position && j < 40; j++) {
                        sprintf(&hex_buffer[2*j], "%02X", m_message_buffer[j + i*40]);
                    }
                    if (j >= m_message_buffer_position) {
                        hex_buffer[j] = '\0';
                    }
                    ESP_LOGD("p1reader", "%s", hex_buffer);
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
                while (*end_of_line != '\n' && *end_of_line != '\r' && *end_of_line != '\0') ++end_of_line;
                char const end_of_line_char{ *end_of_line };
                *end_of_line = '\0';

                if (end_of_line != m_start_of_data) {
                    int minor{ -1 }, major{ -1 }, micro{ -1 };
                    double value{ -1.0 };
                    if (sscanf(m_start_of_data, "1-0:%d.%d.%d(%lf", &major, &minor, &micro, &value) != 4) {
                        ESP_LOGD("p1reader", "Could not parse value from line '%s'", m_start_of_data);
                    }
                    else {
                        uint32_t const obisCode{ OBIS(major, minor, micro) };
                        Sensor *S{ GetSensor(obisCode) };
                        if (S != nullptr) S->publish_state(value);
                        else {
                            ESP_LOGD("p1reader", "No sensor matching: %d.%d.%d (0x%x)", major, minor, micro, obisCode);
                        }
                    }
                }
                *end_of_line = end_of_line_char;
                if (end_of_line_char == '\0') {
                    ChangeState(states::RESENDING);
                    return;
                }
                m_start_of_data = end_of_line + 1;
            } while (millis() - loop_start_time < 25);
            break;
        case states::PROCESSING_BINARY: {
            int current_data_pos = 3;
            while (m_message_buffer[current_data_pos] != 0x13 && current_data_pos <= m_crc_position) ++current_data_pos;
            if (current_data_pos > m_crc_position) {
                ESP_LOGW("p1reader", "Could not find control byte. Resetting.");
                ChangeState(states::ERROR_RECOVERY);
                return;
            }
            current_data_pos += 6;

            uint32_t obis_code = 0x00;
            char value_buffer[101];
            while (current_data_pos < m_crc_position) {
                uint8_t type  = m_message_buffer[current_data_pos];
                switch (type) {
                case 0x00:
                    current_data_pos++;
                    break;
                case 0x01: // array
                    current_data_pos += 2;
                    break;
                case 0x02: // struct
                    current_data_pos += 2;
                    break;
                case 0x06: {// unsigned double long
                    uint32_t v = (m_message_buffer[current_data_pos + 1] << 24 | m_message_buffer[current_data_pos + 2] << 16 | m_message_buffer[current_data_pos + 3] << 8 | m_message_buffer[current_data_pos + 4]);
                    float fv = v * 1.0 / 1000;
                    Sensor *S{ GetSensor(obis_code) };
                    if (S != nullptr) S->publish_state(fv);
                    current_data_pos += 1 + 4;
                    break;
                }
                case 0x09: // octet
                    if (m_message_buffer[current_data_pos + 1] == 0x06) {
                        int minor{ -1 }, major{ -1 }, micro{ -1 };
                        major = m_message_buffer[current_data_pos + 4];
                        minor = m_message_buffer[current_data_pos + 5];
                        micro = m_message_buffer[current_data_pos + 6];

                        obis_code = OBIS(major, minor, micro);
                    }
                    current_data_pos += 2 + (int) m_message_buffer[current_data_pos + 1];
                    break;
                case 0x0a: // string
                    current_data_pos += 2 + (int) m_message_buffer[current_data_pos + 1];
                    break;
                case 0x0c: // datetime
                    current_data_pos += 13;
                    break;
                case 0x0f: // scalar
                    current_data_pos += 2;
                    break;
                case 0x10: {// unsigned long
                    uint16_t v = (m_message_buffer[current_data_pos + 1] << 8 | m_message_buffer[current_data_pos + 2]);
                    float fv = v * 1.0 / 10;
                    Sensor *S{ GetSensor(obis_code) };
                    if (S != nullptr) S->publish_state(fv);
                    current_data_pos += 3;
                    break;
                }
                case 0x12: {// signed long
                    uint16_t v = (m_message_buffer[current_data_pos + 1] << 8 | m_message_buffer[current_data_pos + 2]);
                    float fv = v * 1.0 / 10;
                    Sensor *S{ GetSensor(obis_code) };
                    if (S != nullptr) S->publish_state(fv);
                    current_data_pos += 3;
                    break;
                }
                case 0x16: // enum
                    current_data_pos += 2;
                    break;
                default:
                    ESP_LOGW("p1reader", "Unsupported data type 0x%02x. Resetting.", type);
                    ChangeState(states::ERROR_RECOVERY);
                    return;
                }
            }

            //uint8_t nbr_of_data_points = m_message_buffer[19];
            //int current_data_point_pos = 19 + 1;

            //ESP_LOGD("p1reader", "Found %d data points to process.", nbr_of_data_points);

            //while (nbr_of_data_points-- > 0) {
                //if (m_message_buffer[current_data_point_pos] = 0x02) {
                //    ESP_LOGW("p1reader", "Expected data point not found. Resetting.");
                //    ChangeState(states::ERROR_RECOVERY);
                //    return;
                //}
                //uint8_t data_class = m_message_buffer[current_data_point_pos + 1];

                //int current_value_pos = current_data_point_pos + 2;
                //while (m_message_buffer[current_value_pos] != 0x02) {
                    
                //}

                //char obisCode[12];
                //uint32_t const obisCode{ OBIS(m_message_buffer[current_data_point_pos + 6], m_message_buffer[current_data_point_pos + 7], m_message_buffer[m_message_buffer_position + 8]) };
                //sprintf(obisCode, "%u.%u.%u", m_message_buffer[current_data_point_pos + 6], m_message_buffer[current_data_point_pos + 7], m_message_buffer[m_message_buffer_position + 8]);
//ESP_LOGD("obis", "OBIS %s", obisCode);

                // char value[16];
                // uint8_t value_format = m_message_buffer[current_data_point_pos + 10];
                // if (value_format == 0x09) {
                //   uint8_t value_length = m_message_buffer[current_data_point_pos + 11];
                //   if (value_length > 0) {
                //     // No value read but still parseRow may be called below
                //   }
                // } else if (value_format == 0x06) {
                //   uint32_t v = (m_message_buffer[current_data_point_pos + 11] << 24 | m_message_buffer[current_data_point_pos + 12] << 16 | m_message_buffer[m_message_buffer_position + 13] << 8 | m_message_buffer[m_message_buffer_position + 14]);
                //   float fv = v * 1.0 / 1000;
                //   sprintf(value, "%.3f", fv);
                // } else if (value_format == 0x10) {
                //   int16_t v = (m_message_buffer[current_data_point_pos + 11] << 8 | m_message_buffer[current_data_point_pos + 12]);
                //   float fv = v * 1.0 / 10;
                //   sprintf(value, "%.1f", fv);
                // } else if (value_format == 0x12) {
                //   uint16_t v = (m_message_buffer[current_data_point_pos + 11] << 8 | m_message_buffer[current_data_point_pos + 12]);
                //   float fv = v * 1.0 / 10;
                //   sprintf(value, "%.1f", fv);
                // } else {
                //   ESP_LOGW("p1reader", "unknown value format %d", value_format);
                // }

                //if (data_class == 0x03) {
                //    ESP_LOGD("p1reader", "read a data point");
                //   //ESP_LOGD("obis", "OBIS %s = %s", obisCode, value);
                //   //parseRow(parsed, obisCode, value);
                //   uint32_t const obisCode{ OBIS(major, minor, micro) };
                //   Sensor *S{ GetSensor(obisCode) };
                //   if (S != nullptr) S->publish_state(value);
                //   else {
                //     ESP_LOGD("p1reader", "No sensor matching: %d.%d.%d (0x%x)", major, minor, micro, obisCode);
                //   }
                //}

            //}
            ChangeState(states::RESENDING);
            return;
        }
        case states::RESENDING:
            if (m_bytes_resent < m_message_buffer_position) {
                int max_bytes_to_send{ 200 };
                do {
                    write(m_message_buffer[m_bytes_resent++]);
                } while (m_bytes_resent < m_message_buffer_position && max_bytes_to_send-- != 0);
            }
            else {
                ChangeState(states::WAITING);
            }
            break;
        case states::WAITING:
            if (m_display_time_stats) {
                m_display_time_stats = false;
                ESP_LOGD("p1reader", "Cycle times: Message = %d ms (%d loops), Processing = %d ms (%d loops), (Total = %d ms)",
                    m_processing_time - m_reading_message_time,
                    m_num_message_loops,
                    m_waiting_time - m_processing_time,
                    m_num_processing_loops,
                    m_waiting_time - m_reading_message_time
                );
            }
            if (m_minimum_period_ms < loop_start_time - m_reading_message_time) {
                ChangeState(states::READING_MESSAGE);
            }
            break;
        case states::ERROR_RECOVERY:
            if (available()) {
                int max_bytes_to_discard{ 200 };
                do { read(); } while (available() && max_bytes_to_discard-- != 0);
            }
            else if (500 < loop_start_time - m_error_recovery_time) ChangeState(states::WAITING);
            break;
        }
    }

private:
    uint16_t crc16_ccitt_false(char* pData, int length) {
        int i;
        uint16_t wCrc = 0xffff;
        while (length--) {
            wCrc ^= *(unsigned char *)pData++ << 8;
            for (i=0; i < 8; i++)
                wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
        }
        return wCrc;
    }

    uint16_t crc16_x25(char* pData, int length) {
        int i;
        uint16_t wCrc = 0xffff;
        while (length--) {
            wCrc ^= *(unsigned char *)pData++ << 0;
            for (i=0; i < 8; i++)
                wCrc = wCrc & 0x0001 ? (wCrc >> 1) ^ 0x8408 : wCrc >> 1;
        }
        return wCrc ^ 0xffff;
    }


    // Find the matching sensor in the linked list (or return nullptr
    // if it does not exist.
    Sensor *GetSensor(uint32_t obisCode) const
    {
        SensorListItem *sensor_list{ m_sensor_list };
        while (sensor_list != nullptr) {
            if (obisCode == sensor_list->GetCode()) return sensor_list->GetSensor();
            sensor_list = sensor_list->Next();
        }
        return nullptr;
    }

};
