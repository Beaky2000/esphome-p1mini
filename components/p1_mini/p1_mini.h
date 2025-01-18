#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"

//#include "esphome/components/mqtt/mqtt_datetime.h"

#include <map>

#define OBIS_CODE( major, minor, micro) ((major & 0xfff) << 16 | (minor & 0xff) << 8 | (micro & 0xff))

namespace esphome {
    namespace p1_mini {


        class IP1MiniSensor
        {
        public:
            virtual ~IP1MiniSensor() = default;
            virtual void publish_val(double) = 0;
            virtual uint32_t Obis() const = 0;
            virtual double Multiplier() const = 0;
        };

        class P1MiniSensorBase : public IP1MiniSensor
        {
            uint32_t const m_obis;
            double const m_multiplier;
        public:
            P1MiniSensorBase(std::string obis_code, double multiplier);
            virtual uint32_t Obis() const { return m_obis; }
            virtual double Multiplier() const { return m_multiplier; }
        };

        class IP1MiniTextSensor
        {
        public:
            virtual ~IP1MiniTextSensor() = default;
            virtual void publish_val(std::string) = 0;
            virtual std::string Identifier() const = 0;
            virtual uint32_t Obis() const = 0;
        };

        class P1MiniTextSensorBase : public IP1MiniTextSensor
        {
            std::string const m_identifier;
            uint32_t const m_obis;
        public:
            P1MiniTextSensorBase(std::string identifier);
            virtual std::string Identifier() const { return m_identifier; }
            virtual uint32_t Obis() const { return m_obis; }
        };

        class ReadyToReceiveTrigger : public Trigger<> { };
        class UpdateReceivedTrigger : public Trigger<> { };
        class CommunicationErrorTrigger : public Trigger<> { };

        enum class data_formats {
            ASCII,
            BINARY
        };

        class P1Mini : public uart::UARTDevice, public Component {
        public:


            P1Mini(uint32_t min_period_ms, int buffer_size, bool secondary_p1, data_formats data_format);

            void setup() override;
            void loop() override;
            void dump_config() override;

            void register_sensor(IP1MiniSensor *sensor)
            {
                m_sensors.emplace(sensor->Obis(), sensor);
            }

            void register_text_sensor(IP1MiniTextSensor *sensor);

            void register_ready_to_receive_trigger(ReadyToReceiveTrigger *trigger) { m_ready_to_receive_triggers.push_back(trigger); }
            void register_update_received_trigger(UpdateReceivedTrigger *trigger) { m_update_received_triggers.push_back(trigger); }
            void register_communication_error_trigger(CommunicationErrorTrigger *trigger) { m_communication_error_triggers.push_back(trigger); }

        private:

            unsigned long m_identifying_message_time{ 0 };
            unsigned long m_reading_message_time{ 0 };
            unsigned long m_verifying_crc_time{ 0 };
            unsigned long m_processing_time{ 0 };
            unsigned long m_waiting_time{ 0 };
            unsigned long m_error_recovery_time{ 0 };
            int m_num_message_loops{ 0 };
            int m_num_processing_loops{ 0 };
            bool m_display_time_stats{ false };
            uint32_t obis_code{ 0x00 };

            // Store the message as it is being received:
            std::unique_ptr<char> m_message_buffer_UP;
            int m_message_buffer_size;
            char *m_message_buffer{ nullptr };
            int m_message_buffer_position{ 0 };
            int m_crc_position{ 0 };

            std::unique_ptr<char> m_dlms_message_buffer_UP;
            char *m_dlms_message_buffer{ nullptr };
            int m_dlms_message_buffer_position{ 0 };

            unsigned char m_dlms_struct_size[10];
            unsigned char m_dlms_struct_offset[10];
            int m_dlms_struct_level{ 0 };

            // Keeps track of the start of the data record while processing.
            char *m_start_of_data;

            char GetByte()
            {
                char const C{ static_cast<char>(read()) };
                if (m_secondary_p1) write(C);
                return C;
            }

            enum class states {
                IDENTIFYING_MESSAGE,
                READING_MESSAGE,
                VERIFYING_CRC,
                PROCESSING_ASCII,
                PROCESSING_BINARY,
                WAITING,
                ERROR_RECOVERY
            };
            enum states m_state { states::ERROR_RECOVERY };

            void ChangeState(enum states new_state);

            uint32_t const m_min_period_ms;
            bool const m_secondary_p1;
            enum data_formats const m_data_format;

            std::map<uint32_t, IP1MiniSensor *> m_sensors;
            std::map<uint32_t, IP1MiniTextSensor *> m_obis_text_sensors;
            std::vector<IP1MiniTextSensor *> m_text_sensors; // Keep sorted so longer identifiers are first!
            std::vector<ReadyToReceiveTrigger *> m_ready_to_receive_triggers;
            std::vector<UpdateReceivedTrigger *> m_update_received_triggers;
            std::vector<CommunicationErrorTrigger *> m_communication_error_triggers;

            constexpr static int discard_log_num_bytes{ 32 };
            char m_discard_log_buffer[discard_log_num_bytes * 2 + 1];
            char *m_discard_log_position{ m_discard_log_buffer };
            char *const m_discard_log_end{ m_discard_log_buffer + (discard_log_num_bytes * 2) };

            void AddByteToDiscardLog(uint8_t byte);
            void FlushDiscardLog();

            bool m_useComputedTariff{ true };
            uint32_t m_counter_import[4];
            uint32_t m_counter_import_previous[4];
            uint32_t m_counter_export[4];
            uint32_t m_counter_export_previous[4];

            char m_text_value[100];
            double m_value{ 0.0 };
            int m_scalar{ 0 };
            int m_unit{ 0 };

            enum class value_kind {
                UNKNOW,
                NUMBER,
                TEXT,
            };
            enum value_kind m_value_kind { value_kind::UNKNOW };

            const char* UnitToString(int unit);

            bool IsDlsmBufferSizeAvailable(int size);

        };


    }  // namespace p1_mini_component
}  // namespace esphome
