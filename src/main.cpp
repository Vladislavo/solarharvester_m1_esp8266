#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include <INA226.h>
#include "MAX17043.h"

#include "gateway_protocol.h"
#include "device_control.h"

#define BAUDRATE                            115200

#define I2C_SCLK                            D5
#define I2C_SDATA                           D4

#define WIFI_SSID                           "ISRcomunicaciones*34*"
#define WIFI_PASSWORD                       "52dq4yk9"

#define GATEWAY_IP_ADDRESS                  IPAddress(51,254,120,244)
#define GATEWAY_PORT                        54345
#define GATEWAY_APP_KEY                     "04676630"
#define GATEWAY_DEV_ID                      1
#define SECURE                              0 // encrypted payload

#define DATA_SEND_RETRIES_MAX               3

#define GET_DATA                            0
#define SET_SAMPLING_PERIOD                 1
#define DEV_REBOOT                          2

#define DEFAULT_SAMPLE_PERIOD               60000 // 1min


typedef struct {
    uint32_t sample_period; // 1 min default
} dev_conf_t;

typedef struct {
    uint32_t utc;

    float ina_pv_voltage;
    float ina_pv_power;
    float ina_shunt_voltage;
    float ina_shunt_current;

    float max_battery_voltage;
    float max_battery_state_of_charge;
} sensor_data_t;

WiFiUDP clientUDP;

INA226 ina;
MAX17043 batteryMonitor;

uint32_t last_millis = 0;


void gateway_protocol_send_stat(gateway_protocol_stat_t stat);
// encode sensors data for sending (martialize)
void  gateway_protocol_send_data_payload_encode (
    const sensor_data_t *sensor_data, 
    uint8_t *payload, 
    uint8_t *payload_length);
// request pending message from the gateway
void gateway_protocol_req_pend(void);
// send sensors data
gateway_protocol_stat_t send_sensor_data(const sensor_data_t *sensor_data);

uint8_t send_udp_datagram (
    const IPAddress ip, 
    const uint16_t port, 
    const uint8_t *packet, 
    const uint8_t packet_length);


void print_array_hex(uint8_t *array, uint8_t array_length, const char *sep);

void checkConfig(void);

sensor_data_t sensor_data;
dev_conf_t dev_conf;
gateway_protocol_stat_t g_stat = GATEWAY_PROTOCOL_STAT_NACK;

uint8_t secure_key[GATEWAY_PROTOCOL_SECURE_KEY_SIZE] = { 0x59, 0x51, 0xf9, 0xd2, 0xb4, 0xb5, 0xe5, 0xa2, 0x08, 0x26, 0xf5, 0x84, 0xd6, 0x29, 0x38, 0x43 };


void setup() {
    Serial.begin(BAUDRATE);
    Wire.begin(I2C_SDATA, I2C_SCLK);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("WiFi connected");
    Serial.println("IP address set: ");
    Serial.println(WiFi.localIP()); //print LAN IP

    Serial.println("Initialize INA226");
    Serial.println("-----------------------------------------------");

    // Default INA226 address is 0x40
    ina.begin();

    // Configure INA226
    ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);

    // Calibrate INA226. Rshunt = 0.56 ohm, Max excepted current = 1A
    ina.calibrate(0.56, 0.15);

    // Display configuration
    checkConfig();

    Serial.println("-----------------------------------------------");

    batteryMonitor.reset();
    batteryMonitor.quickStart();
    delay(1000);

    clientUDP.begin(GATEWAY_PORT);

    gateway_protocol_init((uint8_t *)GATEWAY_APP_KEY, GATEWAY_DEV_ID, secure_key, SECURE);

    dev_conf.sample_period = DEFAULT_SAMPLE_PERIOD;

    last_millis = millis();
}

void loop() {
    if (last_millis + dev_conf.sample_period < millis()) {
        sensor_data.ina_pv_voltage = ina.readBusVoltage();
        sensor_data.ina_pv_power = ina.readBusPower();
        sensor_data.ina_shunt_voltage = ina.readShuntVoltage();
        sensor_data.ina_shunt_current = ina.readShuntCurrent();

        sensor_data.max_battery_voltage = batteryMonitor.getVCell();
        sensor_data.max_battery_state_of_charge = batteryMonitor.getSoC();
        
        sensor_data.utc = 0;

        g_stat = send_sensor_data(&sensor_data);
        
        if (g_stat == GATEWAY_PROTOCOL_STAT_ACK) {
            Serial.println("ACK");

        } else if (g_stat == GATEWAY_PROTOCOL_STAT_ACK_PEND) {
            Serial.println("ACK_PEND received");
            gateway_protocol_req_pend();
        } else {
            Serial.printf("NACK %02X", g_stat);
        }
        last_millis = millis();
        
        Serial.print("Bus voltage:     ");
        Serial.print(sensor_data.ina_pv_voltage, 5);
        Serial.println(" V");

        Serial.print("Bus power:       ");
        Serial.print(sensor_data.ina_pv_power, 5);
        Serial.println(" W");

        Serial.print("Shunt voltage:   ");
        Serial.print(sensor_data.ina_shunt_voltage, 5);
        Serial.println(" V");

        Serial.print("Shunt current:   ");
        Serial.print(sensor_data.ina_shunt_current, 5);
        Serial.println(" A");

        Serial.print("Voltage:         ");
        Serial.print(sensor_data.max_battery_voltage, 4);
        Serial.println(" V");

        Serial.print("State of charge: ");
        Serial.print(sensor_data.max_battery_state_of_charge);
        Serial.println(" %");

        Serial.println();
    }
}


void checkConfig()
{
  Serial.print("Mode:                  ");
  switch (ina.getMode())
  {
    case INA226_MODE_POWER_DOWN:      Serial.println("Power-Down"); break;
    case INA226_MODE_SHUNT_TRIG:      Serial.println("Shunt Voltage, Triggered"); break;
    case INA226_MODE_BUS_TRIG:        Serial.println("Bus Voltage, Triggered"); break;
    case INA226_MODE_SHUNT_BUS_TRIG:  Serial.println("Shunt and Bus, Triggered"); break;
    case INA226_MODE_ADC_OFF:         Serial.println("ADC Off"); break;
    case INA226_MODE_SHUNT_CONT:      Serial.println("Shunt Voltage, Continuous"); break;
    case INA226_MODE_BUS_CONT:        Serial.println("Bus Voltage, Continuous"); break;
    case INA226_MODE_SHUNT_BUS_CONT:  Serial.println("Shunt and Bus, Continuous"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Samples average:       ");
  switch (ina.getAverages())
  {
    case INA226_AVERAGES_1:           Serial.println("1 sample"); break;
    case INA226_AVERAGES_4:           Serial.println("4 samples"); break;
    case INA226_AVERAGES_16:          Serial.println("16 samples"); break;
    case INA226_AVERAGES_64:          Serial.println("64 samples"); break;
    case INA226_AVERAGES_128:         Serial.println("128 samples"); break;
    case INA226_AVERAGES_256:         Serial.println("256 samples"); break;
    case INA226_AVERAGES_512:         Serial.println("512 samples"); break;
    case INA226_AVERAGES_1024:        Serial.println("1024 samples"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus conversion time:   ");
  switch (ina.getBusConversionTime())
  {
    case INA226_BUS_CONV_TIME_140US:  Serial.println("140uS"); break;
    case INA226_BUS_CONV_TIME_204US:  Serial.println("204uS"); break;
    case INA226_BUS_CONV_TIME_332US:  Serial.println("332uS"); break;
    case INA226_BUS_CONV_TIME_588US:  Serial.println("558uS"); break;
    case INA226_BUS_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_BUS_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_BUS_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_BUS_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Shunt conversion time: ");
  switch (ina.getShuntConversionTime())
  {
    case INA226_SHUNT_CONV_TIME_140US:  Serial.println("140uS"); break;
    case INA226_SHUNT_CONV_TIME_204US:  Serial.println("204uS"); break;
    case INA226_SHUNT_CONV_TIME_332US:  Serial.println("332uS"); break;
    case INA226_SHUNT_CONV_TIME_588US:  Serial.println("558uS"); break;
    case INA226_SHUNT_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_SHUNT_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_SHUNT_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_SHUNT_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Max possible current:  ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current:           ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage:     ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power:             ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
}

void gateway_protocol_send_stat(gateway_protocol_stat_t stat) {
    uint8_t buffer[32];
    uint8_t buffer_length = 0;

    gateway_protocol_packet_encode (
        GATEWAY_PROTOCOL_PACKET_TYPE_STAT,
        1, (uint8_t *)&stat,
        &buffer_length, buffer);

    send_udp_datagram(GATEWAY_IP_ADDRESS, GATEWAY_PORT, buffer, buffer_length);
}

void  gateway_protocol_send_data_payload_encode (
    const sensor_data_t *sensor_data, 
    uint8_t *payload, 
    uint8_t *payload_length) 
{
    *payload_length = 0;

    memcpy(&payload[*payload_length], &sensor_data->utc, sizeof(sensor_data->utc));
    (*payload_length) += sizeof(sensor_data->utc);

    memcpy(&payload[*payload_length], &sensor_data->ina_pv_voltage, sizeof(sensor_data->ina_pv_voltage));
    (*payload_length) += sizeof(sensor_data->ina_pv_voltage);

    memcpy(&payload[*payload_length], &sensor_data->ina_pv_power, sizeof(sensor_data->ina_pv_power));
    (*payload_length) += sizeof(sensor_data->ina_pv_power);

    memcpy(&payload[*payload_length], &sensor_data->ina_shunt_voltage, sizeof(sensor_data->ina_shunt_voltage));
    (*payload_length) += sizeof(sensor_data->ina_shunt_voltage);

    memcpy(&payload[*payload_length], &sensor_data->ina_shunt_current, sizeof(sensor_data->ina_shunt_current));
    (*payload_length) += sizeof(sensor_data->ina_shunt_current);

    memcpy(&payload[*payload_length], &sensor_data->max_battery_voltage, sizeof(sensor_data->max_battery_voltage));
    (*payload_length) += sizeof(sensor_data->max_battery_voltage);

    memcpy(&payload[*payload_length], &sensor_data->max_battery_state_of_charge, sizeof(sensor_data->max_battery_state_of_charge));
    (*payload_length) += sizeof(sensor_data->max_battery_state_of_charge);
}

void gateway_protocol_req_pend() {
    uint8_t buffer[GATEWAY_PROTOCOL_MAX_PACKET_SIZE];
    uint8_t buffer_length = 0;
    uint8_t payload[GATEWAY_PROTOCOL_MAX_PACKET_SIZE];
    uint8_t payload_length = 0;

    gateway_protocol_packet_encode(
        GATEWAY_PROTOCOL_PACKET_TYPE_PEND_REQ,
        0, buffer,
        &buffer_length, buffer);

    send_udp_datagram(GATEWAY_IP_ADDRESS, GATEWAY_PORT, buffer, buffer_length);
    
    uint32_t wait_ms = millis() + 1000;
    while(!clientUDP.parsePacket() && wait_ms > millis()) {}
    if ((buffer_length = clientUDP.read(buffer, sizeof(buffer)))) {
        gateway_protocol_packet_type_t p_type;
        if (gateway_protocol_packet_decode(
            &p_type,
            &payload_length, payload,
            buffer_length, buffer))
        {
            if (p_type == GATEWAY_PROTOCOL_PACKET_TYPE_PEND_SEND) {
                Serial.println("PEND SEND received");
                print_array_hex(payload, payload_length, " : ");

                uint8_t op, arg_len, args[32];

                device_control_packet_decode(&op, &arg_len, args, payload_length, payload);

                Serial.printf("PEND SEND decoded op = %d, arg_len = %d, args : \n", op, arg_len);
                print_array_hex(args, arg_len, " : ");

                if (op == GET_DATA) {
                    // extra data_send
                    gateway_protocol_send_stat(GATEWAY_PROTOCOL_STAT_ACK);
                    // assign extra data send
                    last_millis = 0;
                } else if (op == SET_SAMPLING_PERIOD) {
                    uint32_t samp_period;
                    // if (arg_len == sizeof(samp_period)) {
                        samp_period = atoi((char *)args);
                        // memcpy(&samp_period, args, sizeof(samp_period));
                        dev_conf.sample_period = samp_period;

                        Serial.printf("sampling period set to %lu from %s\n", dev_conf.sample_period, args);

                        gateway_protocol_send_stat(GATEWAY_PROTOCOL_STAT_ACK);
                    // } else {
                    //     ESP_LOGE(TAG, "arg_len error %d != 4", arg_len);
                    //     gateway_protocol_send_stat(GATEWAY_PROTOCOL_STAT_NACK);
                    // }
                } else if (op == DEV_REBOOT) {
                    Serial.println("going to restart...");
                    gateway_protocol_send_stat(GATEWAY_PROTOCOL_STAT_ACK);
                    ESP.restart();
                    // see peripherals reset
                } else {
                    // error unknown op
                    Serial.println("UNKNOWN OPERATION");
                    gateway_protocol_send_stat(GATEWAY_PROTOCOL_STAT_NACK);
                }
            }
        }
    } else {
        Serial.println("NO PEND SEND received");
    }
}

gateway_protocol_stat_t send_sensor_data(const sensor_data_t *sensor_data) {
    gateway_protocol_stat_t g_stat = GATEWAY_PROTOCOL_STAT_NACK;
    uint8_t data_send_retries = DATA_SEND_RETRIES_MAX;
    uint8_t received_ack = 0;
    uint8_t buffer[GATEWAY_PROTOCOL_MAX_PACKET_SIZE];
    uint8_t buffer_length = 0;
    uint8_t payload[GATEWAY_PROTOCOL_MAX_PACKET_SIZE];
    uint8_t payload_length = 0;

    gateway_protocol_send_data_payload_encode(sensor_data, payload, &payload_length);
    
    do {
        gateway_protocol_packet_encode(
            GATEWAY_PROTOCOL_PACKET_TYPE_DATA_SEND,
            payload_length, payload,
            &buffer_length, buffer);

        Serial.printf("sending %d bytes...\n", buffer_length);
    
        if (send_udp_datagram(GATEWAY_IP_ADDRESS, GATEWAY_PORT, buffer, buffer_length)) {
            Serial.println("data send done!");
        } else {
            Serial.println("data send error");
        }
    
        uint32_t wait_ms = millis() + 1000;
        while(!clientUDP.parsePacket() && wait_ms > millis()) {}
        if ((buffer_length = clientUDP.read((unsigned char *)buffer, sizeof(buffer)))) {
            gateway_protocol_packet_type_t p_type;
            if (gateway_protocol_packet_decode(
                &p_type,
                &payload_length, payload,
                buffer_length, buffer)) 
            {
                print_array_hex(payload, payload_length, " : ");
                if (p_type == GATEWAY_PROTOCOL_PACKET_TYPE_STAT &&
                    payload_length == 1)
                {
                    g_stat = (gateway_protocol_stat_t) payload[0];
                    received_ack = 1;
                    Serial.printf("STAT RECEIVED %02X\n", g_stat);
                } else {
                    Serial.printf("STAT content error p_type = %02X, buf = %02X\n", p_type, payload[0]);
                }
            } else {
                Serial.println("STAT packet decode error");
            }
        } else {
            Serial.println("NO STAT RECEIVED");
            delay(20);
        }
    } while (!received_ack && --data_send_retries);

    return g_stat;
}

uint8_t send_udp_datagram (
    const IPAddress ip, 
    const uint16_t port, 
    const uint8_t *packet, 
    const uint8_t packet_length) 
{
    clientUDP.beginPacket(GATEWAY_IP_ADDRESS, GATEWAY_PORT);
    clientUDP.write(packet, packet_length);

    return clientUDP.endPacket();
}

void print_array_hex(uint8_t *array, uint8_t array_length, const char *sep) {
    #if CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    for(uint8_t i = 0; i < array_length-1; i++) {
        Serial.printf("%02X%s", array[i], sep);
    }
    Serial.printf("%02X\r\n", array[array_length-1]);
    #endif
}