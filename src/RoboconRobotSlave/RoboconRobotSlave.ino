#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include <AHRSProtocol.h>
#include <IMUProtocol.h>
#include <IMURegisters.h>
#include <Wire.h>

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include "driver/twai.h"
#include "ODriveESP32TWAI.hpp"

#include <math.h>



// this one is connected to ROBOT  //

// --- function declarations --- //

void processController(String receivedInput);

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void OnDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);

void driveTrain_setSpeed(int imuFail = 0);
void stopAll();

bool setupCan();

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
void onCanMessage(const CanMsg& msg);

bool receiveHeartbeats();
bool putClosedLoops();

void handleDriveErrors();



// --- variable declarations --- //

String incomingMessage;
unsigned long lastMessageTime = 0; // millis() when the last controller message arrived
unsigned long lastTelemetry = 0;

float TRVel = 0; float TLVel = 0; float BRVel = 0; float BLVel = 0;
int KP = 0; int KI = 0; int KD = 0;
float transVel = 0; float transAngle = 0; float cmdRot = 0; float driveAngle = 0;
float yaw = 0; float pitch = 0; float roll = 0; float heading = 0; float zeroOffset = 0;

bool zeroYawCmd = false;       
bool lastZeroYawCmd = false;   

int latchedError[4] = {0, 0, 0, 0};    
bool clearErrorsCmd = false;           
bool lastClearErrorsCmd = false;

float COMMAND_PERIOD_MS = 20;          // loop() period
float lastCommandTime = 0;

esp_now_peer_info_t peerInfo;
uint8_t masterAddress[] = {0x78, 0x1C, 0x3C, 0xF5, 0x09, 0x20}; // the mac of the one sending to


// same as on transmitter
static const char* PMK_KEY_STR = "XABqkVajNdMXUG8t";
static const char* LMK_KEY_STR = "JUO+D7GGRsJDhi1V";

int register_address = NAVX_REG_YAW_L;

// --- Odrive declarations --- //

#define CAN_BAUDRATE  250000   // CAN bus baudrate. Max`ke sure this matches for every device on the bus

#define TRodrv1_NODE_ID 1  // ODrive node_id for TRodrv1
#define TLodrv2_NODE_ID 2
#define BLodrv3_NODE_ID 3
#define BRodrv4_NODE_ID 4 // watchdog enabled 16/10/26

// Pins used to connect to CAN bus transceiver. the CRX is to the RX pin, CTX to the TX pin
#define ESP32_TWAI_TX_PIN 26
#define ESP32_TWAI_RX_PIN 25

#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT  0x32
#define NUM_BYTES_TO_READ                    8

byte imuData[512];

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
  unsigned long last_heartbeat_ms = 0; // millis() when the last heartbeat arrived, used to detect CAN loss
};

ESP32TWAIIntf can_intf;

ODriveCAN TRodrv1(wrap_can_intf(can_intf), TRodrv1_NODE_ID); 
ODriveCAN TLodrv2(wrap_can_intf(can_intf), TLodrv2_NODE_ID); 
ODriveCAN BLodrv3(wrap_can_intf(can_intf), BLodrv3_NODE_ID);
ODriveCAN BRodrv4(wrap_can_intf(can_intf), BRodrv4_NODE_ID);

ODriveCAN* odrives[] = {&TRodrv1, &TLodrv2, &BLodrv3, &BRodrv4}; // Make sure all ODriveCAN instances are accounted for here

ODriveUserData odrv_user_data[4]; // index thru for 1 to 4


void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Wire.begin(22,23); // join i2c bus (address optional for master)

    for ( int i = 0; i < sizeof(imuData); i++ ) {
    imuData[i] = 0;
    }
    


    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

    delay(1000);
    Serial.print("mac address: ");
    Serial.println(WiFi.macAddress());

    // --- ESPNOW Setup --- //
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

    // Register the master as peer
    memcpy(peerInfo.peer_addr, masterAddress, 6);
    peerInfo.channel = 0;
    // Setting the master device LMK key
    for (uint8_t i = 0; i < 16; i++) {
        peerInfo.lmk[i] = LMK_KEY_STR[i];
    }
    // Set encryption to true
    peerInfo.encrypt = true;

    esp_now_register_recv_cb(OnDataReceive); // always gets data when it receives, calls wtv is the argument on receive
    esp_now_register_send_cb(OnDataSent); // register sending, calls wtv is the argument on send

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_rate_config_t rateConfig = {};
    rateConfig.phymode = WIFI_PHY_MODE_LR;
    rateConfig.rate = WIFI_PHY_RATE_LORA_500K;
    if (esp_now_set_peer_rate_config(masterAddress, &rateConfig) != ESP_OK) {
        Serial.println("Failed to set peer rate config");
    }

    Serial.println("ESP-NOW Receiver Ready");

    

    // setting up odrive //
    
    // Register callbacks for the heartbeat and encoder feedback messages
    for (int i = 0; i < 4; i++) {
        odrives[i]->onFeedback(onFeedback, &odrv_user_data[i]);
        odrives[i]->onStatus(onHeartbeat, &odrv_user_data[i]);
    }

    // Configure and initialize the CAN bus interface. 
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true); // spin indefinitely
    } else {
        Serial.println("success");
    };

    Serial.println("Waiting for ODrives...");
    float lastPrint = 0;
    while (!receiveHeartbeats()) {
        pumpEvents(can_intf); // keep pumping continuously so the rx queue never overflows

        if (millis() - lastPrint > 500) {
            lastPrint = millis();
            twai_status_info_t s;
            twai_get_status_info(&s);
            Serial.printf("missed=%lu overrun=%lu tx_err=%lu rx_err=%lu bus_err=%lu\n",
                        s.rx_missed_count, s.rx_overrun_count, s.tx_error_counter, s.rx_error_counter, s.bus_error_count);
        }
    }

    Serial.println("found ODrives");

    // request bus voltage and current (1sec timeout)
    Serial.println("attempting to read bus voltage and current");
    Get_Bus_Voltage_Current_msg_t vbus;
    bool vbusFound = false;
    
    while (!vbusFound) {
        if (!TLodrv2.request(vbus, 1000)) {
            Serial.println("vbus request failed!");
            // spin indefinitely
            vbusFound = false;
        }
        else { 
            vbusFound = true;
        }
    }

    Serial.print("DC voltage [V]: ");
    Serial.println(vbus.Bus_Voltage);
    Serial.print("DC current [A]: ");
    Serial.println(vbus.Bus_Current);

    Serial.println("Enabling closed loop control...");
    while (!putClosedLoops()) {

        for (int i = 0; i < 4; i++) {
            if (odrv_user_data[i].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrives[i]->clearErrors();
                delay(1);
                odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
        }
        // Pump events for 150ms. This delay is needed for two reasons;
        // 1. If there is an error condition, such as missing DC power, the ODrive might
        //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
        //    on the first heartbeat response, so we want to receive at least two
        //    heartbeats (100ms default interval).
        // 2. If the bus is congested, the setState command won't get through
        //    immediately but can be delayed.
        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }

    Serial.println("ODrives running!");

    while (!processIMU());
    zeroOffset = yaw; // for field oriented drive

    
}

void loop() {

    pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages. This has been found to reduce the number of dropped messages, however it can be removed for applications requiring loop times over 100Hz.
    if (millis() - lastCommandTime < COMMAND_PERIOD_MS) {
         return;
    }
    lastCommandTime = millis();

        
    bool controllerLost = (millis() - lastMessageTime > 1000);
    bool canLost = false;                                      // CAN from any ODrive
    for (int i = 0; i < 4; i++) {
        if (millis() - odrv_user_data[i].last_heartbeat_ms > 1000) {
            canLost = true;
        }
    }

    if (controllerLost || canLost) {
        transVel = 0;
        cmdRot = 0;
    }

    if (!processIMU()) {
        driveTrain_setSpeed(1); // if IMU messes up we gonna just do robot oriented drive
    } else {
        if (zeroYawCmd && !lastZeroYawCmd) {
            zeroOffset = yaw;
            Serial.println("yaw zeroed");
        }
        lastZeroYawCmd = zeroYawCmd;
        driveTrain_setSpeed(0);
    }

    if (canLost) {
        // the TWAI controller stays bus-off or stopped until restarted, so try to bring it back
        twai_status_info_t s;
        twai_get_status_info(&s);
        if (s.state == TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();
        } else if (s.state == TWAI_STATE_STOPPED) {
            twai_start();
        }
    }

    handleDriveErrors();

    // one combined message at 20 Hz, not one packet per CAN feedback frame
    if (millis() - lastTelemetry >= 50) {
        lastTelemetry = millis();
        String outgoingMessage = "{";
        for (int i = 0; i < 4; i++) {
            outgoingMessage += "\"odrv" + String(i + 1) + "-speed\":"
                             + String(odrv_user_data[i].last_feedback.Vel_Estimate, 3);
            if (i < 3) {
                outgoingMessage += ",";
            }
        }
        outgoingMessage += ",\"err\":[";
        for (int i = 0; i < 4; i++) {
            outgoingMessage += String(latchedError[i]);
            if (i < 3) {
                outgoingMessage += ",";
            }
        }
        outgoingMessage += "],\"state\":[";
        for (int i = 0; i < 4; i++) {
            outgoingMessage += String((int)odrv_user_data[i].last_heartbeat.Axis_State);
            if (i < 3) {
                outgoingMessage += ",";
            }
        }
        outgoingMessage += "]}";

        esp_err_t result = esp_now_send(masterAddress, (uint8_t*)outgoingMessage.c_str(), outgoingMessage.length() + 1);
        if (result != ESP_OK) {
            Serial.print("Send error: ");
            Serial.println(esp_err_to_name(result));
        }
    }
}

// --- self-defined functions --- //

void processMaster(String receivedInput) {
    JsonDocument commands;
    deserializeJson(commands, receivedInput);

    KP = commands["KP"];
    KI  = commands["KI"];
    KD  = commands["KD"];
    transVel  = commands["transVel"];
    transAngle  = commands["transAngle"];
    cmdRot = commands["cmdRot"];
    zeroYawCmd = commands["zeroYaw"] | false;
    clearErrorsCmd = commands["clearErrors"] | false;

}

// --- motor control functions --- //


void driveTrain_setSpeed(int imuFail) {

    if (imuFail) {
        driveAngle = transAngle;
    } else {
        driveAngle = transAngle - ((yaw - zeroOffset) * PI / 180); // confirm if i need to yaw + zeroOffset instead
    }


    float TLVel = transVel * sin(driveAngle) - cmdRot;
    float TRVel = transVel * cos(driveAngle) + cmdRot;
    float BLVel = transVel * cos(driveAngle) - cmdRot;
    float BRVel = transVel * sin(driveAngle) + cmdRot;

    TRodrv1.setVelocity(-TRVel);
    TLodrv2.setVelocity(TLVel);
    BLodrv3.setVelocity(BLVel);
    BRodrv4.setVelocity(-BRVel);
};

// --- ESPNOW specific functions --- //

void OnDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) { // can change what we do with received data within this func
    // keep this fast: just parse and save, no prints
    incomingMessage = String((const char*)incomingData, len);
    processMaster(incomingMessage);
    lastMessageTime = millis();
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Fail");
    }
}

// --- Odrive CAN specific functions --- //

bool setupCan() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)ESP32_TWAI_TX_PIN,
        (gpio_num_t)ESP32_TWAI_RX_PIN,
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 128; // default is 5, which overflows with 4 ODrives sending feedback + heartbeats

    twai_timing_config_t t_config;
    switch (CAN_BAUDRATE) {
        case 1000000: t_config = TWAI_TIMING_CONFIG_1MBITS(); break;
        case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 100000:  t_config = TWAI_TIMING_CONFIG_100KBITS(); break;
        default:      t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }

    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }

    return true;
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
  odrv_user_data->last_heartbeat_ms = millis();
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
    if ((msg.id & 0x1F) == 0x01) { // heartbeat cmd id; node id is the upper bits
        Serial.printf("heartbeat from node %lu\n", (unsigned long)(msg.id >> 5));
    }
    for (auto odrive: odrives) {
        onReceive(msg, *odrive);
    }
}

bool receiveHeartbeats() {

        for (int i = 0; i < 4; i++) {
        if (!odrv_user_data[i].received_heartbeat) {
            return false;
        }
    }
    return true;
    /*
    for testing CAN bus in isolation, its using TLodrv2

    if (!odrv_user_data[1].received_heartbeat) {
        Serial.print(String(1));
        Serial.println("failed heartbeat");
        return false;
    }
    return true;
    */
}

bool putClosedLoops() {
    bool allClosed = true;
    for (int i = 0; i < 4; i++) {
        if (odrv_user_data[i].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
            Serial.printf("node %d not in closed loop\n", i + 1);
            allClosed = false;
        }
    }
    return allClosed;
}

// latch every ODrive axis error until the PC sends clearErrors
void handleDriveErrors() {
    if (clearErrorsCmd && !lastClearErrorsCmd) {
        for (int i = 0; i < 4; i++) {
            odrives[i]->clearErrors();
            delay(1);
            odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            odrv_user_data[i].last_heartbeat.Axis_Error = 0;   // stops the old heartbeat relatching it
            latchedError[i] = 0;
        }
        Serial.println("errors cleared");
    } else {
        for (int i = 0; i < 4; i++) {
            latchedError[i] |= odrv_user_data[i].last_heartbeat.Axis_Error;
        }
    }
    lastClearErrorsCmd = clearErrorsCmd;
}

/* --- IMU-based functions  --- */

bool processIMU() {
    // point the navX at the yaw register and tell it how many bytes we want
    Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);
    Wire.write(register_address);
    Wire.write(NUM_BYTES_TO_READ);
    if (Wire.endTransmission() != 0) {
        Serial.println("no ACK from IMU");
        return false;                                
    }

    int received = Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ);
    delay(1);
    if (received != NUM_BYTES_TO_READ) {
        Serial.println("fewer bytes read");
        return false;                                 
    }

    int imuBytes = 0;
    while (Wire.available() && imuBytes < NUM_BYTES_TO_READ) {
        imuData[imuBytes] = Wire.read();
        imuBytes++;
    }
    if (imuBytes != NUM_BYTES_TO_READ) {
        return false;
    }

    // Decode received imuData to floating-point orientation values
    yaw     = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&imuData[0]);
    pitch   = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&imuData[2]);
    roll    = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&imuData[4]);
    heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&imuData[6]);
    Serial.print("yaw:  ");
    Serial.print(yaw,2);
    Serial.print("  pitch:  ");
    Serial.print(pitch,2);
    Serial.print("  roll:  ");
    Serial.print(roll,2);
    Serial.print("  heading:  ");
    Serial.print(heading,2);  
    Serial.println("");
    return true;
}


