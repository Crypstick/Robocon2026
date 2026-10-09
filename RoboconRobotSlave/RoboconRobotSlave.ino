#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include "CytronMotorDriver.h"
#include "ODriveUART.h"

#include <math.h>


// this one is connected to ROBOT  //

// --- function declarations --- //

void processController(String receivedInput);

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void OnDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);

void driveTrain_setSpeed(int topLeftSpeed, int topRightSpeed, int bottomLeftSpeed, int bottomRightSpeed);
void stopAll();

// --- variable declarations --- //

String incomingMessage;
unsigned long lastMessageTime = 0; // millis() when the last controller message arrived

//
float TRVel = 0; float TLVel = 0; float BRVel = 0; float BLVel = 0;
int KP = 0; int KI = 0; int KD = 0;
float transVel = 0; float transAngle = 0; float cmdRot = 0;

esp_now_peer_info_t peerInfo;
uint8_t masterAddress[] = {0x78, 0x1C, 0x3C, 0xF6, 0x75, 0x24}; // the mac of the one sending to

// same as on transmitter
static const char* PMK_KEY_STR = "XABqkVajNdMXUG8t";
static const char* LMK_KEY_STR = "JUO+D7GGRsJDhi1V";

// Odrive declarations
HardwareSerial& odrive_serial = Serial2;
ODriveUART odrive(odrive_serial);

void setup() {
    odrive_serial.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

    delay(1000);
    Serial.print("mac address: ");
    Serial.println(WiFi.macAddress());

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
    Serial.println("Waiting for ODrive...");
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
        delay(100);
    }
    Serial.println("found ODrive");
    Serial.print("DC voltage: ");
    Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

    // make sure the ODrive is in velocity mode with a ramp (defaults to position mode)
    Serial.print("control_mode: ");
    Serial.println(odrive.getParameterAsInt("axis0.controller.config.control_mode"));
    Serial.print("input_mode: ");
    Serial.println(odrive.getParameterAsInt("axis0.controller.config.input_mode"));
    Serial.print("vel_limit: ");
    Serial.println(odrive.getParameterAsFloat("axis0.controller.config.vel_limit"));
    Serial.println("Enabling closed loop control..."); // basically use realtime feedback to manage stuff
    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }
    
    Serial.println("ODrive running!");
}

void loop() {
    // all ODrive talking and sending happens here, so the receive callback stays fast

    // if the ODrive disarmed (e.g. current limit), print why and re-enable it
    if (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        Serial.print("ODrive disarmed, reason: 0x");
        Serial.println(odrive.getParameterAsInt("axis0.disarm_reason"), HEX);
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    if (millis() - lastMessageTime > 500) { // controller lost -> stop
        transVel = 0;
        cmdRot = 0;
    }
    // driveTrain_setSpeed(leftPower, rightPower, leftPower, rightPower);
    TLVel = transVel * sin(transAngle);

    odrive.setVelocity(TLVel);

    // send the current speed to the master
    ODriveFeedback feedback = odrive.getFeedback();
    String outgoingMessage = "{\"speed\":" + String(feedback.vel, 4) + "}";
    esp_err_t result = esp_now_send(masterAddress, (uint8_t*)outgoingMessage.c_str(), outgoingMessage.length() + 1);
    if (result != ESP_OK) {
        Serial.print("Send error: ");
        Serial.println(esp_err_to_name(result));
    }

    // debug: commanded vs measured speed
    
    Serial.print("cmd vel: ");
    Serial.print(TLVel);
    Serial.print("  measured vel: ");
    Serial.print(feedback.vel);
    // TEMP debug: what the ODrive thinks it is doing
    Serial.print("  input_vel: ");      // the command the ODrive received
    Serial.print(odrive.getParameterAsFloat("axis0.controller.input_vel"));
    Serial.print("  vel_setpoint: ");   // the ramped target it is currently aiming for
    Serial.println(odrive.getParameterAsFloat("axis0.controller.vel_setpoint"));
    delay(20); // sends speed to the master (50 ms = 20 Hz)
}

// --- self-defined functions --- //

// runs inside the receive callback: only parse and save values here.
// no ODrive calls or Serial prints - those are slow and block WiFi.

void processMaster(String receivedInput) {
    JsonDocument commands;
    deserializeJson(commands, receivedInput);

    KP = commands["KP"];
    KI  = commands["KI"];
    KD  = commands["KD"];
    transVel  = commands["transVel"];
    transAngle  = commands["transAngle"];
    cmdRot = commands["cmdRot"];
}

// --- motor control functions --- //

/*
void stopAll() {
  bottomLeftMotor.setSpeed(0);
  bottomRightMotor.setSpeed(0);
  topLeftMotor.setSpeed(0);
  topRightMotor.setSpeed(0);
}

void driveTrain_setSpeed(int topLeftSpeed, int topRightSpeed, int bottomLeftSpeed, int bottomRightSpeed) { //max 255 
  bottomLeftMotor.setSpeed(bottomLeftSpeed*-1);
  bottomRightMotor.setSpeed(bottomRightSpeed);
  topLeftMotor.setSpeed(topLeftSpeed*-1);
  topRightMotor.setSpeed(topRightSpeed);
};
*/

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
