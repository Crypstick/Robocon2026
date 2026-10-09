#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <map>
#include <string>

#include <ArduinoJson.h>

// this is the one connected to pc

uint8_t slaveAddress[] = {0x00, 0x4B, 0x12, 0xEC, 0x59, 0x98}; // the mac address of the one we sending to

// PMK and LMK keys pls dont touch and ensure they match in master and slave programs
static const char* PMK_KEY_STR = "XABqkVajNdMXUG8t";
static const char* LMK_KEY_STR = "JUO+D7GGRsJDhi1V";

String outgoingMessage;
String incomingMessage;  
unsigned long lastMessageTime = 0; // millis() when the last controller message arrived


esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void OnDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);

StaticJsonDocument<200> slaveCommands;


void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

    delay(1000);
    Serial.print("mac address: ");
    Serial.println(WiFi.macAddress()); // must match masterAddress in RoboconRobotSlave.ino

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_set_pmk((uint8_t *)PMK_KEY_STR);


    esp_now_register_recv_cb(OnDataReceive);
    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo.peer_addr, slaveAddress, 6);
    peerInfo.channel = 0;
    //Set the receiver device LMK key
    for (uint8_t i = 0; i < 16; i++) {
        peerInfo.lmk[i] = LMK_KEY_STR[i];
    }
    // Set encryption to true
    peerInfo.encrypt = true;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_rate_config_t rateConfig = {};
    rateConfig.phymode = WIFI_PHY_MODE_LR;
    rateConfig.rate = WIFI_PHY_RATE_LORA_500K;
    if (esp_now_set_peer_rate_config(slaveAddress, &rateConfig) != ESP_OK) {
        Serial.println("Failed to set peer rate config");
    }

    Serial.println("Ready");
}

void loop() {
    if (Serial.available()) {
        String controllerInputs = Serial.readStringUntil('\n');
        controllerInputs.trim();

        if (controllerInputs.length() == 0 || controllerInputs.length() > 249) { 
            Serial.println("Skipped: empty or too long");
            return;
        }

        // serializeJson(slaveCommands, outgoingMessage);
        outgoingMessage = controllerInputs;

        esp_err_t result = esp_now_send(slaveAddress,(uint8_t*)outgoingMessage.c_str(),  outgoingMessage.length() + 1);
        if (result != ESP_OK) { // only print failures, to keep the PC's serial input clean
            Serial.print("Send error: ");
            Serial.println(esp_err_to_name(result)); // prints the reason, e.g. ESP_ERR_ESPNOW_NO_MEM
        }
    }
}

// --- processing functions --- //
void processPC(String receivedInput) {
    JsonDocument doc;


    deserializeJson(doc, receivedInput);

    // joysticks = [[left_x, left_y], [right_x, right_y]]
    slaveCommands["KP"] = doc["KP"];
    slaveCommands["KI"]  = doc["KI"];
    slaveCommands["KD"]  = doc["KD"];
    slaveCommands["transVel"]  = doc["transVel"];
    slaveCommands["transAngle"]  = doc["transAngle"];
    slaveCommands["cmdRot"] = doc["cmdRot"];
    // hand to loop()
}


// --- ESPNOW specific functions --- //

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Delivery Fail");
    }
}



void OnDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    incomingMessage = String((const char*)incomingData, len);

    // one line per message, e.g. {"speed":1.2345}
        Serial.println(incomingMessage);
}