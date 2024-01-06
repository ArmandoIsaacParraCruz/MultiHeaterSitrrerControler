#include "RemoteCommunication_2.h"


uint8_t RemoteCommunication_2::mac_multiHeaterStirrer[6] = {0x34, 0x85, 0x18, 0xBD, 0x2B, 0x1C};
uint8_t RemoteCommunication_2::mac_HMI[6] = {0x40, 0x91, 0x51, 0xAB, 0x1B, 0xC0};

struct ProcessesSpecificationsMessage RemoteCommunication_2::processesSpecificationsMessage;

ProcessMessageStatus RemoteCommunication_2::processesSpecificationsMessageStatus;

esp_now_peer_info_t RemoteCommunication_2::peerInfo;


/**Set up the parameters to stablish the remote communication
 * Configura los parámetros para establecer la comunicación remota
*/
void RemoteCommunication_2::OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
    	Serial.println("Delivery Success");
    } else {
    	Serial.println("Delivery Fail");
    }

}

void IRAM_ATTR RemoteCommunication_2::OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if(len == sizeof(processesSpecificationsMessage)) {
        memcpy(&processesSpecificationsMessage, incomingData, sizeof(processesSpecificationsMessage));
        processesSpecificationsMessageStatus = ProcessMessageStatus::Received;
        Serial.println("The processesSpecificationsMessage was received");
    }
}


void RemoteCommunication_2::beginRemoteCommunication()
{
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    

    memcpy(peerInfo.peer_addr, mac_HMI, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    } else{
        Serial.println("Succeed to add peer");
    }
	
    
}


void RemoteCommunication_2::sendMeasurements(measurements& message)
{
    esp_err_t internalProcessStatus = esp_now_send(mac_HMI, (uint8_t*)&message, sizeof(message));
	
    if(internalProcessStatus == ESP_OK) {
        Serial.println("Internal Process to send data was succeed");
    } else {
		Serial.println("Internal Process to send data failed");
	}
}

void RemoteCommunication_2::sendManualAdjustmentParameters(manualAdjustmentParameters &message)
{
    esp_err_t internalProcessStatus = esp_now_send(mac_HMI, (uint8_t*)&message, sizeof(message));
	
    if(internalProcessStatus == ESP_OK) {
        Serial.println("Internal Process to send data was succeed");
    } else {
		Serial.println("Internal Process to send data failed");
	}
}

ProcessMessageStatus RemoteCommunication_2::getProcessesSpecificationsMessageStatus()
{
    return processesSpecificationsMessageStatus;
}

void RemoteCommunication_2::setProcessesSpecificationsMessageStatus(ProcessMessageStatus status)
{
    processesSpecificationsMessageStatus = status;
}
