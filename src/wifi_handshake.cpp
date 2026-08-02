// ============================================================
// wifi_handshake.cpp - v4.2 Stream Edition + PCAP Export
// ============================================================
#include "wifi_handshake.h"
#include "config.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <sys/time.h>

#define MAX_EAPOL_FRAMES    24
#define MAX_FRAME_SIZE      512
#define EAPOL_ETHERTYPE     0x888E

struct EapolFrame {
    uint32_t ts_sec;
    uint32_t ts_usec;
    uint16_t len;
    uint8_t  data[MAX_FRAME_SIZE];
    uint8_t  msgType;  // 1=M1, 2=M2, 3=M3, 4=M4
};

static EapolFrame eapolBuffer[MAX_EAPOL_FRAMES];
static uint8_t eapolCount = 0;
static bool handshakeCapturing = false;
static bool handshakeComplete = false;
static uint8_t messagesFound = 0;  // bitmask
static char handshakeStatus[32] = "Parado";

static const char base64Chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static uint16_t read16be(const uint8_t* p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

static bool isEapolFrame(const uint8_t* payload, uint16_t len, uint16_t* dataOffset) {
    if (len < 32) return false;
    uint8_t type = payload[0] & 0x0C;
    if (type != 0x08) return false;

    uint16_t offset = 24;
    uint8_t subtype = payload[0] & 0xF0;
    if (subtype == 0x80 || subtype == 0x88 || subtype == 0x90 || subtype == 0x98) offset += 2;
    uint8_t toFromDs = payload[1] & 0x03;
    if (toFromDs == 0x03) offset += 6;
    if (len < offset + 8) return false;

    if (payload[offset] != 0xAA || payload[offset+1] != 0xAA || payload[offset+2] != 0x03)
        return false;

    uint16_t etherType = ((uint16_t)payload[offset+6] << 8) | payload[offset+7];
    if (etherType != EAPOL_ETHERTYPE) return false;

    *dataOffset = offset + 8;
    return true;
}

static uint8_t identifyEapolMessage(const uint8_t* eapolBody, uint16_t bodyLen) {
    if (bodyLen < 7) return 0;
    uint8_t eapolType = eapolBody[1];
    if (eapolType != 3) return 0;  // EAPOL-Key

    uint16_t keyInfo = read16be(&eapolBody[4]);
    bool keyAck = (keyInfo & 0x0200) != 0;
    bool keyMic = (keyInfo & 0x0400) != 0;
    bool secure = (keyInfo & 0x0800) != 0;
    bool keyType = (keyInfo & 0x0008) != 0;
    if (!keyType) return 0;

    if (keyAck && !keyMic && !secure) return 1;   // M1
    if (!keyAck && keyMic && !secure) return 2;   // M2
    if (keyAck && keyMic && secure)   return 3;   // M3
    if (!keyAck && keyMic && secure)  return 4;   // M4
    return 0;
}

static void addEapolFrame(const uint8_t* frameData, uint16_t frameLen, uint8_t msgType) {
    if (eapolCount >= MAX_EAPOL_FRAMES) {
        for (int i = 0; i < MAX_EAPOL_FRAMES - 1; i++) eapolBuffer[i] = eapolBuffer[i+1];
        eapolCount = MAX_EAPOL_FRAMES - 1;
    }
    struct timeval tv;
    gettimeofday(&tv, NULL);
    EapolFrame* f = &eapolBuffer[eapolCount];
    f->ts_sec  = tv.tv_sec;
    f->ts_usec = tv.tv_usec;
    f->len     = (frameLen > MAX_FRAME_SIZE) ? MAX_FRAME_SIZE : frameLen;
    f->msgType = msgType;
    memcpy(f->data, frameData, f->len);
    eapolCount++;

    if (msgType >= 1 && msgType <= 4) messagesFound |= (1 << (msgType - 1));
    if ((messagesFound & 0x03) == 0x03) handshakeComplete = true;
    if ((messagesFound & 0x0F) == 0x0F) handshakeComplete = true;

    snprintf(handshakeStatus, 32, "EAPOL:%d M:%d%d%d%d",
        eapolCount,
        (messagesFound & 1) ? 1 : 0,
        (messagesFound & 2) ? 1 : 0,
        (messagesFound & 4) ? 1 : 0,
        (messagesFound & 8) ? 1 : 0);
}

static void handshake_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (!handshakeCapturing) return;
    if (type == WIFI_PKT_MISC) return;
    const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;
    if (len > 4) len -= 4;
    if (len > MAX_FRAME_SIZE) len = MAX_FRAME_SIZE;

    uint16_t eapolOffset = 0;
    if (!isEapolFrame(payload, len, &eapolOffset)) return;

    uint16_t bodyLen = len - eapolOffset;
    uint8_t msgType = identifyEapolMessage(&payload[eapolOffset], bodyLen);
    if (msgType > 0) {
        addEapolFrame(payload, len, msgType);
        Serial.printf("[HS] M%d captured (%d total)\n", msgType, eapolCount);
    }
}

void startHandshakeCapture() {
    if (handshakeCapturing) return;
    eapolCount = 0;
    messagesFound = 0;
    handshakeComplete = false;
    handshakeCapturing = true;
    strcpy(handshakeStatus, "Capturando...");
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&handshake_promiscuous_cb);
    wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA };
    esp_wifi_set_promiscuous_filter(&filter);
    Serial.println("[HS] Capture started");
}

void stopHandshakeCapture() {
    if (!handshakeCapturing) return;
    handshakeCapturing = false;
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(NULL);
    snprintf(handshakeStatus, 32, "Parado (%d frames)", eapolCount);
    Serial.println("[HS] Capture stopped");
}

bool isHandshakeCapturing() { return handshakeCapturing; }
const char* getHandshakeStatus() { return handshakeStatus; }
uint8_t getHandshakeMessageCount() { return eapolCount; }
bool isHandshakeComplete() { return handshakeComplete; }

void clearHandshakeBuffer() {
    eapolCount = 0;
    messagesFound = 0;
    handshakeComplete = false;
    strcpy(handshakeStatus, "Parado");
}

// ============================================================
// EXPORTAR PARA PCAP (Para o Termux quebrar a senha)
// ============================================================
String getPcapData() {
    size_t totalSize = 24; // Global header size
    for (int i = 0; i < eapolCount; i++) {
        totalSize += 16 + eapolBuffer[i].len; // Packet header + data
    }
    
    uint8_t* buf = (uint8_t*)malloc(totalSize);
    if (!buf) return "";
    
    size_t offset = 0;
    
    // PCAP Global Header (Little Endian)
    uint32_t magic = 0xa1b2c3d4;
    uint16_t verMaj = 2;
    uint16_t verMin = 4;
    int32_t tz = 0;
    uint32_t sigfigs = 0;
    uint32_t snaplen = 65535;
    uint32_t network = 105; // LINKTYPE_IEEE802_11
    
    memcpy(&buf[offset], &magic, 4); offset += 4;
    memcpy(&buf[offset], &verMaj, 2); offset += 2;
    memcpy(&buf[offset], &verMin, 2); offset += 2;
    memcpy(&buf[offset], &tz, 4); offset += 4;
    memcpy(&buf[offset], &sigfigs, 4); offset += 4;
    memcpy(&buf[offset], &snaplen, 4); offset += 4;
    memcpy(&buf[offset], &network, 4); offset += 4;
    
    // Pacotes
    for (int i = 0; i < eapolCount; i++) {
        EapolFrame* f = &eapolBuffer[i];
        
        uint32_t ts_sec = f->ts_sec;
        uint32_t ts_usec = f->ts_usec;
        uint32_t incl_len = f->len;
        uint32_t orig_len = f->len;
        
        memcpy(&buf[offset], &ts_sec, 4); offset += 4;
        memcpy(&buf[offset], &ts_usec, 4); offset += 4;
        memcpy(&buf[offset], &incl_len, 4); offset += 4;
        memcpy(&buf[offset], &orig_len, 4); offset += 4;
        
        memcpy(&buf[offset], f->data, f->len); offset += f->len;
    }
    
    String result = String((const char*)buf, totalSize);
    free(buf);
    return result;
}
