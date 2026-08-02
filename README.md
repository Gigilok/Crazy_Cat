============================================================
 GATO LOUCO v3.1 - LEIA-ME
============================================================
Firmware para ESP32 - ferramenta multiuso de RF/WiFi/BLE
inspirado no Flipper Zero, com tela OLED, menu navegavel
por botoes e API REST para controle via APK Android.


============================================================
 ÍNDICE
============================================================

1. Hardware necessário
2. Esquema de Pinos
3. Módulos e Bibliotecas
4. Estrutura do Projeto
5. Funcionalidades
6. API REST (Porta 8080)
7. Compilação e Gravação
8. APK Android
9. Solução de Problemas


============================================================
 1. HARDWARE NECESSÁRIO
============================================================

Placa:
- ESP32 DevKit V1 (38 pinos, 4MB Flash)
Processador: dual core Xtensa LX6 240 MHz
RAM: 320 KB (usados ​​~ 80 KB / 24,5%)
Flash: 4 MB (usados ​​~ 1,65 MB / 52,5%)

Módulos externos:
- Scanner/Jammer NRF24L01+ 2,4 GHz 2400-2525 MHz
- CC1101 Captura/Reprodução Sub-GHz 300-348/387-464/779-928 MHz
Tela OLED SSD1306 128x64 monocromática I2C 400kHz
Buzzer 5V Feedback sonoro ~2kHz
- 4 Botões Navegação no menu INPUT_PULLUP

Alimentacao:
- USB 5V / bateria LiPo 3.7V com regulador 3.3V
- Consumo médio: ~80mA (até 200mA com TX ativo)


============================================================
 2. ESQUEMA DE PINOS
============================================================

Tela OLED (I2C):
  Pino ESP32 -> Pino OLED
  GPIO 21 -> SDA
  GPIO 22 -> SCL (SCK)
  3,3V -> VCC
  GND -> GND

Botoes (4x, pull-up interno):
  Pino ESP32 -> Botao -> GND
  GPIO 17 -> ATIVADO
  GPIO 15 -> PARA BAIXO
  GPIO 32 -> SELECIONAR
  GPIO 33 -> VOLTAR

NRF24L01+ (SPI2 / VSPI):
  Pino ESP32 -> Pino NRF24
  GPIO 26 -> CE
  GPIO 25 -> CSN (SS)
  GPIO 18 -> SCK
  GPIO 23 -> MOSI
  GPIO 19 -> MISO
  3,3V -> VCC *** NAO usar 5V ***
  GND -> GND

CC1101 (SPI3 / HSPI):
  Pino ESP32 -> Pino CC1101
  GPIO 4 -> GDO0 (transmissão de dados/assíncrona)
  GPIO 16 -> GDO2 (status)
  GPIO 27 -> CSN (SS)
  GPIO 14 -> SCK
  GPIO 13 -> MOSI
  GPIO 12 -> MISO
  3,3V -> VCC
  GND -> GND

Campainha:
  Pino ESP32 -> Buzzer
  GPIO 5 -> + (positivo)
  GND -> - (negativo)

LED integrado:
  GPIO 2 -> LED (feedback de status)


============================================================
 3. MÓDULOS E BIBLIOTECAS
============================================================

Dependências (platformio.ini):
  lib_deps =
      Adafruit/Adafruit SSD1306 @ ^2.5.7
      adafruit/Biblioteca GFX da Adafruit @ ^1.11.9
      nrf24/RF24 @ ^1.4.7
      bblanchon/ArduinoJson @ ^6.21.0

Bibliotecas internas do ESP32 (já incluídas no framework):
- WiFi.h Modo AP/STA, escanear, desautenticar
- API esp_wifi.h de baixo nível (transmissão 802.11, modo promíscuo)
- BLEDevice.h Varredura BLE e publicidade
- WebServer.h Servidor HTTP (porta 8080)
- DNSServer.h Portal cativo DNS (porta 53)
- SPI.h Barramento SPI (2 instâncias: VSPI + HSPI)
- Wire.h I2C para OLED

Plataforma:
  plataforma = espressif32@6.5.0

Pinado em v6.5.0 porque usa Arduino-ESP32 v2.0.17 (ESP-IDF v4.4).
Versões mais novas (v3.x) usam NimBLE em vez de Bluedroid e
quebram a compatibilidade do BLE.


============================================================
 4. ESTRUTURA DO PROJETO
============================================================

Gato_Louco/
|
+-- platformio.ini Configuração do PlatformIO
+-- .github/workflows/build.yml CI para GitHub Actions
|
+-- incluir/
| +-- config.h Pinos, estruturas, estados do menu
| +-- wifi_api.h Declarações da API REST
| +-- wifi_handshake.h Captura EAPOL e PCAP
| +-- wifi_portal.h Portal cativo do Gêmeo Maligno
|
+-- src/
    +-- main.cpp setup() e loop() principais
    +-- config.cpp Variáveis ​​globais e toggleWiFi()
    +-- globals.cpp Variáveis ​​globais de estado
    |
    +-- display.cpp OLED + imagens do Garfield
    +-- input.cpp Leitura dos 4 botões
    +-- buzzer.cpp Feedback sonoro
    +-- menu.cpp Menu navegavel + todos os renderizadores
    +-- settings.cpp Configurações (brilho, pinos, módulos)
    |
    +-- nrf24.cpp Scanner/Jammer/Analyzer 2.4GHz
    +-- cc1101.cpp Captura/Reprodução/Interferência Sub-GHz
    +-- bruteforce.cpp Brute force portao e carro
    |
    +-- bluetooth.cpp BLE scan + spam (4 marcas)
    +-- wifi_attacks.cpp Desautenticação, Evil Twin, clone de BSSID
    +-- wifi_handshake.cpp Captura EAPOL e export PCAP
    +-- wifi_portal.cpp Portal cativo HTML
    +-- wifi_api.cpp Servidor HTTP REST (porta 8080)
    +-- wifi_bypass.cpp Ignorar verificação de integridade de quadros brutos ieee80211


============================================================
 5. FUNCIONALIDADES
============================================================

NRF24 (2,4 GHz):
- Jammer: troca de canal 0-125 onda portadora de comunicação
- Scanner 16ch: gráfico de barras RSSI
- Spectrum 64ch: estilo Flipper Zero com cachoeira
- Analisador: detecta sinais e salva no buffer

CC1101 (Sub-GHz):
- Copiar Sinal: captura RF em 4 bandas (315/433/868/915 MHz)
- Reproduzir: replay de sinais capturados
- Interferidor de RF: onda portadora em 433,92 MHz
- RollJam: captura + bloqueio (catch & jam)
- Analisador: espectro de 64 pontos (300-928 MHz)
- Transmit Raw: envia timings do Keeloq via APK

Wi-fi:
- Desautenticação: clonar BSSID + envio de frames 802.11 desautenticação
- Evil Twin: clona rede + portal cativo para capturar senha
- Handshake Capture: captura EAPOL M1-M4 e exporta PCAP
- Scan: lista de redes com RSSI/canal/BSSID
- AP: roda rede "CrazyCat" na porta 8080

Bluetooth (BLE):
- Scan: encontra dispositivos BLE próximos
- Spam: inunda com payloads Apple/Samsung/Google/Xiaomi
  - AirPods Pro, AirPods 3, AirPods Max
  Galaxy Buds, Pixel Buds, JBL, Sony
  - Xiaomi/Redmi

Ataques:
- Bloqueador de drones: portadora de 915 MHz via CC1101
- Localização de drones: scanner RSSI em 868/915MHz
- Congelamento da câmera: bloqueador de 2,4 GHz via NRF24
- Portao de Força Bruta: 16,7M de códigos em 433,92MHz
- Carro Força Bruta: 10 marcas (Toyota, Honda, Ford, etc.)

Configurações:
- Teste de ato
- Teste de módulos
- Ajuste de caixa do OLED
- Gerenciar gravacoes salvas
- Ativar/desativar o Wi-Fi
- Status da conexão


============================================================
 6. API REST (Porta 8080)
============================================================

Principais endpoints:

  GET /api/status Status completo do sistema
  GET /api/networks Lista de redes WiFi
  POST /api/networks/scan Escaneia redes
  POST /api/deauth/start?id=N Iniciar desautenticação
  POST /api/deauth/stop Para desautenticar
  POST /api/eviltwin/start?id=N Iniciando Evil Twin
  POST /api/eviltwin/stop Para Evil Twin
  GET /api/handshake Status do handshake
  GET /api/handshake/download Baixa PCAP
  POST /api/nrf24/jammer/start Jammer 2.4GHz
  POST /api/nrf24/jammer/stop Para jammer NRF24
  POST /api/nrf24/scanner/start Scanner 2.4GHz
  GET /api/nrf24/scan Dados do scanner (16 canais)
  GET /api/nrf24/spec Dados spectrum (64ch)
  POST /api/cc1101/copy Captura de sinal
  POST /api/cc1101/replay?id=N Sinal de reprodução
  GET /api/cc1101/signals Lista de sinais salvos
  GET /api/cc1101/raw?id=N Sinal bruto (timings)
  POST /api/cc1101/transmit_raw TX Keeloq (corpo JSON)
  POST /api/cc1101/jammer/start Jammer Sub-GHz
  POST /api/cc1101/rolljam/start Inicia RollJam
  POST /api/cc1101/analyzer/start Analisador Sub-GHz
  GET /api/cc1101/analyzer/data Dados do analisador
  POST /api/attack/bt/scan Scan BLE
  GET /api/attack/bt/devices Lista de dispositivos BLE
  POST /api/attack/bt/jammer/start Spam BLE
  POST /api/attack/drone/jammer/start Drone de bloqueio 915MHz
  POST /api/attack/camera/freeze/start Jammer camera 2.4GHz
  POST /api/attack/bf/gate/start Força bruta portao
  POST /api/attack/bf/car/start Ataque de força bruta carro
  GET /api/attack/bf/status Status do brute force
  GET /api/attack/bf/brands Marcas de carro
  POST /api/settings/brightness Ajusta brilho OLED
  POST /api/settings/wifi/toggle Liga/desliga WiFi
  POST /api/menu/navigate Navega sem menu
  POST /api/btn Simula botao

Exemplo de resposta JSON (GET /api/status):
  {
    "menu": 0,
    "nome_do_menu": "MENU",
    "wifi_enabled": verdadeiro,
    "desauth_active": falso,
    "eviltwin_active": falso,
    "handshake_status": "Parado",
    "handshake_complete": falso,
    "nrf24_jammer": falso,
    "nrf24_scanning": falso,
    "cc1101_capturando": falso,
    "drone_jammer": falso,
    "congelar_câmera": falso,
    "bt_jammer": falso,
    "ble_available": falso,
    "ble_scanning": falso,
    "força bruta": falso,
    "contagem_de_redes": 5,
    "btdevice_count": 3,
    "cc1101_signals": 2,
    "nrf24_signals": 0
  }

Conexão:
- IP: 192.168.4.1 (ponto de acesso Modo)
- Porta 8080: API REST
- Porta 80: portal cativo do Gêmeo Maligno
- SSID AP: CrazyCat
- Senha AP: crazycat123


============================================================
 7. COMPILAÇÃO E GRAVATAÇÃO
============================================================

Pré-requisitos:
- Python 3.11+
- PlatformIO Core 6.1.x
  pip install "platformio>=6.1,<7"

Compilador:
  CD Crazy_Cat
  corrida pio

Gravar (4 binários necessários):
  esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600
    write_flash 0x1000 .pio/build/esp32dev/bootloader.bin \
                0x8000 .pio/build/esp32dev/partitions.bin \
                0xe000 boot_app0.bin \
                0x10000 .pio/build/esp32dev/firmware.bin

*** IMPORTANTE ***
Grave SEMPRE os 4 arquivos. Se gravar então o firmware.bin,
o bootloader antigo pode ser incompatível e causar bootloop.

Número de série do monitor:
  monitor de dispositivo pio --porta /dev/ttyUSB0 --baud 115200

Tabela de partições (huge_app.csv):
  nvs 0x9000 20 KB
  otadata 0xe000 8KB
  app0 0x10000 3072KB (firmware do aplicativo)
  espiões 0x310000 896 KB
  coredump 0x3f0000 64KB


============================================================
 8. APK ANDROID
============================================================

O APK se conecta via WiFi ao ESP32 (IP 192.168.4.1:8080).

Fluxo de uso:
1. Ligue o ESP32
2. Conecte o celular na rede WiFi CrazyCat (senha crazycat123)
3. Abra o APK - ele detecta o ESP32 automaticamente
4. Use como ferramentas pela interface gráfica

Endpoints usados ​​pelo APK:
- Polling de status a cada 2s
- Gráficos de scanner atualizados a cada 200-300ms
- Baixe o PCAP para crack com aircrack-ng

Ferramentas do APK:
- Sub-GHz: copiar, reproduzir, código rolante, bloqueador, analisador
- NRF24: scanner de 64 canais, bloqueador de sinal
- WiFi: escanear, desautenticar, gêmeo malvado, crackear (aircrack-ng)
- Bluetooth: escanear, spam
- Ataques: drone jammer, câmera congelada, força bruta


============================================================
 9. SOLUÇÃO DE PROBLEMAS
============================================================

Loop de inicialização:
  Sintoma: ESP reinicia a cada ~600ms, não passa do bootloader

  Causas:
  1. PlatformIO 7.x instalou framework v3.x
     Correção: platform = espressif32@6.5.0 sem platformio.ini
  2. Nao gravou os 4 binarios
     Correção: bootloader grave + partições + boot_app0 + firmware
  3. Cache corrompido
     Correção: rm -rf .pio && pio run

Flash corrompida:
  esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
  Depois grave os 4 binários novamente

NRF24 não responde:
- Verifique alimentação 3,3V (NAO 5V)
- Confirme pinos: CE=26, CSN=25, SCK=18, MOSI=23, MISO=19
- Veja no seriado: [NRF24] radio.begin() OK ou FAIL

CC1101 não inicializa:
- Verifique SPI: CSN=27, SCK=14, MOSI=13, MISO=12, GDO0=4
- Serial deve mostrar [CC1101] Configurado com sucesso!
- Se falhar: partnum == 0x00 ou 0xFF = módulo não responde

BLE não funciona:
- BLE e inicializado sob demanda (menu Bluetooth)
- Se crashar: veja serial [BT] FATAL: pServer null
- Conflito WiFi+BLE: firmware gerencia com inicialização lenta

PCAP®/corrompido:
- Use Evil Twin primeiro para capturar handshake
- Status: GET /api/handshake deve mostrar completo: true
- Download: GET /api/handshake/download

APK naovi:
- Confira que está na rede CrazyCat
- IP deve ser 192.168.4.1
- Porta 8080
- Teste no navegador: http://192.168.4.1:8080/api/status

Evil Twin não mostra página de login:
- O portal cativo roda na porta 80
- O cliente deve acessar http://192.168.4.1/ (sem :8080)
- Redirecionamento de DNS automaticamente quando conectado no AP


============================================================
 ESPECIFICAÇÕES TÉCNICAS
============================================================

Plataforma ESP32 DevKit V1
Framework Arduino-ESP32 v2.0.17 (ESP-IDF v4.4)
PlatformIO 6.1.x (pinado)
RAM usada 80.492 / 327.680 bytes (24,6%)
Flash usada 1.650.029 / 3.145.728 bytes (52,5%)
Tamanho do firmware: aproximadamente 1,65 MB
Partição huge_app.csv (aplicativo de 3 MB)
Bootloader 17.536 bytes
API REST Porta 8080
Portal Cativo Porta 80 + DNS 53
Serial 115200 baud
Bibliotecas externas 4 (Adafruit SSD1306, GFX, RF24, ArduinoJson)


============================================================
 AVISO LEGAL
============================================================

Este projeto e para fins educacionais e de pesquisa.
O uso de ferramentas de jamming, deauth e captura de handshake
em redes que você não possui e ILEGAL na maioria dos paises.
Use apenas em seu próprio equipamento ou com autorização
explícita.


============================================================
 SOBRE
============================================================

Gato Louco v3.1
Desenvolvido por Gigilok

- Tela de bota com Garfield
API REST para APK Android
- 17 módulos de ferramentas RF/WiFi/BLE
- Tela OLED 128x64
- 2 barramentos SPI independentes (VSPI + HSPI)
