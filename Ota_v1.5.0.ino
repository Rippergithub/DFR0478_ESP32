// =====================================================================================
// FAYDAM DATALOGGER - ESP32
// Versiyon: v1.4.0
// Açıklama: Bu kod, DS18B20 sıcaklık sensörü ve bir analog sensörden veri okuyan,
// bu verileri belirli periyotlarla veya alarm durumlarında bir web sunucusuna gönderen
// bir IoT cihazı yazılımıdır. Cihaz, enerji tasarrufu için deep sleep modunu kullanır.
// Bağlantı kuramadığında verileri LittleFS dosya sisteminde kuyruklar.
// Wi-Fi kurulumu için bir web arayüzü sunar.
// v1.4.0: Enerji optimizasyonu eklenmiştir. Cihaz artık her uyandığında değil,
// sadece veri göndermesi gerektiğinde Wi-Fi'ye bağlanmayı dener.
// =====================================================================================

// --- Kütüphaneler ---
#include <WiFi.h>              // WiFi bağlantısı için
#include <HTTPClient.h>        // HTTP istekleri için
#include <ArduinoJson.h>       // JSON ayrıştırma için
#include <time.h>              // Zaman fonksiyonları için (struct tm, mktime, settimeofday, getLocalTime)
#include <sys/time.h>          // settimeofday için gerekli olabilir
#include <OneWire.h>           // DS18B20 sıcaklık sensörü için
#include <DallasTemperature.h> // DS18B20 sensöründen sıcaklık okuma için
#include <WebServer.h>         // Web sunucusu için
#include <LittleFS.h>          // LittleFS dosya sistemi için
#include <WiFiClient.h>        // WebServer kütüphanesi için gerekli olabilir (ESP32 için genellikle dahil edilir ama emin olmak için ekliyorum)
#include <WiFiClientSecure.h>  // OTA için HTTPS bağlantısı
#include <Update.h>            // OTA firmware güncellemesi için


// --- VERSİYON BİLGİSİ ---
#define FIRMWARE_VERSION "1.4.0" // Cihazın mevcut firmware versiyonu (STRING OLMALI!)

// --- Sabit Tanımlamaları ---
#define DEFAULT_AP_PASSWORD "12345678" // Kurulum AP'si şifresi

#define API_HOST "datalogger.faydam.io" // API sunucusu adresi
#define API_USER "faydam"               // API kullanıcı adı
#define API_PASS "faydam1998"           // API şifresi
#define API_KEY "faydam"                // API anahtarı

#define ONE_WIRE_BUS 5             // DS18B20 sıcaklık sensörü için veri pini (D8 pini GPIO5'e karşılık gelir)
#define ANALOG_INPUT_PIN 32        // Analog sensör girişi (örn. A0 gibi ESP kart pinleri)

// Manuel kurulum modu tetikleyici butonu (örnek: GPIO0 = BOOT butonu, veya başka bir pin)
#define CONFIG_BUTTON_PIN 0

// --- ANALOG OKUMA VE LİNEER HARİTALAMA DEĞERLERİ ---
// Bu değerler, pil voltajını doğrudan ham ADC değerinden hesaplamak için kullanılır.
// Lütfen BATTERY_VOLTAGE_TO_MV_FACTOR değerini kendi cihazınızın ölçümlerine göre ayarlayın.
// Örneğin: Pil 3.99V iken Ham ADC 310 ise, 3.99 / 310 = 0.01287 V/ham_birim olur.
const float BATTERY_VOLTAGE_TO_MV_FACTOR = 0.01287; // V/ham_birim
// --- ANALOG OKUMA VE LİNEER HARİTALAMA SONU ---

// Periyotlar dakika cinsinden tanımlandı ve Deep Sleep saniyeye çevrilecek
// Bu define'lar artık TOPLAM SÜRE'yi (uyku + aktif) saniye cinsinden ifade ediyor.
const int NORMAL_TOTAL_PERIOD_SECONDS = 600; // Normal veri gönderimi için toplam hedef süre (10 dakika)
const int ALARM_TOTAL_PERIOD_SECONDS = 300; // Alarm durumu veri gönderimi için toplam hedef süre (5 dakika)
const int ESTIMATED_ACTIVE_TIME_SECONDS = 15; // Cihazın her aktif döngüde harcadığı tahmini süre (saniye)

#define MINUTE_SLEEP_SECONDS 60            // Her zaman 1 dakika uykuya geç

#define DEVICE_DISCONNECTED_C -127.00 // DS18B20 sensör bağlantı hatası kodu
#define uS_TO_S_FACTOR 1000000ULL      // Saniyeyi mikrosaniyeye çevirme faktörü

// HTTP isteği için maksimum tekrar deneme sayısı ve gecikme
#define MAX_HTTP_RETRIES 3
#define HTTP_RETRY_DELAY_MS 3000 // Her deneme arası gecikme (milisaniye)

// LittleFS dosya yolları
const char* WIFI_STA_CONFIG_FILE = "/wifi_sta_config.json"; // Wi-Fi STA bilgileri için dosya adı
const char* AP_SSID_FILE = "/ap_ssid.json";             // AP SSID bilgisi için dosya adı
const char* DATALOG_FILE = "/datalog.csv";             // Kuyruğa alınmış veri için dosya adı
const char* KNOWN_NETWORKS_FILE = "/wifi_known_networks.json"; // Bilinen Wi-Fi ağları için dosya adı

// --- OTA Güncelleme Ayarları ---
const char* GITHUB_USER = "Rippergithub";                 // GitHub kullanıcı adınız
const char* GITHUB_REPO = "DFR0478_ESP32";               // GitHub depo adınız
const char* GITHUB_HOST = "raw.githubusercontent.com";    // GitHub raw dosyalarının sunucusu (Bu sabit kalır)
const char* GITHUB_VERSION_FILE_PATH = "/main/version.json"; // Depodaki versiyon dosyasının yolu
// Bu CA sertifikası, GitHub'a HTTPS üzerinden güvenli bağlantı kurmak için gereklidir.
// Kendi cihazınız için güncel ve doğru sertifikayı buraya eklemeniz ÇOK ÖNEMLİDİR.
// https://www.digicert.com/kb/ssl-support/pem-file-for-digicert-root-certificates.htm adresinden
// veya tarayıcınızdan raw.githubusercontent.com adresine gidip sertifika detaylarından
// "DigiCert Global Root G2" veya "ISRG Root X1" gibi root sertifikayı alabilirsiniz.
const char* GITHUB_ROOT_CA = \
"-----BEGIN CERTIFICATE-----\n" \
"MIICjzCCAhWgAwIBAgIQXIuZxVqUxdJxVt7NiYDMJjAKBggqhkjOPQQDAzCBiDEL\n" \
"MAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNl\n" \
"eSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMT\n" \
"JVVTRVJUcnVzdCBFQ0MgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAwMjAx\n" \
"MDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNVBAgT\n" \
"Ck5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVUaGUg\n" \
"VVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBFQ0MgQ2VydGlm\n" \
"aWNhdGlvbiBBdXRob3JpdHkwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAQarFRaqflo\n" \
"I+d61SRvU8Za2EurxtW20eZzca7dnNYMYf3boIkDuAUU7FfO7l0/4iGzzvfUinng\n" \
"o4N+LZfQYcTxmdwlkWOrfzCjtHDix6EznPO/LlxTsV+zfTJ/ijTjeXmjQjBAMB0G\n" \
"A1UdDgQWBBQ64QmG1M8ZwpZ2dEl23OA1xmNjmjAOBgNVHQ8BAf8EBAMCAQYwDwYD\n" \
"VR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAwNoADBlAjA2Z6EWCNzklwBBHU6+4WMB\n" \
"zzuqQhFkoJ2UOQIReVx7Hfpkue4WQrO/isIJxOzksU0CMQDpKmFHjFJKS04YcPbW\n" \
"RNZu9YO6bVi9JNlWSOrvxKJGgYhqOkbRqZtNyWHa0V1Xahg=\n" \
"-----END CERTIFICATE-----\n";

// RTC belleğinde kalıcı olacak sayaçlar ve veriler (Deep Sleep sonrası değerlerini korur)
RTC_DATA_ATTR int bootCount = 0;
// RTC_DATA_ATTR int shortSleepCounter = 0; // shortSleepCounter artık kullanılmıyor, yerine accumulatedSleepSeconds var
RTC_DATA_ATTR bool inAlarmState = false; // Cihaz alarm durumunda mı?
RTC_DATA_ATTR int otaCheckCounter = 0;   // OTA kontrolü için sayaç
RTC_DATA_ATTR bool shouldCheckOTA = false; // OTA kontrolünün yapılması gerektiğini belirtir
RTC_DATA_ATTR unsigned long accumulatedSleepSeconds = 0; // Toplam birikmiş uyku süresi (saniye cinsinden)

#define OTA_CHECK_INTERVAL_MAIN_SENDS 5 // Her 5 başarılı ana veri gönderiminde bir OTA kontrolü yap

#define MAX_ALARM_TEMP_READINGS (NORMAL_TOTAL_PERIOD_SECONDS / MINUTE_SLEEP_SECONDS) // Alarm sıcaklık okuma kapasitesi (yaklaşık 10 okuma)
RTC_DATA_ATTR int rtcAlarmTemperatureReadings[MAX_ALARM_TEMP_READINGS]; // Alarm durumundaki sıcaklık okumaları
RTC_DATA_ATTR int alarmTempIndex = 0; // Alarm sıcaklık dizisindeki indeks

// Sunucudan alınan ve RTC bellekte kalıcı olacak eşik değerleri
RTC_DATA_ATTR float TLow_threshold = -100.0; // Varsayılan düşük değer (cihaz ilk kez çalıştığında)
RTC_DATA_ATTR float THigh_threshold = 100.0; // Varsayılan yüksek değer (cihaz ilk kez çalıştığında)

// Diğer global değişkenler
bool shouldSendMainData = false; // Ana veri gönderme zamanı mı?
float temperature_c;             // Okunan sıcaklık (DS18B20)
int current_value_int;           // Okunan analog sensör değeri (Haritalanmış)
String deviceID;                 // Cihazın MAC adresi tabanlı ID'si

// Web Sunucusu ve LittleFS için global değişkenler
WebServer server(80);
String saved_ssid = "";      // Kaydedilen STA Wi-Fi SSID
String saved_password = ""; // Kaydedilen STA Wi-Fi parolası
String ap_ssid_to_use = ""; // AP'nin kullanacağı SSID (MAC adresinden veya dosyadan çekilir)

// Wi-Fi tarama işlemleri için global değişkenler
String lastScanResultsJson = "[]"; // Son tarama sonuçlarını JSON olarak depolar
unsigned long lastScanTime = 0;    // Son tarama zamanı (millis())
const long SCAN_COOLDOWN_TIME = 20000; // Tarama arası minimum süre 20 saniyeye çıkarıldı

// Bilinen ağları depolamak için bir DynamicJsonDocument
DynamicJsonDocument knownNetworksDoc(2048); // Yeterli boyutta bir belge

// DS18B20 sensörü için nesneler
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


// --- LittleFS Fonksiyonları (Wi-Fi STA Ayarları için) ---
// Wi-Fi STA kimlik bilgilerini LittleFS'e JSON formatında kaydeder
void saveWiFiCredentials(const String& ssid, const String& password) {
    DynamicJsonDocument doc(256); // JSON belgesi için bellek ayır
    doc["ssid"] = ssid;
    doc["password"] = password;

    File configFile = LittleFS.open(WIFI_STA_CONFIG_FILE, "w"); // Dosyayı yazma modunda aç
    if (!configFile) {
        Serial.println("LittleFS STA yapılandırma dosyası yazılamadı.");
        return;
    }
    serializeJson(doc, configFile); // JSON belgesini dosyaya yaz
    configFile.close(); // Dosyayı kapat
    Serial.println("Wi-Fi STA bilgileri LittleFS'e kaydedildi.");
}

// Wi-Fi STA kimlik bilgilerini LittleFS'ten okur
void loadWiFiCredentials() {
    if (LittleFS.exists(WIFI_STA_CONFIG_FILE)) { // Dosya var mı kontrol et
        File configFile = LittleFS.open(WIFI_STA_CONFIG_FILE, "r"); // Dosyayı okuma modunda aç
        if (configFile) {
            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, configFile); // Dosyadan JSON oku ve ayrıştır
            if (!error) {
                saved_ssid = doc["ssid"].as<String>();      // SSID'yi oku
                saved_password = doc["password"].as<String>(); // Parolayı oku
                Serial.printf("LittleFS'ten okunan STA SSID: '%s'\n", saved_ssid.c_str());
                Serial.printf("LittleFS'ten okunan STA Parola: '%s'\n", saved_password.c_str());
            } else {
                Serial.println("LittleFS STA yapılandırma dosyası okunamadı veya bozuk.");
            }
            configFile.close();
        } else {
            Serial.println("LittleFS STA yapılandırma dosyası açılamadı.");
        }
    } else {
        Serial.println("LittleFS STA yapılandırma dosyası bulunamadı.");
    }
}

// Wi-Fi STA kimlik bilgilerini LittleFS'ten siler (dosyayı siler)
void clearWiFiCredentials() {
    if (LittleFS.exists(WIFI_STA_CONFIG_FILE)) {
        if (LittleFS.remove(WIFI_STA_CONFIG_FILE)) {
            Serial.println("Wi-Fi STA bilgileri LittleFS'ten temizlendi.");
        } else {
            Serial.println("LittleFS Wi-Fi STA bilgileri silinemedi.");
        }
    } else {
        Serial.println("LittleFS'te temizlenecek Wi-Fi STA bilgisi yok.");
    }
    saved_ssid = "";
    saved_password = "";
}

// --- LittleFS Fonksiyonları (AP SSID Ayarı için) ---
// AP SSID'yi dosyaya kaydeder
void saveApSsidToFile(const String& ap_ssid) {
    DynamicJsonDocument doc(128); // JSON belgesi için bellek ayır (daha küçük olabilir)
    doc["ap_ssid"] = ap_ssid;

    File apFile = LittleFS.open(AP_SSID_FILE, "w"); // Dosyayı yazma modunda aç
    if (!apFile) {
        Serial.println("LittleFS AP SSID dosyası yazılamadı.");
        return;
    }
    serializeJson(doc, apFile); // JSON belgesini dosyaya yaz
    apFile.close(); // Dosyayı kapat
    Serial.println("AP SSID LittleFS'e kaydedildi.");
}

// AP SSID'yi dosyadan okur
String loadApSsidFromFile() {
    String loadedApSsid = "";
    if (LittleFS.exists(AP_SSID_FILE)) { // Dosya var mı kontrol et
        File apFile = LittleFS.open(AP_SSID_FILE, "r"); // Dosyayı okuma modunda aç
        if (apFile) {
            DynamicJsonDocument doc(128);
            DeserializationError error = deserializeJson(doc, apFile); // Dosyadan JSON oku ve ayrıştır
            if (!error) {
                loadedApSsid = doc["ap_ssid"].as<String>(); // AP SSID'yi oku
                Serial.printf("LittleFS'ten okunan AP SSID: '%s'\n", loadedApSsid.c_str());
            } else {
                Serial.println("LittleFS AP SSID dosyası okunamadı veya bozuk. Dosya siliniyor.");
                LittleFS.remove(AP_SSID_FILE); // Hata durumunda dosyayı sil, böylece yeniden oluşturulur
            }
            apFile.close();
        } else {
            Serial.println("LittleFS AP SSID dosyası açılamadı.");
        }
    } else {
        Serial.println("LittleFS AP SSID dosyası bulunamadı.");
    }
    return loadedApSsid;
}

// --- Bilinen Ağ Yönetimi Fonksiyonları ---
// Bilinen Wi-Fi Ağlarını Dosyadan Oku
void loadKnownNetworksFromFile() {
    Serial.println("Bilinen Wi-Fi ağları dosyadan okunuyor...");
    File knownNetworksFile = LittleFS.open(KNOWN_NETWORKS_FILE, "r");
    if (!knownNetworksFile) {
        Serial.println("Known Wi-Fi networks dosyasi bulunamadi. Bos liste ile baslatiliyor.");
        knownNetworksDoc.to<JsonArray>(); // Dosya yoksa veya açılamazsa, boş bir JSON dizisi ile başlat
        return;
    }

    DeserializationError error = deserializeJson(knownNetworksDoc, knownNetworksFile);
    if (error) {
        Serial.print(F("deserializeJson() hatasi (known networks): "));
        Serial.println(error.f_str());
        knownNetworksDoc.to<JsonArray>(); // Hata durumunda da boş bir JSON dizisi ile başlat
    } else {
        Serial.println("Bilinen Wi-Fi ağları basariyla yuklendi.");
    }
    knownNetworksFile.close();
}

// Bilinen Wi-Fi Ağlarını Dosyaya Kaydet
void saveKnownNetworksToFile() {
    Serial.println("Bilinen Wi-Fi ağları dosyaya kaydediliyor...");
    File knownNetworksFile = LittleFS.open(KNOWN_NETWORKS_FILE, "w"); // Yazma modunda aç (varsa üzerine yazar)
    if (!knownNetworksFile) {
        Serial.println("Known Wi-Fi networks dosyasi yazılamadı.");
        return;
    }

    if (serializeJson(knownNetworksDoc, knownNetworksFile) == 0) {
        Serial.println("Bilinen Wi-Fi ağları JSON dosyaya yazılamadı.");
    } else {
        Serial.println("Bilinen Wi-Fi ağları basariyla dosyaya kaydedildi.");
    }
    knownNetworksFile.close();
}

// İstenen SSID için bilinen parolayı döndürür (Web Sunucu Endpoint'i)
void handleGetKnownPassword() {
    String requestedSsid = server.arg("ssid");
    String foundPassword = "";

    Serial.printf("Known password istegi alindi for SSID: '%s'\n", requestedSsid.c_str());

    // knownNetworksDoc bir JsonArray içeriyor mu kontrol et
    if (knownNetworksDoc.is<JsonArray>()) {
        JsonArray networks = knownNetworksDoc.as<JsonArray>();
        for (JsonObject network : networks) {
            String ssid = network["ssid"].as<String>();
            if (ssid == requestedSsid) {
                foundPassword = network["password"].as<String>();
                Serial.printf("Password bulundu for SSID '%s': '%s'\n", requestedSsid.c_str(), foundPassword.c_str());
                break;
            }
        }
    } else {
        Serial.println("knownNetworksDoc gecerli bir JsonArray degil. Yeniden yukleniyor.");
        loadKnownNetworksFromFile(); // Olur da bir nedenle knownNetworksDoc bozulursa, yeniden yüklemeyi dene
        if (knownNetworksDoc.is<JsonArray>()) { // Yeniden yüklendikten sonra tekrar dene
            JsonArray networks = knownNetworksDoc.as<JsonArray>();
            for (JsonObject network : networks) {
                String ssid = network["ssid"].as<String>();
                if (ssid == requestedSsid) {
                    foundPassword = network["password"].as<String>();
                    Serial.printf("Password bulundu (yeniden yuklendi) for SSID '%s': '%s'\n", requestedSsid.c_str(), foundPassword.c_str());
                    break;
                }
            }
        }
    }

    DynamicJsonDocument responseDoc(128);
    responseDoc["password"] = foundPassword; // Bulunan şifreyi gönder (boşsa boş string)

    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    server.send(200, "application/json", jsonResponse);
}
// --- Bilinen Ağ Yönetimi Fonksiyonları SONU ---


// HTML özniteliklerinde kullanılacak karakterleri kaçış yapar
String escapeHtmlAttribute(const String& input) {
  String output = "";
  for (int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (c == '"') {
      output += "&quot;"; // Çift tırnak
    } else if (c == '&') {
      output += "&amp;"; // Ampersand
    } else if (c == '<') {
      output += "&lt;";  // Küçüktür işareti
    } else if (c == '>') {
      output += "&gt;";  // Büyüktür işareti
    } else {
      output += c;
    }
  }
  return output;
}

// Web sunucusunun kök dizinine gelen isteği işler
const char PROGMEM SETUP_HTML[] = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Wi-Fi Ayarları</title>
<style>
body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #f4f7f6; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; }
.container { background-color: #ffffff; padding: 30px; border-radius: 12px; box-shadow: 0 6px 20px rgba(0, 0, 0, 0.1); width: 100%; max-width: 400px; text-align: center; }
h2 { color: #333; margin-bottom: 25px; font-size: 24px; }
label { display: block; text-align: left; margin-bottom: 8px; font-weight: 600; color: #555; font-size: 15px; }
input[type="text"], input[type="password"], select {
    width: 100%; padding: 12px; margin-bottom: 20px; border: 1px solid #e0e0e0;
    border-radius: 8px; box-sizing: border-box; font-size: 16px; transition: border-color 0.3s;
}
button {
    width: 100%; padding: 12px; background-color: #007bff; color: white; border: none;
    border-radius: 8px; cursor: pointer; font-size: 17px; font-weight: 600;
    transition: background-color 0.3s, transform 0.2s; margin-top: 10px;
}
button:hover { background-color: #0056b3; transform: translateY(-2px); }
#wifiSelect { margin-bottom: 20px; }
#confirmationBox button { width: auto; padding: 10px 15px; margin: 5px; display: inline-block; }
.logo { display: block; margin: 0 auto 20px auto; height: 60px; width: auto; max-width: 100%; }
</style>
</head>
<body>
<div class="container">
<img src="/faydamlogo.png" alt="Faydam Logo" class="logo">
<h2>Wi-Fi Ayarları</h2>
<button id="scanButton" onclick="startScanAndRefresh()">Kablosuz Ağları Tara</button>
<label for="wifiSelect">Mevcut Ağlar:</label>
<select id="wifiSelect" onchange="selectSSID(this.value)">
    <option value="">Ağ taraması için butona basın...</option>
</select>
<form id="wifiForm" action="/save" method="post">
<label for="ssid">Wi-Fi Ağı (SSID):</label>
<input type="text" id="ssid" name="ssid" value="" required><br>
<label for="password">Parola:</label>
<input type="password" id="password" name="password" value=""><br>
<button type="button" onclick="showConfirmation()">Kaydet ve Yeniden Başlat</button>
</form>
<div style="margin-top: 20px; text-align: left;">
    <label>AP (Kurulum) Ağı SSID:</label>
    <input type="text" value="%AP_SSID_VALUE%" readonly style="background-color: #e9e9e9; cursor: default;"><br>
    <p style="font-size: 0.9em; color: #666;">Bu ağ, cihaz Wi-Fi bağlantısı kuramazsa otomatik olarak etkinleşir.</p>
</div>
<div id="confirmationBox" style="display: none; background-color: #f0f8ff; border: 1px solid #c0e0ff; border-radius: 8px; padding: 15px; margin-top: 20px;">
    <h3>Girilen Bilgileri Onaylayın</h3>
    <p><strong>SSID:</strong> <span id="confirmSsid"></span></p>
    <p><strong>Parola:</strong> <span id="confirmPassword"></span></p>
    <button onclick="confirmAndSave()">Onayla ve Yeniden Başlat</button>
    <button onclick="hideConfirmation()" style="background-color: #6c757d;">Geri Dön ve Düzenle</button>
</div>
<div class="firmware-info" style="margin-top: 30px; font-size: 14px; color: #888; text-align: center;">
    Firmware Version: %FIRMWARE_VERSION%
</div>
</div>
<script>
let isScanning = false;
let scanTimeoutId = null;
// SETUP_HTML içindeki <script> etiketinde yer alan getNetworks fonksiyonu

function getNetworks() {
  fetch('/getnetworks')
    .then(response => response.json())
    .then(data => {
      // Sunucudan gelen status'ü kontrol et
      if (data.status === 'scanning') {
        document.getElementById('wifiSelect').innerHTML = '<option value="">Ağlar taranıyor... Lütfen bekleyin.</option>';
        // Tarama devam ediyorsa, 2 saniye sonra tekrar sor
        setTimeout(getNetworks, 2000); 
      } else if (data.status === 'completed') {
        isScanning = false;
        const scanButton = document.getElementById('scanButton');
        if (scanButton) {
            scanButton.disabled = false;
            scanButton.textContent = 'Kablosuz Ağları Tara';
        }
        let optionsHtml = '';
        if (data.networks.length === 0) {
            optionsHtml = '<option value="">Hiç ağ bulunamadı. Tekrar tarayın.</option>';
        } else {
          optionsHtml += '<option value="">Listeden Ağ seçiniz...</option>';
          data.networks.forEach(network => {
            optionsHtml += '<option value="' + network.ssid + '">' + network.ssid + ' (' + network.rssi + ' dBm)</option>';
          });
        }
        document.getElementById('wifiSelect').innerHTML = optionsHtml;
      } else { // 'failed' veya bilinmeyen bir durum
        throw new Error('Ağ taraması başarısız oldu: ' + (data.message || 'Bilinmeyen sunucu hatası'));
      }
    })
    .catch(error => {
      console.error('Ağları alırken hata:', error);
      isScanning = false;
      const scanButton = document.getElementById('scanButton');
      if (scanButton) {
          scanButton.disabled = false;
          scanButton.textContent = 'Kablosuz Ağları Tara';
      }
      document.getElementById('wifiSelect').innerHTML = '<option value="">Ağlar yüklenemedi. Tekrar deneyin.</option>';
    });
}

function startScanAndRefresh() {
  if (isScanning) { return; }
  isScanning = true;
  const scanButton = document.getElementById('scanButton');
  if (scanButton) {
      scanButton.disabled = true;
      scanButton.textContent = 'Taranıyor...';
  }
  document.getElementById('wifiSelect').innerHTML = '<option value="">Ağlar taranıyor, lütfen bekleyin...</option>';
  fetch('/startscan')
    .then(response => {
      return response.json().then(data => {
        if (!response.ok) {
            throw new Error('Tarama başlatılamadı: ' + (data.message || 'Bilinmeyen hata'));
        }
        setTimeout(getNetworks, 1000);
      });
    })
    .catch(error => {
      console.error('Tarama başlatılırken hata:', error);
      clearTimeout(scanTimeoutId);
      isScanning = false;
      const scanButton = document.getElementById('scanButton');
      if (scanButton) {
          scanButton.disabled = false;
          scanButton.textContent = 'Kablosuz Ağları Tara';
      }
      document.getElementById('wifiSelect').innerHTML = '<option value="">Tarama başlatılırken hata oluştu: ' + error.message + '</option>';
    });
}
function selectSSID(selectedSsid) {
  const ssidInput = document.getElementById('ssid');
  const passwordInput = document.getElementById('password');
  ssidInput.value = selectedSsid;
  fetch('/getKnownPassword?ssid=' + encodeURIComponent(selectedSsid))
    .then(response => response.json())
    .then(data => {
      if (data && data.password) {
        passwordInput.value = data.password;
      } else {
        passwordInput.value = '';
      }
      passwordInput.focus();
    })
    .catch(error => {
      console.error('Bilinen şifre alınırken hata:', error);
      passwordInput.value = '';
      passwordInput.focus();
    });
}
function showConfirmation() {
    const ssidInput = document.getElementById('ssid');
    const passwordInput = document.getElementById('password');
    const confirmationBox = document.getElementById('confirmationBox');
    const wifiForm = document.getElementById('wifiForm');
    if (!ssidInput.value) {
        ssidInput.reportValidity();
        return;
    }
    document.getElementById('confirmSsid').textContent = ssidInput.value;
    document.getElementById('confirmPassword').textContent = passwordInput.value;
    wifiForm.style.display = 'none';
    confirmationBox.style.display = 'block';
}
function hideConfirmation() {
    document.getElementById('confirmationBox').style.display = 'none';
    document.getElementById('wifiForm').style.display = 'block';
}
function confirmAndSave() {
    const ssid = document.getElementById('ssid');
    const password = document.getElementById('password');
    fetch('/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'ssid=' + encodeURIComponent(ssid.value) + '&password=' + encodeURIComponent(password.value)
    })
    .then(response => {
        if (!response.ok) {
            return response.text().then(text => {
                try {
                    const errorJson = JSON.parse(text);
                    throw new Error(errorJson.message || 'Bilinmeyen bir hata oluştu.');
                } catch (e) {
                    throw new Error('Kaydetme başarısız oldu: ' + response.statusText + ' - ' + text);
                }
            });
        }
        return response.text();
    })
    .then(htmlMessage => {
        document.body.innerHTML = htmlMessage;
    })
    .catch(error => {
        console.error('Wi-Fi bilgileri kaydedilirken hata:', error);
        const confirmationBox = document.getElementById('confirmationBox');
        confirmationBox.innerHTML = '<h2>Hata!</h2><p>Bilgiler kaydedilemedi: ' + error.message + '</p><button onclick="hideConfirmation()" style="background-color: #6c757d; margin-top: 20px;">Geri Dön</button>';
        confirmationBox.style.backgroundColor = '#ffe0e0';
        confirmationBox.style.display = 'block';
        document.getElementById('wifiForm').style.display = 'none';
    });
}
window.onload = function() {
    document.getElementById('wifiSelect').innerHTML = '<option value="">Ağ taraması için "Kablosuz Ağları Tara" butonuna basınız.</option>';
    document.getElementById('ssid').value = window.__savedSsid__ || '';
    document.getElementById('password').value = window.__savedPassword__ || '';
};
</script>
</body>
</html>
)=====";

void handleRoot() {
    String htmlContent = (const char*)SETUP_HTML;
    htmlContent.replace("%AP_SSID_VALUE%", escapeHtmlAttribute(ap_ssid_to_use));
    htmlContent.replace("%FIRMWARE_VERSION%", FIRMWARE_VERSION);
    String scriptBlock = "<script>";
    scriptBlock += "window.__savedSsid__ = '" + escapeHtmlAttribute(saved_ssid) + "';";
    scriptBlock += "window.__savedPassword__ = '" + escapeHtmlAttribute(saved_password) + "';";
    scriptBlock += "window.__apSsidToUse__ = '" + escapeHtmlAttribute(ap_ssid_to_use) + "';";
    scriptBlock += "</script>";
    server.send(200, "text/html", htmlContent + scriptBlock);
}

void handleSave() {
    String new_ssid = server.arg("ssid");
    String new_password = server.arg("password");
    String statusMessage, extraInfo, statusColor;

    if (new_ssid.length() > 0) {
        saveWiFiCredentials(new_ssid, new_password);

        bool foundInKnownNetworks = false;
        if (knownNetworksDoc.is<JsonArray>()) {
            JsonArray networks = knownNetworksDoc.as<JsonArray>();
            for (JsonObject network : networks) {
                if (network["ssid"].as<String>() == new_ssid) {
                    network["password"] = new_password;
                    foundInKnownNetworks = true;
                    Serial.println("Bilinen ağlarda güncellendi.");
                    break;
                }
            }
            if (!foundInKnownNetworks) {
                JsonObject newEntry = networks.createNestedObject();
                newEntry["ssid"] = new_ssid;
                newEntry["password"] = new_password;
                Serial.println("Bilinen ağlara yeni giriş eklendi.");
            }
        } else {
            knownNetworksDoc.clear();
            JsonArray networks = knownNetworksDoc.to<JsonArray>();
            JsonObject newEntry = networks.createNestedObject();
            newEntry["ssid"] = new_ssid;
            newEntry["password"] = new_password;
            Serial.println("Bilinen ağlar listesi oluşturuldu ve yeni giriş eklendi.");
        }
        saveKnownNetworksToFile();

        String htmlResponsePage = R"=====(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wi-Fi Bilgileri Kaydedildi</title><style>body{font-family: Arial, sans-serif; background-color: #f4f7f6; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0;}
.message-box { background-color: #ffffff; padding: 30px; border-radius: 12px; box-shadow: 0 6px 20px rgba(0, 0, 0, 0.1); width: 100%; max-width: 400px; text-align: center; }
h2{color: %STATUS_COLOR%;} p{color: #555;}</style></head><body><div class="message-box"><h2>%STATUS_MESSAGE%</h2><p>%EXTRA_INFO%</p></div></body></html>
)=====";
        statusMessage = "Wi-Fi bilgileri kaydedildi!";
        extraInfo = "Cihaz, girdiğiniz ağa bağlanmak için yeniden başlatılıyor.<br>Lütfen mobil cihazınızın/bilgisayarınızın Wi-Fi bağlantısını kontrol edin.<br>Eğer bağlanamazsa, <span style='font-weight:bold;'>%AP_SSID%</span> ağına tekrar bağlanarak kurulumu tekrar yapmanız gerekebilir.";
        statusColor = "#333";
        htmlResponsePage.replace("%STATUS_MESSAGE%", statusMessage);
        htmlResponsePage.replace("%EXTRA_INFO%", extraInfo);
        htmlResponsePage.replace("%STATUS_COLOR%", statusColor);
        htmlResponsePage.replace("%AP_SSID%", ap_ssid_to_use);
        server.send(200, "text/html", htmlResponsePage);
        delay(1000);
        ESP.restart();
    } else {
        String htmlResponsePage = R"=====(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Hata!</title>
<style>body{font-family: Arial, sans-serif; background-color: #f4f7f6; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0;}
.message-box { background-color: #ffffff; padding: 30px; border-radius: 12px; box-shadow: 0 6px 20px rgba(0, 0, 0, 0.1); width: 100%; max-width: 400px; text-align: center; }
h2{color: #dc3545;} p{color: #555;}</style></head><body><div class="message-box"><h2>Hata: SSID boş olamaz!</h2>
<p><a href="/">Geri dönmek için tıklayın</a></p></div></body></html>
)=====";
        server.send(400, "text/html", htmlResponsePage);
        delay(100);
    }
}

void handleStartScan() {
    unsigned long currentTime = millis();
    if (currentTime - lastScanTime < SCAN_COOLDOWN_TIME) {
        server.send(202, "application/json", "{\"status\":\"scanning\", \"message\":\"Tarama bekleme süresinde, devam ediyor.\"}");
        return;
    }
    Serial.println("Yeni Wi-Fi tarama başlatılıyor...");
    int n = WiFi.scanNetworks(true);
    if (n == WIFI_SCAN_RUNNING) {
        server.send(202, "application/json", "{\"status\":\"scanning\", \"message\":\"Wi-Fi taraması başlatıldı (arka planda).\"}");
        lastScanTime = currentTime;
    } else if (n == WIFI_SCAN_FAILED) {
        server.send(500, "application/json", "{\"status\":\"failed\", \"message\":\"Wi-Fi taraması başlatılırken hata oluştu.\"}");
    } else {
        Serial.println("Tarama hemen tamamlandı (beklenmedik durum).");
        handleGetNetworks();
        lastScanTime = currentTime;
    }
}

void handleGetNetworks() {
    Serial.println("Ağ listesi isteği alındı...");
    DynamicJsonDocument doc(4096);
    JsonArray networks = doc.createNestedArray("networks");

    int n = WiFi.scanComplete(); // Tarama sonuçlarını kontrol et
    String scan_status = "completed";

    if (n == WIFI_SCAN_RUNNING) { // -1: Tarama hala devam ediyor
        Serial.println("Tarama hala devam ediyor. Durum: scanning.");
        scan_status = "scanning";
    } else if (n == WIFI_SCAN_FAILED) { // -2: Tarama başarısız oldu VEYA henüz başlamadı
        // Eğer tarama butonu yeni basıldıysa, bu durumu "başarısız" değil,
        // "hazırlanıyor" olarak kabul edip client'a beklemesini söyleyelim.
        unsigned long currentTime = millis();
        if (currentTime - lastScanTime < 5000) { // Tarama 5 saniyeden daha yeni başlatıldıysa
            Serial.println("Tarama hazırlanıyor/başlatılıyor... Durum: scanning.");
            scan_status = "scanning"; // Hata yerine "tarama devam ediyor" de.
        } else {
            Serial.println("Tarama başarısız oldu. Durum: failed.");
            scan_status = "failed"; // 5 saniye geçtiyse ve hala -2 ise gerçek bir hatadır.
        }
    } else if (n == 0) {
        Serial.println("Hiç ağ bulunamadı.");
        // Boş dizi dönecek
    } else { // n > 0 (tarama tamamlandı ve sonuçlar var)
        Serial.printf("%d ağ bulundu.\n", n);
        for (int i = 0; i < n; ++i) {
            JsonObject network = networks.createNestedObject();
            network["ssid"] = WiFi.SSID(i);
            network["rssi"] = WiFi.RSSI(i);
        }
    }

    doc["status"] = scan_status;
    serializeJson(doc, lastScanResultsJson);
    Serial.print("handleGetNetworks - Gonderilen JSON: ");
    Serial.println(lastScanResultsJson);
    server.send(200, "application/json", lastScanResultsJson);
}


void startWebServer() {
    Serial.println("Wi-Fi kurulumu için Erişim Noktası (AP) başlatılıyor...");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ap_ssid_to_use.c_str(), DEFAULT_AP_PASSWORD);
    Serial.printf("AP SSID: %s, AP IP: %s\n", ap_ssid_to_use.c_str(), WiFi.softAPIP().toString().c_str());

    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/startscan", HTTP_GET, handleStartScan);
    server.on("/getnetworks", HTTP_GET, handleGetNetworks);
    server.on("/getKnownPassword", HTTP_GET, handleGetKnownPassword);
    server.serveStatic("/faydamlogo.png", LittleFS, "/faydamlogo.png", "max-age=86400");
    server.serveStatic("/favicon.ico", LittleFS, "/favicon.ico", "max-age=86400");

    server.begin();
    Serial.println("Web sunucusu başladı.");

    while (true) {
        server.handleClient();
    }
}

// Analog değeri lineer olarak haritalama fonksiyonu (artık doğrudan pil voltajı için kullanılmıyor)
float map_linear(int x, float x1_in, float y1_out, float x2_in, float y2_out) {
    if (x2_in == x1_in) {
        return y1_out;
    }
    return y1_out + (x - x1_in) * (y2_out - y1_out) / (x2_in - x1_in);
}

// DEĞİŞTİRİLECEK FONKSİYON: connectToWiFi
bool connectToWiFi() {
    Serial.print("WiFi baglaniliyor...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(saved_ssid.c_str(), saved_password.c_str());

    wl_status_t status = (wl_status_t)WiFi.waitForConnectResult(30000); // 30 saniye bekle

    if (status == WL_CONNECTED) {
        Serial.println("WiFi baglantisi basarili.");
        // YENİ EKLENEN SATIR: IP adresini yazdır
        Serial.printf("IP Adresi: %s\n", WiFi.localIP().toString().c_str()); 
        Serial.printf("Cihaz SENSOR_ID: %s\n", deviceID.c_str());
        String desiredHostname = "Faydam-" + deviceID;
        WiFi.setHostname(desiredHostname.c_str());
        Serial.printf("WiFi Hostname ayarlandi: %s\n", desiredHostname.c_str());
        return true;
    } else {
        Serial.printf("\nWiFi baglantisi kurulamadi. Durum kodu: %d\n", status);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        Serial.println("WiFi baglantisi kurulamadigi icin kapatildi.");
        return false;
    }
}

// Zamanı belirli bir string formatına dönüştürme (yyyyMMddHHmmss)
String formatTime(struct tm timeinfo) {
    char timeString[15];
    sprintf(timeString, "%04d%02d%02d%02d%02d%02d",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return String(timeString);
}

// Sunucu yanıtından eşik değerleri ve ZAMANI ayrıştırma ve sistem saatini ayarlama
void parseThresholdsAndSyncTime(const String& jsonResponse, String& currentFormattedTime) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonResponse);

    if (error) {
        Serial.print(F("deserializeJson() hatasi: "));
        Serial.println(error.f_str());
        currentFormattedTime = "00000000000000";
        return;
    }
    Serial.printf("Yanit Uzunlugu: %d\n", jsonResponse.length());
    Serial.printf("JSON Oge Sayis: %d\n", doc.size());
    bool thresholdsUpdated = false;

    if (doc.containsKey("TLow")) {
        String tLowStr = doc["TLow"].as<String>();
        float temp_tLow = tLowStr.substring(0, tLowStr.indexOf(',')).toFloat();
        if (temp_tLow != -99999999.00f) {
            TLow_threshold = temp_tLow;
            Serial.printf("Alinan TLow Esik Degeri: %.2f (Gelen String: %s)\n", TLow_threshold, tLowStr.c_str());
            thresholdsUpdated = true;
        } else {
            Serial.println("TLow esik degeri API'den default geldi, guncellenmedi.");
        }
    } else {
        Serial.println("API yanitinda 'TLow' bulunamadi, mevcut TLow esigi korunuyor.");
    }
    if (doc.containsKey("THigh")) {
        String tHighStr = doc["THigh"].as<String>();
        float temp_tHigh = tHighStr.substring(0, tHighStr.indexOf(',')).toFloat();
        if (temp_tHigh != -99999999.00f) {
            THigh_threshold = temp_tHigh;
            Serial.printf("Alinan THigh Esik Degeri: %.2f (Gelen String: %s)\n", THigh_threshold, tHighStr.c_str());
            thresholdsUpdated = true;
        } else {
            Serial.println("THigh esik degeri API'den default geldi, guncellenmedi.");
        }
    } else {
        Serial.println("API yanitinda 'THigh' bulunamadi, mevcut THigh esigi korunuyor.");
    }
    if (thresholdsUpdated) {
        Serial.println("Esik degerleri basariyla guncellendi (non-default degerler alindi).");
    } else {
        Serial.println("API'den yeni esik degeri alinamadi veya default geldi, mevcut esikler korunuyor.");
    }
    if (doc.containsKey("DateTime")) {
        String dateTimeStr = doc["DateTime"].as<String>();
        if (dateTimeStr.length() == 0) {
            Serial.println("Zaman API yanitinda 'DateTime' alani bulunamadi veya bos.");
            currentFormattedTime = "00000000000000";
            return;
        }
        Serial.printf("Alinan zaman stringi: %s\n", dateTimeStr.c_str());

        struct tm timeinfo;
        int year, month, day, hour, minute, second;
        int parsed_elements = sscanf(dateTimeStr.c_str(), "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);

        if (parsed_elements != 6) {
            Serial.printf("Zaman verisi format uyumsuzluk! %d/%d eleman okundu.\n", parsed_elements, 6);
            currentFormattedTime = "00000000000000";
            return;
        }
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = minute;
        timeinfo.tm_sec = second;
        timeinfo.tm_isdst = -1;
        time_t epochTime = mktime(&timeinfo);

        if (epochTime == -1) {
            Serial.println("Zaman dönüştürme hatası (mktime).");
            currentFormattedTime = "00000000000000";
            return;
        }
        struct timeval tv;
        tv.tv_sec = epochTime;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        Serial.println("Sistem zamani sunucudan gelen veriyle ayarlandi.");
        struct tm current_time_info;
        getLocalTime(&current_time_info);
        currentFormattedTime = formatTime(current_time_info);
        Serial.printf("Guncel RTC Zamani (Sunucudan): %s\n", currentFormattedTime.c_str());
    } else {
        Serial.println("API yanitinda 'DateTime' alani bulunamadi.");
        currentFormattedTime = "00000000000000";
    }
}

// Sensör verisini datalog.csv'ye kaydet
void saveSensorDataToFile(float temp_c, int analog_val, time_t timestamp, int rssi) {
    File dataFile = LittleFS.open(DATALOG_FILE, FILE_APPEND);
    if (!dataFile) {
        Serial.println("datalog.csv dosya acilamadi/olusturulamadi!");
        return;
    }
    String dataLine = String((int)(temp_c * 100)) + "," + String(analog_val) + "," + String(timestamp) + "," + String(rssi);
    dataFile.println(dataLine);
    dataFile.close();
    Serial.printf("Veri datalog.csv'ye kaydedildi: %s\n", dataLine.c_str());
}

// datalog.csv'den verileri göndermeye çalış
bool sendQueuedDataFromFile() {
    if (!LittleFS.exists(DATALOG_FILE)) {
        Serial.println("datalog.csv dosyasi yok, gonderilecek bekleyen veri bulunamadi.");
        return true;
    }
    File dataFile = LittleFS.open(DATALOG_FILE, FILE_READ);
    if (!dataFile) {
        Serial.println("datalog.csv dosyasi okuma modunda acilamadi!");
        return false;
    }
    File tempFile = LittleFS.open("/temp_datalog.csv", FILE_WRITE);
    if (!tempFile) {
        Serial.println("temp_datalog.csv dosyasi olusturulamadi!");
        dataFile.close();
        return false;
    }
    Serial.println("datalog.csv'den bekleyen veriler gonderiliyor...");
    HTTPClient http;
    bool allQueuedDataSent = true;
    //bool currentConnectionStatus = false;
    
    while (dataFile.available()) {
        String line = dataFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        int thirdComma = line.indexOf(',', secondComma + 1);

        if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
            Serial.printf("Gecersiz CSV satiri atlandi: %s\n", line.c_str());
            tempFile.println(line);
            allQueuedDataSent = false;
            continue;
        }
        int temp_val_int = line.substring(0, firstComma).toInt();
        int analog_val = line.substring(firstComma + 1, secondComma).toInt();
        time_t timestamp = (time_t)line.substring(secondComma + 1, thirdComma).toInt();
        int rssi_from_file = line.substring(thirdComma + 1).toInt();

        struct tm queued_timeinfo;
        if (!gmtime_r(&timestamp, &queued_timeinfo)) {
            Serial.println("Kuyruk zaman damgasi gecersiz, bu veri atlandi ve dosyaya geri yazildi.");
            tempFile.println(line);
            allQueuedDataSent = false;
            continue;
        }
        String formattedQueuedTime = formatTime(queued_timeinfo);
        String dataToSend = String(temp_val_int) + "," + String(rssi_from_file) + "," + String(analog_val);
        String request_url = "https://" + String(API_HOST) + "/ext/wdata?";
        request_url += "id=" + deviceID + "01";
        request_url += "&vtid=2,97,1";
        request_url += "&dt=" + formattedQueuedTime;
        request_url += "&vl=" + dataToSend;
        request_url += "&u=" + String(API_USER);
        request_url += "&p=" + String(API_PASS);
        request_url += "&API=" + String(API_KEY);

        Serial.printf("Kuyruktan gonderilen URL: %s\n", request_url.c_str());
        http.begin(request_url);
        int httpCode = http.GET();

        if (httpCode > 0 && (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)) {
            String payload = http.getString();
            Serial.printf("[HTTP] Kuyruk GET... code: %d, Yanit: %s\n", httpCode, payload.c_str());
        } else {
            Serial.printf("[HTTP] Kuyruk GET... basarisiz: %d, hata: %s. Veri dosyada kaldi.\n", httpCode, http.errorToString(httpCode).c_str());
            tempFile.println(line);
            //currentConnectionStatus = false;
            allQueuedDataSent = false;
            String remainingLine;
            while(dataFile.available()) {
                remainingLine = dataFile.readStringUntil('\n');
                remainingLine.trim();
                tempFile.println(remainingLine);
            }
            break;
        }
        http.end();
    }
    dataFile.close();
    tempFile.close();

    //WiFi.disconnect(true);
    //WiFi.mode(WIFI_OFF);
    //Serial.println("Kuyruk veri gonderimi tamamlandi/kesildi, WiFi kapatildi.");

    LittleFS.remove(DATALOG_FILE);
    if (LittleFS.rename("/temp_datalog.csv", DATALOG_FILE)) {
        Serial.println("datalog.csv guncellendi.");
    } else {
        Serial.println("datalog.csv guncellenirken hata olustu. temp_datalog.csv dosyasi kalmis olabilir.");
    }
    if (allQueuedDataSent && LittleFS.exists(DATALOG_FILE)) {
        Serial.println("Tum bekleyen veriler basariyla gonderildi ve datalog.csv siliniyor.");
        LittleFS.remove(DATALOG_FILE);
    } else if (allQueuedDataSent) {
        Serial.println("Tum bekleyen veriler basariyla gonderildi. datalog.csv zaten yok/bos.");
    } else {
        Serial.println("Bekleyen verilerin bir kismi gonderilemedi, dosyada tutuluyor.");
    }
    return allQueuedDataSent;
}

void sendGateData() {
    HTTPClient http;
    String formattedTime = "";
    bool gateRequestSuccess = false;

    for (int retry = 0; retry < MAX_HTTP_RETRIES; retry++) {
        if (retry > 0) {
            Serial.printf("GATE istegi (yeniden deneme %d/%d) icin bekleniyor (%d ms)...\n", retry + 1, MAX_HTTP_RETRIES, HTTP_RETRY_DELAY_MS);
            delay(HTTP_RETRY_DELAY_MS);
        }
        if (!connectToWiFi()) {
            Serial.println("GATE istegi icin WiFi baglantisi kurulamadi. Tekrar denenecek.");
            if (retry == MAX_HTTP_RETRIES - 1) {
                Serial.println("Tum denemeler basarisiz oldu. GATE istegi yapilamadi.");
                break;
            }
            continue;
        }
        String gate_request_url = "https://" + String(API_HOST) + "/ext/getdate";
        Serial.printf("Gonderilecek GATE URL (Deneme %d/%d): %s\n", retry + 1, MAX_HTTP_RETRIES, gate_request_url.c_str());
        http.begin(gate_request_url);
        int httpCode = http.GET();

        if (httpCode > 0) {
            Serial.printf("[HTTP] GATE GET... code: %d\n", httpCode);
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
                String payload = http.getString();
                Serial.printf("Sunucu Yaniti: %s\n", payload.c_str());
                parseThresholdsAndSyncTime(payload, formattedTime);
                gateRequestSuccess = true;
                break;
            } else {
                Serial.printf("[HTTP] GATE GET... hatali kod: %d, hata: %s. Yeniden denenecek.\n", httpCode, http.errorToString(httpCode).c_str());
            }
        } else {
            Serial.printf("[HTTP] GATE GET... basarisiz, hata: %s. Yeniden denenecek.\n", http.errorToString(httpCode).c_str());
        }
        http.end();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("GATE istegi tamamlandi/denemeleri bitti, WiFi kapatildi.");
}

// Versiyon numaralarını karşılaştıran yardımcı fonksiyon (örn: "1.10" > "1.9")
bool isNewVersionAvailable(const char* currentVer, const char* latestVer) {
    String current = String(currentVer);
    String latest = String(latestVer);

    // Basit bir versiyon karşılaştırması: x.y.z formatı
    // Daha karmaşık versiyonlar için (örn. 1.0.0-beta), daha detaylı ayrıştırma gerekebilir.
    int currentMajor = 0, currentMinor = 0, currentPatch = 0;
    int latestMajor = 0, latestMinor = 0, latestPatch = 0;

    sscanf(current.c_str(), "%d.%d.%d", &currentMajor, &currentMinor, &currentPatch);
    sscanf(latest.c_str(), "%d.%d.%d", &latestMajor, &latestMinor, &latestPatch);

    if (latestMajor > currentMajor) return true;
    if (latestMajor < currentMajor) return false;

    if (latestMinor > currentMinor) return true;
    if (latestMinor < currentMinor) return false;

    if (latestPatch > currentPatch) return true;
    if (latestPatch < currentPatch) return false;

    return false; // Versiyonlar aynı veya daha eski
}

// OTA güncelleme kontrolünü ve başlatmayı yapan fonksiyon
void checkAndDoUpdate() {
    // Sadece Wi-Fi bağlıysa OTA kontrolü yap
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("OTA kontrolü için WiFi bağlı değil.");
        return;
    }

    WiFiClientSecure client;
    client.setCACert(GITHUB_ROOT_CA); // Güvenli bağlantı için CA sertifikası

    HTTPClient http;
    String versionUrl = "https://" + String(GITHUB_HOST) + "/" + String(GITHUB_USER) + "/" + String(GITHUB_REPO) + String(GITHUB_VERSION_FILE_PATH);

    Serial.printf("OTA: GitHub versiyon kontrol ediliyor: %s\n", versionUrl.c_str());

    http.begin(client, versionUrl);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        String latestFirmwareVersion = "";
        String firmwareDownloadUrl = "";

        StaticJsonDocument<200> doc; 
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
            Serial.print(F("OTA: JSON ayrıştırma hatası: "));
            Serial.println(error.f_str());
            http.end();
            return;
        }

        latestFirmwareVersion = doc["version"].as<String>();
        firmwareDownloadUrl = doc["url"].as<String>();

        Serial.printf("OTA: Mevcut versiyon: %s, GitHub versiyon: %s\n", FIRMWARE_VERSION, latestFirmwareVersion.c_str());

        if (isNewVersionAvailable(FIRMWARE_VERSION, latestFirmwareVersion.c_str())) {
            Serial.println("OTA: Yeni firmware versiyonu bulundu! Güncelleme başlatılıyor...");
            Serial.printf("OTA: İndirme URL: %s\n", firmwareDownloadUrl.c_str());
            
            http.end(); // Önceki HTTPClient bağlantısını kapat

            http.begin(client, firmwareDownloadUrl);
            int firmwareHttpCode = http.GET();

            if (firmwareHttpCode == HTTP_CODE_OK) {
                int contentLength = http.getSize();
                if (contentLength > 0) {
                    bool canBegin = Update.begin(contentLength);
                    if (canBegin) {
                        WiFiClient* stream = http.getStreamPtr();
                        size_t written = Update.writeStream(*stream);
                        if (written == contentLength) {
                            Serial.println("OTA: Yazma başarılı.");
                        } else {
                            Serial.printf("OTA: Yazma hatası! Yazılan bayt: %zu, Beklenen: %d\n", written, contentLength);
                        }
                        if (Update.end()) { // Bu kontrol hem yazmayı bitirir hem de CRC doğrulaması yapar
                            Serial.println("OTA: Güncelleme başarılı. Cihaz yeniden başlatılıyor...");
                            ESP.restart(); // Cihazı yeniden başlat
                        } else {
                            Serial.printf("OTA: Bitirme hatası! Hata kodu: %d - %s\n", Update.getError(), Update.errorString());
                        }
                    } else {
                        Serial.println("OTA: Güncelleme başlatılamadı (Update.begin hatası).");
                    }
                } else {
                    Serial.println("OTA: Firmware boyutu alınamadı veya sıfır.");
                }
            } else {
                Serial.printf("OTA: Firmware indirme hatası! HTTP Kodu: %d\n", firmwareHttpCode);
            }
        } else {
            Serial.println("OTA: Mevcut firmware en güncel versiyon.");
        }
    } else {
        Serial.printf("OTA: Versiyon dosyası indirme hatası! HTTP Kodu: %d\n", httpCode);
    }
    http.end();
}

void sendMainData() {
    String formattedTime;
    struct tm timeinfo;
    time_t current_epoch_time;
    int current_rssi = -1;

    if (!getLocalTime(&timeinfo)) {
        Serial.println("Zaman bilgisi alinamadi (getLocalTime). Guncel veri datalog.csv'ye eklenecek.");
        current_epoch_time = 0;
    } else {
        current_epoch_time = mktime(&timeinfo);
        formattedTime = formatTime(timeinfo);
    }
    sendQueuedDataFromFile();
    Serial.println("Guncel ana veriyi gondermeye calisiliyor...");
    HTTPClient http;
    bool mainRequestSuccessful = false;

    for (int retry = 0; retry < MAX_HTTP_RETRIES; retry++) {
        if (retry > 0) {
            Serial.printf("Guncel ana veri gonderme (yeniden deneme %d/%d) icin bekleniyor (%d ms)....\n", retry + 1, MAX_HTTP_RETRIES, HTTP_RETRY_DELAY_MS);
            delay(HTTP_RETRY_DELAY_MS);
        }

        current_rssi = WiFi.RSSI();
        String dataLine = String((int)(temperature_c * 100)) + "," + String(current_rssi) + "," + String(current_value_int);
        Serial.printf("Olusturulan ana veri satiri: %s\n", dataLine.c_str());
        String main_request_url = "https://" + String(API_HOST) + "/ext/wdata?";
        main_request_url += "id=" + deviceID + "01";
        main_request_url += "&vtid=2,97,1";
        main_request_url += "&dt=" + formattedTime;
        main_request_url += "&vl=" + dataLine;
        main_request_url += "&u=" + String(API_USER);
        main_request_url += "&p=" + String(API_PASS);
        main_request_url += "&API=" + String(API_KEY);

        Serial.printf("Gonderilecek Guncel Ana URL (Deneme %d/%d): %s\n", retry + 1, MAX_HTTP_RETRIES, main_request_url.c_str());
        http.begin(main_request_url);
        int httpCode = http.GET();

        if (httpCode > 0) {
            Serial.printf("[HTTP] Guncel Ana GET... code: %d\n", httpCode);
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
                String payload = http.getString();
                Serial.printf("Sunucu Yaniti: %s\n", payload.c_str());
                parseThresholdsAndSyncTime(payload, formattedTime);
                if (alarmTempIndex > 0) {
                    Serial.println("Alarm verileri ana istek ile gonderildi, RTC'deki alarm dizisi temizleniyor.");
                    for(int i = 0; i < MAX_ALARM_TEMP_READINGS; i++) {
                        rtcAlarmTemperatureReadings[i] = -9999;
                    }
                    alarmTempIndex = 0;
                }
                mainRequestSuccessful = true;
                break;
            } else {
                Serial.printf("[HTTP] Guncel Ana GET... hatali kod: %d, hata: %s. Yeniden denenecek.\n", httpCode, http.errorToString(httpCode).c_str());
                if (retry == MAX_HTTP_RETRIES - 1) {
                    Serial.println("Tum denemeler basarisiz oldu. Guncel veri datalog.csv'ye ekleniyor.");
                    saveSensorDataToFile(temperature_c, current_value_int, current_epoch_time, current_rssi);
                }
            }
        } else {
            Serial.printf("[HTTP] Guncel Ana GET... basarisiz, hata: %s. Yeniden denenecek.\n", http.errorToString(httpCode).c_str());
            if (retry == MAX_HTTP_RETRIES - 1) {
                Serial.println("Tum denemeler basarisiz oldu. Guncel veri datalog.csv'ye ekleniyor.");
                saveSensorDataToFile(temperature_c, current_value_int, current_epoch_time, current_rssi);
            }
        }
        http.end();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF); // WiFi bağlantısını kapat
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF); // WiFi bağlantısını kapat
    if (mainRequestSuccessful) {
        Serial.println("Guncel Ana veri sunucuya basariyla gonderildi, WiFi kapatildi.");
        otaCheckCounter++; // Sadece ana veri başarılı gönderilirse OTA sayacını artır
        if (otaCheckCounter >= OTA_CHECK_INTERVAL_MAIN_SENDS) {
            shouldCheckOTA = true; // OTA kontrolü yapılması gerektiğini işaretle
            otaCheckCounter = 0; // Sayacı sıfırla
        }
        // Başarılı gönderimde birikmiş uyku süresini sıfırla
        accumulatedSleepSeconds = 0; 
    } else {
        Serial.println("Guncel Ana veri sunucuya gonderilemedi, datalog.csv'ye kaydedildi ve WiFi kapatildi.");
        // Başarısız gönderimde de birikmiş uyku süresini sıfırlayalım ki sonraki periyot bozulmasın
        accumulatedSleepSeconds = 0; 
        // OTA sayacını sıfırlama (başarılı gönderim olmadığı için artırmadık)
        otaCheckCounter = 0; // Başarısız bağlantı varsa OTA kontrolünü ertelemek için sayacı sıfırlayalım
    }
}

// --- Setup Fonksiyonu ---
void setup() {
    unsigned long setupStartTimeMillis = millis(); // setup'ın başlangıç zamanı

    Serial.begin(115200);
    delay(100);

    Serial.printf("Cihaz Firmware Versiyonu: %s\n", FIRMWARE_VERSION);

    if (!LittleFS.begin()) {
        Serial.println("LittleFS başlatılamadı, formatlanıyor...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("LittleFS formatlandıktan sonra da başlatılamadı! Hata!");
            while(true) { delay(100); }
        }
    }
    Serial.println("LittleFS başlatıldı.");

    Serial.println("\n--- LittleFS Dosya Listesi ---");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    if (!file) {
        Serial.println(" - Kök dizinde dosya yok.");
    } else {
        while(file){
            Serial.print(" - ");
            Serial.print(file.name());
            Serial.print(" (");
            Serial.print(file.size());
            Serial.print(" bayt)");
            file = root.openNextFile();
        }
    }
    Serial.println("--- Dosya Listesi Sonu ---\n");

    loadKnownNetworksFromFile();

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.printf("\nCihaz %d. kez baslatildi.\n", ++bootCount);
    const int MAX_BOOT_COUNT = 100000;
    if (bootCount >= MAX_BOOT_COUNT) {
        bootCount = 0;
        Serial.printf("Boot sayaci %d'ye ulasti, sifirlandi.\n", MAX_BOOT_COUNT);
    }

    // WiFi modu ayarı ve MAC ID oluşturma
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    deviceID = mac;
    Serial.printf("Cihazın okunan MAC Adresi (kolonsuz): %s\n", deviceID.c_str());

    // AP SSID'sini yükle veya oluştur
    ap_ssid_to_use = loadApSsidFromFile();
    if (ap_ssid_to_use.length() == 0 || ap_ssid_to_use.equals("FAYDAM_000000000000")) {
        String generatedApSsid = "FAYDAM_" + deviceID;
        Serial.printf("AP SSID bulunamadi veya gecersiz, MAC adresinden olusturuldu ve kaydediliyor: %s\n", generatedApSsid.c_str());
        saveApSsidToFile(generatedApSsid);
        ap_ssid_to_use = generatedApSsid;
    } else {
        Serial.printf("Kayitli AP SSID kullaniliyor: %s\n", ap_ssid_to_use.c_str());
    }

    if (ap_ssid_to_use.length() > 0 && ap_ssid_to_use.startsWith("FAYDAM_") && ap_ssid_to_use.substring(7).length() == 12) {
        Serial.println("AP SSID dogrulandi: Gecerli formatta ve kaydedilmis.");
    } else {
        Serial.println("UYARI: AP SSID beklenen formatta degil veya gecersiz! LittleFS'ten okuma hatasi olabilir.");
    }

    bool should_enter_setup_mode = false;
    pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
    delay(100);

    bool button_pressed_for_long = false;
    unsigned long button_press_start_time = millis();
    Serial.println("Manuel kurulum butonu kontrol ediliyor (5 saniye basili tutun)...");
    while (digitalRead(CONFIG_BUTTON_PIN) == LOW && (millis() - button_press_start_time < 5000)) {
        delay(50);
        if (millis() - button_press_start_time >= 3000) {
            Serial.println("Manuel kurulum moduna gecis icin bekleniyor...");
        }
    }

    if (millis() - button_press_start_time >= 5000) {
        button_pressed_for_long = true;
        Serial.println("\nManuel kurulum moduna gecis tetiklendi! Tum Wi-Fi bilgileri ve datalog temizleniyor.");
        clearWiFiCredentials();
        if (LittleFS.exists(DATALOG_FILE)) { if (LittleFS.remove(DATALOG_FILE)) { Serial.println("datalog.csv temizlendi."); } else { Serial.println("datalog.csv temizlenirken hata olustu."); } }
        if (LittleFS.exists(AP_SSID_FILE)) { if (LittleFS.remove(AP_SSID_FILE)) { Serial.println("ap_ssid.json temizlendi."); } else { Serial.println("ap_ssid.json temizlenirken hata olustu."); } }
        if (LittleFS.exists(KNOWN_NETWORKS_FILE)) { if (LittleFS.remove(KNOWN_NETWORKS_FILE)) { Serial.println("wifi_known_networks.json temizlendi."); } else { Serial.println("wifi_known_networks.json temizlenirken hata olustu."); } }
        should_enter_setup_mode = true;
    } else {
        Serial.println("\nManuel kurulum modu tetiklenmedi.");
    }

    loadWiFiCredentials();

    // --- YENİ MANTIK BURADA BAŞLIYOR (setup akışı) ---
    // Eğer manuel kurulum butonu basılı değilse:
    if (!button_pressed_for_long) {
        if (saved_ssid.length() == 0) { // Kayıtlı Wi-Fi bilgisi yoksa (ilk kurulum)
            Serial.println("Kaydedilen Wi-Fi bilgisi yok (ilk baslatma), kurulum moduna geciliyor.");
            startWebServer(); // Blocking çağrı, buradan dönülmez. setup() burada biter.
        }
        // KAYITLI BİLGİ VARSA, ARTIK BURADA BAĞLANMIYORUZ.
        // SADECE AKIŞIN DEVAM ETMESİNE İZİN VERİYORUZ.
        // Bağlantı denemesi sadece veri gönderileceği zaman yapılacak.
    } else { // Eğer button_pressed_for_long true ise (manuel kurulum)
        Serial.println("Manuel kurulum butonu ile kurulum moduna geciliyor.");
        startWebServer(); // Blocking çağrı, buradan dönülmez. setup() burada biter.
    }
    // --- YENİ MANTIK BURADA BİTİYOR (setup akışı) ---

    // RTC sayaçlarını ve alarm durumunu ilk açılışta veya güç kesintisinden sonra sıfırla
    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED && !should_enter_setup_mode) {
        Serial.println("Normal boot veya guc kesintisi sonrasi. Sayaclar ve alarm durumu sifirlaniyor.");
        accumulatedSleepSeconds = 0; // Birikmiş uyku süresini sıfırla
        inAlarmState = false;
        alarmTempIndex = 0;
        otaCheckCounter = 0; // OTA sayacını da sıfırla
        shouldCheckOTA = true; // İlk açılışta OTA kontrolü yap
        for(int i = 0; i < MAX_ALARM_TEMP_READINGS; i++) {
            rtcAlarmTemperatureReadings[i] = -9999;
        }
        shouldSendMainData = true; // İlk açılışta veri gönder

        // Eşik değerlerini ve zamanı almak için GATE isteği
        Serial.println("İlk başlatma, /ext/getdate isteği gonderiliyor...");
        sendGateData(); // API'den zaman ve eşik değerlerini al (retry'li)

    } else if (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED) {
        // Deep-sleep'ten uyandıysa birikmiş uyku süresini artır
        accumulatedSleepSeconds += MINUTE_SLEEP_SECONDS;
        Serial.printf("Deep-sleep'ten Timer ile uyandi. Birikmis Uyku Suresi: %lu sn. Alarm Durumu: %s, OTA Sayaci: %d\n", accumulatedSleepSeconds, inAlarmState ? "EVET" : "HAYIR", otaCheckCounter);
    }
    
    // OTA Kontrolü Zamanı Geldiyse
    if (shouldCheckOTA) {
        Serial.println("OTA kontrol zamanı geldi. Wi-Fi'ye bağlanılıp OTA kontrolü yapılacak...");
        if (connectToWiFi()) { // OTA için Wi-Fi bağlantısı kur
            checkAndDoUpdate(); // OTA güncellemesini kontrol et ve yap
            // OTA tamamlandıktan sonra Wi-Fi'yi kapat
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            Serial.println("OTA kontrolü tamamlandı, WiFi kapatıldı.");
        } else {
            Serial.println("OTA kontrolü için WiFi bağlantısı kurulamadı.");
        }
        shouldCheckOTA = false; // Kontrol yapıldı, bayrağı sıfırla
    }

    sensors.begin();
    Serial.println("DS18B20 sensörleri baslatildi.");
    sensors.requestTemperatures();
    delay(750); // Sensör okuması için gerekli bekleme
    temperature_c = sensors.getTempCByIndex(0);

    if (temperature_c == DEVICE_DISCONNECTED_C) {
        Serial.println("Sicaklik okumasi basarisiz.");
    } else {
        Serial.printf("Sicaklik okumasi isteniyor...Basarili. Sicaklik: %.2f C\n", temperature_c);
    }

    // --- Analog Sensör İşlemleri (Pil Voltajı) ---
    Serial.println("--- Analog Sensor Islemleri (Pil Voltajı) ---");
    // Voltaj okumasını daha kararlı hale getirmek için ortalama alma
    const int numReadings = 20; // Ortalama alınacak okuma sayısı (daha yüksek sayı daha kararlı, daha yavaş)
    long sumRawAnalogValue = 0; // Toplam için 'long' kullan, olası taşmayı önlemek için
    for (int i = 0; i < numReadings; i++) {
        sumRawAnalogValue += analogRead(ANALOG_INPUT_PIN);
        delay(2); // Okumalar arasında kısa bir gecikme bırakın (isteğe bağlı, gürültüye bağlı olarak ayarlanabilir)
    }
    int raw_analog_value_avg = sumRawAnalogValue / numReadings; // Ortalama değeri al
    
    // Doğrudan çarpan kullanarak pil voltajını hesapla (Volt cinsinden)
    float battery_voltage_float = raw_analog_value_avg * BATTERY_VOLTAGE_TO_MV_FACTOR;

    // Integer (tam sayı) değerine dönüştürme (örn. 3.93V -> 393)
    current_value_int = static_cast<int>(battery_voltage_float * 100); 
    Serial.printf("current_value_int (atama sonrası): %d\n", current_value_int); // Eklenen print satırı

    Serial.printf("Ham analog okuma (Pin %d): %d\n", ANALOG_INPUT_PIN, raw_analog_value_avg);
    Serial.printf("Hesaplanan Pil Voltajı (Float): %.2f V\n", battery_voltage_float); 
    Serial.printf("Hesaplanan Pil Voltajı (Int - mV*100): %d\n", current_value_int);
    Serial.printf("RTC'den yuklenen Esik Degerleri: TLow=%.2f, THigh=%.2f\n", TLow_threshold, THigh_threshold);

    bool isTemperatureAlarmTriggered = false;
    if (temperature_c != DEVICE_DISCONNECTED_C) {
        Serial.printf("Alarm Kontrolu (Guncel Esiklerle): Sicaklik=%.2f, TLow=%.2f, THigh=%.2f\n", temperature_c, TLow_threshold, THigh_threshold);
        if (temperature_c < TLow_threshold || temperature_c > THigh_threshold) {
            isTemperatureAlarmTriggered = true;
            Serial.println("!!! SICAKLIK ALARM DURUMU ALGILANDI !!!");
        } else {
            Serial.println("Sicaklik normal aralikta.");
        }
    } else {
        Serial.println("Sicaklik okunamadi, alarm durumu kontrol edilemiyor.");
    }

    if (temperature_c != DEVICE_DISCONNECTED_C) {
        if (alarmTempIndex < MAX_ALARM_TEMP_READINGS) {
            rtcAlarmTemperatureReadings[alarmTempIndex++] = (int)(temperature_c * 100);
            Serial.printf("Sicaklik (%.2f C) alarm dizisine kaydedildi. Indeks: %d\n", temperature_c, alarmTempIndex);
        } else {
            for(int i = 0; i < MAX_ALARM_TEMP_READINGS - 1; i++) {
                rtcAlarmTemperatureReadings[i] = rtcAlarmTemperatureReadings[i+1];
            }
            rtcAlarmTemperatureReadings[MAX_ALARM_TEMP_READINGS - 1] = (int)(temperature_c * 100);
            Serial.println("Alarm sicaklik dizisi dolu, en eski deger atildi ve yeni deger eklendi.");
        }
    }

    // Veri gönderme zamanının gelip gelmediğini kontrol et
    long currentTargetTotalPeriodSeconds;
    if (inAlarmState) {
        currentTargetTotalPeriodSeconds = ALARM_TOTAL_PERIOD_SECONDS;
    } else {
        currentTargetTotalPeriodSeconds = NORMAL_TOTAL_PERIOD_SECONDS;
    }
    
    // Yalnızca uyku süresi yeterli olduğunda shouldSendMainData'yı true yap
    // shouldSendMainData alarm başlangıcı/bitişi ile de true olabilir, o önceliklidir.
    if (!shouldSendMainData) { // Eğer zaten alarm tetiklememişse
        if (accumulatedSleepSeconds >= (currentTargetTotalPeriodSeconds - ESTIMATED_ACTIVE_TIME_SECONDS)) {
            shouldSendMainData = true;
            Serial.printf("Hedef periyot doldu (%lu sn uyku): Veri gonderiliyor!\n", accumulatedSleepSeconds);
        }
    }

    // Alarm durumu değişiklikleri (Eğer alarm durumu değiştiyse, hemen veri gönder)
    if (isTemperatureAlarmTriggered && !inAlarmState) {
        Serial.println("Yeni Alarm Durumu Baslatiliyor...Hemen veri gonderiliyor!");
        shouldSendMainData = true;
        inAlarmState = true;
        // Alarm başlangıcında birikmiş uyku süresini sıfırla, yeni alarm periyodu başlıyor.
        accumulatedSleepSeconds = 0; 
    }
    else if (!isTemperatureAlarmTriggered && inAlarmState) {
        Serial.println("Alarm durumu sona erdi. Hemen veri gonderiliyor!");
        shouldSendMainData = true;
        inAlarmState = false;
        // Alarm bitişinde birikmiş uyku süresini sıfırla, yeni normal periyot başlıyor.
        accumulatedSleepSeconds = 0; 
    }
    else if (inAlarmState) {
        Serial.printf("Alarm devam ediyor. Birikmis Uyku Suresi: %lu sn. Hedef Periyot: %d sn.\n", accumulatedSleepSeconds, ALARM_TOTAL_PERIOD_SECONDS);
        // Zaten yukarıdaki kontrol ile shouldSendMainData ayarlanacak.
    }
    else {
        Serial.printf("Normal periyot devam ediyor. Birikmis Uyku Suresi: %lu sn. Hedef Periyot: %d sn.\n", accumulatedSleepSeconds, NORMAL_TOTAL_PERIOD_SECONDS);
        // Zaten yukarıdaki kontrol ile shouldSendMainData ayarlanacak.
    }

    // --- Gerekli Veri Gönderimini Yap (Sadece shouldSendMainData true ise) ---
    if (shouldSendMainData) {
        Serial.println("Veri gönderme zamanı geldi. Şimdi Wi-Fi'ye bağlanılacak...");

        // BAĞLANTIYI SADECE ŞİMDİ DENE
        if (connectToWiFi()) {
            // Bağlantı başarılıysa ana veriyi gönder.
            Serial.println("WiFi bağlantısı başarılı. sendMainData() çağrılıyor...");
            sendMainData(); // Ana veriyi gönder (retry'li).
        } else {
            // Bağlantı başarısız olursa, veriyi dosyaya kaydet
            Serial.println("WiFi bağlantısı kurulamadı. Veri datalog.csv'ye kaydediliyor.");
            time_t now_epoch = 0;
            struct tm current_time_info_for_queue;
            if (getLocalTime(&current_time_info_for_queue)) {
                now_epoch = mktime(&current_time_info_for_queue);
            } else {
                Serial.println("Zaman bilgisi alınamadığı için datalog.csv'ye eklenen verinin zaman damgası 0 olarak ayarlandı.");
            }
            saveSensorDataToFile(temperature_c, current_value_int, now_epoch, -1); // RSSI bilinmediği için -1
            
            // Veri gönderimi başarısız olsa bile, periyodu tamamladığımızı varsayarak sayaçları sıfırla
            accumulatedSleepSeconds = 0;
            otaCheckCounter = 0; // Bağlantı olmadığı için OTA da yapılamadı, sayacı sıfırla
        }
    } else {
        // Veri gönderme zamanı değilse, Wi-Fi hiç açılmaz.
        Serial.println("Veri gonderme zamani gelmedi. Wi-Fi bağlantısı denenmedi, enerji tasarrufu yapıldı.");
    }

    Serial.println("--- Islemler Tamamlandi ---");

    // --- DEEP SLEEP ---
    int sleepDurationSeconds;
    long currentTargetTotalPeriodSecondsCalculated = inAlarmState ? ALARM_TOTAL_PERIOD_SECONDS : NORMAL_TOTAL_PERIOD_SECONDS;
    long targetSleepSecondsForThisPeriod = currentTargetTotalPeriodSecondsCalculated - ESTIMATED_ACTIVE_TIME_SECONDS;

    // Kalan uyku süresi hesaplaması
    long remainingSleepNeeded = targetSleepSecondsForThisPeriod - accumulatedSleepSeconds;

    if (remainingSleepNeeded <= 0) {
        // Hedeflenen uyku süresini zaten aştıysak veya tam tamına ulaştıysak
        // (bu durum genellikle veri gönderimi sonrasında sayaçların sıfırlandığı yerde ele alınır),
        // minimum 1 saniye uyu. Bu bir sonraki periyodun başlangıcı için hazırlar.
        sleepDurationSeconds = 1;
    } else if (remainingSleepNeeded < MINUTE_SLEEP_SECONDS) {
        // Eğer kalan uyku süresi 1 dakikadan azsa, sadece o kalan süre kadar uyu.
        sleepDurationSeconds = remainingSleepNeeded;
    } else {
        // Eğer kalan uyku süresi 1 dakikadan fazlaysa, standart 1 dakikalık bir parçayı uyu.
        sleepDurationSeconds = MINUTE_SLEEP_SECONDS;
    }
    
    // Asla 1 saniyeden az uyuma (esp_deep_sleep_enable_timer_wakeup minimum 1 saniye ister)
    if (sleepDurationSeconds < 1) sleepDurationSeconds = 1;

    Serial.printf("ESP32 %d saniye Deep-sleep'e gidiyor. (Birikmis uyku: %lu sn, Hedef: %ld sn)\n", sleepDurationSeconds, accumulatedSleepSeconds, targetSleepSecondsForThisPeriod);
    esp_sleep_enable_timer_wakeup(sleepDurationSeconds * uS_TO_S_FACTOR);
    Serial.flush();
    esp_deep_sleep_start();
}

void loop() {
    // loop() fonksiyonu deep-sleep kullandığımız için çalışmaz.
}
