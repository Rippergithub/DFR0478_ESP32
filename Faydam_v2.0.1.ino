// =====================================================================================
// FAYDAM DATALOGGER - ESP32 (Python Aracı Entegrasyonu ile)
// Versiyon: v2.0.1
// Açıklama: Bu kod, DS18B20 sıcaklık sensörü ve bir analog sensörden veri okuyan,
// bu verileri belirli periyotlarla veya alarm durumlarında bir web sunucusuna gönderen
// bir IoT cihazı yazılımıdır. Cihaz, enerji tasarrufu için deep sleep modunu kullanır.
// Ek olarak, seri port üzerinden JSON komutları ile çalışan bir Python yapılandırma
// aracıyla tam entegrasyon için bir 'Yapılandırma Modu' içerir.
//
// v2.0.0: Python Yapılandırma Aracı Entegrasyonu ve Stabilite Güncellemeleri
// - Python Aracı Entegrasyonu (Yapılandırma Modu) eklendi:
//   - Cihaz ilk açıldığında, 10 saniye boyunca seri porttan komut bekler.
//   - Komut alınırsa, Deep Sleep devre dışı bırakılır ve loop() fonksiyonu aktif olarak komut dinler.
//   - Bu modda aşağıdaki işlemler seri port üzerinden yapılabilir:
//     - Cihaz bilgilerini (MAC, FW Versiyonu) okuma.
//     - Wi-Fi ağlarını tarama ve sonuçları listeleme.
//     - Wi-Fi ayarlarını (SSID/Parola) cihaza kaydetme ve silme.
//     - Kaydedilmiş Wi-Fi bilgileriyle anlık bağlantı testi yapma (IP ve Sinyal Gücü alma).
//     - UDP Log ayarlarını (IP/Port) okuma ve cihaza kalıcı olarak kaydetme.
//     - Cihazı yeniden başlatarak normal otonom moda döndürme.
//
// - Stabilite ve Optimizasyon İyileştirmeleri:
//   - `setup()` fonksiyonu mantığı, modlar arası (otonom, web, yapılandırma) geçişin net
//     ve hatasız olması için tamamen yeniden düzenlendi.
//   - Yapılandırma modundaki Wi-Fi tarama fonksiyonu, kilitlenmeleri önlemek için stabilize edildi.
//   - Otonom modda, tüm ağ işlemleri (Zaman alma, OTA, Veri gönderimi) tek bir Wi-Fi
//     bağlantısı altında birleştirilerek güç ve zaman verimliliği artırıldı.
//   - Deep sleep'ten uyanıldığında gereksiz 10 saniyelik bekleme kaldırıldı.
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
#include <WiFiClient.h>        // WebServer kütüphanesi için gerekli olabilir
#include <WiFiClientSecure.h>  // OTA için HTTPS bağlantısı
#include <Update.h>            // OTA firmware güncellemesi için
#include <WiFiUdp.h>           // UDP loglama için

// --- Remote Logger Ayarları ---
String remoteLogHost = "192.168.2.223"; // Varsayılan IP adresi
int remoteLogPort = 4210;              // Varsayılan Port

const char* UDP_CONFIG_FILE = "/udp_config.json"; // Ayarların kaydedileceği dosya adı

WiFiUDP Udp; // UDP objesi
char logBuffer[256]; // Yeterli boyutta bir buffer

// --- VERSİYON BİLGİSİ ---
#define FIRMWARE_VERSION "2.0.1" // Cihazın mevcut firmware versiyonu (STRING OLMALI!)

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
const float BATTERY_VOLTAGE_TO_MV_FACTOR = 0.0109329; // V/ham_birim
// --- Mantıksız Okuma Aralıkları (Plausible Ranges) ---

RTC_DATA_ATTR int tempSensorFaultSkipCounter = 0;   // Sıcaklık sensörü okumasını atlama sayacı
RTC_DATA_ATTR int battSensorFaultSkipCounter = 0;   // Pil sensörü okumasını atlama sayacı

// --- Mantıksız Okuma Aralıkları (Plausible Ranges) ---
// Bu değerleri sensörlerinizin ve pilinizin fiziksel limitlerine göre ayarlayın.
const float MIN_PLAUSIBLE_TEMP_C = -50.0;  // DS18B20 için minimum kabul edilebilir sıcaklık (°C)
const float MAX_PLAUSIBLE_TEMP_C = 125.0; // DS18B20 için maksimum kabul edilebilir sıcaklık (°C)

const int MIN_PLAUSIBLE_BATT_MV100 = 280;  // Minimum kabul edilebilir pil voltajı (2.80V için 280, mV*100 formatında)
const int MAX_PLAUSIBLE_BATT_MV100 = 430;  // Maksimum kabul edilebilir pil voltajı (4.30V için 430, mV*100 formatında)

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

// NOT: GITHUB_USER ve GITHUB_REPO artık doğrudan URL içinde yer alacak,
// bu yüzden sadece okunabilirlik için tutulabilirler ama URL oluşturmada kullanılmayacaklar.
//const char* GITHUB_PAT = "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"; // Kendi PAT'nızın doğru olduğundan emin olun
const char* GITHUB_HOST_FOR_OTA = "raw.githubusercontent.com"; // <<<<<<<<< BURAYI DÜZELTİN <<<<<<<<<

const char* GITHUB_USER = "Rippergithub"; // GitHub kullanıcı adınız
const char* GITHUB_REPO = "DFR0478_ESP32"; // GitHub depo adınız
// Versiyon dosyasına ve firmware'e giden tam yol (host hariç)
const char* GITHUB_BASE_RAW_PATH = "/Rippergithub/DFR0478_ESP32/main/"; //

// Versiyon dosyasının adını da ayrı tutalım.
const char* GITHUB_VERSION_FILE_NAME = "version.json"; 

// Bu CA sertifikası, GitHub'a HTTPS üzerinden güvenli bağlantı kurmak için gereklidir.
// Kendi cihazınız için güncel ve doğru sertifikayı buraya eklemeniz ÇOK ÖNEMLİDİR.
// https://www.digicert.com/kb/ssl-support/pem-file-for-digicert-root-certificates.html adresinden
// veya tarayıcınızdan raw.githubusercontent.com adresine gidip sertifika detaylarından
// "DigiCert Global Root G2" veya "ISRG Root X1" gibi root sertifikayı alabilirsiniz.
const char* GITHUB_ROOT_CA = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n" \
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n" \
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n" \
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n" \
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n" \
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n" \
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n" \
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n" \
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n" \
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n" \
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n" \
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n" \
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n" \
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n" \
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n" \
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n" \
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n" \
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n" \
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n" \
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n" \
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n" \
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n" \
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n" \
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n" \
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n" \
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n" \
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n" \
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n" \
"-----END CERTIFICATE-----\n";

// RTC belleğinde kalıcı olacak sayaçlar ve veriler (Deep Sleep sonrası değerlerini korur)
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool inAlarmState = false; // Cihaz alarm durumunda mı?
RTC_DATA_ATTR int otaCheckCounter = 0;    // OTA kontrolü için sayaç
RTC_DATA_ATTR bool shouldCheckOTA = false; // OTA kontrolünün yapılması gerektiğini belirtir
RTC_DATA_ATTR unsigned long accumulatedSleepSeconds = 0; // Toplam birikmiş uyku süresi (saniye cinsinden)

// --- Son Geçerli Sensör Değerleri (RTC Bellekte Saklanır) ---
// Cihaz ilk açıldığında veya manuel reset sonrası bu değerler varsayılan değerlerinde olacaktır.
// DEVICE_DISCONNECTED_C veya 0 gibi başlangıç değerleri, henüz geçerli bir okuma olmadığını gösterir.
RTC_DATA_ATTR float lastValidTemperatureC = DEVICE_DISCONNECTED_C; // Son geçerli sıcaklık okuması
RTC_DATA_ATTR int lastValidBatteryMV100 = 0;                     // Son geçerli pil voltajı okuması (mV*100)

// --- Mantıksız Okuma Sayaçları (Hata Tespiti ve Güç Optimizasyonu için) ---
RTC_DATA_ATTR int consecutiveInvalidTempReadings = 0; // Ardışık geçersiz sıcaklık okuma sayısı
RTC_DATA_ATTR int consecutiveInvalidBatteryReadings = 0; // Ardışık geçersiz pil okuma sayısı

const int MAX_CONSECUTIVE_INVALID_READINGS = 5; // Bir sensörün arızalı sayılması için ardışık geçersiz okuma eşiği
const int SENSOR_FAULT_SKIP_CYCLES = 10;        // Arızalı sensörü okumayı atlama süresi (Dakika bazında veya 1 dakikalık deep sleep döngüsü sayısı)

#define OTA_CHECK_INTERVAL_MAIN_SENDS 72 // Her 5 başarılı ana veri gönderiminde bir OTA kontrolü yap
#define MAX_ALARM_TEMP_READINGS (NORMAL_TOTAL_PERIOD_SECONDS / MINUTE_SLEEP_SECONDS) // Alarm sıcaklık okuma kapasitesi (yaklaşık 10 okuma)

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

bool inConfigMode = false; // Cihazın Python arayüzü için yapılandırma modunda olup olmadığını belirtir.
// Global değişkenler bölümüne ekleyin
bool debugMode = false;

// Wi-Fi tarama işlemleri için global değişkenler
String lastScanResultsJson = "[]"; // Son tarama sonuçlarını JSON olarak depolar
unsigned long lastScanTime = 0;    // Son tarama zamanı (millis())
const long SCAN_COOLDOWN_TIME = 20000; // Tarama arası minimum süre 20 saniyeye çıkarıldı

// Bilinen ağları depolamak için bir DynamicJsonDocument
DynamicJsonDocument knownNetworksDoc(2048); // Yeterli boyutta bir belge

// DS18B20 sensörü için nesneler
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void loadUdpConfig();
void saveUdpConfig();
void handleGetUdpSettings();
void handleSetUdpSettings(JsonDocument& doc);
void processSerialCommands();
void handleWifiTest();
void handleScanWifi();
void handleClearWifiSettings();
void handleSetWifi(JsonDocument& doc);
void handleGetWifiSettings();
void handleGetDeviceInfo();

// Wi-Fi STA kimlik bilgilerini LittleFS'e JSON formatında kaydeder (Debug Moduna uygun)
void saveWiFiCredentials(const String& ssid, const String& password) {
    DynamicJsonDocument doc(256);
    doc["ssid"] = ssid;
    doc["password"] = password;

    File configFile = LittleFS.open(WIFI_STA_CONFIG_FILE, "w");
    if (!configFile) {
        // Hata durumunu her zaman logla
        log_message("HATA: LittleFS STA yapilandirma dosyasi yazilamadi.");
        return;
    }
    serializeJson(doc, configFile);
    configFile.close();
    
    // Başarılı kaydetme işlemini sadece debug modunda logla
    log_debug("Wi-Fi STA bilgileri LittleFS'e kaydedildi.");
}

// Wi-Fi STA kimlik bilgilerini LittleFS'ten okur (Debug Moduna uygun)
void loadWiFiCredentials() {
    if (LittleFS.exists(WIFI_STA_CONFIG_FILE)) { 
        File configFile = LittleFS.open(WIFI_STA_CONFIG_FILE, "r"); 
        if (configFile) {
            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, configFile);
            if (!error) {
                saved_ssid = doc["ssid"].as<String>();
                saved_password = doc["password"].as<String>();
                
                // Sadece debug modunda logla
                sprintf(logBuffer, "LittleFS'ten okunan STA SSID: '%s'", saved_ssid.c_str());
                log_debug(logBuffer);

                // Güvenlik için parolayı maskele
                String masked_pass = "";
                for(int i=0; i < saved_password.length(); i++) {
                  masked_pass += "*";
                }
                sprintf(logBuffer, "LittleFS'ten okunan STA Parola: '%s'", masked_pass.c_str());
                log_debug(logBuffer);

            } else {
                log_debug("LittleFS STA yapilandirma dosyasi okunamadi veya bozuk.");
            }
            configFile.close();
        } else {
            log_debug("LittleFS STA yapilandirma dosyasi acilamadi.");
        }
    } else {
        log_debug("LittleFS STA yapilandirma dosyasi bulunamadi.");
    }
}

// Wi-Fi STA kimlik bilgilerini LittleFS'ten siler (Debug Moduna uygun)
void clearWiFiCredentials() {
    if (LittleFS.exists(WIFI_STA_CONFIG_FILE)) {
        if (LittleFS.remove(WIFI_STA_CONFIG_FILE)) {
            // Başarılı silme işlemini sadece debug modunda logla
            log_debug("Wi-Fi STA bilgileri LittleFS'ten temizlendi.");
        } else {
            // Hata durumunu her zaman logla
            log_message("HATA: LittleFS Wi-Fi STA bilgileri silinemedi.");
        }
    } else {
        // Dosyanın zaten olmadığını sadece debug modunda logla
        log_debug("LittleFS'te temizlenecek Wi-Fi STA bilgisi yok.");
    }
    saved_ssid = "";
    saved_password = "";
}

// AP SSID'yi dosyaya kaydeder (Debug Moduna uygun)
void saveApSsidToFile(const String& ap_ssid) {
    DynamicJsonDocument doc(128);
    doc["ap_ssid"] = ap_ssid;

    File apFile = LittleFS.open(AP_SSID_FILE, "w");
    if (!apFile) {
        // Hata durumunu her zaman logla
        log_message("HATA: LittleFS AP SSID dosyasi yazilamadi.");
        return;
    }
    serializeJson(doc, apFile);
    apFile.close();
    
    // Başarılı kaydetme işlemini sadece debug modunda logla
    log_debug("AP SSID LittleFS'e kaydedildi.");
}

// AP SSID'yi dosyadan okur (Debug Moduna uygun)
String loadApSsidFromFile() {
    String loadedApSsid = "";
    if (LittleFS.exists(AP_SSID_FILE)) {
        File apFile = LittleFS.open(AP_SSID_FILE, "r");
        if (apFile) {
            DynamicJsonDocument doc(128);
            DeserializationError error = deserializeJson(doc, apFile);
            if (!error) {
                loadedApSsid = doc["ap_ssid"].as<String>();
                // Başarılı okumayı sadece debug modunda logla
                sprintf(logBuffer, "LittleFS'ten okunan AP SSID: '%s'", loadedApSsid.c_str());
                log_debug(logBuffer);
            } else {
                // Hata durumunu her zaman logla
                log_message("HATA: LittleFS AP SSID dosyasi okunamadi veya bozuk. Dosya siliniyor.");
                LittleFS.remove(AP_SSID_FILE);
            }
            apFile.close();
        } else {
            // Hata durumunu her zaman logla
            log_message("HATA: LittleFS AP SSID dosyasi acilamadi.");
        }
    } else {
        // Dosyanın olmaması bir hata değil, rutin bir durum. Sadece debug'da göster.
        log_debug("LittleFS AP SSID dosyasi bulunamadi.");
    }
    return loadedApSsid;
}

// Bilinen Wi-Fi Ağlarını Dosyadan Oku (Debug Moduna uygun)
void loadKnownNetworksFromFile() {
    log_debug("Bilinen Wi-Fi aglari dosyadan okunuyor...");
    File knownNetworksFile = LittleFS.open(KNOWN_NETWORKS_FILE, "r");
    if (!knownNetworksFile) {
        log_debug("Known Wi-Fi networks dosyasi bulunamadi. Bos liste ile baslatiliyor.");
        knownNetworksDoc.to<JsonArray>();
        return;
    }

    DeserializationError error = deserializeJson(knownNetworksDoc, knownNetworksFile);
    if (error) {
        // Hata durumunu her zaman logla
        log_message("HATA: deserializeJson (known networks): " + String(error.c_str()));
        knownNetworksDoc.to<JsonArray>();
    } else {
        // Başarılı yüklemeyi sadece debug modunda logla
        log_debug("Bilinen Wi-Fi aglari basariyla yuklendi.");
    }
    knownNetworksFile.close();
}

// Bilinen Wi-Fi Ağlarını Dosyaya Kaydet (Debug Moduna uygun)
void saveKnownNetworksToFile() {
    log_debug("Bilinen Wi-Fi aglari dosyaya kaydediliyor...");
    File knownNetworksFile = LittleFS.open(KNOWN_NETWORKS_FILE, "w");
    if (!knownNetworksFile) {
        // Hata durumunu her zaman logla
        log_message("HATA: Known Wi-Fi networks dosyasi yazilamadi.");
        return;
    }

    if (serializeJson(knownNetworksDoc, knownNetworksFile) == 0) {
        // Hata durumunu her zaman logla
        log_message("HATA: Bilinen Wi-Fi aglari JSON dosyaya yazilamadi.");
    } else {
        // Başarılı kaydı sadece debug modunda logla
        log_debug("Bilinen Wi-Fi aglari basariyla dosyaya kaydedildi.");
    }
    knownNetworksFile.close();
}

// İstenen SSID için bilinen parolayı döndürür (Web Sunucu Endpoint'i - Debug ve Hata Kurtarma ile)
void handleGetKnownPassword() {
    String requestedSsid = server.arg("ssid");
    String foundPassword = "";

    sprintf(logBuffer, "Web'den bilinen parola istegi alindi: '%s'", requestedSsid.c_str());
    log_debug(logBuffer);

    if (knownNetworksDoc.is<JsonArray>()) {
        JsonArray networks = knownNetworksDoc.as<JsonArray>();
        for (JsonObject network : networks) {
            if (network["ssid"].as<String>() == requestedSsid) {
                foundPassword = network["password"].as<String>();
                log_debug("Parola bilinen aglar listesinde bulundu.");
                break;
            }
        }
    } else {
        // <<< DÜZELTİLMİŞ HATA KURTARMA MANTIĞI >>>
        log_message("HATA: knownNetworksDoc gecerli bir JsonArray degil. Dosya yeniden yukleniyor.");
        loadKnownNetworksFromFile(); // Dosyayı yeniden yüklemeyi dene
        
        if (knownNetworksDoc.is<JsonArray>()) { // Yeniden yüklendikten sonra tekrar ara
            JsonArray networks = knownNetworksDoc.as<JsonArray>();
            for (JsonObject network : networks) {
                if (network["ssid"].as<String>() == requestedSsid) {
                    foundPassword = network["password"].as<String>();
                    log_debug("Parola, dosya yeniden yuklendikten sonra bulundu.");
                    break;
                }
            }
        }
    }

    DynamicJsonDocument responseDoc(128);
    responseDoc["password"] = foundPassword;

    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    server.send(200, "application/json", jsonResponse);
}

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
            optionsHtml = '<option value="">Lütfen Bekleyiniz...</option>';
            getNetworks();
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

// Web arayüzünden gelen tarama isteğini başlatır (Debug Moduna uygun)
void handleStartScan() {
    unsigned long currentTime = millis();
    if (currentTime - lastScanTime < SCAN_COOLDOWN_TIME) {
        server.send(202, "application/json", "{\"status\":\"scanning\", \"message\":\"Tarama bekleme süresinde, devam ediyor.\"}");
        return;
    }
    
    // Sadece debug modunda logla
    log_debug("Yeni Wi-Fi tarama baslatiliyor (web arayuzunden)...");

    int n = WiFi.scanNetworks(true);
    if (n == WIFI_SCAN_RUNNING) {
        server.send(202, "application/json", "{\"status\":\"scanning\", \"message\":\"Wi-Fi taraması başlatıldı (arka planda).\"}");
        lastScanTime = currentTime;
    } else if (n == WIFI_SCAN_FAILED) {
        server.send(500, "application/json", "{\"status\":\"failed\", \"message\":\"Wi-Fi taraması başlatılırken hata oluştu.\"}");
    } else {
        // Sadece debug modunda logla
        log_debug("Tarama hemen tamamlandi (beklenmedik durum).");
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


// <<< YUKARIDAKİ FONKSİYONU BUNUNLA DEĞİŞTİRİN >>>

// Web sunucusunu başlatır (Debug Moduna uygun)
void startWebServer() {
    log_message("Wi-Fi kurulumu icin Erisim Noktasi (AP) baslatiliyor...");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ap_ssid_to_use.c_str(), DEFAULT_AP_PASSWORD);
    
    sprintf(logBuffer, "AP SSID: %s, AP IP: %s", ap_ssid_to_use.c_str(), WiFi.softAPIP().toString().c_str());
    log_message(logBuffer);

    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/startscan", HTTP_GET, handleStartScan);
    server.on("/getnetworks", HTTP_GET, handleGetNetworks);
    server.on("/getKnownPassword", HTTP_GET, handleGetKnownPassword);
    server.serveStatic("/faydamlogo.png", LittleFS, "/faydamlogo.png", "max-age=86400");
    server.serveStatic("/favicon.ico", LittleFS, "/favicon.ico", "max-age=86400");

    server.begin();
    log_message("Web sunucusu basladi.");

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


// <<< YUKARIDAKİ FONKSİYONU BUNUNLA DEĞİŞTİRİN >>>

// Wi-Fi'ye bağlanmayı dener (Debug Moduna uygun)
bool connectToWiFi() {
    log_message("WiFi baglaniliyor..."); // Bağlantı denemesi kritik bir bilgi olduğu için log_message
    WiFi.mode(WIFI_STA);
    WiFi.begin(saved_ssid.c_str(), saved_password.c_str());

    wl_status_t status = (wl_status_t)WiFi.waitForConnectResult(30000);

    if (status == WL_CONNECTED) {
        log_message("WiFi baglantisi basarili."); // Başarılı bağlantı kritik bir bilgi

        sprintf(logBuffer, "IP Adresi: %s", WiFi.localIP().toString().c_str());
        log_debug(logBuffer); // IP adresi gibi detaylar debug için

        String desiredHostname = "Faydam_" + deviceID;
        WiFi.setHostname(desiredHostname.c_str());
        log_debug("WiFi Hostname ayarlandi: " + desiredHostname);

        // DNS çözümlemesi gibi detaylı ağ bilgilerini sadece debug modunda göster
        IPAddress resolvedIP;
        if (WiFi.hostByName(API_HOST, resolvedIP) == 1) {
            sprintf(logBuffer, "DNS Cozumlemesi basarili. %s -> %s", API_HOST, resolvedIP.toString().c_str());
            log_debug(logBuffer);
        } else {
            // DNS hatası önemli olabilir, bunu her zaman gösterelim
            sprintf(logBuffer, "HATA: DNS Cozumlemesi basarisiz! Host: %s", API_HOST);
            log_message(logBuffer);
        }
        return true;
    } else {
        // Bağlantı hatalarını her zaman göster
        sprintf(logBuffer, "HATA: WiFi baglantisi kurulamadi. Durum kodu: %d", status);
        log_message(logBuffer);
        
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        log_message("WiFi baglantisi kurulamadigi icin kapatildi.");
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

// Sunucu yanıtından eşik değerleri ve zamanı ayrıştırır (Debug Moduna uygun)
String parseThresholdsAndSyncTime(const String& jsonResponse) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonResponse);

    if (error) {
        log_message("HATA: Sunucu yaniti (JSON) ayristirilamadi: " + String(error.c_str()));
        return "00000000000000";
    }

    sprintf(logBuffer, "Sunucu yanit uzunlugu: %d, JSON Oge Sayisi: %d", jsonResponse.length(), doc.size());
    log_debug(logBuffer);
    
    bool thresholdsUpdated = false;

    if (doc.containsKey("TLow")) {
        String tLowStr = doc["TLow"].as<String>();
        float temp_tLow = tLowStr.substring(0, tLowStr.indexOf(',')).toFloat();
        if (temp_tLow != -99999999.00f) {
            TLow_threshold = temp_tLow;
            log_debug("Alinan TLow Esik Degeri: " + String(TLow_threshold));
            thresholdsUpdated = true;
        } else {
            log_debug("TLow esik degeri API'den default geldi, guncellenmedi.");
        }
    } else {
        log_debug("API yanitinda 'TLow' bulunamadi.");
    }

    if (doc.containsKey("THigh")) {
        String tHighStr = doc["THigh"].as<String>();
        float temp_tHigh = tHighStr.substring(0, tHighStr.indexOf(',')).toFloat();
        if (temp_tHigh != -99999999.00f) {
            THigh_threshold = temp_tHigh;
            log_debug("Alinan THigh Esik Degeri: " + String(THigh_threshold));
            thresholdsUpdated = true;
        } else {
            log_debug("THigh esik degeri API'den default geldi, guncellenmedi.");
        }
    } else {
        log_debug("API yanitinda 'THigh' bulunamadi.");
    }

    if (thresholdsUpdated) {
        log_debug("Esik degerleri basariyla guncellendi.");
    } else {
        log_debug("API'den yeni esik degeri alinamadi, mevcut esikler korunuyor.");
    }

    if (doc.containsKey("DateTime")) {
        String dateTimeStr = doc["DateTime"].as<String>();
        if (dateTimeStr.length() == 0) {
            log_message("UYARI: Sunucu yanitinda 'DateTime' alani bos geldi.");
            return "00000000000000";
        }
        log_debug("Alinan zaman stringi: " + dateTimeStr);

        struct tm timeinfo;
        int year, month, day, hour, minute, second;
        int parsed_elements = sscanf(dateTimeStr.c_str(), "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);

        if (parsed_elements != 6) {
            log_message("HATA: Zaman verisi format uyumsuzluk!");
            return "00000000000000";
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
            log_message("HATA: Zaman donusturme hatasi (mktime).");
            return "00000000000000";
        }
        struct timeval tv;
        tv.tv_sec = epochTime;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        log_debug("Sistem zamani sunucudan gelen veriyle ayarlandi.");
        
        struct tm current_time_info;
        getLocalTime(&current_time_info);
        String formattedTime = formatTime(current_time_info);
        log_debug("Guncel RTC Zamani (Sunucudan): " + formattedTime);    
        return formattedTime;
    } else {
        log_message("UYARI: Sunucu yanitinda 'DateTime' alani bulunamadi.");
        return "00000000000000";
    }   
}

// Sensör verisini datalog.csv'ye kaydeder (Debug Moduna uygun)
void saveSensorDataToFile(float temp_c, int analog_val, time_t timestamp, int rssi) {
    File dataFile = LittleFS.open(DATALOG_FILE, FILE_APPEND);
    if (!dataFile) {
        // Hata durumunu her zaman logla
        log_message("HATA: datalog.csv dosyasi acilamadi/olusturulamadi!");
        return;
    }
    String dataLine = String((int)(temp_c * 100)) + "," + String(analog_val) + "," + String(timestamp) + "," + String(rssi);
    dataFile.println(dataLine);
    dataFile.close();
    
    // Başarılı kaydı sadece debug modunda logla
    sprintf(logBuffer, "Veri datalog.csv'ye kaydedildi: %s", dataLine.c_str());
    log_debug(logBuffer);
}

// datalog.csv'den verileri göndermeye çalışır (Debug Moduna uygun)
bool sendQueuedDataFromFile() {
    if (!LittleFS.exists(DATALOG_FILE)) {
        log_debug("datalog.csv dosyasi yok, gonderilecek bekleyen veri bulunamadi.");
        return true;
    }
    File dataFile = LittleFS.open(DATALOG_FILE, FILE_READ);
    if (!dataFile) {
        log_message("HATA: datalog.csv dosyasi okuma modunda acilamadi!");
        return false;
    }
    File tempFile = LittleFS.open("/temp_datalog.csv", FILE_WRITE);
    if (!tempFile) {
        log_message("HATA: temp_datalog.csv dosyasi olusturulamadi!");
        dataFile.close();
        return false;
    }
    
    log_message("datalog.csv'den bekleyen veriler gonderiliyor...");
    HTTPClient http;
    WiFiClientSecure client_secure; // HTTPS için gerekli
    client_secure.setInsecure();    // Sertifika kontrolünü atla
    
    bool allQueuedDataSent = true;
    
    while (dataFile.available()) {
        String line = dataFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        int thirdComma = line.indexOf(',', secondComma + 1);

        if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
            sprintf(logBuffer, "UYARI: Gecersiz CSV satiri atlandi: %s", line.c_str());
            log_message(logBuffer);
            tempFile.println(line);
            allQueuedDataSent = false;
            continue;
        }
        int temp_val_int = line.substring(0, firstComma).toInt();
        int analog_val = line.substring(firstComma + 1, secondComma).toInt();
        time_t timestamp = (time_t)line.substring(secondComma + 1, thirdComma).toInt();
        int rssi_from_file = line.substring(thirdComma + 1).toInt();

        struct tm queued_timeinfo;
        gmtime_r(&timestamp, &queued_timeinfo);
        
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

        log_debug("Kuyruktan gonderilen URL: " + request_url);
        
        http.begin(client_secure, request_url);
        int httpCode = http.GET();

        if (httpCode > 0 && (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)) {
            String payload = http.getString();
            sprintf(logBuffer, "[HTTP] Kuyruk GET... code: %d", httpCode);
            log_message(logBuffer);
            log_debug("Kuyruk Yaniti: " + payload);
        } else {
            sprintf(logBuffer, "HATA: [HTTP] Kuyruk GET... basarisiz: %d, hata: %s.", httpCode, http.errorToString(httpCode).c_str());
            log_message(logBuffer);
            tempFile.println(line);
            allQueuedDataSent = false;
            // Bir hata olduğunda geri kalan verileri de geçici dosyaya kopyala ve döngüden çık
            while(dataFile.available()) {
                tempFile.println(dataFile.readStringUntil('\n'));
            }
            break;
        }
        http.end();
    }
    dataFile.close();
    tempFile.close();

    LittleFS.remove(DATALOG_FILE);
    if (!LittleFS.rename("/temp_datalog.csv", DATALOG_FILE)) {
        log_message("HATA: datalog.csv guncellenirken hata olustu.");
    }

    if (allQueuedDataSent) {
        log_message("Bekleyen tum veriler basariyla gonderildi.");
        LittleFS.remove(DATALOG_FILE);
    } else {
        log_message("Bekleyen verilerin bir kismi gonderilemedi, dosyada tutuluyor.");
    }
    return allQueuedDataSent;
}



// Diğer fonksiyonların yanına bu yeni fonksiyonu ekleyin
void log_debug(String message) {
    if (debugMode) {
        log_message(message); // Sadece debug modu aktifse log gönder
    }
}

// Sunucudan zaman ve eşik değerlerini alır (Debug Moduna uygun)
void sendGateData() {
    // Fonksiyonun başında, Wi-Fi'nin zaten bağlı olup olmadığını bir değişkende sakla
    bool connectionWasActive = (WiFi.status() == WL_CONNECTED);
    
    // Eğer zaten bağlı değilse, bağlanmayı dene.
    if (!connectionWasActive) {
        if (!connectToWiFi()) {
            log_message("HATA: sendGateData icin WiFi baglantisi kurulamadi.");
            return; // Bağlanamazsa fonksiyondan çık.
        }
    }

    // --- Buradan sonrası, bağlantının artık var olduğu varsayılarak devam eder ---
    HTTPClient http;
    WiFiClientSecure client_secure;
    client_secure.setInsecure();

    String gate_request_url = "https://" + String(API_HOST) + "/ext/getdate";
    log_debug("Gonderilecek GATE URL: " + gate_request_url);
    
    http.begin(client_secure, gate_request_url);
    int httpCode = http.GET();

    if (httpCode > 0 && (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)) {
        String payload = http.getString();
        log_message("GATE istegi basarili. Zaman ve esik degerleri alindi.");
        log_debug("GATE Sunucu Yaniti: " + payload);
        parseThresholdsAndSyncTime(payload);
    } else {
        sprintf(logBuffer, "HATA: [HTTP] GATE GET... basarisiz. Kod: %d", httpCode);
        log_message(logBuffer);
    }
    http.end();

    // Sadece ve sadece bu fonksiyon özel olarak bağlantı kurduysa bağlantıyı kapat.
    // Eğer setup tarafından zaten açık bırakılmış bir bağlantıyı kullandıysa, ona dokunma.
    if (!connectionWasActive) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        log_debug("sendGateData kendi actigi baglantiyi kapatti.");
    }
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

// OTA güncelleme kontrolünü ve başlatmayı yapar (Debug Moduna uygun)
void checkAndDoUpdate() {
    if (WiFi.status() != WL_CONNECTED) {
        log_message("HATA: OTA kontrolu icin WiFi bagli degil.");
        return;
    }

    WiFiClientSecure client;
    client.setCACert(GITHUB_ROOT_CA);
    client.setInsecure(); // Geliştirme aşamasında sertifika kontrolünü atla

    HTTPClient http;
    String versionUrl = "https://" + String(GITHUB_HOST_FOR_OTA) + String(GITHUB_BASE_RAW_PATH) + String(GITHUB_VERSION_FILE_NAME);

    log_debug("OTA: GitHub versiyon kontrol ediliyor: " + versionUrl);

    http.begin(client, versionUrl);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        String latestFirmwareVersion = "";
        String firmwareDownloadUrl = "";

        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
            log_message("HATA: OTA JSON ayristirma hatasi: " + String(error.c_str()));
            http.end();
            return;
        }

        latestFirmwareVersion = doc["version"].as<String>();
        firmwareDownloadUrl = doc["url"].as<String>();

        sprintf(logBuffer, "OTA: Mevcut versiyon: %s, Sunucu versiyon: %s", FIRMWARE_VERSION, latestFirmwareVersion.c_str());
        log_debug(logBuffer);

        if (isNewVersionAvailable(FIRMWARE_VERSION, latestFirmwareVersion.c_str())) {
            log_message("OTA: Yeni firmware versiyonu bulundu! Guncelleme baslatiliyor...");
            log_debug("OTA: Indirme URL: " + firmwareDownloadUrl);
            
            http.end(); 

            http.begin(client, firmwareDownloadUrl);
            int firmwareHttpCode = http.GET();

            if (firmwareHttpCode == HTTP_CODE_OK) {
                int contentLength = http.getSize();
                if (contentLength > 0) {
                    if (Update.begin(contentLength)) {
                        size_t written = Update.writeStream(*http.getStreamPtr());
                        if (written == contentLength) {
                            log_debug("OTA: Yazma basarili.");
                        } else {
                            sprintf(logBuffer, "HATA: OTA Yazma hatasi! Yazilan: %zu, Beklenen: %d", written, contentLength);
                            log_message(logBuffer);
                        }
                        if (Update.end()) {
                            log_message("OTA: Guncelleme basarili. Cihaz yeniden baslatiliyor...");
                            ESP.restart();
                        } else {
                            sprintf(logBuffer, "HATA: OTA Bitirme hatasi! Kod: %d - %s", Update.getError(), Update.errorString());
                            log_message(logBuffer);
                        }
                    } else {
                        log_message("HATA: OTA Guncelleme baslatilamadi (Update.begin hatasi).");
                    }
                } else {
                    log_message("HATA: OTA Firmware boyutu alinamadi veya sifir.");
                }
            } else {
                sprintf(logBuffer, "HATA: OTA Firmware indirme hatasi! HTTP Kodu: %d", firmwareHttpCode);
                log_message(logBuffer);
            }
        } else {
            log_debug("OTA: Mevcut firmware en guncel versiyon.");
        }
    } else {
        sprintf(logBuffer, "HATA: OTA Versiyon dosyasi indirme hatasi! HTTP Kodu: %d", httpCode);
        log_message(logBuffer);
    }
    http.end();
}

/*
// Ana sensör verisini sunucuya gönderir (Debug Moduna uygun)
void sendMainData() {
    String formattedTime;
    struct tm timeinfo;
    time_t current_epoch_time;
    int current_rssi = -1;
    String deviceIpAddress = "0.0.0.0";

    if (!getLocalTime(&timeinfo)) {
        log_message("UYARI: Zaman bilgisi RTC'den alinamadi. Veri kuyruga eklenecek.");
        current_epoch_time = 0;
    } else {
        current_epoch_time = mktime(&timeinfo);
        formattedTime = formatTime(timeinfo);
    }

    // Bekleyen veri varsa önce onu gönder
    if (LittleFS.exists(DATALOG_FILE)) {
        sendQueuedDataFromFile();
    } else {
        log_debug("Gonderilecek bekleyen veri bulunamadi (datalog.csv bos).");
    }

    log_message("Guncel ana veri gonderiliyor...");
    HTTPClient http;
    WiFiClientSecure client_secure;
    client_secure.setInsecure();
    
    bool mainRequestSuccessful = false;

    // Bu fonksiyon sadece Wi-Fi'ye bağlıyken çağrıldığı için yeniden bağlantı döngüsüne gerek yok.
    if (WiFi.status() == WL_CONNECTED) {
        deviceIpAddress = WiFi.localIP().toString();
        current_rssi = WiFi.RSSI();
        String dataLine = String((int)(temperature_c * 100)) + "," + String(current_rssi) + "," + String(current_value_int);
        log_debug("Olusturulan ana veri satiri: " + dataLine);

        String main_request_url = "https://" + String(API_HOST) + "/ext/wdata?";
        main_request_url += "id=" + deviceID + "01";
        main_request_url += "&vtid=2,97,1";
        main_request_url += "&dt=" + formattedTime;
        main_request_url += "&vl=" + dataLine;
        main_request_url += "&ip=" + deviceIpAddress;
        main_request_url += "&vrs=" + String(FIRMWARE_VERSION);
        main_request_url += "&u=" + String(API_USER);
        main_request_url += "&p=" + String(API_PASS);
        main_request_url += "&API=" + String(API_KEY);
        
        log_debug("Gonderilen URL: " + main_request_url);

        http.begin(client_secure, main_request_url);
        int httpCode = http.GET();

        if (httpCode > 0 && (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)) {
            String payload = http.getString();
            sprintf(logBuffer, "[HTTP] Guncel Ana GET... code: %d", httpCode);
            log_message(logBuffer);
            log_debug("Sunucu Yaniti: " + payload);
            parseThresholdsAndSyncTime(payload);
            mainRequestSuccessful = true;
        } else {
            sprintf(logBuffer, "HATA: [HTTP] Guncel Ana GET... basarisiz: %d, hata: %s", httpCode, http.errorToString(httpCode).c_str());
            log_message(logBuffer);
        }
        http.end();
    } else {
        log_message("HATA: sendMainData cagrildi ancak WiFi baglantisi yoktu.");
    }

    if (mainRequestSuccessful) {
        log_message("Guncel ana veri sunucuya basariyla gonderildi.");
        otaCheckCounter++;
        if (otaCheckCounter >= OTA_CHECK_INTERVAL_MAIN_SENDS) {
            shouldCheckOTA = true;
            otaCheckCounter = 0;
            log_debug("Bir sonraki dongude OTA kontrolu yapilacak.");
        }
        accumulatedSleepSeconds = 0; // Başarılı gönderimden sonra uyku sayacını sıfırla
    } else {
        log_message("Guncel ana veri sunucuya gonderilemedi, datalog.csv'ye kaydediliyor.");
        saveSensorDataToFile(temperature_c, current_value_int, current_epoch_time, current_rssi);
        accumulatedSleepSeconds = 0; // Başarısız gönderimden sonra da periyodu yeniden başlat
    }
}

*/

void sendMainData() {
    String formattedTime;
    struct tm timeinfo;
    time_t current_epoch_time;
    int current_rssi = -1;
    String deviceIpAddress = "0.0.0.0";

    if (!getLocalTime(&timeinfo)) {
        log_message("UYARI: Zaman bilgisi RTC'den alinamadi.");
        current_epoch_time = 0;
    } else {
        current_epoch_time = mktime(&timeinfo);
        formattedTime = formatTime(timeinfo);
    }

    if (LittleFS.exists(DATALOG_FILE)) {
        sendQueuedDataFromFile();
    } else {
        log_debug("Gonderilecek bekleyen veri bulunamadi (datalog.csv bos).");
    }

    log_message("Guncel ana veri gonderiliyor...");
    HTTPClient http;
    WiFiClientSecure client_secure;
    client_secure.setInsecure();
    
    bool mainRequestSuccessful = false;

    if (WiFi.status() == WL_CONNECTED) {
        deviceIpAddress = WiFi.localIP().toString();
        current_rssi = WiFi.RSSI();
        String dataLine = String((int)(temperature_c * 100)) + "," + String(current_rssi) + "," + String(current_value_int);
        log_debug("Olusturulan ana veri satiri: " + dataLine);

        String main_request_url = "https://" + String(API_HOST) + "/ext/wdata?";
        main_request_url += "id=" + deviceID + "01";
        main_request_url += "&vtid=2,97,1";
        main_request_url += "&dt=" + formattedTime;
        main_request_url += "&vl=" + dataLine;
        main_request_url += "&ip=" + deviceIpAddress;
        main_request_url += "&vrs=" + String(FIRMWARE_VERSION);
        main_request_url += "&u=" + String(API_USER);
        main_request_url += "&p=" + String(API_PASS);
        main_request_url += "&API=" + String(API_KEY);
        
        log_debug("Gonderilen URL: " + main_request_url);

        http.begin(client_secure, main_request_url);
        int httpCode = http.GET();

        if (httpCode > 0 && (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)) {
            String payload = http.getString();
            sprintf(logBuffer, "[HTTP] Guncel Ana GET... code: %d", httpCode);
            log_message(logBuffer);
            log_debug("Sunucu Yaniti: " + payload);
            parseThresholdsAndSyncTime(payload);
            mainRequestSuccessful = true;
        } else {
            sprintf(logBuffer, "HATA: [HTTP] Guncel Ana GET... basarisiz: %d, hata: %s", httpCode, http.errorToString(httpCode).c_str());
            log_message(logBuffer);
        }
        http.end();
    } else {
        log_message("HATA: sendMainData cagrildi ancak WiFi baglantisi yoktu.");
    }

    if (mainRequestSuccessful) {
        log_message("Guncel ana veri sunucuya basariyla gonderildi.");
        otaCheckCounter++;
        if (otaCheckCounter >= OTA_CHECK_INTERVAL_MAIN_SENDS) {
            shouldCheckOTA = true;
            otaCheckCounter = 0;
            log_debug("Bir sonraki dongude OTA kontrolu yapilacak.");
        }
        accumulatedSleepSeconds = 0;
    } else {
        log_message("Guncel ana veri sunucuya gonderilemedi, datalog.csv'ye kaydediliyor.");
        saveSensorDataToFile(temperature_c, current_value_int, current_epoch_time, current_rssi);
        accumulatedSleepSeconds = 0;
    }
}

void log_message(String message) {
    // Cihazın MAC adresini (deviceID) her mesajın başına ekle
    String prefixedMessage = "[" + deviceID + "] " + message;

    // Mesajı hem Seri Monitör'e hem de UDP'ye ön ekli olarak gönder
    Serial.println(prefixedMessage);
    
    if (WiFi.status() == WL_CONNECTED && remoteLogHost.length() > 0) {
        Udp.beginPacket(remoteLogHost.c_str(), remoteLogPort);
        Udp.print(prefixedMessage);
        Udp.endPacket();
    }
}
/*
void setup() {
  
    // --- 1. Başlangıç Kurulumları ---
    Serial.begin(115200);
    Serial.setDebugOutput(false);
    delay(100);
    log_message("\n\n--- Cihaz Baslatiliyor ---");
    
    sprintf(logBuffer, "Cihaz Firmware Versiyonu: %s", FIRMWARE_VERSION);
    log_message(logBuffer);

    if (!LittleFS.begin()) {
        log_message("LittleFS baslatilamadi, formatlaniyor...");
        LittleFS.format();
        LittleFS.begin();
    }
    log_debug("LittleFS baslatildi.");
    debugMode = LittleFS.exists("/debug_mode.on");
    loadUdpConfig();

    WiFi.mode(WIFI_AP_STA);
    delay(100);
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    deviceID = mac;
    log_message("Cihaz MAC Adresi: " + deviceID);

    log_message("Cihaz " + String(++bootCount) + ". kez baslatildi.");

    ap_ssid_to_use = loadApSsidFromFile();
    if (ap_ssid_to_use.length() == 0) {
        ap_ssid_to_use = "FAYDAM_" + deviceID;
        saveApSsidToFile(ap_ssid_to_use);
        log_message("AP SSID olusturuldu ve kaydedildi: " + ap_ssid_to_use);
    } else {
        log_debug("Kayitli AP SSID kullaniliyor: " + ap_ssid_to_use);
    }
    
    loadWiFiCredentials();

    // --- 2. Mod Belirleme ve Özel Durumların Yönetimi ---
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) { 
        log_debug("Cihaz ilk kez calisiyor (power-on/reset).");
        
        inAlarmState = false;
        accumulatedSleepSeconds = 0;
        otaCheckCounter = 0;
        shouldCheckOTA = true;
        lastValidTemperatureC = DEVICE_DISCONNECTED_C;
        lastValidBatteryMV100 = 0;
        consecutiveInvalidTempReadings = 0;
        consecutiveInvalidBatteryReadings = 0;
        tempSensorFaultSkipCounter = 0;
        battSensorFaultSkipCounter = 0;
        
        if (Serial) {
            log_debug("Seri baglanti algilandi. Komut icin 10 saniye bekleniyor...");
            unsigned long serialWaitStartTime = millis();
            while (millis() - serialWaitStartTime < 10000) {
                if (Serial.available() > 0) {
                    inConfigMode = true;
                    break; 
                }
                delay(50);
            }
        }
        
        if (inConfigMode) {
            log_message("Komut alindi. Yapilandirma modu aktif.");
            return;
        }

        log_debug("Zaman asimi. Otonom calisma baslatiliyor.");

        pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
        bool button_pressed_for_long = false;
        unsigned long button_press_start_time = millis();
        while (digitalRead(CONFIG_BUTTON_PIN) == LOW && (millis() - button_press_start_time < 5000)) {
            delay(50);
        }
        if (millis() - button_press_start_time >= 5000) {
            button_pressed_for_long = true;
        }

        if (button_pressed_for_long || saved_ssid.length() == 0) {
            log_message("Wi-Fi bilgisi yok veya manuel kurulum tetiklendi. Web sunucusu baslatiliyor...");
            clearWiFiCredentials();
            startWebServer();
        }

        shouldSendMainData = true;
        sendGateData();

    } else { // Deep-sleep'ten uyandıysa
        sprintf(logBuffer, "Deep-sleep'ten uyanildi. Gecen sure: %lu sn. Alarm Durumu: %s", accumulatedSleepSeconds, inAlarmState ? "EVET" : "HAYIR");
        log_debug(logBuffer);
        accumulatedSleepSeconds += MINUTE_SLEEP_SECONDS;
        if (tempSensorFaultSkipCounter > 0) tempSensorFaultSkipCounter--;
        if (battSensorFaultSkipCounter > 0) battSensorFaultSkipCounter--;
    }

    // --- 3. Otonom Çalışma Mantığı ---
    log_debug("Otonom calisma dongusu basladi.");

    sensors.begin();
    if (tempSensorFaultSkipCounter > 0) {
       log_debug("Sicaklik sensoru ariza durumunda, okuma atlaniyor.");
       temperature_c = lastValidTemperatureC;
    } else {
       sensors.requestTemperatures();
       delay(750);
       temperature_c = sensors.getTempCByIndex(0);
       
       if (temperature_c == DEVICE_DISCONNECTED_C || temperature_c < MIN_PLAUSIBLE_TEMP_C || temperature_c > MAX_PLAUSIBLE_TEMP_C) {
           consecutiveInvalidTempReadings++;
           temperature_c = lastValidTemperatureC;
       } else {
           lastValidTemperatureC = temperature_c; 
           consecutiveInvalidTempReadings = 0; 
       }
       if (consecutiveInvalidBatteryReadings >= MAX_CONSECUTIVE_INVALID_READINGS) {
           log_message("!!! KRITIK UYARI: Pil sensoru hatali okunuyor! 10 dongu atlanacak.");
           battSensorFaultSkipCounter = SENSOR_FAULT_SKIP_CYCLES;
       }
    }
    if (battSensorFaultSkipCounter > 0) {
       log_debug("Pil sensoru ariza durumunda, okuma atlaniyor.");
       current_value_int = lastValidBatteryMV100;
    } else {
       const int numReadings = 20;
       long sumRawAnalogValue = 0; 
       for (int i = 0; i < numReadings; i++) { sumRawAnalogValue += analogRead(ANALOG_INPUT_PIN); delay(2); }
       int raw_analog_value_avg = sumRawAnalogValue / numReadings;
       float battery_voltage_float = raw_analog_value_avg * BATTERY_VOLTAGE_TO_MV_FACTOR;
       int calculated_current_value_int = static_cast<int>(battery_voltage_float * 100);
       if (calculated_current_value_int < MIN_PLAUSIBLE_BATT_MV100 || calculated_current_value_int > MAX_PLAUSIBLE_BATT_MV100) {
           consecutiveInvalidBatteryReadings++;
           if (lastValidBatteryMV100 != 0) { 
               current_value_int = lastValidBatteryMV100;
           } else {
               current_value_int = calculated_current_value_int; 
           }
       } else {
           current_value_int = calculated_current_value_int;
           lastValidBatteryMV100 = current_value_int;         
           consecutiveInvalidBatteryReadings = 0; 
       }
       if (consecutiveInvalidBatteryReadings >= MAX_CONSECUTIVE_INVALID_READINGS) {
           battSensorFaultSkipCounter = SENSOR_FAULT_SKIP_CYCLES;
       }
    }
    bool isTemperatureAlarmTriggered = false;
    if (temperature_c != DEVICE_DISCONNECTED_C) {
        if (temperature_c < TLow_threshold || temperature_c > THigh_threshold) {
            isTemperatureAlarmTriggered = true;
        }
    }
    
    long currentTargetTotalPeriodSeconds = inAlarmState ? ALARM_TOTAL_PERIOD_SECONDS : NORMAL_TOTAL_PERIOD_SECONDS;
    if (!shouldSendMainData) {
        if (accumulatedSleepSeconds >= (currentTargetTotalPeriodSeconds - ESTIMATED_ACTIVE_TIME_SECONDS)) {
            shouldSendMainData = true;
        }
    }
    if (isTemperatureAlarmTriggered && !inAlarmState) {
        log_message("!!! ALARM BASLANGICI: Sicaklik esik disina cikti!");
        shouldSendMainData = true; inAlarmState = true; accumulatedSleepSeconds = 0;
    } else if (!isTemperatureAlarmTriggered && inAlarmState) {
        log_message("--- ALARM SONA ERDI: Sicaklik normale dondu. ---");
        shouldSendMainData = true; inAlarmState = false; accumulatedSleepSeconds = 0;
    }

    if (shouldSendMainData) {
        log_message("Gonderim zamani geldi. Ag islemleri baslatiliyor...");
        if (connectToWiFi()) {
            // BAĞLANTI BAŞARILI, ŞİMDİ TÜM İŞLEMLERİ YAP
            
            // Eğer ilk açılışsa, önce zamanı al
            if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
                sendGateData();
            }

            // Eğer OTA kontrol zamanı geldiyse, kontrol et
            if (shouldCheckOTA) {
                checkAndDoUpdate();
                shouldCheckOTA = false;
            }

            // Her durumda ana veriyi gönder
            sendMainData(); 
            
            // TÜM İŞLEMLER BİTTİKTEN SONRA Wi-Fi'yi kapat
            if (WiFi.status() == WL_CONNECTED) {
                WiFi.disconnect(true);
                WiFi.mode(WIFI_OFF);
                log_debug("Tum ag islemleri bitti, WiFi kapatildi.");
            }

        } else {
            // BAĞLANTI BAŞARISIZ OLURSA VERİYİ KAYDET
            log_message("WiFi baglantisi kurulamadi. Veri datalog.csv'ye kaydediliyor.");
            time_t now_epoch = 0;
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) { now_epoch = mktime(&timeinfo); }
            saveSensorDataToFile(temperature_c, current_value_int, now_epoch, -1);
            accumulatedSleepSeconds = 0;
        }
    } else {
        log_debug("Veri gonderme zamani gelmedi. Enerji tasarrufu yapildi.");
    }

    log_message("--- Islemler Tamamlandi ---");
    
    long remainingSeconds = currentTargetTotalPeriodSeconds - accumulatedSleepSeconds;
    sprintf(logBuffer, "Deep-sleep baslatiliyor (%d sn)... Sonraki gonderime yaklasik %ld sn kaldi.", MINUTE_SLEEP_SECONDS, remainingSeconds > 0 ? remainingSeconds : 0);
    log_debug(logBuffer);

    esp_sleep_enable_timer_wakeup(MINUTE_SLEEP_SECONDS * uS_TO_S_FACTOR);
    Serial.flush();
    esp_deep_sleep_start();
}
*/

void setup() {
  
    // --- 1. Başlangıç Kurulumları ---
    Serial.begin(115200);
    Serial.setDebugOutput(false);
    delay(100);
    log_message("\n\n--- Cihaz Baslatiliyor ---");
    
    sprintf(logBuffer, "Cihaz Firmware Versiyonu: %s", FIRMWARE_VERSION);
    log_message(logBuffer);

    if (!LittleFS.begin()) {
        log_message("LittleFS baslatilamadi, formatlaniyor...");
        LittleFS.format();
        LittleFS.begin();
    }
    log_debug("LittleFS baslatildi.");
    debugMode = LittleFS.exists("/debug_mode.on");
    loadUdpConfig();

    WiFi.mode(WIFI_AP_STA);
    delay(100);
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    deviceID = mac;
    log_message("Cihaz MAC Adresi: " + deviceID);
    log_message("Cihaz " + String(++bootCount) + ". kez baslatildi.");

    ap_ssid_to_use = loadApSsidFromFile();
    if (ap_ssid_to_use.length() == 0) {
        ap_ssid_to_use = "FAYDAM_" + deviceID;
        saveApSsidToFile(ap_ssid_to_use);
        log_message("AP SSID olusturuldu ve kaydedildi: " + ap_ssid_to_use);
    } else {
        log_debug("Kayitli AP SSID kullaniliyor: " + ap_ssid_to_use);
    }
    
    loadWiFiCredentials();

    // --- 2. Mod Belirleme ---
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool isFirstBoot = (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED);

    if (isFirstBoot) { 
        log_debug("Cihaz ilk kez calisiyor (power-on/reset).");
        
        inAlarmState = false;
        accumulatedSleepSeconds = 0;
        otaCheckCounter = 0;
        shouldCheckOTA = true;
        lastValidTemperatureC = DEVICE_DISCONNECTED_C;
        lastValidBatteryMV100 = 0;
        consecutiveInvalidTempReadings = 0;
        consecutiveInvalidBatteryReadings = 0;
        tempSensorFaultSkipCounter = 0;
        battSensorFaultSkipCounter = 0;
        
        if (Serial) {
            log_debug("Seri baglanti algilandi. Komut icin 10 saniye bekleniyor...");
            unsigned long serialWaitStartTime = millis();
            while (millis() - serialWaitStartTime < 10000) {
                if (Serial.available() > 0) {
                    inConfigMode = true;
                    break; 
                }
                delay(50);
            }
        }
        
        if (inConfigMode) {
            log_message("Komut alindi. Yapilandirma modu aktif.");
            return;
        }

        log_debug("Zaman asimi. Otonom calisma baslatiliyor.");

        pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
        bool button_pressed_for_long = (digitalRead(CONFIG_BUTTON_PIN) == LOW);
        
        if (button_pressed_for_long || saved_ssid.length() == 0) {
            log_message("Wi-Fi bilgisi yok veya manuel kurulum tetiklendi. Web sunucusu baslatiliyor...");
            clearWiFiCredentials();
            startWebServer();
        }
        
        shouldSendMainData = true;

    } else { // Deep-sleep'ten uyandıysa
        accumulatedSleepSeconds += MINUTE_SLEEP_SECONDS;
        sprintf(logBuffer, "Deep-sleep'ten uyanildi. Gecen sure: %lu sn. Alarm Durumu: %s", accumulatedSleepSeconds, inAlarmState ? "EVET" : "HAYIR");
        log_debug(logBuffer);
        if (tempSensorFaultSkipCounter > 0) tempSensorFaultSkipCounter--;
        if (battSensorFaultSkipCounter > 0) battSensorFaultSkipCounter--;
    }

    // --- 3. Otonom Çalışma Mantığı ---
    log_debug("Otonom calisma dongusu basladi.");
    
    // <<< DEĞİŞİKLİK: SENSÖR OKUMA VE ALARM KONTROLÜ ÖNE ALINDI >>>
    sensors.begin();
    if (tempSensorFaultSkipCounter > 0) {
       log_debug("Sicaklik sensoru ariza durumunda, okuma atlaniyor.");
       temperature_c = lastValidTemperatureC;
    } else {
       sensors.requestTemperatures();
       delay(750);
       temperature_c = sensors.getTempCByIndex(0);
       if (temperature_c == DEVICE_DISCONNECTED_C || temperature_c < MIN_PLAUSIBLE_TEMP_C || temperature_c > MAX_PLAUSIBLE_TEMP_C) {
           consecutiveInvalidTempReadings++;
           temperature_c = lastValidTemperatureC;
       } else {
           lastValidTemperatureC = temperature_c; 
           consecutiveInvalidTempReadings = 0; 
       }
       if (consecutiveInvalidTempReadings >= MAX_CONSECUTIVE_INVALID_READINGS) {
           log_message("!!! KRITIK UYARI: Sicaklik sensoru hatali okunuyor! 10 dongu atlanacak.");
           tempSensorFaultSkipCounter = SENSOR_FAULT_SKIP_CYCLES;
       }
    }
    if (battSensorFaultSkipCounter > 0) {
       log_debug("Pil sensoru ariza durumunda, okuma atlaniyor.");
       current_value_int = lastValidBatteryMV100;
    } else {
       const int numReadings = 20;
       long sumRawAnalogValue = 0; 
       for (int i = 0; i < numReadings; i++) { sumRawAnalogValue += analogRead(ANALOG_INPUT_PIN); delay(2); }
       int raw_analog_value_avg = sumRawAnalogValue / numReadings;
       float battery_voltage_float = raw_analog_value_avg * BATTERY_VOLTAGE_TO_MV_FACTOR;
       int calculated_current_value_int = static_cast<int>(battery_voltage_float * 100);
       if (calculated_current_value_int < MIN_PLAUSIBLE_BATT_MV100 || calculated_current_value_int > MAX_PLAUSIBLE_BATT_MV100) {
           consecutiveInvalidBatteryReadings++;
           if (lastValidBatteryMV100 != 0) { 
               current_value_int = lastValidBatteryMV100;
           } else {
               current_value_int = calculated_current_value_int; 
           }
       } else {
           current_value_int = calculated_current_value_int;
           lastValidBatteryMV100 = current_value_int;         
           consecutiveInvalidBatteryReadings = 0; 
       }
       if (consecutiveInvalidBatteryReadings >= MAX_CONSECUTIVE_INVALID_READINGS) {
           log_message("!!! KRITIK UYARI: Pil sensoru hatali okunuyor! 10 dongu atlanacak.");
           battSensorFaultSkipCounter = SENSOR_FAULT_SKIP_CYCLES;
       }
    }
    bool isTemperatureAlarmTriggered = false;
    if (temperature_c != DEVICE_DISCONNECTED_C) {
        if (temperature_c < TLow_threshold || temperature_c > THigh_threshold) {
            isTemperatureAlarmTriggered = true;
        }
    }

    // Gönderme periyodunu ve kararını, güncel sensör verilerine göre belirle
    long currentTargetTotalPeriodSeconds = inAlarmState ? ALARM_TOTAL_PERIOD_SECONDS : NORMAL_TOTAL_PERIOD_SECONDS;
    if (!shouldSendMainData) {
        if (accumulatedSleepSeconds >= currentTargetTotalPeriodSeconds) {
            shouldSendMainData = true;
        }
    }
    if (isTemperatureAlarmTriggered && !inAlarmState) {
        log_message("!!! ALARM BASLANGICI: Sicaklik esik disina cikti!");
        shouldSendMainData = true; inAlarmState = true; accumulatedSleepSeconds = 0;
    } else if (!isTemperatureAlarmTriggered && inAlarmState) {
        log_message("--- ALARM SONA ERDI: Sicaklik normale dondu. ---");
        shouldSendMainData = true; inAlarmState = false; accumulatedSleepSeconds = 0;
    }

    // --- BİRLEŞTİRİLMİŞ AĞ İŞLEMLERİ BLOĞU ---
    if (shouldSendMainData) {
        log_message("Gonderim zamani geldi. Ag islemleri baslatiliyor...");
        if (connectToWiFi()) {
            if (isFirstBoot) {
                sendGateData();
            }
            if (shouldCheckOTA) {
                checkAndDoUpdate();
            }
            sendMainData();
            if (WiFi.status() == WL_CONNECTED) {
                WiFi.disconnect(true);
                WiFi.mode(WIFI_OFF);
                log_debug("Tum ag islemleri bitti, WiFi kapatildi.");
            }
        } else {
            log_message("WiFi baglantisi kurulamadi. Veri datalog.csv'ye kaydediliyor.");
            time_t now_epoch = 0;
            // ... (veri kaydetme kodları) ...
            accumulatedSleepSeconds = 0;
        }
    } else {
        log_debug("Veri gonderme zamani gelmedi. Enerji tasarrufu yapildi.");
    }

    log_message("--- Islemler Tamamlandi ---");

    long remainingSeconds = currentTargetTotalPeriodSeconds - accumulatedSleepSeconds;
    sprintf(logBuffer, "Deep-sleep baslatiliyor (%d sn)... Sonraki gonderime yaklasik %ld sn kaldi.", MINUTE_SLEEP_SECONDS, remainingSeconds > 0 ? remainingSeconds : 0);
    log_debug(logBuffer);
    
    esp_sleep_enable_timer_wakeup(MINUTE_SLEEP_SECONDS * uS_TO_S_FACTOR);
    Serial.flush();
    esp_deep_sleep_start();
}
  
void loop() {
    if (inConfigMode) {
        // Yapılandırma modundaysak sürekli seri portu dinle ve işle.
        processSerialCommands();
    }
    delay(10);
    // Normal modda cihaz deep-sleep'te olacağı için bu döngü çalışmaz.
}

// --- Python Aracı için JSON Komut İşleyicileri ---
void handleGetDeviceInfo() {
    StaticJsonDocument<256> doc;
    doc["type"] = "DEVICE_INFO";
    doc["mac_address"] = deviceID;
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["ip_address"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "N/A";
    
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    Serial.println(jsonResponse);
}

void handleGetWifiSettings() {
    StaticJsonDocument<256> doc;
    doc["cmd"] = "GET_WIFI_SETTINGS_RESPONSE";
    doc["status"] = "OK";
    doc["ssid"] = saved_ssid;
    doc["password"] = saved_password;
    
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    Serial.println(jsonResponse);
}

void handleSetWifi(JsonDocument& doc) {
    const char* ssid = doc["ssid"];
    const char* password = doc["password"];
    
    if (ssid) {
        saveWiFiCredentials(String(ssid), String(password));
        loadWiFiCredentials(); // Değişkenleri güncelle
        Serial.println("{\"status\":\"OK\", \"message\":\"Wi-Fi credentials saved.\"}");
        delay(1000);
        ESP.restart(); // Ayarların geçerli olması için yeniden başlat
    } else {
        Serial.println("{\"status\":\"ERROR\", \"message\":\"SSID not provided.\"}");
    }
}

void handleScanWifi() {
    log_message("Wi-Fi taramasi isteniyor...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    log_message("Aglar taraniyor, bu islem birkac saniye surebilir...");
    int n = WiFi.scanNetworks();
    log_message(String(n) + " adet ag bulundu.");
    DynamicJsonDocument doc(2048);
    doc["cmd"] = "SCAN_RESULTS";
    doc["status"] = "completed";
    JsonArray networks = doc.createNestedArray("networks");
    if (n > 0) {
        for (int i = 0; i < n; ++i) {
            JsonObject network = networks.createNestedObject();
            network["ssid"] = WiFi.SSID(i);
            network["rssi"] = WiFi.RSSI(i);
        }
    }
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    Serial.println(jsonResponse);
    log_message("Tarama sonuclari Python aracina gonderildi.");
}

void handleClearWifiSettings() {
    clearWiFiCredentials();
    Serial.println("{\"status\":\"OK\", \"message\":\"Wi-Fi settings cleared.\"}");
    delay(1000);
    ESP.restart(); // Temizlenmiş ayarlarla başlaması için yeniden başlat
}

void handleWifiTest() {
    log_message("Wi-Fi bağlantı testi başlatılıyor...");
    
    if (saved_ssid.length() == 0) {
        log_message("Test için kayıtlı Wi-Fi bilgisi bulunamadı.");
        // String() dönüşümü ile uyumluluk artırıldı
        Serial.println(String("{\"type\":\"WIFI_TEST_RESULT\", \"status\":\"FAILED\", \"message\":\"Kayıtlı SSID Yok\"}"));
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(saved_ssid.c_str(), saved_password.c_str());
    log_message("Ağa bağlanmaya çalışılıyor: " + saved_ssid);

    wl_status_t status = (wl_status_t)WiFi.waitForConnectResult(15000);

    StaticJsonDocument<256> doc;
    doc["type"] = "WIFI_TEST_RESULT";

    if (status == WL_CONNECTED) {
        log_message("Bağlantı testi başarılı.");
        doc["status"] = "SUCCESS";
        doc["ip"] = WiFi.localIP().toString();
        doc["rssi"] = WiFi.RSSI();
    } else {
        log_message("Bağlantı testi başarısız. Durum kodu: " + String(status));
        doc["status"] = "FAILED";
        
        String message = "Bilinmeyen Hata";
        // WL_WRONG_PASSWORD yerine daha uyumlu olan WL_CONNECT_FAILED kullanılır.
        // Cihaz zaten yanlış parola durumunda da bu hatayı verir.
        if (status == WL_NO_SSID_AVAIL) message = "Ağ Bulunamadı";
        else if (status == WL_CONNECT_FAILED) message = "Bağlantı Kurulamadı veya Parola Yanlış";
        else if (status == WL_IDLE_STATUS) message = "Zaman Aşımı";
        
        doc["message"] = message;
    }
    
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    Serial.println(jsonResponse);

    WiFi.disconnect(true);
    log_message("Bağlantı testi tamamlandı, Wi-Fi kapatıldı.");
}

void loadUdpConfig() {
    if (LittleFS.exists(UDP_CONFIG_FILE)) {
        File configFile = LittleFS.open(UDP_CONFIG_FILE, "r");
        if (configFile) {
            DynamicJsonDocument doc(128);
            if (deserializeJson(doc, configFile) == DeserializationError::Ok) {
                remoteLogHost = doc["ip"].as<String>();
                remoteLogPort = doc["port"].as<int>();
                // Sadece debug modunda logla
                log_debug("UDP ayarlari dosyadan yuklendi: " + remoteLogHost + ":" + String(remoteLogPort));
            }
            configFile.close();
        }
    } else {
        log_debug("UDP ayar dosyasi bulunamadi, varsayilanlar kullaniliyor.");
    }
}

void saveUdpConfig() {
    DynamicJsonDocument doc(128);
    doc["ip"] = remoteLogHost;
    doc["port"] = remoteLogPort;
    File configFile = LittleFS.open(UDP_CONFIG_FILE, "w");
    if (configFile) {
        serializeJson(doc, configFile);
        configFile.close();
        // Başarılı kaydı sadece debug modunda logla
        log_debug("UDP ayarlari dosyaya kaydedildi.");
    } else {
        // Hatayı her zaman logla
        log_message("HATA: UDP ayar dosyasi olusturulamadi.");
    }
}

void handleGetUdpSettings() {
    StaticJsonDocument<128> doc;
    doc["cmd"] = "GET_UDP_SETTINGS_RESPONSE";
    doc["status"] = "OK";
    doc["ip"] = remoteLogHost;
    doc["port"] = remoteLogPort;
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    Serial.println(jsonResponse);
}

void handleSetUdpSettings(JsonDocument& doc) {
    if (doc.containsKey("ip") && doc.containsKey("port")) {
        remoteLogHost = doc["ip"].as<String>();
        remoteLogPort = doc["port"].as<int>();
        saveUdpConfig(); // Ayarları dosyaya kaydet
        Serial.println(String("{\"status\":\"OK\", \"message\":\"UDP settings saved.\"}"));
    } else {
        Serial.println(String("{\"status\":\"ERROR\", \"message\":\"IP or Port not provided.\"}"));
    }
}

// Gelen seri komutları okur, JSON olarak ayrıştırır ve ilgili fonksiyonu çağırır.
void processSerialCommands() {
    if (Serial.available() > 0) {
        String commandJson = Serial.readStringUntil('\n');
        commandJson.trim();
        log_message("Python Aracindan komut alindi: " + commandJson);

        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, commandJson);

        if (error) {
            log_message("JSON ayrıştırma hatası: " + String(error.c_str()));
            Serial.println("{\"status\":\"ERROR\", \"message\":\"Invalid JSON\"}");
            return;
        }

        const char* cmd = doc["cmd"];
        if (cmd) {
            String command = String(cmd);
            if (command.equals("GET_DEVICE_INFO")) {
                handleGetDeviceInfo();
            } else if (command.equals("GET_WIFI_SETTINGS")) {
                handleGetWifiSettings();
            } else if (command.equals("SET_WIFI")) {
                handleSetWifi(doc);
            } else if (command.equals("SCAN_WIFI")) {
                handleScanWifi(); 
            } else if (command.equals("CONNECT_WIFI_TEST")) {
                handleWifiTest();
            } else if (command.equals("CLEAR_WIFI_SETTINGS")) {
                handleClearWifiSettings();
            } else if (command.equals("ENTER_CONFIG_MODE")) {
                // Zaten bu moddayız, sadece bir onay gönderelim.
                Serial.println("{\"status\":\"OK\", \"message\":\"Already in config mode.\"}");
            } else if (command.equals("START_NORMAL_MODE")) {
                log_message("Normal çalışma moduna geçiliyor, cihaz yeniden başlatılacak.");
                Serial.println(String("{\"status\":\"OK\", \"message\":\"Rebooting to normal mode.\"}"));
                delay(200); // Seri mesajın gönderilmesi için kısa bir bekleme
                ESP.restart();
            } else if (command.equals("GET_DEBUG_MODE")) {
                Serial.println(String("{\"cmd\":\"GET_DEBUG_MODE_RESPONSE\", \"enabled\":") + (debugMode ? "true" : "false") + "}");
            } else if (command.equals("SET_DEBUG_MODE")) {
                bool enabled = doc["enabled"];
                if (enabled) {
                    File file = LittleFS.open("/debug_mode.on", "w");
                    if(file) file.close();
                } else {
                    LittleFS.remove("/debug_mode.on");
                }
                debugMode = enabled;
                Serial.println(String("{\"status\":\"OK\", \"message\":\"Debug mode set to ") + (debugMode ? "ON" : "OFF") + "\"}");    
            } else if (command.equals("GET_UDP_SETTINGS")) {
                handleGetUdpSettings();
            } else if (command.equals("SET_UDP_SETTINGS")) {
                handleSetUdpSettings(doc);
            }
            
            // Not: OTA güncellemesi gibi daha karmaşık komutlar buraya eklenebilir.
        }
    }
}
