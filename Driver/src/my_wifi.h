#ifndef MY_WIFI_H
#define MY_WIFI_H

#include <WiFi.h>

const char *AP_ssid = "ArRobotDriver";
const char *AP_password = "89106472476";

int flag_wifi = 0; // Флаг вкючена ли стеть вайфай на котроллере

// Настройки для офисной сети
const char *Net_ssid = "Cons24";
const char *Net_password = "89106472476";
IPAddress staticIP(192, 168, 0, 41);
IPAddress gateway(192, 168, 0, 1);
IPAddress dns1(192, 168, 0, 1);

// //Настроки для домашней сети
// const char *Net_ssid = "MVV-24";
// const char *Net_password = "89106472476";
// IPAddress staticIP(192, 168, 100, 40);
// IPAddress gateway(192, 168, 100, 1);
// IPAddress dns1(192, 168, 100, 1);

//Общие настройки
IPAddress subnet(255, 255, 255, 0);
IPAddress dns2(8, 8, 8, 8);

/*
* WiFi Events Table

0  SYSTEM_EVENT_WIFI_READY               < ESP32 WiFi ready
1  SYSTEM_EVENT_SCAN_DONE                < ESP32 finish scanning AP
2  SYSTEM_EVENT_STA_START                < ESP32 station start
3  SYSTEM_EVENT_STA_STOP                 < ESP32 station stop
4  SYSTEM_EVENT_STA_CONNECTED            < ESP32 station connected to AP
5  SYSTEM_EVENT_STA_DISCONNECTED         < ESP32 station disconnected from AP
6  SYSTEM_EVENT_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by ESP32 station changed
7  SYSTEM_EVENT_STA_GOT_IP               < ESP32 station got IP from connected AP
8  SYSTEM_EVENT_STA_LOST_IP              < ESP32 station lost IP and the IP is reset to 0
9  SYSTEM_EVENT_STA_WPS_ER_SUCCESS       < ESP32 station wps succeeds in enrollee mode
10 SYSTEM_EVENT_STA_WPS_ER_FAILED        < ESP32 station wps fails in enrollee mode
11 SYSTEM_EVENT_STA_WPS_ER_TIMEOUT       < ESP32 station wps timeout in enrollee mode
12 SYSTEM_EVENT_STA_WPS_ER_PIN           < ESP32 station wps pin code in enrollee mode
13 SYSTEM_EVENT_AP_START                 < ESP32 soft-AP start
14 SYSTEM_EVENT_AP_STOP                  < ESP32 soft-AP stop
15 SYSTEM_EVENT_AP_STACONNECTED          < a station connected to ESP32 soft-AP
16 SYSTEM_EVENT_AP_STADISCONNECTED       < a station disconnected from ESP32 soft-AP
17 SYSTEM_EVENT_AP_STAIPASSIGNED         < ESP32 soft-AP assign an IP to a connected station
18 SYSTEM_EVENT_AP_PROBEREQRECVED        < Receive probe request packet in soft-AP interface
19 SYSTEM_EVENT_GOT_IP6                  < ESP32 station or ap or ethernet interface v6IP addr is preferred
20 SYSTEM_EVENT_ETH_START                < ESP32 ethernet start
21 SYSTEM_EVENT_ETH_STOP                 < ESP32 ethernet stop
22 SYSTEM_EVENT_ETH_CONNECTED            < ESP32 ethernet phy link up
23 SYSTEM_EVENT_ETH_DISCONNECTED         < ESP32 ethernet phy link down
24 SYSTEM_EVENT_ETH_GOT_IP               < ESP32 ethernet got IP from connected AP
25 SYSTEM_EVENT_MAX
*/
//Функция обработчика события Подключение к домашней сети WiFi
void OnWiFiEvent(WiFiEvent_t event)
{
    Serial.println("");
    Serial.printf("[WiFi-event] event: %d -> ", event);
    switch (event)
    {
    case SYSTEM_EVENT_WIFI_READY:
        Serial.println("WiFi interface ready");
        break;
    case SYSTEM_EVENT_SCAN_DONE:
        Serial.println("Completed scan for access points");
        break;
    case SYSTEM_EVENT_STA_START:
        Serial.println("WiFi client started");
        break;
    case SYSTEM_EVENT_STA_STOP:
        Serial.println("WiFi clients stopped");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        Serial.println("Connected to access point WiFi Network");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("Disconnected from WiFi access point");
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        Serial.println("Authentication mode of access point has changed");
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("Obtained IP address");
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        Serial.println("Lost IP address and IP address is reset to 0");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
        Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
        Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
        Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
        break;
    case SYSTEM_EVENT_AP_START:
        Serial.println("ESP32 soft AP started");
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        Serial.println("Station connected to ESP32 soft AP");
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        Serial.println("Station disconnected from ESP32 soft AP");
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        Serial.println("18 - Receive probe request packet in soft-AP interface");
        break;
    default:
        break;
    }
}

//Функция запускающая точку доступа
void Start_WiFi_AP()
{
    WiFi.softAP(AP_ssid, AP_password); // Запускаем точку доступа
    delay(500);
    Serial.print("ESP32 IP as soft AP: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("***");
}
//Инициализация вайфай в нужных режимах
void init_WiFi()
{
    WiFi.onEvent(OnWiFiEvent); // Обьявляем обработчик по событию
    WiFi.disconnect();
    WiFi.softAPdisconnect(true);

    //WiFi.mode(WIFI_MODE_STA);
    WiFi.mode(WIFI_MODE_APSTA); // Устанавливаем режим работы и точка доступа и подключение к доманшему WiFi
    Start_WiFi_AP();            // Заспускаем точку доступа
    flag_wifi = 1;              // Отмечаем что включили сеть
    // WiFi.config(staticIP, gateway, subnet, dns1, dns2); //Задаем конфигурацию параметров с которыми будем подключаться к существующей сети WiFi
    // Start_WiFi_Net();                                   // Подключаемся к сети WiFi
}


// //Функция подключения к существующей сети WiFi
// void Start_WiFi_Net()
// {

//     Serial.println(" Connecting to WiFi... SSID= " + String(Net_ssid) + " Pass= " + String(Net_password));
//     WiFi.begin(Net_ssid, Net_password); // Подключаемся к домашней WiFi
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(100);
//         Serial.println(millis());
//         Serial.print(".");

//         //WiFi.begin(Net_ssid, Net_password);
//     }
//     delay(500);
//     Serial.println("");
//     Serial.println("---"); // Печатаем с каким параметрами подключились
//     Serial.print("Local IP: ");
//     Serial.println(WiFi.localIP());
//     Serial.print("MAC Address: ");
//     Serial.println(WiFi.macAddress());
//     Serial.print("Subnet Mask: ");
//     Serial.println(WiFi.subnetMask());
//     Serial.print("Gateway IP: ");
//     Serial.println(WiFi.gatewayIP());
//     Serial.print("DNS 1: ");
//     Serial.println(WiFi.dnsIP(0));
//     Serial.print("DNS 2: ");
//     Serial.println(WiFi.dnsIP(1));
//     Serial.println("---");
//     delay(500);
// }



#endif