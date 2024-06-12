/* <<<< csi261M2 >>>> - Startover Version 3.Jan.2023

 esp_now (mac p2p): 2 Masters (M1,M2), 6 Sensors, 1 CSI Collector (M1)

    Based on esp-csi/examples/get-started/csi_send - ESPRESSIF
    @2022 Adolfo Bauchspiess - ITIV/KIT

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "nvs_flash.h"
#include "rom/ets_sys.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#define configTICK_RATE_HZ                  1000
#define portTICK_RATE_MS = 1000/configTICK_RATE_HZ;


#define CONFIG_LESS_INTERFERENCE_CHANNEL    11
#define CONFIG_SEND_FREQUENCY               1000
//#define LED 23

typedef struct struct_bc {  // Broadcast Msg Received
    uint8_t  idMaster; // origin of bcMsg
    uint8_t  debugFlag; // 0b0000 0001 dbug(M1), 0b0000 0010 dbug(M2), 0b1111 1100 dbug(S:6)
    uint16_t  delta;
    uint32_t bcCount;
} struct_bc;
struct_bc bcM1, bcM2;

//uint32_t LbcCount=1;
uint32_t Lbc1=0;

struct timeval tv_now;
int64_t time_us;
int64_t time_st; // start of running (or reset by M1 "0" gCount)
int64_t time_cb;  // time elapsed since send_cb
int32_t dtime;

int32_t ts2, tsS;


int64_t time_bc;

int64_t tStart=0;
int32_t t2start=0;
int32_t lt2start=0; // Save latest t2start in order to display the difference between gCount (CSI arrivals)

uint8_t myMAC[6];
uint8_t sendNow = 0; // Received req. from Master; CSI ready in CallBack function

static const char *TAG = "csi_send";
//static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0xc0, 0x49, 0xef, 0x4b, 0xa2, 0x98}; // "2"
/*
esp_now_peer_info_t peerM1 = {
    .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
    .ifidx     = WIFI_IF_STA,
    .encrypt   = false,
    .peer_addr = {0xc4, 0xde, 0xe2, 0xc0, 0x10, 0xc8}, // "8"
};
*/
esp_now_peer_info_t peerBC = {
    .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
    .ifidx     = WIFI_IF_STA,
    .encrypt   = false,
    .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
};

#define DEC_PLACE_MULT 1000

void print_dtime() {
    gettimeofday(&tv_now, NULL);
    time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    dtime = time_us-time_cb;  // dtime in us

//    ets_printf("dtime(us)=%u ",dtime);
    ets_printf(" dt(ms): %d.%02u",  dtime / DEC_PLACE_MULT, abs(dtime) % DEC_PLACE_MULT);
}
void print_stime() {
    gettimeofday(&tv_now, NULL);
    time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    dtime = time_us-time_st;  // dtime in us
    time_cb = time_us;

//    ets_printf("dtime(us)=%u ",dtime);
    ets_printf(" t(ms): %d.%02u",  dtime / DEC_PLACE_MULT, abs(dtime) % DEC_PLACE_MULT);
}

uint8_t na4MAC(uint8_t *mac) {
    uint8_t name;
    uint8_t mac4=mac[4], mac5=mac[5];
    
    if (mac5==0x14) name=1;
    else if (mac5==0x98) name=2; // ITIV
    else if (mac5==0x2c) name=3;
    else if (mac5==0x78) name=4;
    else if (mac5==0x1c) name=5;
    else if (mac5==0xb8) name=6;
    else if (mac5==0x90) {name=7; if (mac4==0xb4) name=10;}
    else if (mac5==0xc8) name=8;
    
    else if (mac5==0x94) name=11; // GDH
    else if (mac5==0x60) {name=12;  if (mac4==0x62) name=9;}
    else if (mac5==0x30) name=13;
    else if (mac5==0x7c) name=14;
    else if (mac5==0xb0) name=15;
    else name=9;
    return name;
}



static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, WIFI_PHY_RATE_MCS0_SGI));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}

/* It is of NO USE - receiver should process BC!!
 ************************************************
static void send_ask_csi_cb(const uint8_t *mac, esp_now_send_status_t status) {
    ets_printf("\n\rASK_CSI_CB() gCount=%u, bcCount=%u, status=%u\n\r", bcMsg.gCount,bcMsg.bcCount,status);
}
*/
/*****************/
static void now_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    
    ets_printf("\n\r\033[0;33m     now_send_cb\033[0m");
}
/*********************/

static void now_recv_cb(const uint8_t *mac, const uint8_t *incoming, int len) {

    memcpy(&bcM1,incoming, sizeof(bcM1));
    
    if (bcM1.debugFlag & 0b010){
        ets_printf("\n\r\033[0;36m now_recv_cb M%u, bcCount:%u",bcM1.idMaster, bcM1.bcCount);
        print_dtime();
        ets_printf("\033[0m");
    }
    
    if (bcM1.idMaster == 1) return;

    // keeps receiving now_cb from other sensor nodes - do not send CSI!!
    if (bcM1.idMaster == 136) {
  
        if (bcM1.bcCount == 0) bcM2.bcCount = 0;
        
        sendNow = 1;
            // Enable Transmission of CSI to the master, and only to the master node, for this bcCounter value
        if (bcM1.debugFlag & 0b010){
            ets_printf(" sendNow:%u",sendNow);
        }
    }
}



static void now_csi_init()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));

//    ESP_ERROR_CHECK(esp_now_register_recv_cb(csi_rx_cb));
//    ESP_ERROR_CHECK(esp_now_register_send_cb(send_ask_csi_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(now_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(now_recv_cb));
    
//    ESP_ERROR_CHECK(esp_wifi_set_csi(false));
}


void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    wifi_init();
    now_csi_init();
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT); //pinMode(LED,OUTPUT)
    gpio_reset_pin(GPIO_NUM_23);
    
    // peer register the Master8 to receive BroadCasting and itself to Broacast
    ESP_ERROR_CHECK(esp_now_add_peer(&peerBC));    // Auto register??
    //    ESP_ERROR_CHECK(esp_now_add_peer(&peerM1));
    
    ESP_LOGI(TAG, "================ CSI SEND ================");
    ESP_LOGI(TAG, "wifi_channel: %d, send_frequency: %d, mac: " MACSTR,
             CONFIG_LESS_INTERFERENCE_CHANNEL, CONFIG_SEND_FREQUENCY, MAC2STR(CONFIG_CSI_SEND_MAC));
    
    bcM1.bcCount=0; // BroadCast counter - count CSI acquisition cycles
    bcM2.bcCount=0; // BroadCast counter - count CSI acquisition cycles
    
    ESP_ERROR_CHECK(esp_base_mac_addr_get(myMAC));
    if (myMAC[5]==0xc8) bcM1.idMaster=1;        // "8" Red Label on ESP32
    else if (myMAC[5]==0x98) bcM1.idMaster=2;   // "2"
    else bcM1.idMaster=3;                       // "1" myMAC[5]==0x14
    
    //    bool ledOn=true;
/*
     for(int i=0 ; i<=255 ; i++ ) //ASCII values ranges from 0-255
    {
        printf("ASCII value of character %c = %d\n", i, i);
    }
*/
    
    bcM2.idMaster = 2;
    
    // MAIN LOOP
    for ( ; ; ) { // infinite loop
        if (sendNow) {
            sendNow = 0;   // send was OK!!
            
            gpio_set_level(GPIO_NUM_23,0);
            
            ets_printf("\n\r\033[0;34mmain(%u) START",bcM2.idMaster);
            print_stime();
            time_bc = time_cb;

            bcM2.bcCount=bcM1.bcCount;  // better then bcM2.bcCount++ discard old

            if (bcM1.bcCount == 0) {  // Start over - from M1
                print_stime();
                time_st = time_us;
                ts2 = time_us;
                tsS = time_us;
                bcM2.delta=bcM1.delta;
                bcM2.debugFlag=bcM1.debugFlag;
            }

            // Wait 1 ms so that the sensors can save the CSI asked by M1
            // First delta reserver for BC1 + BC2
            vTaskDelay(1);   // Important - otherwise data mess: delta, seqSens
            // BC to Sensors - acquire CSI 2
            esp_err_t ret = esp_now_send(peerBC.peer_addr, (uint8_t *) &bcM2, sizeof(struct_bc));
            if(ret != ESP_OK) ESP_LOGW(TAG, "<%s> ESP-NOW send error", esp_err_to_name(ret));
            
  //          vTaskDelay(1);

            ets_printf("\n\r\033[0;34mmain(%u) END bcC1:%u, delta:%u, dFlag:%u", bcM2.idMaster, bcM1.bcCount,bcM1.delta,bcM1.debugFlag);
            print_dtime();
            ets_printf("\n\r\033[0m");
            
            gpio_set_level(GPIO_NUM_23,1);
        }
    }
}
