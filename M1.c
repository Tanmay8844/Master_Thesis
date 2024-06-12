/*  <<<< csi261M1 >>>> Startover Version 3.Jan.2023

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


#define CONFIG_LESS_INTERFERENCE_CHANNEL    11
#define CONFIG_SEND_FREQUENCY               1000

//#define configTICK_RATE_HZ                  1000
/*  FREE_RTOS */
//#define portTICK_RATE_MS = 1000/configTICK_RATE_HZ;

// {myMsg, bcM1 and bcM2} once received can be separeted by the first field {3:15, 1, 2}
typedef struct struct_msg {     // Sensor Msg Sended
    uint8_t  idSens;
    uint32_t sCount;            // sensor Counter for CSI packages
    uint32_t bc1;
    uint32_t bc2;
    uint32_t tstamp1;           // timestamp of M1 CSI acquisition 时间截
    uint32_t tstamp2;           // timestamp of M2 CSI acquisition
    uint32_t tstampS;           // send to M1 timestamp (32 bit in us)
    int8_t   RSSI1;             // from Master 1
    int8_t   RSSI2;             // from Master 2
    uint8_t  retry;
    int8_t  data[216];         // Each CFR(Channel Freq. Resp.) registers as 2 signed chars
                                // Im1 Re1
} struct_msg;
struct_msg myMsg;

uint32_t idCount[8];           // 改动counter for each sensor received data (max 15 Devices)

typedef struct struct_bc {      // Broadcast Msg Received
    uint8_t   idMaster;         // origin of bcMsg
    uint8_t   debugFlag;        // 0b0000 0001 dbug(M1), 0b0000 0010 dbug(M2), 0b1111 1100 dbug(S:6)
    uint16_t  delta;
    uint32_t  bcCount;
} struct_bc;
struct_bc bcM1, bcM2;

uint32_t Lbc2=0;
uint32_t gCount;

struct timeval tv_now;
int32_t time_us;
int32_t time_bc;
int64_t time_st;                // start of running (or reset by M1 "0" gCount)
int64_t time_cb;                // time elapsed since send_cb
int32_t dtime;

int32_t ts2, tsS;

int32_t tStart=0;
int32_t t2start=0;
int32_t lt2start=0; // Save latest t2start in order to display the difference between gCount (CSI arrivals)

int32_t last_time[15]; //us time differe


uint8_t myMAC[6];

static const char *TAG = "csi_send";
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0xc4, 0xde, 0xe2, 0xc0, 0x10, 0xc8}; // "8"


#define DEC_PLACE_MULT 1000

/* ***********************************************
   print time since start of a new cycle, 'dtime':
   ***********************************************/
void print_dtime() {
    gettimeofday(&tv_now, NULL);
    time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    dtime = time_us-time_cb;  // dtime in us
    ets_printf(" dt(ms): %d.%02u",  dtime / DEC_PLACE_MULT, abs(dtime) % DEC_PLACE_MULT);
}

/* ***********************************************
   print time since new run (reset ESP32)
   resets time count, time_cb = time_us
   ***********************************************/
void print_stime() {
    gettimeofday(&tv_now, NULL);
    time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    dtime = time_us-time_st;  // dtime in us
    time_cb = time_us;
    ets_printf(" t(ms): %d.%02u",  dtime / DEC_PLACE_MULT, abs(dtime) % DEC_PLACE_MULT);
}

/* ***********************************************
   Get Device names {idSensor | idMaster} from mac
   ***********************************************/
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


/* ***********************************************
 
   ***********************************************/
static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT40)); // non HT20 MHz !!!!!!!!
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, WIFI_PHY_RATE_MCS0_SGI));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}


/* ***********************************************
   Receives CSI from Sensors. ets_printf monitor | grep "CSI_DATA" > file.csv
   ***********************************************/
void csi_rx_cb(const uint8_t* mac, const uint8_t* incoming, int len)
{

    memcpy(&myMsg,incoming, sizeof(myMsg));

/* Must be commented when saving CSI to file.csv with grep >
 ***********************************************************
 if (bcM1.debugFlag & 0b001) {
        ets_printf("\n\r\033[0;36m    CSI_RX_cb() idS%u sC=%u",myMsg.idSens, myMsg.sCount);
        print_dtime(); // Blue
        ets_printf("\033[0m");
    }
*/
    
    if (myMsg.idSens == 1 || myMsg.idSens == 2 || myMsg.idSens == 136 ) {
//        ets_printf("=== but NOT from M2! Skip!!");  // See above comment
        return;
    }

    /*
    if (LmyMsg[myMsg.idSens-1] < bcM1.bcCount) {  // ******* myMsg.sCount LmyMsg == idCount
        
        if (myMsg.idSens == 2) return;
        
        if (myMsg.sCount == idCount[myMsg.idSens-1]) {  // return; ==> Loose data!
            if (myMsg.retry == 1) myMsg.retry = 3;      // Reshape myMsg.retry to sign both
            else myMsg.retry = 2;                       // "twin" CSI
        }
        
        //retry==1 => now_send_cb of idSens with "status != 0"
        if (myMsg.retry > 0) return; //retry=-2 || retry==3 => CSI already registered!!
        
        idCount[myMsg.idSens-1] = myMsg.sCount;  // store sCount from idSens
        LmyMsg[myMsg.idSens-1]++;
*/
    if (idCount[myMsg.idSens-1] < myMsg.sCount) {
        
        idCount[myMsg.idSens-1] = myMsg.sCount;  // store sCount from idSens (to discard already received myMsg and Old Sensor Values => )
        
        gCount++;
        // print labels in the 1st line
        if (gCount==1) {
            ESP_LOGI(TAG, "================ Received CSI ================");
            ets_printf("CSI_DATA: gC, idS, sC, sC_M, bc1, bc2, bcM1, retry, RSSI1, RSSI2, tCSI1_S, tCSI2_S, tdma_S, t_M(ms), dt_rxM, tin_cycle, [data]\n\r\n");
            tStart = time_us; tsS = myMsg.tstampS; ts2 = myMsg.tstamp2;
        }
        
        gettimeofday(&tv_now, NULL);
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        
        last_time[myMsg.idSens-1] = time_us;
        
        gettimeofday(&tv_now, NULL);
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        
        int32_t t2bc = time_us - time_bc;
        t2start = time_us - tStart;
        int32_t tdiff = t2start-lt2start;
        
        // print only relevant data
        ets_printf("CSI_DATA: %u,%u,%u,%u,%u,%u,%u,%u,%d,%d,",
            gCount, myMsg.idSens, myMsg.sCount, idCount[myMsg.idSens - 1], myMsg.bc1, myMsg.bc2, bcM1.bcCount, myMsg.retry, myMsg.RSSI1, myMsg.RSSI2);

        ets_printf("%d.%02u,%d.%02u,%d.%02u,%d.%02u,%d.%02u,%d.%02u,\"[%d",
            myMsg.tstamp1 / 1000, (myMsg.tstamp1) % 1000, myMsg.tstamp2 / 1000, (myMsg.tstamp2) % 1000,
            myMsg.tstampS / 1000, (myMsg.tstampS) % 1000, t2start / 1000, (t2start) % 1000,
            tdiff / 1000, (tdiff) % 1000, t2bc / 1000, (t2bc) % 1000, myMsg.data[0]);

        for (int i = 1; i < 216; i++) {
            ets_printf(",%d", myMsg.data[i]);
        }

        ets_printf("]\"\n\r\n");
    }
//    else ets_printf("\n\r\033[0;36m ===> DISCARD S%u ==> already registered!! idCount:%u, sCount:%u\033[0m",myMsg.idSens, idCount[myMsg.idSens-1], myMsg.sCount);
 }

//***********************************************
//***********************************************
/* It is of NO USE - receiver should process BC!!
 ************************************************/
static void send_ask_csi_cb(const uint8_t *mac, esp_now_send_status_t status) {
    
    if (bcM1.debugFlag & 0b001) {
//        ets_printf("\n\r  BC%u SEND_cb OK gCount=%u, bcCount=%u, status=%u", bcM1.idMaster, gCount,bcM1.bcCount,status);
//        print_dtime();
    }
}



/* ***********************************************
 
   ***********************************************/
static void now_csi_init()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));

    ESP_ERROR_CHECK(esp_now_register_recv_cb(csi_rx_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_ask_csi_cb));
}



/* ***********************************************
 
   ***********************************************/
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
    
    esp_now_peer_info_t peerBC = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
    // Its only necessary to peer register the Master as BroadCasting
    ESP_ERROR_CHECK(esp_now_add_peer(&peerBC));    // Auto register??
    
    ESP_LOGI(TAG, "================ CSI SEND ================");
    ESP_LOGI(TAG, "wifi_channel: %d, send_frequency: %d, mac: " MACSTR,
             CONFIG_LESS_INTERFERENCE_CHANNEL, CONFIG_SEND_FREQUENCY, MAC2STR(CONFIG_CSI_SEND_MAC));
    
    
    ESP_ERROR_CHECK(esp_base_mac_addr_get(myMAC));
    if (myMAC[5]==0xc8) bcM1.idMaster=1;        // "8" Red Label on ESP32
    else if (myMAC[5]==0x98) bcM1.idMaster=2;   // "2"
    else bcM1.idMaster=3;                       // "1" myMAC[5]==0x14
    
    // ets_printf() allways for 'grep "CSI_DATA" > file.csv'
    //    bcMsg.debugFlag = 0b001;  // M1 debug prints
    //    bcMsg.debugFlag = 0b010;  // M2 debug prints
    //    bcMsg.debugFlag = 0b100;  // Sensor debug prints
    //超时
    TickType_t openTimeout;
    TimeOut_t x_timeout;
    vTaskSetTimeOutState(&x_timeout);

    bcM1.debugFlag = 0b000;    // NO debug prints

// M1, M2, S3, S9   "csi221"
//#define DELTA  5     //  10 ms TDMA interval
//#define CYCLE  10    //  20 ms                NO DEBUG prints

// M1, M2, S3   "csi211"
//#define DELTA  5     //  50 ms TDMA interval
//#define CYCLE  5    //  100 ms                NO DEBUG prints
    
// M1, M2, S3, S9, S11, S12, S13, S15   "csi261"
#define DELTA  5  //  60 ms TDMA interval
#define CYCLE  40   //  500 ms                NO DEBUG prints

// M1, M2, S3, S9   "csi221"
//#define DELTA  5     //  50 ms TDMA interval
//#define CYCLE  10    //  100 ms                NO DEBUG prints
    
//    bcM1.debugFlag = 0b111;    // All debug prints//
//#define DELTA  7     //  60 ms TDMA interval
//#define CYCLE  100    //  600 ms                  with DEBUG prints

    
    
    bcM1.delta = DELTA; // Interval between aquisitions - 60 ms; Cycle = 6 x 60ms
    
    for (uint8_t i=0; i<15;i++) idCount[i]=0;
    
    openTimeout = 0; // start sending BC
    
    gCount=0;        // global experiment counter - count received CSI
    bcM1.bcCount=-1; // BroadCast counter - count CSI acquisition cycles
    
    // MAIN LOOP
    for (;;) { // infinite loop
        if(xTaskCheckForTimeOut(&x_timeout, &openTimeout) == pdTRUE) {
            
            bcM1.bcCount++;
            bcM1.idMaster = 1;
            if (bcM1.debugFlag & 0b001) {
                ets_printf("\n\n\r\033[0;34m MAIN%u START CYCLE gC=%u, bcC=%u, bcM1.delta:%u ",bcM1.idMaster, gCount,bcM1.bcCount,bcM1.delta);
                print_stime();
                time_bc = time_cb;
                print_dtime();
                ets_printf("\033[0m\n\r ");
            }
            else {  // Mark time without ets_print()
                    gettimeofday(&tv_now, NULL);
                    time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
                    dtime = time_us-time_st;  // dtime in us
                    time_cb = time_us;
                time_bc = time_cb;
                    dtime = time_us-time_cb;  // dtime in us
            }
                

            // BC to enable CSI acquisition
            esp_err_t ret1 = esp_now_send(peerBC.peer_addr, (uint8_t *) &bcM1, sizeof(struct_bc));
            if(ret1 != ESP_OK) ESP_LOGW(TAG, "<%s> ESP-NOW send error", esp_err_to_name(ret1));
            
                           /* *****************************************************************/
            vTaskDelay(1); // 1 tick(10 ms) ===> ALLOW next send(M136) to produce CSI on Sensors

            // BC to acquire CSI 1
            bcM1.idMaster = 136;
            ret1 = esp_now_send(peerBC.peer_addr, (uint8_t *) &bcM1, sizeof(struct_bc));
            if(ret1 != ESP_OK) ESP_LOGW(TAG, "<%s> ESP-NOW send error", esp_err_to_name(ret1));
    
            
            if (bcM1.debugFlag & 0b001) {
                ets_printf("\n\r\033[0;34mMAIN%u END CYCLE gC=%u, bcC=%u, bcM1.delta:%u ",bcM1.idMaster, gCount,bcM1.bcCount,bcM1.delta);
                print_dtime();
                ets_printf("\033[0m\n\r ");
            }
            lt2start = t2start;                         //每次循环结束时，lt2start 就会更新为上一次循环的 t2start 值，然后在下一次循环时，就可以用新的 t2start 值减去旧的 lt2start 值，得到时间差 tdiff。
            openTimeout = CYCLE;
        }
    }
}
