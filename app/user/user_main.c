/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "eagle_soc.h"
#include "driver/uart.h"  
#include "c_types.h"	
#include "user_config.h"
#include "os_type.h"	
#include "mem.h"
#include "espconn.h"
#include "ip_addr.h"
#include "user_interface.h"


#define MESSAGE_QUEUE_LEN 4 
#define	ESP8266_AP_SSID		"WiFi_OBDII"		// 创建的WIFI名
#define	ESP8266_AP_PASS		"11111111"		


#define		LED_ON				GPIO_OUTPUT_SET(GPIO_ID_PIN(4),0)		// LED亮
#define		LED_OFF				GPIO_OUTPUT_SET(GPIO_ID_PIN(4),1)

#define	HUMITURE_WIFI_LED_IO_MUX				PERIPHS_IO_MUX_GPIO2_U
#define	HUMITURE_WIFI_LED_IO_NUM				2
#define	HUMITURE_WIFI_LED_IO_FUNC				FUNC_GPIO2
ETSEvent *Point_task_1;
u8 F_LED = 0;
ETSTimer OS_Timer_1;
struct espconn ST_NetCon;

extern uint8 uartRxBuffer[256];

//void ICACHE_FLASH_ATTR ESP8266_NetCon_Init();

void ICACHE_FLASH_ATTR ESP8266_NetConTCP_Init();

void ICACHE_FLASH_ATTR 
LED_Init(void)
{
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,FUNC_GPIO4);
    //PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO2_U);
    GPIO_OUTPUT_SET(GPIO_ID_PIN(4),1);
}


void ICACHE_FLASH_ATTR 
OS_Timer_1_cb(void)
{   
     
   // struct ip_info ST_ESP8266_IP;
    system_soft_wdt_feed();
   // u8 ESP8266_IP[4];
   if(wifi_softap_get_station_num() != 0){
        
        //wifi_status_led_uninstall();
        //os_printf("set=0\n");
        GPIO_OUTPUT_SET(GPIO_ID_PIN(2),0);
   }
   else
   {    //os_printf("set=1\n");
        GPIO_OUTPUT_SET(GPIO_ID_PIN(2),1);
        //wifi_status_led_install(4, PERIPHS_IO_MUX_GPIO4_U,	FUNC_GPIO4);
   }
   
    //os_timer_disarm(&OS_Timer_1);
    //ESP8266_NetConTCP_Init();

}


void ICACHE_FLASH_ATTR 
OS_Timer_1_INIT(u32 time_ms, u8 time_repetitive)
{
    os_timer_disarm(&OS_Timer_1);	
	os_timer_setfn(&OS_Timer_1,(os_timer_func_t *)OS_Timer_1_cb, NULL);	// 设置定时器
	os_timer_arm(&OS_Timer_1, time_ms, time_repetitive);  
}


void ICACHE_FLASH_ATTR
ESP8266_AP_INIT()
{
    struct	ip_info	info;
    struct softap_config AP_Config; //结构体初始化
    wifi_set_opmode(0x02);
    os_memset(&AP_Config, 0, sizeof(struct softap_config));	// 开辟内存空间 写到flash
	os_strcpy(AP_Config.ssid,ESP8266_AP_SSID);		// 设置SSID(将字符串复制到ssid数组)
	os_strcpy(AP_Config.password,ESP8266_AP_PASS);	// 设置密码(将字符串复制到password数组)
	AP_Config.ssid_len=os_strlen(ESP8266_AP_SSID);	// 设置ssid长度(和SSID的长度一致)
	AP_Config.channel=1;                      		// 通道号1～13
	AP_Config.authmode=AUTH_OPEN;           	// 设置加密模式
	AP_Config.ssid_hidden=0;                  		// 不隐藏SSID
	AP_Config.max_connection=4;               		// 最大连接数
	AP_Config.beacon_interval=100;            		// 信标间隔时槽100～60000 ms

    wifi_softap_dhcps_stop();
    IP4_ADDR(&info.ip,	192,	168,	0,	10);
    IP4_ADDR(&info.gw,	192,	168,	0,	10);
    IP4_ADDR(&info.netmask,	255,	255,	255,	0);
    wifi_set_ip_info(SOFTAP_IF,	&info);
    wifi_softap_dhcps_start();

	wifi_softap_set_config(&AP_Config);				// ！！！设置soft-AP，并保存到Flash
}



void ICACHE_FLASH_ATTR 
ESP8266_TCP_Disconnect_Cb_JX(void *arg)
{
   // os_printf("\nESP8266_tcp_disconnect\n");
   // GPIO_OUTPUT_SET(GPIO_ID_PIN(2),1);
    
}

void ICACHE_FLASH_ATTR 
ESP8266_TCP_Break_Cb_JX(void *arg,sint8 err)
{
     //os_printf("\nESP8266_tcp_BREAK====%S\n",err);
}



void ICACHE_FLASH_ATTR 
ESP8266_WIFI_Send_Cb_JX(void *arg)
{
	//os_printf("\nESP8266_WIFI_Send_OK\n");
}

void ICACHE_FLASH_ATTR 
ESP8266_WIFI_Recv_Cb_JX(void * arg, char * pdata, unsigned short len)
{
    uint8 i;
    struct espconn * T_arg = arg;
    remot_info * P_port_info = NULL;
  //send
    uart0_sendStr(pdata);
}

void ICACHE_FLASH_ATTR 
ESP8266_TCP_Connect_Cb_JX(void *arg)
{
    //os_printf("\nESP8266_tcp_connect_OK\n");
    //espconn_regist_sentcb(&ST_NetCon,ESP8266_WIFI_Send_Cb_JX);
    //espconn_regist_recvcb(&ST_NetCon,ESP8266_WIFI_Recv_Cb_JX);
    espconn_regist_disconcb((struct espconn *)arg,ESP8266_TCP_Disconnect_Cb_JX);
    espconn_set_opt((struct espconn *)arg,0x04);
   // os_printf("\narg=%p st=%p\n",(struct espconn *)arg,ST_NetCon);
   //GPIO_OUTPUT_SET(GPIO_ID_PIN(2),0);

}

//TCP初始化
void ICACHE_FLASH_ATTR
ESP8266_NetConTCP_Init()
{
    ST_NetCon.type = ESPCONN_TCP;
    ST_NetCon.proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
    ST_NetCon.proto.tcp->local_port = 35000;
    
    espconn_regist_connectcb(&ST_NetCon, ESP8266_TCP_Connect_Cb_JX);	
	espconn_regist_reconcb(&ST_NetCon, ESP8266_TCP_Break_Cb_JX);
     espconn_regist_sentcb(&ST_NetCon,ESP8266_WIFI_Send_Cb_JX);
    espconn_regist_recvcb(&ST_NetCon,ESP8266_WIFI_Recv_Cb_JX);
    espconn_accept(&ST_NetCon);
    espconn_regist_time(&ST_NetCon, 300, 0); 
    
}

void ICACHE_FLASH_ATTR
user_init(void)
{
    uart_init(38400,38400);
    os_delay_us(10000);
    LED_Init();
    ESP8266_AP_INIT();
    OS_Timer_1_INIT(100,1);   
    ESP8266_NetConTCP_Init();
    //wifi_status_led_install(HUMITURE_WIFI_LED_IO_NUM, HUMITURE_WIFI_LED_IO_MUX,	HUMITURE_WIFI_LED_IO_FUNC);
    
    //int led1 = wifi_softap_get_station_num();
    //os_printf("\nst_init=%p\n",ST_NetCon);
    //system_soft_wdt_feed();
    //cjson_creat();
    //cjson_ana();   
    //os_printf("\r\n------------------ user_init OVER ----------------\r\n");
}


void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}

uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
            rf_cal_sec = 512 - 5;
            break;
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
            rf_cal_sec = 1024 - 5;
            break;
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

