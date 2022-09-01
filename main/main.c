
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/task.h"
#include "driver/uart.h"

//Pins: CLK, MOSI, RES, DC, BLK, CS, MSIO

#define V_PIN_BLK           12
#define V_PIN_SCLK          14
#define V_PIN_MOSI          13
#define V_PIN_DC            27
#define V_PIN_CS_LCD        15
#define V_PIN_RST           16
#define V_PANEL_WIDTH       320
#define V_PANEL_HEIGHT      240

esp_err_t esp_lcd_new_panel_ili9341(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

void app_main(void)
{
    //IO Installation
    //  BLK: GPIO
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << V_PIN_BLK
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(V_PIN_BLK, 1);
    //  CLK, MSIO, MOSI: SPI
    spi_bus_config_t buscfg = {
        .sclk_io_num = V_PIN_SCLK,
        .mosi_io_num = V_PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = V_PANEL_WIDTH * 160 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = V_PIN_DC,
        .cs_gpio_num = V_PIN_CS_LCD,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = V_PIN_RST,
        .flags.reset_active_high = 0,
        //.rgb_endian = LCD_RGB_ENDIAN_RGB,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 18,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    /*
    static unsigned char color_map[48]={
        //0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,
        //0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,0x3,0xF0,0x0,
        0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,
        0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,0xF3,0,0,
    };

    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 10, 1, color_map);
    */
    /*
    unsigned int iConX;
    unsigned int iConY;
    for(iConY = 0 ; iConY < 320 ; iConY++){
        for(iConX = 0 ; iConX < 15 ; iConX++){
            esp_lcd_panel_draw_bitmap(panel_handle, 0 + iConX * 16, 0 + iConY, 16 + iConX * 16, iConY + 1, color_map);
        }
    }
    */
    /*
    unsigned int iConX;
    unsigned int iConY;
    for(iConY = 0 ; iConY < 240 ; iConY++){
        for(iConX = 0 ; iConX < 20 ; iConX++){
            esp_lcd_panel_draw_bitmap(panel_handle, 0 + iConX * 16, 0 + iConY, 16 + iConX * 16, iConY + 1, color_map);
        }
    }
    */
    /*
    esp_lcd_panel_draw_bitmap(panel_handle, iCon, iCon+1, 0, 0, color_map);


    static unsigned char color_map[32]={0xFF,0xFF,0xFF,0xFF,0xFF};
    unsigned short iCon;
    iCon = 0 ;
    while(1){
        esp_lcd_panel_draw_bitmap(panel_handle, iCon, iCon+1, 0, 0, color_map);
        vTaskDelay(100);
        iCon++;
        iCon %= 100;
        printf(".");
    }
    */
    /*
    
    
    ESP_ERROR_CHECK(esp_lcd_panel_disp_off(panel_handle, false));
    gpio_set_level(V_PIN_BLK, 0);
    
    unsigned int iCon;
    static unsigned char color_map[32]={0xFF,0xFF,0xFF,0xFF,0xFF};
    iCon = 0 ;
    while(1){
        esp_lcd_panel_draw_bitmap(panel_handle, iCon, iCon+1, 0, 0, color_map);
        vTaskDelay(100);
        iCon++;
        printf(".");
    }
    
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    */
}