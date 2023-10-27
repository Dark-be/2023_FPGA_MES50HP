#include <stdint.h>

#include "../include/utils.h"
#include "../include/uart.h"
#include "../include/xprintf.h"
#include "../include/gpio.h"
#include "../include/spi.h"

#define DELAY_US(a) busy_wait(a)
#define DELAY_MS(a) busy_wait(a * 1000)

// gpio 0 1 已被用作dc rst
#pragma region LCD

#define LCD_WIDTH   240
#define LCD_HEIGHT  320

#define LCD_DC_HIGH()   GPIO_REG(GPIO_DATA) |= 0x1;  // 1 DC data
#define LCD_DC_LOW()    GPIO_REG(GPIO_DATA) &= ~0x1; // 0 DC command

#define LCD_RST_HIGH()  GPIO_REG(GPIO_DATA) |= 0x2;  // 1 RST
#define LCD_RST_LOW()   GPIO_REG(GPIO_DATA) &= ~0x2; // 0 RST

void lcd_write(uint8_t data){//SPI_MODE0 common low up sample
    spi_write_byte(data);
}

void lcd_reset(){
    LCD_RST_LOW();
    DELAY_MS(100);
    LCD_RST_HIGH();
    DELAY_MS(100);
}
// Command
void lcd_write_command(uint8_t command){
    LCD_DC_LOW();
    lcd_write(command);
}
// Data
void lcd_write_data(uint8_t data){
    LCD_DC_HIGH();
    lcd_write(data);
}
// Configure
void lcd_configure()
{
    //功耗控制B
    DELAY_MS(1);
    lcd_write_command(0xCF);
	lcd_write_data(0x00);
	lcd_write_data(0xC1);
	lcd_write_data(0X30);
    //电源时序控制
    DELAY_MS(1);
	lcd_write_command(0xED);
	lcd_write_data(0x64);
	lcd_write_data(0x03);
	lcd_write_data(0X12);
	lcd_write_data(0X81);
    //驱动时序控制A
    DELAY_MS(1);
	lcd_write_command(0xE8); 
	lcd_write_data(0x85);
	lcd_write_data(0x10);
	lcd_write_data(0x7A);
    //功耗控制A
    DELAY_MS(1);
	lcd_write_command(0xCB);
	lcd_write_data(0x39);
	lcd_write_data(0x2C);
	lcd_write_data(0x00);
	lcd_write_data(0x34);
	lcd_write_data(0x02);
    //泵比控制
    DELAY_MS(1);
	lcd_write_command(0xF7);
	lcd_write_data(0x20);
    //驱动时序控制B
    DELAY_MS(1);
	lcd_write_command(0xEA);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
    //功耗控制1
    DELAY_MS(1);
	lcd_write_command(0xC0);  //Power control 
	lcd_write_data(0x1B);  //VRH[5:0]
    //功耗控制2
    DELAY_MS(1);
	lcd_write_command(0xC1);  //Power control 
	lcd_write_data(0x01);  //SAP[2:0];BT[3:0]
    //VCOM控制1
    DELAY_MS(1);
	lcd_write_command(0xC5);
	lcd_write_data(0x30);
	lcd_write_data(0x30);
    //VCOM控制2
    DELAY_MS(1);
	lcd_write_command(0xC7);
	lcd_write_data(0XB7);
    //存储器访问控制
    DELAY_MS(1);
	lcd_write_command(0x36);
	lcd_write_data(0x48);
    //像素格式设置
    DELAY_MS(1);
	lcd_write_command(0x3A);
	lcd_write_data(0x55);
    //帧速率控制（正常模式/全色模式）
    DELAY_MS(1);
	lcd_write_command(0xB1);
	lcd_write_data(0x00);
	lcd_write_data(0x1A);
    //显示功能设置控制
    DELAY_MS(1);
	lcd_write_command(0xB6);
	lcd_write_data(0x0A);
	lcd_write_data(0xA2);
    //使能3G
    DELAY_MS(1);
	lcd_write_command(0xF2);  
	lcd_write_data(0x00);  //3Gamma Function Disable 
    //伽马设置
    DELAY_MS(1);
	lcd_write_command(0x26);  
	lcd_write_data(0x01);  //Gamma curve selected 
    //正极伽马校正
    DELAY_MS(1);
	lcd_write_command(0xE0);
	lcd_write_data(0x0F);
	lcd_write_data(0x2A);
	lcd_write_data(0x28);
	lcd_write_data(0x08);
	lcd_write_data(0x0E);
	lcd_write_data(0x08);
	lcd_write_data(0x54);
	lcd_write_data(0XA9);
	lcd_write_data(0x43);
	lcd_write_data(0x0A);
	lcd_write_data(0x0F);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
    //负极伽马校正
    DELAY_MS(1);
	lcd_write_command(0XE1);
	lcd_write_data(0x00);
	lcd_write_data(0x15);
	lcd_write_data(0x17);
	lcd_write_data(0x07);
	lcd_write_data(0x11);
	lcd_write_data(0x06);
	lcd_write_data(0x2B);
	lcd_write_data(0x56);
	lcd_write_data(0x3C);
	lcd_write_data(0x05);
	lcd_write_data(0x10);
	lcd_write_data(0x0F);
	lcd_write_data(0x3F);
	lcd_write_data(0x3F);
	lcd_write_data(0x0F);
    //行地址设置
    DELAY_MS(1);
	lcd_write_command(0x2B);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x01);
	lcd_write_data(0x3f);
    //列地址设置
    DELAY_MS(1);
	lcd_write_command(0x2A);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0xef);
    //退出睡眠模式
    DELAY_MS(1);
	lcd_write_command(0x11);
    //开显示(0x28为关显示)
    DELAY_MS(300);
	lcd_write_command(0x29);
}

void lcd_set_windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end){
    lcd_write_command(0x2a);
    lcd_write_data(x_start >> 8);
    lcd_write_data(x_start & 0xff);
    lcd_write_data(x_end >> 8);
    lcd_write_data(x_end & 0xff);

    lcd_write_command(0x2b);
    lcd_write_data(y_start >> 8);
    lcd_write_data(y_start & 0xff);
    lcd_write_data(y_end >> 8);
    lcd_write_data(y_end & 0xff);

    lcd_write_command(0x2c);
}

void lcd_set_cursor(uint16_t x, uint16_t y){
    lcd_write_command(0x2a);
    lcd_write_data(x >> 8);
    lcd_write_data(x & 0xff);
    lcd_write_command(0x2b);
    lcd_write_data(y >> 8);
    lcd_write_data(y & 0xff);
    lcd_write_command(0x2c);
}

void lcd_write_gram(uint16_t data){
    lcd_write_data(data >> 8);
    lcd_write_data(data & 0xff);
}

void lcd_init(){
    lcd_reset();
    lcd_configure();
}

void lcd_clear(uint16_t color){
    lcd_set_windows(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    for(int i=0;i<LCD_WIDTH;i++){
        for(int j=0;j<LCD_HEIGHT;j++){
            lcd_write_gram(color);
        }
    }
}

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color){
    lcd_set_cursor(x, y);
    lcd_write_gram(color);
}

#pragma endregion

void gpio_init(){
    GPIO_REG(GPIO_CTRL) = 0x5555;       // gpio0输出模式 0101 0101 0101 0101 0x5555
    lcd_reset();
}

int main()
{
    uart_init();
    xprintf("UART done!\r\n");
    gpio_init();
    xprintf("GPIO done!\r\n");
    lcd_init();
    lcd_clear(0xF800);
    xprintf("LCD done!\r\n");


    while(1){
        //GPIO_REG(GPIO_DATA) |= 0x5555;
        //GPIO_REG(GPIO_DATA) &= ~0xAAAA;
        busy_wait(1000000);
        xprintf("high\r\n");

        //GPIO_REG(GPIO_DATA) |= 0xAAAA;
        //GPIO_REG(GPIO_DATA) &= ~0x5555;
        busy_wait(1000000);
        xprintf("low\r\n");

    }
}
