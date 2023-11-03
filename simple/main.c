#include <stdint.h>

#include "../include/utils.h"
#include "../include/uart.h"
#include "../include/xprintf.h"
#include "../include/gpio.h"
#include "../include/spi.h"

//global param
#define GAME_COUNT 3
#define GAME_RANDOM (uint8_t)frame_time*113
#define COMMAND_A 'A'
#define COMMAND_W 'W'
#define COMMAND_S 'S'
#define COMMAND_D 'D'
#define COMMAND_Q 'Q'

//game param
#define SNAKE_MAP_HEIGHT 12
#define SNAKE_MAP_WIDTH 16
#define SNAKE_DELAY 16//800/50

#define TETRIS_MAP_HEIGHT 12
#define TETRIS_MAP_WIDTH 16
#define TETRIS_DELAY 20//1000/50

#define FLAPPYBIRD_DELAY 2
#define HOLE_WIDTH 20
#define HOLE_HEIGHT 60

//color
#define COLOR_RED 0xF800
#define COLOR_YELLOW 0xFFE0
#define COLOR_GREEN 0x07E0
#define COLOR_CYAN 0x07FF
#define COLOR_BLUE 0x001F
#define COLOR_PUPPLE 0xF81F
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_GRAY 0x8410

#define DELAY_US(a) busy_wait(a)
#define DELAY_MS(a) busy_wait(a * 1000)

#define GPIO_INPUT(a) GPIO_REG(GPIO_CTRL) |= ((uint32_t)0x1 << (a*2+1))
#define GPIO_OUTPUT(a) GPIO_REG(GPIO_CTRL) |= ((uint32_t)0x1 << (a*2))
#define GPIO_STATE(a) (GPIO_REG(GPIO_DATA) & ((uint32_t)0x1 << a))
#define GPIO_HIGH(a) GPIO_REG(GPIO_DATA) |= ((uint32_t)0x1 << a)
#define GPIO_LOW(a) GPIO_REG(GPIO_DATA) &= ~((uint32_t)0x1 << a)
#define GPIO_TOGGLE(a) GPIO_REG(GPIO_DATA) ^= ((uint32_t)0x1 << a)

void gpio_init(){
    GPIO_OUTPUT(0);//对应40pin
    GPIO_OUTPUT(1);
    GPIO_OUTPUT(2);
    GPIO_OUTPUT(3);
    GPIO_INPUT(4);
    GPIO_INPUT(5);
    GPIO_INPUT(6);
    GPIO_INPUT(7);

    GPIO_OUTPUT(8);//LED
    GPIO_OUTPUT(9);
    GPIO_OUTPUT(10);
    GPIO_OUTPUT(11);

    GPIO_INPUT(12);//按键
    GPIO_INPUT(13);
    GPIO_INPUT(14);
    GPIO_INPUT(15);

    GPIO_LOW(2);//led
    GPIO_HIGH(8);
    GPIO_LOW(9);
    GPIO_HIGH(10);
    GPIO_LOW(11);
}

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

void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color){
    //trans
    //               y       y
    //                  x =>    x
    uint8_t tmp = y;
    y=LCD_HEIGHT-x-width;
    x=tmp;
    lcd_set_windows(x, y, x + height - 1, y + width - 1);
    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++){
            lcd_write_gram(color);
        }
    }
}

#pragma endregion

void led_send_0(){
    GPIO_HIGH(2);
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    GPIO_LOW(2);
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
}

void led_send_1(){
    GPIO_HIGH(2);
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    GPIO_LOW(2);
    __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
    // __asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");__asm__("nop");
}
void led_send_data(uint8_t data){
    for(int i=0;i<8;i++){
        if(data&0x80){
            led_send_1();
        }
        else{
            led_send_0();
        }
        data<<=1;
    }
}
void led_send_rgb(uint8_t r, uint8_t g, uint8_t b){
    led_send_data(g);
    led_send_data(r);
    led_send_data(b);
    busy_wait(3);
}

void led_init(){
    led_send_rgb(128,0,0);
    led_send_rgb(128,128,0);
    led_send_rgb(0,128,0);
    led_send_rgb(0,128,128);
    led_send_rgb(0,0,128);
    
    led_send_rgb(128,0,128);
    led_send_rgb(128,128,128);
    led_send_rgb(128,0,0);
    led_send_rgb(128,128,0);
    led_send_rgb(0,128,0);

    led_send_rgb(0,128,128);
    led_send_rgb(0,0,128);
    led_send_rgb(128,0,128);
    led_send_rgb(128,128,128);
}

int main()
{
    uart_init();
    xprintf("UART done!\r\n");
    gpio_init();
    xprintf("GPIO done!\r\n");
    lcd_init();
    lcd_clear(COLOR_RED);
    xprintf("LCD done!\r\n");
    led_init();
    
    // state 0: menu
    // state 1: GreedySnake
    // state 2: Tetris
    uint8_t state = 0;
    int8_t menu_choice = 1;

    lcd_draw_rect(LCD_HEIGHT/2-40, LCD_WIDTH/2-40, 80, 80, COLOR_GREEN);
    lcd_draw_rect(LCD_HEIGHT/2-10, LCD_WIDTH/2-10-60, 20, 20, COLOR_BLACK);

    uint8_t frame_time = 0;

    // snake_init()
    uint8_t snake_time=0;
    int8_t snake_map[SNAKE_MAP_HEIGHT][SNAKE_MAP_WIDTH] = {0};
    int8_t snake_head_x = SNAKE_MAP_WIDTH/2-1;
    int8_t snake_head_y = SNAKE_MAP_HEIGHT/2-1;
    uint8_t apple_x = snake_head_x+1;
    uint8_t apple_y = snake_head_y+2;
    uint8_t snake_length = 3;
    uint8_t snake_direction = 4;//COMMAND_D
    snake_map[snake_head_y][snake_head_x-2]=3;
    snake_map[snake_head_y][snake_head_x-1]=2;
    snake_map[snake_head_y][snake_head_x]=1;
    snake_map[apple_y][apple_x]=-1;

    // tetris_init()
    uint8_t tetris_time=0;
    int8_t tetris_map[TETRIS_MAP_HEIGHT][TETRIS_MAP_WIDTH] = {0};

    // flappybird_init()
    uint8_t flappybird_time=0;
    uint8_t bird_x = LCD_HEIGHT*2/7;
    uint16_t bird_y = LCD_WIDTH/2 * 10;
    int16_t bird_v = 0;
    int16_t hole_y[2]={LCD_HEIGHT/2, LCD_HEIGHT/2};
    int16_t hole_x[2]={LCD_HEIGHT-HOLE_WIDTH/2, LCD_HEIGHT+LCD_HEIGHT/2-HOLE_WIDTH/2};
    
    uint8_t key_press = 0;
    while(1){
        busy_wait(20000);// 20 ms
        frame_time++;
        uint8_t data = uart_getc_nowait();
        if(key_press>0){
            key_press--;
        }
        else if(data==0){
            if(GPIO_STATE(4)==0){
                data = COMMAND_A;
            }
            else if(GPIO_STATE(5)==0){
                data = COMMAND_W;
            }
            else if(GPIO_STATE(6)==0){
                data = COMMAND_S;
            }
            else if(GPIO_STATE(7)==0){
                data = COMMAND_D;
            }
            key_press=10;
        }
        if(data != 0){// 有数据改变部分
            xprintf("Command: %d\r\n", data);
            //Main Menu
            if(state==0){
                if(data == COMMAND_A || data == COMMAND_D){
                    if(data == COMMAND_A){
                        menu_choice--;
                        if(menu_choice<1){
                            menu_choice=GAME_COUNT;
                        }
                    }
                    else if(data == COMMAND_D){
                        menu_choice++;
                        if(menu_choice>GAME_COUNT){
                            menu_choice=1;
                        }
                    }
                    lcd_draw_rect(LCD_HEIGHT/2-10, LCD_WIDTH/2-10-60, 20, 20, COLOR_BLACK);
                    if(menu_choice==1){
                        lcd_draw_rect(LCD_HEIGHT/2-40, LCD_WIDTH/2-40, 80, 80, COLOR_GREEN);
                    }
                    else if(menu_choice==2){
                        lcd_draw_rect(LCD_HEIGHT/2-40, LCD_WIDTH/2-40, 80, 80, COLOR_PUPPLE);
                    }
                    else if(menu_choice==3){
                        lcd_draw_rect(LCD_HEIGHT/2-40, LCD_WIDTH/2-40, 80, 80, COLOR_CYAN);
                    }
                }

                else if(data == COMMAND_W){
                    state = menu_choice;
                    //reset game
                    //snake_init()
                    if(menu_choice==1){
                        lcd_clear(COLOR_RED);
                        snake_time=0;
                        snake_head_x = SNAKE_MAP_WIDTH/2-1;
                        snake_head_y = SNAKE_MAP_HEIGHT/2-1;
                        apple_x = snake_head_x+1;
                        apple_y = snake_head_y+2;
                        snake_length = 3;
                        snake_direction = 4;//COMMAND_D
                        snake_map[snake_head_y][snake_head_x-2]=3;
                        snake_map[snake_head_y][snake_head_x-1]=2;
                        snake_map[snake_head_y][snake_head_x]=1;
                        snake_map[apple_y][apple_x]=-1;
                        continue;
                    }
                    else if(menu_choice==2){
                        //tetris_init()
                        tetris_time=0;
                    }
                    else if(menu_choice==3){
                        //flappybird_init()
                        flappybird_time=0;
                        bird_x = LCD_HEIGHT*2/7;
                        bird_y = LCD_WIDTH/2 * 10;
                        bird_v = 0;
                        hole_y[0]=LCD_HEIGHT/2;hole_y[1]=LCD_HEIGHT/2;
                        hole_x[0]=LCD_HEIGHT-HOLE_WIDTH/2;hole_x[1]=LCD_HEIGHT/2-HOLE_WIDTH/2;
                    }
                }
            }

            //GreedySnake
            if(state==1){
                if(data == COMMAND_A){
                    if(snake_direction!=4)
                        snake_direction=1;
                }
                else if(data == COMMAND_W){
                    if(snake_direction!=3)
                        snake_direction=2;
                }
                else if(data == COMMAND_S){
                    if(snake_direction!=2)
                        snake_direction=3;
                }
                else if(data == COMMAND_D){
                    if(snake_direction!=1)
                        snake_direction=4;
                }
                else if(data == COMMAND_Q){
                    state=0;
                    continue;
                }
            }

            //Tetris
            if(state==2){
                if(data == COMMAND_A){
                    //tetris_move_left()
                }
                else if(data == COMMAND_D){
                    //tetris_move_right()
                }
                else if(data == COMMAND_W){
                    //tetris_rotate()
                }
                else if(data == COMMAND_S){
                    //tetris_move_down()
                }
                else if(data == COMMAND_Q){
                    state=0;
                    continue;
                }
            }


            if(state==3){
                if(data == COMMAND_W){
                    bird_v=10;
                }
                else if(data == COMMAND_Q){
                    state=0;
                    continue;
                }
            }
        }
        if(state==0){
            //lcd_load(menu_choice)
        }

        else if(state==1){
            if(snake_time<SNAKE_DELAY){
                snake_time++;
                continue;
            }
            else{
                snake_time=0;
            }
            uint8_t snake_tail_x = 0;
            uint8_t snake_tail_y = 0;
            for(int i=0;i<SNAKE_MAP_HEIGHT;i++){
                for(int j=0;j<SNAKE_MAP_WIDTH;j++){
                    // body
                    if(snake_map[i][j]>0){
                        snake_map[i][j]++;
                        if(snake_map[i][j]>snake_length){
                            snake_map[i][j]=0;
                            snake_tail_x = j;
                            snake_tail_y = i;
                            
                        }
                    }
                }
            }
            if(snake_direction==1){
                snake_head_x--;
                if(snake_head_x<0){
                    snake_head_x=SNAKE_MAP_WIDTH-1;
                }
            }
            else if(snake_direction==2){
                snake_head_y++;
                if(snake_head_y>=SNAKE_MAP_HEIGHT){
                    snake_head_y=0;
                }
            }
            else if(snake_direction==3){
                snake_head_y--;
                if(snake_head_y<0){
                    snake_head_y=SNAKE_MAP_HEIGHT-1;
                }
            }
            else if(snake_direction==4){
                snake_head_x++;
                if(snake_head_x>=SNAKE_MAP_WIDTH){
                    snake_head_x=0;
                }
            }
            if(snake_map[snake_head_y][snake_head_x]>0){
                //show game over
                state=0;
                continue;
            }
            else if(snake_map[snake_head_y][snake_head_x]==-1){
                snake_length++;
                // summon apple
                int random = GAME_RANDOM%(SNAKE_MAP_HEIGHT*SNAKE_MAP_WIDTH-snake_length-1);
                for(int i=0;i<SNAKE_MAP_HEIGHT;i++){
                    for(int j=0;j<SNAKE_MAP_WIDTH;j++){
                        if(snake_map[i][j]>0){
                            continue;
                        }
                        random--;
                        if(random==0){
                            snake_map[i][j]=-1;
                            apple_x=j;
                            apple_y=i;
                            break;
                        }
                    }
                }
                //show score
            }
            snake_map[snake_head_y][snake_head_x]=1;
            lcd_draw_rect(apple_x*LCD_HEIGHT/SNAKE_MAP_WIDTH, apple_y*LCD_WIDTH/SNAKE_MAP_HEIGHT, LCD_HEIGHT/SNAKE_MAP_WIDTH, LCD_WIDTH/SNAKE_MAP_HEIGHT, COLOR_YELLOW);
            lcd_draw_rect(snake_tail_x*LCD_HEIGHT/SNAKE_MAP_WIDTH, snake_tail_y*LCD_WIDTH/SNAKE_MAP_HEIGHT, LCD_HEIGHT/SNAKE_MAP_WIDTH, LCD_WIDTH/SNAKE_MAP_HEIGHT, COLOR_RED);
            lcd_draw_rect(snake_head_x*LCD_HEIGHT/SNAKE_MAP_WIDTH, snake_head_y*LCD_WIDTH/SNAKE_MAP_HEIGHT, LCD_HEIGHT/SNAKE_MAP_WIDTH, LCD_WIDTH/SNAKE_MAP_HEIGHT, COLOR_GREEN);
        }
        
        else if(state==2){
            if(tetris_time<TETRIS_DELAY){
                tetris_time++;
                continue;
            }
            else{
                tetris_time=0;
            }
            for(int i=0;i<TETRIS_MAP_HEIGHT;i++){
                for(int j=0;j<TETRIS_MAP_WIDTH;j++){
                    // body
                    if(tetris_map[i][j]>0){
                        tetris_map[i][j]++;
                        if(tetris_map[i][j]>snake_length){
                            tetris_map[i][j]=0;
                            
                        }
                    }
                }
            }
        }


        else if(state==3){
            if(flappybird_time<FLAPPYBIRD_DELAY){
                flappybird_time++;
                continue;
            }
            else{
                flappybird_time=0;
            }
            bird_y+=bird_v;
            bird_v--;

            if(bird_y<0||bird_y>LCD_WIDTH-20){
                // lcd game over
                state=0;
                lcd_clear(COLOR_RED);
                continue;
            }
            lcd_draw_rect(bird_x, bird_y-bird_v, 10, 10, COLOR_RED);
            lcd_draw_rect(bird_x, bird_y, 10, 10, COLOR_YELLOW);

            for(int i=0;i<2;i++){
                hole_x[i]-=3;
                if(hole_x[i]<0){
                    hole_x[i]=LCD_HEIGHT-HOLE_WIDTH/2;
                    hole_y[i]=GAME_RANDOM%HOLE_HEIGHT+LCD_WIDTH/2;
                }
            }
            lcd_draw_rect(bird_x, bird_y, 10, 10, COLOR_BLACK);
            for(int i=0;i<2;i++){
                lcd_draw_rect(hole_x[i]-HOLE_WIDTH/2, 0, HOLE_WIDTH, hole_y[i]-HOLE_HEIGHT/2, COLOR_GREEN);
                lcd_draw_rect(hole_x[i]-HOLE_WIDTH/2, LCD_WIDTH, HOLE_WIDTH, hole_y[i]+HOLE_HEIGHT/2, COLOR_GREEN);
                
                lcd_draw_rect(hole_x[i]+HOLE_WIDTH/2-3, 0, 3, hole_y[i]-HOLE_HEIGHT/2, COLOR_RED);
                lcd_draw_rect(hole_x[i]+HOLE_WIDTH/2-3, LCD_WIDTH, 3, hole_y[i]+HOLE_HEIGHT/2, COLOR_RED);
            }
        }
    }
}
