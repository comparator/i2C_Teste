#include "HAL/hal.h"
#include "HAL/hal_gpio.h"
#include "HAL/hal_uart.h"
#include "HAL/hal_twi.h"
#include "syscal/printf.h"

#include <string.h>



//static const uint8_t errAlreadyRun[] = "Already Run\r\n"

static const uint8_t errBusBusy[] = "Bus Busy\r\n";
static const uint8_t okBus[] = "Bus Ok\r\n";

static uint8_t addr, toWrite, toRead;
static uint8_t datawr[32];
//static uint8_t datard[32];
    
static uint8_t data[6];         // uint16: 0 - 255
static uint8_t cnt = 0, mode = 0;

//static bool run = false;

static void parse(void)
{
    if(cnt == 0)
    {
        return;
    }

    uint16_t val = 0;
    uint8_t pos = 0;
    while(cnt > 0)
    {
        val *= 10;
        val += data[pos++] - '0';
        cnt--;
    }

    switch(mode)
    {
        case 'A':
            if(val > 127)
            {
                val = 127;
            }
            addr = val;
            mode = 0;
            break;

        case 'R':
            if(val > 32)    // bytes to read
            {
                val = 32;
            }
            toRead = val;
            mode = 0;
            break;

        case 'W':
            if(toWrite < 32)
            {
                datawr[toWrite++] = val & 0xFF;
                if((val > 255) && (toWrite < 32))
                {
                    datawr[toWrite++] = val >> 8;
                }
            }
            break;


        default:
            cnt = 0;
            break;
    }
}

static void exec(void)
{
/*
    char print_buf[256];
    uint8_t pos = 0;

    while(hal_uart_free() == false);

    if(run)
    {
        hal_uart_send(sizeof(errAlreadyRun), (uint8_t *)errAlreadyRun);
        return;
    }
    
    pos = hal_twi_status();
    id(
    
    

    if(!hal_twi_configure(1))
    {
        hal_uart_send(sizeof(errBusBusy), (uint8_t *)errBusBusy);
        return;
    }
*/
/*
    uint8_t pos = sprintf(print_buf, "Addr: %d\r\n", addr);

    if(toRead != 0xFF)
    {
        pos += sprintf(&print_buf[pos], "To Read: %d\r\n", toRead);
    }
    
    if(toWrite != 0xFF)
    {
        pos += sprintf(&print_buf[pos], "To Write:");

        uint8_t i;
        for(i = 0; i < toWrite; i++)
        {
            pos += sprintf(&print_buf[pos], " %d", datawr[i]);
        }
        pos += sprintf(&print_buf[pos], "\r\n");
    }

    if(hal_uart_free())
    {
        hal_uart_send(pos, (uint8_t *)print_buf);
    }
*/
}

int main(void)
{
    hal_system_init();
    hal_gpio_init();        // Enable GPIO Clock

    hal_uart_init_hw(115200);

    // Check I2C Pin
    if(hal_twi_check_pin() == false)
    {
        hal_uart_send((sizeof(errBusBusy) - 1), (uint8_t *)errBusBusy);
    }
    else
    {
        hal_uart_send((sizeof(okBus) - 1), (uint8_t *)okBus);
    }

    addr = 0xFF;
    toWrite = 0xFF;
    toRead = 0xFF;
    cnt = 0;
    mode = 0;

    while(1)
    {
        // get data
        if(hal_uart_datardy())
        {
            uint8_t ch = hal_uart_get();

            if((ch >= 'a') && (ch <= 'z'))
            {
                ch -= 0x20;
            }

            switch(ch)
            {
                case 'C':           // clear settings
                    addr = 0xFF;
                    toWrite = 0xFF;
                    toRead = 0xFF;
                    cnt = 0;
                    mode = 0;
                    break;
                
                case 'G':               // Global Call
                    addr = 0x00;
                    toWrite = 0x01;
                    toRead = 0xFF;
                    datawr[0] = 0x06;   //  Reset
                    exec();
                    mode = 0;
                    break;


                case 'W':           // data to write
                    toWrite = 0;
                    __attribute__ ((fallthrough));
                    /* no break */

                case 'A':           // Get Addr
                case 'R':           // bytes to read
                    parse();
                    mode = ch;
                    break;

                case 'E':           // Execute
                    parse();
                    exec();
                    mode = 0;
                    break;

                default:
                    if((mode != 0) && (ch >= '0') && (ch <= '9'))
                    {
                        data[cnt++] = ch;
                        if(cnt > sizeof(data))
                        {
                            parse();
                        }
                    }
                    else
                    {
                        parse();
                    }
                    break;
            }
        }
    }

    return 0;
}
