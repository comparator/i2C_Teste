#include "HAL/hal.h"
#include "HAL/hal_gpio.h"
#include "HAL/hal_uart.h"
#include "HAL/hal_twi.h"
#include "HAL/hal_timer.h"
#include "syscal/printf.h"

#include <string.h>

static const uint8_t strVersion[] = "I2C Scaner Version 0.3\r";
static const uint8_t strBusBusy[] = "Bus Busy\r";
static const uint8_t strBusError[] = "Bus Error\r";
static const uint8_t strNACK[] = "NACK\r";
static const uint8_t strOk[] = "Ok\r";
static const uint8_t strHelp[] = "C - clear settings\rA - Set Current Address\rS - Set Max Scan Address\rR - Bytes to read\rW - data to write\rL - Show Settings\rV[0-2 show dec/hex/ascii]  verbose on/off\rP - pause between executes\rT - Number of executes\rE - execute\r";

static uint8_t addr, StartAddr, StopAddr;
static uint8_t toWrite, toRead;
static uint8_t datawr[32];
static uint8_t datard[32];

static bool  scanmode, verbose;
static uint8_t view;

char print_buf[256];

static uint8_t print_view(uint8_t pos, uint8_t num, uint8_t *pBuf)
{
    uint8_t i;
    for(i = 0; i < num; i++)
    {
        uint8_t ichar = *pBuf;
        pBuf++;

        if(view == 1)
        {
            pos += sprintf(&print_buf[pos], "%02X ", ichar);
        }
        else if(view == 2)
        {
            if((ichar < 0x20) || (ichar > 0x7E))
            {
                ichar = '.';
            }
            pos += sprintf(&print_buf[pos], "%c", ichar);
        }
        else
        {
            pos += sprintf(&print_buf[pos], "%03d ", ichar);
        }
    }
    return pos;
}

static void print_rdy(void)
{
    uint8_t pos = 0;
    uint8_t sent = hal_twi_get_data(datard);

    if(scanmode)
    {
        pos = sprintf(print_buf, "Addr:%03d ", addr);
    }

    if(toWrite != 0xFF)
    {
        if(toWrite != 0x00)
        {
            pos += sprintf(&print_buf[pos], "Wr:%d ", sent);
        }
        else
        {
            pos += sprintf(&print_buf[pos], "WrS ");
        }
    }

    if((toRead != 0xFF) && (toRead != 0x00))
    {
        pos += sprintf(&print_buf[pos], "Rd:");
        pos = print_view(pos, toRead, datard);
    }
    else
    {
        pos += sprintf(&print_buf[pos], "Ok");
    }

    pos += sprintf(&print_buf[pos], "\r");
    hal_uart_send(pos, (uint8_t *)print_buf);
}

static void print_info(void)
{
    uint8_t pos = 0;
    if(scanmode)
    {
        pos = sprintf(print_buf, "Scan %d - %d ", addr, StopAddr);
    }
    else
    {
        pos = sprintf(print_buf, "Addr:%d ", addr);
    }

    if(toWrite != 0xFF)
    {
        pos += sprintf(&print_buf[pos], "Wr:");
        pos  = print_view(pos, toWrite, datawr);
        if(toRead != 0xFF)
        {
            print_buf[pos++] = ' ';
        }
    }

    if(toRead != 0xFF)
    {
        pos += sprintf(&print_buf[pos], "Rd:%d", toRead);
    }

    if(scanmode)
    {
        print_buf[pos++] = '\r';
    }
    else
    {
        print_buf[pos++] = ' ';
    }
    
    hal_uart_send(pos, (uint8_t *)print_buf);
}

static bool exec(void)
{
    // Check GPIO
    if( ((hal_twi_status() & TWI_FL_BUSY) != 0) || 
         (hal_twi_check_pin() == false))
    {
        hal_uart_send((sizeof(strBusBusy) - 1), (uint8_t *)strBusBusy);
        return false;
    }

    if((toWrite == 0xFF) && (toRead == 0xFF))   // No data
    {
        toWrite = 0x00;                         // Write Strobe
    }

    hal_twi_enable();
    uint8_t tErr = hal_twi_start(addr, toWrite, toRead, datawr);
    
    if(tErr == TWI_FL_ERROR)
    {
        hal_twi_disable();
        hal_uart_send((sizeof(strBusError) - 1), (uint8_t *)strBusError);
    }
    else if(tErr == TWI_FL_BUSY)
    {
        hal_twi_disable();
        hal_uart_send((sizeof(strBusBusy) - 1), (uint8_t *)strBusBusy);
    }

    return (tErr == 0);
}

int main(void)
{
    hal_system_init();
    hal_gpio_init();        // Enable GPIO Clock
    hal_timer_init();

    hal_uart_init_hw(115200);
    hal_uart_send((sizeof(strVersion) - 1), (uint8_t *)strVersion);

    addr = 0x7F;
    StartAddr = 0x01;
    StopAddr = 0x7F;
    toWrite = 0xFF;
    toRead = 0xFF;

    scanmode = false;
    verbose = false;
    view = 1;       // HEX

    uint8_t mode = 0;
    uint16_t val = 0;
    uint8_t val_ptr = 0;
    uint16_t pause = 0;
    uint8_t repeat = 0;
    uint8_t pos;

    while(1)
    {
        // get data
        if(hal_uart_datardy() && (mode <= 'Z'))
        {
            uint8_t ch = hal_uart_get();

            if((mode != 0) && (val_ptr < 3) && (ch >= '0') && (ch <= '9'))
            {
                val *= 10;
                val += ch -'0';
                val_ptr++;
            }
            else
            {
                switch(mode)
                {
                    case 'A':           // Set Address
                        if(val_ptr != 0)
                        {
                            if(val > 127)
                            {
                                val = 127;
                            }

                            addr = val;
                            StartAddr = val;
                        }
                        scanmode = false;
                        mode = 0;
                        break;

                    case 'S':           // Scan Address
                        if(val_ptr != 0)
                        {
                            if(val > 127)
                            {
                                val = 127;
                            }

                            StopAddr = val;
                        }
                        scanmode = true;
                        mode = 0;
                        break;

                    case 'R':
                        if(val_ptr != 0)
                        {
                            if(val > 32)    // bytes to read
                            {
                                val = 32;
                            }
                            toRead = val;
                        }
                        else
                        {
                            toRead = 0xFF;
                        }
                        mode = 0;
                        break;

                    case 'W':
                        if(val_ptr != 0)
                        {
                            if(toWrite == 0xFF)
                            {
                                toWrite = 0x00;
                            }

                            if(toWrite < 32)
                            {
                                datawr[toWrite++] = val & 0xFF;
                            }
                        }
                        break;

                    case 'V':           // Scan verbose
                        if((val_ptr != 0) && (val < 3))
                        {
                            view = val;
                        }
                        else
                        {
                            if(verbose)
                            {
                                verbose = false;
                                pos = sprintf(print_buf, "Verbose Off, View:%d\r", view);
                            }
                            else
                            {
                                verbose = true;
                                pos = sprintf(print_buf, "Verbose On, View:%d\r", view);
                            }
                            hal_uart_send(pos, (uint8_t *)print_buf);
                        }
                        mode = 0;
                        break;

                    case 'P':           // Pause between executes
                        if(val_ptr != 0)
                        {
                            if(val > 0x7FFF)
                            {
                                val = 0x7FFF;
                            }

                            pause = val;
                        }
                        else
                        {
                            pause = 0;
                        }
                        mode = 0;
                        break;

                    case 'T':
                        if(val_ptr != 0)
                        {
                            repeat = val;
                        }
                        else
                        {
                            repeat = 0;
                        }
                        mode = 0;
                        break;

                    default:
                        break;
                }
                val = 0;
                val_ptr = 0;

                if(ch >= 'a' && ch <= 'z')
                {
                    ch -= 0x20;
                }

                // Parse New Command
                switch(ch)
                {
                    case 'C':               // Clear Settings
                        addr = 0x7F;
                        StartAddr = 0x01;
                        StopAddr = 0x7F;
                        toWrite = 0xFF;     // Default Write Strobe
                        toRead = 0xFF;
                        mode = 0;
                        break;

                    case 'A':               // Get Addr
                    case 'S':               // Get Scan Start and End Address
                    case 'R':               // bytes to read
                    case 'V':               // Verbose
                    case 'P':               // Pause
                    case 'T':               // Repeat executes
                        mode = ch;
                        break;

                    case 'W':               // data to write
                        toWrite = 0xFF;
                        mode = 'W';
                        break;
                        
                    case 'L':               // Show Settings
                        print_info();
                        mode = 0;
                        break;

                    case 'E':               // Execute
                        if(verbose)
                        {
                            print_info();
                        }
                        mode = 'E';

                        if(exec())
                        {
                            if(scanmode)
                            {
                                mode = 's';
                            }
                            else
                            {
                                mode = 'e';
                            }
                        }
                        break;

                    case 'H':
                    case '?':
                        hal_uart_send((sizeof(strHelp) - 1), (uint8_t *)strHelp);
                        mode = 0;
                        break;

                    default:
                        break;
                }
            }
        }

        if(mode == 'e')     // Execute
        {
            uint8_t stat = hal_twi_status();

            if((stat & (TWI_FL_RDY | TWI_FL_SLANACK | TWI_FL_ERROR)) != 0)
            {
                if(stat & TWI_FL_RDY)  // Ok Data Ready
                {
                    print_rdy();
                }
                else if(stat & TWI_FL_SLANACK)
                {
                    hal_uart_send((sizeof(strNACK) - 1), (uint8_t *)strNACK);
                }
                else
                {
                    hal_uart_send((sizeof(strBusError) - 1), (uint8_t *)strBusError);
                }

                hal_twi_disable();

                if((pause > 0) || (repeat > 0))
                {
                    hal_timer_start(pause);
                    mode = 'p';
                }
                else
                {
                    mode = 0;
                }
            }
        }
        else if(mode == 's')        // Scan Mode
        {
            uint8_t stat = hal_twi_status();

            if(stat & TWI_FL_ERROR)
            {
                hal_uart_send((sizeof(strBusError) - 1), (uint8_t *)strBusError);
                hal_twi_disable();
                mode = 0;
            }
            else if((stat & (TWI_FL_RDY | TWI_FL_SLANACK)) != 0)
            {
                if(stat & TWI_FL_RDY)
                {
                    print_rdy();
                }
                else if(verbose)
                {
                    pos = sprintf(print_buf, "Addr:%03d NACK\r", addr);
                    hal_uart_send(pos, (uint8_t *)print_buf);
                }

                if(addr < StopAddr)
                {
                    if(hal_twi_start(addr + 1, toWrite, toRead, datawr) == 0)
                    {
                        addr++;
                    }
                }
                else    // Scan complete
                {
                    hal_uart_send((sizeof(strOk) - 1), (uint8_t *)strOk);
                    hal_twi_disable();
                    mode = 0;
                }
            }
        }
        else if((mode == 'p') && hal_timer_timeout())
        {
            if(repeat > 1)
            {
                repeat--;
                if(exec())
                {
                    mode = 'e';
                }
                else
                {
                    mode = 0;
                }
            }
            else
            {
                repeat = 0;
                mode = 0;
            }
        }
    }

    return 0;
}
