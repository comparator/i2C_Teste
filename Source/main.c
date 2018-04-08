#include "HAL/hal.h"
#include "HAL/hal_gpio.h"
#include "HAL/hal_uart.h"
#include "HAL/hal_twi.h"
#include "syscal/printf.h"

#include <string.h>

static const uint8_t strWellcome[] = "Wellcome\r\n";
static const uint8_t strBusBusy[] = "Bus Busy\r\n";
static const uint8_t strBusError[] = "Bus Error\r\n";
static const uint8_t strNACK[] = "NACK\r\n";
static const uint8_t strOk[] = "Ok\r\n";
static const uint8_t strHelp[] = "C - clear settings\r\nA - Set Current Address\r\nS - Set Max Scan Address\r\nR - Butes to read\r\nW - data to write\r\nV 0-2 show dec/hex/ascii, 4 - verbose scan\r\nE - execute\r\n";

static uint8_t addr, toWrite, toRead;
static uint8_t mode, view;

static uint8_t datawr[32];
static uint8_t datard[32];

static uint8_t StartAddr, StopAddr;
static bool  scanmode, verbose;
char print_buf[256];

static void exec(void)
{
    uint8_t pos;

    while(hal_uart_free() == false);

    // Check GPIO
    if( ((hal_twi_status() & TWI_FL_BUSY) != 0) || 
         (hal_twi_check_pin() == false))
    {
        hal_uart_send((sizeof(strBusBusy) - 1), (uint8_t *)strBusBusy);
        return;
    }

    if((toWrite == 0xFF) && (toRead == 0xFF))   // No data
    {
        toRead = 0x00;                          // Read Strobe
    }

    if(scanmode)
    {
        pos = sprintf(print_buf, "Scan From:%d To:%d\r\n", StartAddr, StopAddr);
        addr = StartAddr;
    }
    else
    {
        pos = sprintf(print_buf, "Addr:%d ", addr);

        if(toRead != 0xFF)
        {
            pos += sprintf(&print_buf[pos], "Rd:%d ", toRead);
        }

        if(toWrite != 0xFF)
        {
            pos += sprintf(&print_buf[pos], "Wr:");

            uint8_t i;
            for(i = 0; i < toWrite; i++)
            {
                if(view == 1)
                {
                    pos += sprintf(&print_buf[pos], "%02X ", datawr[i]);
                }
                else if(view == 2)
                {
                    pos += sprintf(&print_buf[pos], "%c", datawr[i]);
                }
                else
                {
                    pos += sprintf(&print_buf[pos], "%03d ", datawr[i]);
                }
            }
            if(view == 2)
            {
                print_buf[pos++] =' ';
            }
        }
    }

    hal_twi_enable();
    uint8_t tErr = hal_twi_start(addr, toWrite, toRead, datawr);
    
    if(tErr == TWI_FL_ERROR)
    {
        hal_twi_disable();
        pos += sprintf(&print_buf[pos], "%s", strBusError);
    }
    else if(tErr == TWI_FL_BUSY)
    {
        hal_twi_disable();
        pos += sprintf(&print_buf[pos], "%s", strBusBusy);
    }
    else
    {
        if(scanmode)
        {
            mode = 's';
        }
        else
        {
            mode = 'E';
        }
    }

    hal_uart_send(pos, (uint8_t *)print_buf);
}

static void print_rdy(void)
{
    uint8_t sent, pos = 0;
    sent = hal_twi_get_data(datard);

    if(mode == 's')
    {
        pos = sprintf(print_buf, "Addr:%d", addr);
    }

    if((toWrite != 0xFF) && (toWrite != 0x00))
    {
        pos += sprintf(&print_buf[pos], " Sent:%d", sent);
    }

    if((toRead != 0xFF) && (toRead != 0x00))
    {
        pos += sprintf(&print_buf[pos], " Read:");

        uint8_t i;
        for(i = 0; i < toRead; i++)
        {
            if(view == 1)
            {
                pos += sprintf(&print_buf[pos], "%02X ", datard[i]);
            }
            else if(view == 2)
            {
                uint8_t ichar = datard[i];
                if((ichar < 0x20) || (ichar > 0x7E))
                {
                    ichar = '.';
                }

                pos += sprintf(&print_buf[pos], "%c", ichar);
            }
            else
            {
                pos += sprintf(&print_buf[pos], "%03d ", datard[i]);
            }
        }
    }

    pos += sprintf(&print_buf[pos], "\r\n");
    hal_uart_send(pos, (uint8_t *)print_buf);
}

int main(void)
{
    hal_system_init();
    hal_gpio_init();        // Enable GPIO Clock

    hal_uart_init_hw(115200);
    hal_uart_send((sizeof(strWellcome) - 1), (uint8_t *)strWellcome);

    addr = 0x7F;
    toWrite = 0xFF;
    toRead = 0xFF;
    mode = 0;
    view = 0;
    StartAddr = 0x00;
    StopAddr = 0x00;
    scanmode = false;
    verbose = false;

    static uint8_t val = 0;
    static bool val_pr = false;

    while(1)
    {
        // get data
        if(hal_uart_datardy() && (mode != 'E') && (mode != 's'))
        {
            uint8_t ch = hal_uart_get();

            if((mode != 0) && (ch >= '0') && (ch <= '9'))
            {
                val *= 10;
                val += ch -'0';
                val_pr = true;
            }
            else
            {
                switch(mode)
                {
                    case 'A':           // Set Address
                        if(val_pr)
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
                        if(val_pr)
                        {
                            if(val > 127)
                            {
                                val = 127;
                            }

                            StopAddr = val;
                        }
                        scanmode = true;
                        break;

                    case 'R':
                        if(val_pr)
                        {
                            if(val > 32)    // bytes to read
                            {
                                val = 32;
                            }
                        }
                        else
                        {
                            val = 0xFF;
                        }

                        toRead = val;
                        mode = 0;
                        break;

                    case 'W':
                        if(val_pr)
                        {
                            if(toWrite < 32)
                            {
                                datawr[toWrite++] = val;
                            }
                        }
                        break;

                    case 'V':           // Scan verbose
                        if(val < 3)
                        {
                            view = val;
                        }
                        else if(val == 4)
                        {
                            verbose = true;
                        }
                        else
                        {
                            verbose = false;
                            view = 0;
                        }
                        break;

                    default:
                        break;
                }
                val = 0;
                val_pr = false;

                // Parse New Command
                switch(ch)
                {
                    case 'C':               // Clear Settings
                    case 'c':
                        addr = 0xFF;
                        toWrite = 0xFF;
                        toRead = 0xFF;
                        mode = 0;
                        break;

                    case 'A':               // Get Addr
                    case 'a':
                        mode = 'A';
                        break;
                        
                    case 'S':               // Get Scan Start and End Address
                    case 's':
                        mode = 'S';
                        break;

                    case 'R':               // bytes to read
                    case 'r':
                        mode = 'R';
                        break;

                    case 'W':               // data to write
                    case 'w':
                        toWrite = 0x00;
                        mode = 'W';
                        break;

                    case 'E':               // Execute
                    case 'e':
                        exec();
                        break;
                        
                    case 'V':
                    case 'v':
                        mode = 'V';
                        break;
                        
                    case 'h':
                    case 'H':
                    case '?':
                        while(hal_uart_free() == false);
                        hal_uart_send((sizeof(strHelp) - 1), (uint8_t *)strHelp);
                        break;

                    default:
                        if(mode != 'W')
                        {
                            mode = 0;
                        }
                        break;
                }
            }
        }

        if(mode == 'E')     // Execute
        {
            uint8_t stat = hal_twi_status();
            
            if((stat & (TWI_FL_RDY | TWI_FL_SLANACK | TWI_FL_ERROR)) != 0)
            {
                while(hal_uart_free() == false);

                if(stat & TWI_FL_RDY)  // Ok Data Ready
                {
                    print_rdy();
                }
                else if(stat & TWI_FL_SLANACK)
                {
                    hal_uart_send((sizeof(strNACK) - 1), (uint8_t *)strNACK);
                }
                else if(stat & TWI_FL_ERROR)
                {
                    hal_uart_send((sizeof(strBusError) - 1), (uint8_t *)strBusError);
                }

                hal_twi_disable();
                mode = 0;
            }
        }
        else if(mode == 's')        // Scan Mode
        {
            uint8_t stat = hal_twi_status();

            if(stat & TWI_FL_ERROR)
            {
                while(hal_uart_free() == false);
                hal_uart_send((sizeof(strBusError) - 1), (uint8_t *)strBusError);
                hal_twi_disable();
                mode = 0;
            }
            else if((stat & (TWI_FL_RDY | TWI_FL_SLANACK)) != 0)
            {
                while(hal_uart_free() == false);

                if(stat & TWI_FL_RDY)
                {
                    print_rdy();
                }
                else if(verbose)
                {
                    uint8_t pos = sprintf(print_buf, "Addr: %d NACK\r\n", addr);
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
                    while(hal_uart_free() == false);
                    hal_uart_send((sizeof(strOk) - 1), (uint8_t *)strOk);
                    hal_twi_disable();
                    mode = 0;
                }
            }
        }
    }

    return 0;
}
