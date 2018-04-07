#include "HAL/hal.h"
#include "HAL/hal_gpio.h"
#include "HAL/hal_uart.h"
#include "HAL/hal_twi.h"
#include "syscal/printf.h"

#include <string.h>

static const uint8_t strWellcome[] = "Wellcome\r\n";
static const uint8_t strBusBusy[] = "Bus Busy\r\n";
static const uint8_t strBusError[] = "Bus Error\r\n";
static const uint8_t strAbort[] = "Abort\r\n";
static const uint8_t strNACK[] = "NACK\r\n";
static const uint8_t strOk[] = "Ok\r\n";

static uint8_t addr, toWrite, toRead;
static uint8_t mode;

static uint8_t datawr[32];
static uint8_t datard[32];

static uint8_t StartAddr, StopAddr;
static bool  scanmode, verbose;

static void exec(void)
{
    char print_buf[256];
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
        pos = sprintf(print_buf, "Scan From: %d, To: %d\r\n", StartAddr, StopAddr);
        addr = StartAddr;
    }
    else
    {
        pos = sprintf(print_buf, "Addr: %d\r\n", addr);

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

static void abort(void)
{
    while(hal_uart_free() == false);
    hal_uart_send(sizeof(strAbort) - 1, (uint8_t *)strAbort);
    hal_twi_disable();
    mode = 0;
}

static void print_rdy(void)
{
    uint8_t sent;
    sent = hal_twi_get_data(datard);

    char print_buf[256];
    uint8_t pos = sprintf(print_buf, "Addr: %d ", addr);

    if(toWrite != 0xFF)
    {
        pos += sprintf(&print_buf[pos], "Sent: %d ", sent);
    }
    
    if(toRead != 0xFF)
    {
        pos += sprintf(&print_buf[pos], "Read:");

        uint8_t i;
        for(i = 0; i < toRead; i++)
        {
            pos += sprintf(&print_buf[pos], " %d", datard[i]);
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
    StartAddr = 0x00;
    StopAddr = 0x00;
    scanmode = false;
    verbose = false;

    static uint8_t val = 0;

    while(1)
    {
        // get data
        if(hal_uart_datardy())
        {
            uint8_t ch = hal_uart_get();

            if((mode != 0) && (ch >= '0') && (ch <= '9'))
            {
                val *= 10;
                val += ch -'0';
            }
            else
            {
                switch(mode)
                {
                    case 'A':           // Set Address
                        if(val > 127)
                        {
                            val = 127;
                        }
                        else if(val < 1)
                        {
                            val = 1;
                        }

                        addr = val;
                        StartAddr = val;
                        scanmode = false;
                        mode = 0;
                        break;

                    case 'S':           // Scan Address
                        if(val > 127)
                        {
                            val = 127;
                        }
                        else if(val < 1)
                        {
                            val = 1;
                        }

                        StopAddr = val;
                        scanmode = true;
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
                            datawr[toWrite++] = val;
                        }
                        break;

                    case 'E':           // Abort of Execute or Scan
                    case 's':
                        abort();
                        break;
                        
                    case 'V':           // Scan verbose
                        if(val != 0)
                        {
                            verbose = true;
                        }
                        else
                        {
                            verbose = false;
                        }
                        break;

                    default:
                        break;
                }
                val = 0;

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

                    case 'G':
                    case 'g':
                        addr = 0x00;
                        toWrite = 0x01;
                        datawr[0] = 0x06;   // Global Call - Command Reset
                        toRead = 0xFF;
                        scanmode = false;
                        mode = 0;
                        exec();
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
                        toWrite = 0;
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
                    char print_buf[256];
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
