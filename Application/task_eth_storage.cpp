/*
 * task_eth_storage.cpp
 *
 *  Created on: Apr 16, 2021
 *      Author: Clayton
 */

/* Display Header files */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/display/DisplayExt.h>
#include <ti/display/AnsiColor.h>
#include "stdio.h"
#include <string.h>
#include "ti_drivers_config.h"
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/SPI.h>
#include "ti_drivers_config.h"
#include "rom_jt_154.h"
#include "Application/lib/MCP23017/MCP23017.h"
#include "Application/myconfig.h"

#include "nvintf.h"

#include "lib/W5500/w5500_init.h"
#include "lib/W5500/ioLibrary_Driver-master/Ethernet/W5500/w5500.h"
#include "lib/W5500/ioLibrary_Driver-master/Internet/DHCP/dhcp.h"
#include "lib/W5500/ioLibrary_Driver-master/Internet/DNS/dns.h"
#include "lib/W5500/ioLibrary_Driver-master/Internet/httpServer/httpServer.h"
#include "lib/W5500/ioLibrary_Driver-master/Internet/httpServer/httpParser.h"
#include "lib/W5500/ioLibrary_Driver-master/Internet/httpServer/httpUtil.h"
#include "Web/webassets.h"


#ifdef __cplusplus
extern "C" {
#endif
void ethstore_task(NVINTF_nvFuncts_t *pfnNV);
#ifdef __cplusplus
}
#endif

volatile bool ip_assigned = false;
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2


Semaphore_Struct semSPIStruct;
Semaphore_Handle semSPIHandle;  // Main SPI communication mutex

uint8_t g_send_buf[2048] = { 0 };
uint8_t g_recv_buf[2048] = { 0 };
char data[255] = { 0 };
uint8_t predefined_get_cgi_processor(uint8_t *uri_name, uint8_t *buf, uint16_t *len)
{
    if (strcmp((const char*) uri_name, "gettick.cgi") == 0)
    {
        *len = sprintf((char*) buf, "{\"ticks\":\"%u\"}",
        Clock_getTicks() * Clock_tickPeriod);
        return 1;
    }
}

uint8_t predefined_set_cgi_processor(uint8_t *uri_name, uint8_t *uri, uint8_t *buf, uint16_t *len)
{
  return 0;
}

uint32_t get1sTick(void)
{
  return ((Clock_getTicks() * Clock_tickPeriod) / (1000 * 1000));
}

void Callback_IPAssigned(void)
{
  System_printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
  System_flush();
  ip_assigned = true;
}

void Callback_IPConflict(void)
{
  System_printf("Callback: IP conflict!\r\n");
  System_flush();
}

void getHardwareMac(uint8_t *macAddress)
{
  if (!macAddress)
    return;

  uint64_t const eui64Address = (*((volatile uint64_t*) (FCFG1_BASE + FCFG1_O_MAC_15_4_0)));

  // Per TI, the first 2 bytes are unique (although it looks like 3 are, see OUI. For CC1352 it's 4b:12:00)
  // This means that 64b-16b = 48b, exactly what we need for an unregistered (fully random) eui48/mac address per device
  // As the high bytes are constant going to need to copy the 6 byte address out backwards, index 0 will always be constant (4b in this case).
  // https://e2e.ti.com/support/wireless-connectivity/sub-1-ghz/f/156/t/586103?RTOS-LAUNCHXL-CC1310-Obtaining-a-48-bit-unique-ID-for-each-device
  for (int i = 0; i <= 5; i++)
    macAddress[5 - i] = (((uint8_t*) &eui64Address))[i];

  // Ensure address is unicast, and an OUI respectfully. (Clear 2 LSBs)
  macAddress[0] &= ~((1 << 0) | (1 << 1));
}

#include "lib/SerialFlash/SerialFlash.h"

SerialFlashFile file;
const unsigned long testIncrement = 4096;

// RawHardwareTest - Check if a SPI Flash chip is compatible
// with SerialFlash by performing many read and write tests
// to its memory.
//
// The chip should be fully erased before running this test.
// Use the EraseEverything to do a (slow) full chip erase.
//
// Normally you should NOT access the flash memory directly,
// as this test program does.  You should create files and
// read and write the files.  File creation allocates space
// with program & erase boundaries within the chip, to allow
// reading from any other files while a file is busy writing
// or erasing (if created as erasable).
//
// If you discover an incompatible chip, please report it here:
// https://github.com/PaulStoffregen/SerialFlash/issues
// You MUST post the complete output of this program, and
// the exact part number and manufacturer of the chip.

float eraseBytesPerSecond(const unsigned char *id) {
  if (id[0] == 0x20) return 152000.0; // Micron
  if (id[0] == 0x01) return 500000.0; // Spansion
  if (id[0] == 0xEF) return 419430.0; // Winbond
  if (id[0] == 0xC2) return 279620.0; // Macronix
  return 320000.0; // guess?
}

void erase()
{
  unsigned long startMillis = ((Clock_getTicks() * Clock_tickPeriod) / 1000);
  unsigned char id[5];
  SerialFlash.readID(id);
  unsigned long size = SerialFlash.capacity(id);

  if (size > 0)
  {
    System_printf("\r\nFlash Memory has %u bytes.\r\n", size);
    System_printf("Erasing ALL Flash Memory:\r\n");
    // Estimate the (lengthy) wait time.
    int seconds = (float) size / eraseBytesPerSecond(id) + 0.5;
    System_printf("  estimated wait: %u seconds.\r\n", seconds);
    System_printf("  Yes, full chip erase is SLOW!\r\n");
    System_flush();

    SerialFlash.eraseAll();
    unsigned long dotMillis = ((Clock_getTicks() * Clock_tickPeriod) / 1000);
    unsigned char dotcount = 0;
    while (SerialFlash.ready() == false)
    {
      if (((Clock_getTicks() * Clock_tickPeriod) / 1000) - dotMillis > 1000)
      {
        dotMillis = dotMillis + 1000;
        System_printf(".");
        System_flush();
        dotcount = dotcount + 1;
        if (dotcount >= 60)
        {
          System_printf("\r\n");
          dotcount = 0;
        }
      }
    }
    if (dotcount > 0)
      System_printf("\r\n");
    System_printf("Erase completed\r\n");
    unsigned long elapsed = ((Clock_getTicks() * Clock_tickPeriod) / 1000) - startMillis;
    System_printf("  actual wait: %u seconds.\r\n", elapsed / 1000ul);
    System_flush();
  }
}

const char * id2chip(const unsigned char *id)
{
    if (id[0] == 0xEF) {
        // Winbond
        if (id[1] == 0x40) {
            if (id[2] == 0x14) return "W25Q80BV";
            if (id[2] == 0x15) return "W25Q16DV";
            if (id[2] == 0x17) return "W25Q64FV";
            if (id[2] == 0x18) return "W25Q128FV";
            if (id[2] == 0x19) return "W25Q256FV";
        }
    }
    if (id[0] == 0x01) {
        // Spansion
        if (id[1] == 0x02) {
            if (id[2] == 0x16) return "S25FL064A";
            if (id[2] == 0x19) return "S25FL256S";
            if (id[2] == 0x20) return "S25FL512S";
        }
        if (id[1] == 0x20) {
            if (id[2] == 0x18) return "S25FL127S";
        }
    }
    if (id[0] == 0xC2) {
        // Macronix
        if (id[1] == 0x20) {
            if (id[2] == 0x18) return "MX25L12805D";
        }
    }
    if (id[0] == 0x20) {
        // Micron
        if (id[1] == 0xBA) {
            if (id[2] == 0x20) return "N25Q512A";
            if (id[2] == 0x21) return "N25Q00AA";
        }
        if (id[1] == 0xBB) {
            if (id[2] == 0x22) return "MT25QL02GC";
        }
    }
    if (id[0] == 0xBF) {
        // SST
        if (id[1] == 0x25) {
            if (id[2] == 0x02) return "SST25WF010";
            if (id[2] == 0x03) return "SST25WF020";
            if (id[2] == 0x04) return "SST25WF040";
            if (id[2] == 0x41) return "SST25VF016B";
            if (id[2] == 0x4A) return "SST25VF032";
        }
        if (id[1] == 0x25) {
            if (id[2] == 0x01) return "SST26VF016";
            if (id[2] == 0x02) return "SST26VF032";
            if (id[2] == 0x43) return "SST26VF064";
        }
    }
    if (id[0] == 0x1F) {
            // Adesto
        if (id[1] == 0x89) {
                if (id[2] == 0x01) return "AT25SF128A";
            }
    }
    return "(unknown chip)";
}

void print_signature(const unsigned char *data)
{
  System_printf("data=");
  for (unsigned char i = 0; i < 8; i++)
  {
    System_printf("%c ", data[i]);
  }
  System_printf("\r\n");
}

void create_signature(unsigned long address, unsigned char *data)
{
    data[0] = address >> 24;
    data[1] = address >> 16;
    data[2] = address >> 8;
    data[3] = address;
    unsigned long hash = 2166136261ul;
    for (unsigned char i=0; i < 4; i++) {
        hash ^= data[i];
        hash *= 16777619ul;
    }
    data[4] = hash;
    data[5] = hash >> 8;
    data[6] = hash >> 16;
    data[7] = hash >> 24;
}

bool equal_signatures(const unsigned char *data1, const unsigned char *data2)
{
    for (unsigned char i=0; i < 8; i++) {
        if (data1[i] != data2[i]) return false;
    }
    return true;
}

bool is_erased(const unsigned char *data, unsigned int len)
{
    while (len > 0) {
        if (*data++ != 255) return false;
        len = len - 1;
    }
    return true;
}

void printbuf(const void *buf, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)buf;
  do {
    unsigned char b = *p++;
    System_printf("%x", b >> 4);
    System_printf("%x ", b & 15);
    //Serial.printf("%02X", *p++);
  } while (--len > 0);
  System_printf("\r\n");
}

bool test() {
  unsigned char buf[256], sig[256], buf2[8];
  unsigned long address, count, chipsize, blocksize;
  unsigned long usec;
  bool first;

  // Read the chip identification
  System_printf("\r\n");
  System_printf("Read Chip Identification:\r\n");
  SerialFlash.readID(buf);
  System_printf("  JEDEC ID:     %x %x %x\r\n", buf[0], buf[1], buf[2]);
  System_printf("  Part Number: %s\r\n", id2chip(buf));

  chipsize = SerialFlash.capacity(buf);
  System_printf("  Memory Size:  %u bytes\r\n", chipsize);

  if (chipsize == 0) return false;
  blocksize = SerialFlash.blockSize();
  System_printf("  Block Size:  %u bytes\r\n", blocksize);
  System_flush();

  // Read the entire chip.  Every test location must be
  // erased, or have a previously tested signature
  System_printf("\r\nReading Chip...\r\n");
  System_flush();
  memset(buf, 0, sizeof(buf));
  memset(sig, 0, sizeof(sig));
  memset(buf2, 0, sizeof(buf2));
  address = 0;
  count = 0;
  first = true;
  while (address < chipsize) {
    SerialFlash.read(address, buf, 8);
    //System_printf("  addr = ");
    //System_printf(address, HEX);
    //System_printf(", data = ");
    //printbuf(buf, 8);
    create_signature(address, sig);
    if (is_erased(buf, 8) == false) {
      if (equal_signatures(buf, sig) == false) {
        System_printf("  Previous data found at address %u\r\n", address);
        System_printf("  You must fully erase the chip before this test\r\n");
        System_printf("  found this: ");
        printbuf(buf, 8);
        System_printf("     correct: ");
        printbuf(sig, 8);
        System_flush();
        return false;
      }
    } else {
      count = count + 1; // number of blank signatures
    }
    if (first) {
      address = address + (testIncrement - 8);
      first = false;
    } else {
      address = address + 8;
      first = true;
    }
  }


  // Write any signatures that were blank on the original check
  if (count > 0) {
    System_printf("\r\n");
    System_printf("Writing %u signatures\r\n", count);
    memset(buf, 0, sizeof(buf));
    memset(sig, 0, sizeof(sig));
    memset(buf2, 0, sizeof(buf2));
    address = 0;
    first = true;
    while (address < chipsize) {
      SerialFlash.read(address, buf, 8);
      if (is_erased(buf, 8)) {
        create_signature(address, sig);
        //Serial.printf("write %08X: data: ", address);
        //printbuf(sig, 8);
        SerialFlash.write(address, sig, 8);
        while (!SerialFlash.ready()) ; // wait
        SerialFlash.read(address, buf, 8);
        if (equal_signatures(buf, sig) == false) {
          System_printf("  error writing signature at %u\r\n", address);
          System_printf("  Read this: ");
          printbuf(buf, 8);
          System_printf("  Expected:  ");
          printbuf(sig, 8);
          System_flush();
          return false;
        }
      }
      if (first) {
        address = address + (testIncrement - 8);
        first = false;
      } else {
        address = address + 8;
        first = true;
      }
    }
  } else {
    System_printf("  all signatures present from prior tests\r\n");
  }
  System_flush();

  // Read all the signatures again, just to be sure
  // checks prior writing didn't corrupt any other data
  System_printf("\r\nDouble Checking All Signatures:\r\n");
  memset(buf, 0, sizeof(buf));
  memset(sig, 0, sizeof(sig));
  memset(buf2, 0, sizeof(buf2));
  count = 0;
  address = 0;
  first = true;
  while (address < chipsize) {
    SerialFlash.read(address, buf, 8);
    create_signature(address, sig);
    if (equal_signatures(buf, sig) == false) {
      System_printf("  error in signature at %u\r\n", address);
      System_printf("  Read this: ");
      printbuf(buf, 8);
      System_printf("  Expected:  ");
      printbuf(sig, 8);
      System_flush();
      return false;
    }
    count = count + 1;
    if (first) {
      address = address + (testIncrement - 8);
      first = false;
    } else {
      address = address + 8;
      first = true;
    }
  }
  System_printf("  all %u signatures read ok\r\n", count);
  System_flush();

  // Read pairs of adjacent signatures
  // check read works across boundaries
  System_printf("\r\nChecking Signature Pairs\r\n");
  memset(buf, 0, sizeof(buf));
  memset(sig, 0, sizeof(sig));
  memset(buf2, 0, sizeof(buf2));
  count = 0;
  address = testIncrement - 8;
  first = true;
  while (address < chipsize - 8) {
    SerialFlash.read(address, buf, 16);
    create_signature(address, sig);
    create_signature(address + 8, sig + 8);
    if (memcmp(buf, sig, 16) != 0) {
      System_printf("  error in signature pair at %u\r\n", address);
      System_printf("  Read this: ");
      printbuf(buf, 16);
      System_printf("  Expected:  ");
      printbuf(sig, 16);
      System_flush();
      return false;
    }
    count = count + 1;
    address = address + testIncrement;
  }
  System_printf("  all %u signatures read ok\r\n", count);
  System_flush();

  // Write data and read while write in progress
  System_printf("\r\n");
  System_printf("Checking Read-While-Write (Program Suspend)\r\n");
  address = 256;
  while (address < chipsize) { // find a blank space
    SerialFlash.read(address, buf, 256);
    if (is_erased(buf, 256)) break;
    address = address + 256;
  }
  if (address >= chipsize) {
    System_printf("  error, unable to find any blank space!");
    return false;
  }
  for (int i=0; i < 256; i += 8) {
    create_signature(address + i, sig + i);
  }
  System_printf("  write 256 bytes at %u\r\n", address);
  SerialFlash.write(address, sig, 256);
  usec = Clock_getTicks() * Clock_tickPeriod;
  if (SerialFlash.ready()) {
    System_printf("  error, chip did not become busy after write");
    return false;
  }
  SerialFlash.read(0, buf2, 8); // read while busy writing
  while (!SerialFlash.ready()) ; // wait
  usec = Clock_getTicks() * Clock_tickPeriod - usec;
  System_printf("  write time was at %u microseconds\r\n", usec);
  SerialFlash.read(address, buf, 256);
  if (memcmp(buf, sig, 256) != 0) {
    System_printf("  error writing to flash");
    System_printf("  Read this: ");
    printbuf(buf, 256);
    System_printf("  Expected:  ");
    printbuf(sig, 256);
    return false;
  }
  create_signature(0, sig);
  if (memcmp(buf2, sig, 8) != 0) {
    System_printf("  error, incorrect read while writing");
    System_printf("  Read this: ");
    printbuf(buf2, 256);
    System_printf("  Expected:  ");
    printbuf(sig, 256);
    return false;
  }
  System_printf("  read-while-writing: ");
  printbuf(buf2, 8);
  System_printf("  test passed, good read while writing");
  System_flush();


  // Erase a block and read while erase in progress
  if (chipsize >= 262144 + blocksize + testIncrement) {
    System_printf("\r\n");
    System_printf("Checking Read-While-Erase (Erase Suspend)\r\n");
    memset(buf, 0, sizeof(buf));
    memset(sig, 0, sizeof(sig));
    memset(buf2, 0, sizeof(buf2));
    SerialFlash.eraseBlock(262144);
    usec = Clock_getTicks() * Clock_tickPeriod;
    Task_sleep(50 / Clock_tickPeriod);
    if (SerialFlash.ready()) {
      System_printf("  error, chip did not become busy after erase");
      return false;
    }
    SerialFlash.read(0, buf2, 8); // read while busy writing
    while (!SerialFlash.ready()) ; // wait
    usec = Clock_getTicks() * Clock_tickPeriod - usec;
    System_printf("  erase time was at %u microseconds\r\n", usec);
    // read all signatures, check ones in this block got
    // erased, and all the others are still intact
    address = 0;
    first = true;
    while (address < chipsize) {
      SerialFlash.read(address, buf, 8);
      if (address >= 262144 && address < 262144 + blocksize) {
        if (is_erased(buf, 8) == false) {
          System_printf("  error in erasing at %u\r\n", address);
          System_printf("  Read this: ");
          printbuf(buf, 8);
          return false;
        }
      } else {
        create_signature(address, sig);
        if (equal_signatures(buf, sig) == false) {
          System_printf("  error in signature at %u\r\n", address);
          System_printf("  Read this: ");
          printbuf(buf, 8);
          System_printf("  Expected:  ");
          printbuf(sig, 8);
          return false;
        }
      }
      if (first) {
        address = address + (testIncrement - 8);
        first = false;
      } else {
        address = address + 8;
        first = true;
      }
    }
    System_printf("  erase correctly erased %u bytes\r\n", blocksize);
    // now check if the data we read during erase is good
    create_signature(0, sig);
    if (memcmp(buf2, sig, 8) != 0) {
      System_printf("  error, incorrect read while erasing");
      System_printf("  Read this: ");
      printbuf(buf2, 256);
      System_printf("  Expected:  ");
      printbuf(sig, 256);
      return false;
    }
    System_printf("  read-while-erasing: ");
    printbuf(buf2, 8);
    System_printf("  test passed, good read while erasing\r\n");

  } else {
    System_printf("Skip Read-While-Erase, this chip is too small\r\n");
  }

  return true;
}

void ethstore_task(NVINTF_nvFuncts_t *pfnNV)
{
    while (!hardwareReady)
    {
        Task_sleep(50 * (1000 / Clock_tickPeriod));
    }

    mcpptr->pinMode(MCP_PinMap::ETH_RST, OUTPUT);
    mcpptr->pinMode(MCP_PinMap::FONT_CS, OUTPUT);

    /* W5500 Chip Reset */
    mcpptr->digitalWrite(MCP_PinMap::ETH_RST, false);
    Task_sleep(50 * (1000 / Clock_tickPeriod));

    mcpptr->digitalWrite(MCP_PinMap::ETH_RST, true);
    Task_sleep(100 * (1000 / Clock_tickPeriod));

    static SPI_Handle masterSpi;
    static SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 8000000; // supports up to 80 Mhz but layout / clock speed will limit this
    spiParams.dataSize = 8;
    spiParams.frameFormat = SPI_POL0_PHA0;
    masterSpi = SPI_open(CONFIG_SPI_W5500, &spiParams);

    Semaphore_Params semParams;

    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semSPIStruct, 1, &semParams);
    /* Obtain instance handle */
    semSPIHandle = Semaphore_handle(&semSPIStruct);

    SerialFlash.begin(masterSpi);

    unsigned char buf[256], sig[256], buf2[8];
    unsigned long address, count, chipsize, blocksize;
    unsigned long usec;
    bool first;

    System_printf("Read Chip Identification:\r\n");
    SerialFlash.readID(buf);
    System_printf("  JEDEC ID:     %x %x %x\r\n", buf[0], buf[1], buf[2]);
    System_printf("  Part Number: %s\r\n", id2chip(buf));
    System_flush();

//    erase();
//    test();
//    for (;;)
//    {
//      System_printf(".");
//      System_flush();
//      Task_sleep(500 * (1000 / Clock_tickPeriod));
//    }

    System_flush();
    W5500_Init(masterSpi);

    System_flush();

    // Give time for network to establish and handle ip negotiation
    Task_sleep(5000 * (1000 / Clock_tickPeriod));

    // 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
    uint8_t dhcp_buffer[1024];
    // 1K seems to be enough for this buffer as well
    uint8_t dns_buffer[1024];

    // This is a mock web server handler.
    char requestedURL[] = "index.html";

    web_FileAssetItem_Type match = ASSET_ERROR;
    for (int i = 0; i < NWEBASSETS; i++)
    {
      if (strncmp(requestedURL, webAssets[i].name, 20) == 0)
      {
        // This was a match
        match = (web_FileAssetItem_Type) i;
      }
    }

    if (match != ASSET_ERROR)
    {
      // We need to send this file
      web_FileAssetItem_t *item = &webAssets[match];

      System_printf("Sending: <%s> \r\n", item->name);

      // can not guarentee that there is going to be a \0 at the end of the data string
      // so a memcpy or similar should be used. TODO DMA transfer to W5500
      for (int i = 0; i < item->size; i++)
      {
        System_printf("%c", item->data[i]);
      }
      System_printf("\r\n------------- \r\n");
    }
    System_flush();

    System_printf("Calling DHCP_init()...\r\n");
    System_flush();
    wiz_NetInfo net_info = { .dhcp = NETINFO_DHCP };
    // use unique hardware mac address embedded in the micro
    getHardwareMac(net_info.mac);
    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    System_printf("Registering DHCP callbacks...\r\n");
    System_flush();
    reg_dhcp_cbfunc(Callback_IPAssigned, Callback_IPAssigned, Callback_IPConflict);

    System_printf("Calling DHCP_run()...\r\n");
    System_flush();
    // actually should be called in a loop, e.g. by timer
    uint32_t ctr = 10000;
    while ((!ip_assigned) && (ctr > 0))
    {
      DHCP_run();
      ctr--;
    }
    if (!ip_assigned)
    {
      System_printf("\r\nIP was not assigned :(\r\n");
      System_flush();
      return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);
    getDNSfromDHCP(net_info.dns);

    System_printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nDNS: %d.%d.%d.%d\r\n",
        net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
        net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
        net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3],
        net_info.dns[0], net_info.dns[1], net_info.dns[2], net_info.dns[3]
    );
    System_flush();


    System_printf("Calling wizchip_setnetinfo()...\r\n");
    System_flush();
    wizchip_setnetinfo(&net_info);

    System_printf("Calling DNS_init()...\r\n");
    System_flush();
    DNS_init(DNS_SOCKET, dns_buffer);

    uint8_t addr[4];
    {
      char domain_name[] = "eax.me";
      System_printf("Resolving domain name \"%s\"...\r\n", domain_name);
      System_flush();
      int8_t res = DNS_run(net_info.dns, (uint8_t*) &domain_name, addr);
      if (res != 1)
      {
        System_printf("DNS_run() failed, res = %d", res);
        System_flush();
        return;
      }
      System_printf("Result: %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
      System_flush();
    }

    System_flush();
    Task_sleep(1000 * (1000 / Clock_tickPeriod));

    uint8_t sockets[] = { 4 };
    g_send_buf[2047] = 1;
    g_recv_buf[2047] = 2;

    httpServer_init(g_send_buf, g_recv_buf, sizeof(sockets), sockets);
    reg_httpServer_cbfunc(NULL, NULL, get1sTick);

    reg_httpServer_webContent((uint8_t*) "index.html", (uint8_t*) "<html><body>hi</body></html>");
    for (;;)
    {
      httpServer_run(0);
      Task_sleep(100 * (1000 / Clock_tickPeriod));
    }
}
