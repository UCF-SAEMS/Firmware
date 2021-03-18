#include "w5500_init.h"

#include "socket.h"

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include "ti_drivers_config.h"

#include <ti/display/Display.h>
#include "stdio.h"
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

static SPI_Handle masterSpi;
static SPI_Params spiParams;
static SPI_Transaction transaction;
static uint32_t i;
static bool transferOK;
static int32_t status;

#include "Application/zcl_samplelight.h"

#include "Application/lib/MCP23017/MCP23017.h"
#include "Application/myconfig.h"

/* W5500 Call Back Functions */
void wizchip_select(void)
{
  // wait for spi bus and lock
  Semaphore_pend(semSPIHandle, BIOS_WAIT_FOREVER);

  GPIO_write(GPIO_ETH_CS, false); // SSEL(CS)
}

void wizchip_deselect(void)
{
  GPIO_write(GPIO_ETH_CS, true); // SSEL(CS)

  // unlock spi bus
  Semaphore_post(semSPIHandle);
}

static void wizchip_readburst(uint8_t *pBuf, uint16_t len)
{

  transaction.count = len;
  transaction.txBuf = NULL;
  transaction.rxBuf = (void*) pBuf;

  transferOK = SPI_transfer(masterSpi, &transaction);
}

void wizchip_writeburst(uint8_t *pBuf, uint16_t len)
{
  transaction.count = len;
  transaction.txBuf = (void*) pBuf;
  transaction.rxBuf = NULL;

  transferOK = SPI_transfer(masterSpi, &transaction);
}

uint8_t wizchip_read()
{
  uint8_t rb;
  wizchip_readburst(&rb, 1);
  return rb;
}

static void wizchip_write(uint8_t wb)
{
  wizchip_writeburst(&wb, 1);
}

void delay_cnt(volatile unsigned int nCount)
{
  for (; nCount != 0; nCount--)
    ;
}

void Display_Net_Conf()
{
  uint8_t tmpstr[6] = { 0, };
  wiz_NetInfo gWIZNETINFO;

  ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);
  ctlwizchip(CW_GET_ID, (void*) tmpstr);

  // Display Network Information
  if (gWIZNETINFO.dhcp == NETINFO_DHCP)
    System_printf("\r\n===== %s NET CONF : DHCP =====\r\n", (char*) tmpstr);
  else
    System_printf("\r\n===== %s NET CONF : Static =====\r\n", (char*) tmpstr);

  uint8_t link = 0;
  ctlwizchip(CW_GET_PHYLINK, (void*) &link);
  wiz_PhyConf phyconf = { 0 };
  wizphy_getphystat(&phyconf);

  System_printf("\r\n LINK: %02x | SPEED: %02x | DUPLEX: %02x\r\n", link, phyconf.speed, phyconf.duplex);

  System_printf("\r\nMAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3],
                gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
  System_printf("IP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
  System_printf("GW: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
  System_printf("SN: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
  System_printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
}

void Net_Conf(wiz_NetInfo netinfo)
{
  // wiz_NetInfo structure declared in the main.c
  /*
   wiz_NetInfo gWIZNETINFO = {
   { 0x00, 0x08, 0xDC, 0x44, 0x55, 0x66 },             // Mac address
   { 192, 168, 1, 91 },                                // IP address
   { 255, 255, 255, 0},                                // Subnet mask
   { 192, 168, 1, 1},                                  // Gateway
   { 8, 8, 8, 8},                                      // DNS Server
   };
   */
  ctlnetwork(CN_SET_NETINFO, (void*) &netinfo);
}

void W5500_Init(SPI_Handle& device)
{

//  SPI_Params_init(&spiParams);
//  spiParams.bitRate = 8000000; // supports up to 80 Mhz but layout / clock speed will limit this
//  spiParams.dataSize = 8;
//  spiParams.frameFormat = SPI_POL0_PHA0;
//  masterSpi = SPI_open(CONFIG_SPI_W5500, &spiParams);
masterSpi = device;

  //    VERSIONR 0x0039 should be 0x04
  uint8_t buf[] = { 0x00, 0x39, 0b0000000 };
  wizchip_select();
  wizchip_writeburst(buf, 3);
  uint8_t ver = wizchip_read();
  wizchip_deselect();
  System_printf("WIZCHIP Ver: %02x!\r\n", ver);

  uint8_t memsize[2][8] = { { 2, 2, 2, 2, 2, 2, 2, 2 }, { 2, 2, 2, 2, 2, 2, 2, 2 } };

//    /* W5500 Chip Reset */
//    GPIO_write(GPIO_ETH_CS, true); // SSEL(CS)
//    Chip_GPIO_SetPinState(LPC_GPIO, GPIO_W5500_RST_PORT, GPIO_W5500_RST, false);
//    delay_cnt(5000);
//    GPIO_write(GPIO_ETH_CS, true); // SSEL(CS)
//    Chip_GPIO_SetPinState(LPC_GPIO, GPIO_W5500_RST_PORT, GPIO_W5500_RST, true);
//    delay_cnt(10000);

  /* Register Call back function */
  reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
  reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
  reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);

  /* W5500 Chip Initialization */
  if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1)
  {
    System_printf("WIZCHIP Initialized fail.\r\n");
  }
  else
  {
    System_printf("WIZCHIP Initialized!\r\n");
  }
}
