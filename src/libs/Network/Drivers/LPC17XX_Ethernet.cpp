#if 1
#include "LPC17XX_Ethernet.h"

#include "Kernel.h"

#include <cstring>
#include <cstdio>

#include "lpc17xx_clkpwr.h"

#include <mri.h>

// #include "netcore.h"

#define DEBUG_PRINTF printf

static const uint8_t EMAC_clkdiv[] = { 4, 6, 8, 10, 14, 20, 28 };

/*--------------------------- write_PHY -------------------------------------*/
/*********************************************************************//**
* @brief       Write value to PHY device
* @param[in]   PhyReg: PHY Register address
* @param[in]   Value:  Value to write
* @return      0 - if success
*              1 - if fail
***********************************************************************/
static int32_t write_PHY (uint32_t PhyReg, uint16_t Value)
{
    /* Write a data 'Value' to PHY register 'PhyReg'. */
    uint32_t tout;

    LPC_EMAC->MADR = EMAC_DEF_ADR | PhyReg;
    LPC_EMAC->MWTD = Value;

    /* Wait until operation completed */
    tout = 0;
    for (tout = 0; tout < EMAC_MII_WR_TOUT; tout++) {
        if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
            return (0);
        }
    }
    DEBUG_PRINTF("write PHY %lu %04X failed!\n", PhyReg, Value);
    // Time out!
    return (-1);
}


/*--------------------------- read_PHY --------------------------------------*/
/*********************************************************************//**
* @brief       Read value from PHY device
* @param[in]   PhyReg: PHY Register address
* @return      0 - if success
*              1 - if fail
***********************************************************************/
static int32_t read_PHY (uint32_t PhyReg)
{
    /* Read a PHY register 'PhyReg'. */
    uint32_t tout;

    LPC_EMAC->MADR = EMAC_DEF_ADR | PhyReg;
    LPC_EMAC->MCMD = EMAC_MCMD_READ;

    /* Wait until operation completed */
    tout = 0;
    for (tout = 0; tout < EMAC_MII_RD_TOUT; tout++) {
        if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
            LPC_EMAC->MCMD = 0;
            return (LPC_EMAC->MRDD);
        }
    }
    DEBUG_PRINTF("read PHY %lu failed!\n", PhyReg);
    // Time out!
    return (-1);
}

/*********************************************************************//**
* @brief       Set Station MAC address for EMAC module
* @param[in]   abStationAddr Pointer to Station address that contains 6-bytes
*              of MAC address (should be in order from MAC Address 1 to MAC Address 6)
* @return      None
**********************************************************************/
static void setEmacAddr(uint8_t abStationAddr[])
{
    /* Set the Ethernet MAC Address registers */
    LPC_EMAC->SA0 = ((uint32_t)abStationAddr[5] << 8) | (uint32_t)abStationAddr[4];
    LPC_EMAC->SA1 = ((uint32_t)abStationAddr[3] << 8) | (uint32_t)abStationAddr[2];
    LPC_EMAC->SA2 = ((uint32_t)abStationAddr[1] << 8) | (uint32_t)abStationAddr[0];
}

/*********************************************************************//**
* @brief       Set specified PHY mode in EMAC peripheral
* @param[in]   ulPHYMode   Specified PHY mode, should be:
*                          - EMAC_MODE_AUTO
*                          - EMAC_MODE_10M_FULL
*                          - EMAC_MODE_10M_HALF
*                          - EMAC_MODE_100M_FULL
*                          - EMAC_MODE_100M_HALF
* @return      Return (0) if no error, otherwise return (-1)
**********************************************************************/
int32_t emac_SetPHYMode(uint32_t ulPHYMode)
{
    int32_t id1, id2, tout, regv;

    id1 = read_PHY (EMAC_PHY_REG_IDR1);
    id2 = read_PHY (EMAC_PHY_REG_IDR2);

    if (((id1 << 16) | (id2 & 0xFFF0)) == EMAC_SMSC_8720A) {
        switch(ulPHYMode){
            case EMAC_MODE_AUTO:
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_AUTO_NEG);
                /* Wait to complete Auto_Negotiation */
                for (tout = EMAC_PHY_RESP_TOUT; tout; tout--) {
                    regv = read_PHY (EMAC_PHY_REG_BMSR);
                    if (regv & EMAC_PHY_BMSR_AUTO_DONE) {
                        /* Auto-negotiation Complete. */
                        break;
                    }
                    if (tout == 0){
                        // Time out, return error
                        return (-1);
                    }
                }
                break;
            case EMAC_MODE_10M_FULL:
                /* Connect at 10MBit full-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_FULLD_10M);
                break;
            case EMAC_MODE_10M_HALF:
                /* Connect at 10MBit half-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_HALFD_10M);
                break;
            case EMAC_MODE_100M_FULL:
                /* Connect at 100MBit full-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_FULLD_100M);
                break;
            case EMAC_MODE_100M_HALF:
                /* Connect at 100MBit half-duplex */
                write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_HALFD_100M);
                break;
            default:
                // un-supported
                return (-1);
        }
    }
    // It's not correct module ID
    else {
        DEBUG_PRINTF("PHY reports id %04lX %04lX - not an SMSC 8720A\n", id1, id2);
        return (-1);
    }

    // Update EMAC configuration with current PHY status
    if (EMAC_UpdatePHYStatus() < 0){
        return (-1);
    }

    // Complete
    return (0);
}

_rxbuf_t LPC17XX_Ethernet::rxbuf __attribute__ ((section ("AHBSRAM1"))) __attribute__((aligned(8)));
_txbuf_t LPC17XX_Ethernet::txbuf __attribute__ ((section ("AHBSRAM1"))) __attribute__((aligned(8)));

LPC17XX_Ethernet* LPC17XX_Ethernet::instance;

LPC17XX_Ethernet::LPC17XX_Ethernet()
{
    // TODO these need to be configurable
    // mac_address[0] = 0xAE;
    // mac_address[1] = 0xF0;
    // mac_address[2] = 0x28;
    // mac_address[3] = 0x5D;
    // mac_address[4] = 0x66;
    // mac_address[5] = 0x41;

    // ip_address = IPA(192,168,3,222);
    // ip_mask = 0xFFFFFF00;

    for (int i = 0; i < LPC17XX_RXBUFS; i++) {
        rxbuf.rxdesc[i].packet = rxbuf.buf[i];
        rxbuf.rxdesc[i].control = (LPC17XX_MAX_PACKET - 1) | EMAC_RCTRL_INT;

        rxbuf.rxstat[i].Info = 0;
        rxbuf.rxstat[i].HashCRC = 0;
    }

    for (int i = 0; i < LPC17XX_TXBUFS; i++) {
        txbuf.txdesc[i].packet = txbuf.buf[i];
        txbuf.txdesc[i].control = (LPC17XX_MAX_PACKET - 1) | EMAC_TCTRL_PAD | EMAC_TCTRL_CRC | EMAC_TCTRL_LAST | EMAC_TCTRL_INT;

        txbuf.txstat[i].Info = 0;
    }

    interface_name = (uint8_t*) malloc(5);
    memcpy(interface_name, "eth0", 5);

    instance = this;

    up = false;
}

void LPC17XX_Ethernet::on_module_loaded()
{
    LPC_PINCON->PINSEL2 = (1 << 0) | (1 << 2) | (1 << 8) | (1 << 16) | (1 << 18) | (1 << 20) | (1 << 28) | (1 << 30);
    LPC_PINCON->PINSEL3 &= (2 << 0) | (2 << 2);
    LPC_PINCON->PINSEL3 |= (1 << 0) | (1 << 2);

    DEBUG_PRINTF("EMAC_INIT\n");
    emac_init();
    DEBUG_PRINTF("INIT OK\n");

    //register_for_event(ON_IDLE);
    register_for_event(ON_SECOND_TICK);
}

void LPC17XX_Ethernet::on_idle(void*)
{
    //_receive_frame();
}

void LPC17XX_Ethernet::on_second_tick(void *) {
    check_interface();
}

void LPC17XX_Ethernet::check_interface()
{
//     LPC_EMAC->Command = 0x303;
//     setEmacAddr(mac_address);

    uint32_t st;
    st  = read_PHY (EMAC_PHY_REG_BMSR);

    if ((st & EMAC_PHY_BMSR_LINK_ESTABLISHED) && (st & EMAC_PHY_BMSR_AUTO_DONE) && (up == false))
    {
        // TODO: link up event
        up = true;
//         net->set_interface_status(this, up);
        uint32_t scsr = read_PHY(EMAC_PHY_REG_SCSR);
        DEBUG_PRINTF("%s: link up: ", interface_name);
        switch ((scsr >> 2) & 0x7)
        {
            case 1:
                DEBUG_PRINTF("10MBit Half Duplex\n");
                break;
            case 5:
                DEBUG_PRINTF("10MBit Full Duplex\n");
                break;
            case 2:
                DEBUG_PRINTF("100MBit Half Duplex\n");
                break;
            case 6:
                DEBUG_PRINTF("100MBit Full Duplex\n");
                break;
            default:
                DEBUG_PRINTF("Unknown speed: SCSR = 0x%04lX\n", scsr);
                break;
        }
        DEBUG_PRINTF("MAC Address: %02lX:%02lX:%02lX:%02lX:%02lX:%02lX\n", (LPC_EMAC->SA2) & 0xFF, (LPC_EMAC->SA2 >> 8) & 0xFF, (LPC_EMAC->SA1) & 0xFF, (LPC_EMAC->SA1 >> 8) & 0xFF, (LPC_EMAC->SA0) & 0xFF, (LPC_EMAC->SA0 >> 8) & 0xFF);
    }
    else if (((st & EMAC_PHY_BMSR_LINK_ESTABLISHED) == 0) && up)
    {
        // TODO: link down event
        up = false;
//         net->set_interface_status(this, up);
        DEBUG_PRINTF("%s: link down\n", interface_name);
    }

    //DEBUG_PRINTF("PHY: id:%04lX %04lX st:%04lX\n", id1, id2, st);
    // DEBUG_PRINTF("ETH: Rx:%lu/%lu Tx:%lu/%lu\n", LPC_EMAC->RxConsumeIndex, LPC_EMAC->RxProduceIndex, LPC_EMAC->TxProduceIndex, LPC_EMAC->TxConsumeIndex);
    // DEBUG_PRINTF("MII: 0x%1lX\n", LPC_EMAC->MIND);
    // DEBUG_PRINTF("Command: 0x%03lX Status: 0x%1lX\n", LPC_EMAC->Command, LPC_EMAC->Status);
    // DEBUG_PRINTF("RxN: %lu TxN: %lu\n", LPC_EMAC->RxDescriptorNumber, LPC_EMAC->TxDescriptorNumber);
    // DEBUG_PRINTF("MAC1: 0x%04lX MAC2: 0x%04lX\n", LPC_EMAC->MAC1, LPC_EMAC->MAC2);
    // DEBUG_PRINTF("MAC Address: %02lX:%02lX:%02lX:%02lX:%02lX:%02lX\n", (LPC_EMAC->SA2) & 0xFF, (LPC_EMAC->SA2 >> 8) & 0xFF, (LPC_EMAC->SA1) & 0xFF, (LPC_EMAC->SA1 >> 8) & 0xFF, (LPC_EMAC->SA0) & 0xFF, (LPC_EMAC->SA0 >> 8) & 0xFF);
}

void LPC17XX_Ethernet::emac_init()
{
    /* Initialize the EMAC Ethernet controller. */
    int32_t regv,tout, tmp;
    volatile uint32_t d;

    /* Set up clock and power for Ethernet module */
    CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCENET, ENABLE);

    /* Reset all EMAC internal modules */
    LPC_EMAC->MAC1    = EMAC_MAC1_RES_TX | EMAC_MAC1_RES_MCS_TX | EMAC_MAC1_RES_RX |
    EMAC_MAC1_RES_MCS_RX | EMAC_MAC1_SIM_RES | EMAC_MAC1_SOFT_RES;

    LPC_EMAC->Command = EMAC_CR_REG_RES | EMAC_CR_TX_RES | EMAC_CR_RX_RES;

    /* A short delay after reset. */
    for (d = 256; d; d--);

    /* Initialize MAC control registers. */
    LPC_EMAC->MAC1 = EMAC_MAC1_PASS_ALL;
    LPC_EMAC->MAC2 = EMAC_MAC2_CRC_EN | EMAC_MAC2_PAD_EN | EMAC_MAC2_FULL_DUP;
    LPC_EMAC->MAXF = EMAC_ETH_MAX_FLEN;
    /*
     * Find the clock that close to desired target clock
     */
    tmp = SystemCoreClock / EMAC_MCFG_MII_MAXCLK;
    for (tout = 0; tout < (int32_t) sizeof (EMAC_clkdiv); tout++){
        if (EMAC_clkdiv[tout] >= tmp) break;
    }
    tout++;
    // Write to MAC configuration register and reset
    LPC_EMAC->MCFG = EMAC_MCFG_CLK_SEL(tout) | EMAC_MCFG_RES_MII;
    // release reset
    LPC_EMAC->MCFG &= ~(EMAC_MCFG_RES_MII);
    LPC_EMAC->CLRT = EMAC_CLRT_DEF;
    LPC_EMAC->IPGR = EMAC_IPGR_P2_DEF;

    /* Enable Reduced MII interface. */
    LPC_EMAC->Command = EMAC_CR_RMII;

    /* Reset Reduced MII Logic. */
    LPC_EMAC->SUPP = EMAC_SUPP_RES_RMII;

    for (d = 256; d; d--);
    LPC_EMAC->SUPP = EMAC_SUPP_SPEED;

    /* Put the DP83848C in reset mode */
    write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_BMCR_RESET);

    /* Wait for hardware reset to end. */
    for (tout = EMAC_PHY_RESP_TOUT; tout; tout--) {
        regv = read_PHY (EMAC_PHY_REG_BMCR);
        if (!(regv & (EMAC_PHY_BMCR_RESET | EMAC_PHY_BMCR_POWERDOWN))) {
            /* Reset complete, device not Power Down. */
            break;
        }
        if (tout == 0){
            // Time out, return ERROR
            DEBUG_PRINTF("ETH: PHY TIMEOUT\n");
            return;
        }
    }

    // Set PHY mode
//     if (emac_SetPHYMode(EMAC_MODE_AUTO) < 0){
//         DEBUG_PRINTF("ETH: Error Setting Mode\n");
//         return;
//     }
    write_PHY (EMAC_PHY_REG_BMCR, EMAC_PHY_AUTO_NEG);

    // Set EMAC address
    setEmacAddr(mac_address);

    /* Initialize Tx and Rx DMA Descriptors */
    LPC_EMAC->RxDescriptor       = (uint32_t) rxbuf.rxdesc;
    LPC_EMAC->RxStatus           = (uint32_t) rxbuf.rxstat;
    LPC_EMAC->RxDescriptorNumber = LPC17XX_RXBUFS-1;

    LPC_EMAC->TxDescriptor       = (uint32_t) txbuf.txdesc;
    LPC_EMAC->TxStatus           = (uint32_t) txbuf.txstat;
    LPC_EMAC->TxDescriptorNumber = LPC17XX_TXBUFS-1;

    // Set Receive Filter register: enable broadcast and multicast
    LPC_EMAC->RxFilterCtrl = EMAC_RFC_BCAST_EN | EMAC_RFC_PERFECT_EN;

    /* Enable Rx Done and Tx Done interrupt for EMAC */
    LPC_EMAC->IntEnable = EMAC_INT_RX_DONE | EMAC_INT_TX_DONE;

    /* Reset all interrupts */
    LPC_EMAC->IntClear  = 0xFFFF;

    /* Enable receive and transmit mode of MAC Ethernet core */
    LPC_EMAC->Command  = EMAC_CR_RX_EN | EMAC_CR_TX_EN | EMAC_CR_RMII | EMAC_CR_FULL_DUP | EMAC_CR_PASS_RUNT_FRM;
    LPC_EMAC->MAC1     |= EMAC_MAC1_REC_EN;

    DEBUG_PRINTF("ETH:EMAC INITIALISED\n");
}

void LPC17XX_Ethernet::set_mac(uint8_t* newmac)
{
    memcpy(mac_address, newmac, 6);
}

// size must be preloaded with max size of packet buffer
bool LPC17XX_Ethernet::_receive_frame(void *packet, int *size)
{
    if (can_read_packet() && can_write_packet())
    {
        int i = LPC_EMAC->RxConsumeIndex;
        RX_Stat* stat = &(rxbuf.rxstat[i]);
        int len = (stat->Info & EMAC_RINFO_SIZE) + 1; //this is the index so add one to get the size
        if(len <= *size) { // check against recieving buffer length
            memcpy(packet, rxbuf.buf[i], len);
            *size= len;
        }else{
            // discard frame that is too big for input buffer
            DEBUG_PRINTF("WARNING: Discarded ethernet frame that is too big: %08lX, %d - %d\n", stat->Info, len, *size);
            *size= 0;
        }

        //DEBUG_PRINTF("Received %d byte Ethernet frame %lu/%lu\n", *size, LPC_EMAC->RxProduceIndex, LPC_EMAC->RxConsumeIndex);

        uint32_t r = LPC_EMAC->RxConsumeIndex + 1;
        if (r > LPC_EMAC->RxDescriptorNumber)
            r = 0;
        LPC_EMAC->RxConsumeIndex = r;

        return *size > 0;
    }

    return false;
}

void LPC17XX_Ethernet::irq()
{
    // if (EMAC_IntGetStatus(EMAC_INT_RX_DONE))
    // {
    //     //_receive_frame();
    // }

    // if (EMAC_IntGetStatus(EMAC_INT_TX_DONE))
    // {
    // }
}

bool LPC17XX_Ethernet::can_read_packet()
{
    return (LPC_EMAC->RxProduceIndex != LPC_EMAC->RxConsumeIndex);
}

int LPC17XX_Ethernet::read_packet(uint8_t** buf)
{
    *buf = rxbuf.buf[LPC_EMAC->RxConsumeIndex];
    return rxbuf.rxstat[LPC_EMAC->RxConsumeIndex].Info & EMAC_RINFO_SIZE;
}

void LPC17XX_Ethernet::release_read_packet(uint8_t*)
{
    uint32_t r = LPC_EMAC->RxConsumeIndex + 1;
    if (r > LPC_EMAC->RxDescriptorNumber)
        r = 0;
    LPC_EMAC->RxConsumeIndex = r;
}

bool LPC17XX_Ethernet::can_write_packet()
{
    uint32_t r = LPC_EMAC->TxProduceIndex + 1;
    if (r > LPC_EMAC->TxDescriptorNumber)
        r = 0;
    return (r != LPC_EMAC->TxConsumeIndex);
}

int LPC17XX_Ethernet::write_packet(uint8_t* buf, int size)
{
    txbuf.txdesc[LPC_EMAC->TxProduceIndex].control = ((size - 1) & 0x7ff) | EMAC_TCTRL_LAST | EMAC_TCTRL_CRC | EMAC_TCTRL_PAD | EMAC_TCTRL_INT;

    uint32_t r = LPC_EMAC->TxProduceIndex + 1;
    if (r > LPC_EMAC->TxDescriptorNumber)
        r = 0;

    if (r == LPC_EMAC->TxConsumeIndex)
        return 0;

    LPC_EMAC->TxProduceIndex = r;

    return size;
}

void* LPC17XX_Ethernet::request_packet_buffer()
{
    return txbuf.txdesc[LPC_EMAC->TxProduceIndex].packet;
}

NET_PACKET  LPC17XX_Ethernet::get_new_packet_buffer(NetworkInterface* ni)
{
    if (ni != this)
        return NULL;

    return (NET_PACKET) request_packet_buffer();
}

NET_PAYLOAD LPC17XX_Ethernet::get_payload_buffer(NET_PACKET packet)
{
    return (NET_PAYLOAD) packet;
}

void LPC17XX_Ethernet::set_payload_length(NET_PACKET packet, int length)
{
    uint32_t offset = ((uint8_t*) packet) - txbuf.buf[0];
    int i = (offset / LPC17XX_MAX_PACKET);
    if ((i < LPC17XX_TXBUFS) && ((offset % LPC17XX_MAX_PACKET) == 0))
    {
        txbuf.txdesc[i].control = (txbuf.txdesc[i].control & ~EMAC_TCTRL_SIZE) | (length & EMAC_TCTRL_SIZE);
    }
}

int LPC17XX_Ethernet::receive(NetworkInterface* ni, NET_PACKET packet, int length)
{
    if (can_write_packet())
        return write_packet((uint8_t*) packet, length);
    return 0;
}

int LPC17XX_Ethernet::construct(NetworkInterface* ni, NET_PACKET packet, int length)
{
    return length;
}

extern "C" {
    void ENET_IRQHandler()
    {
        LPC17XX_Ethernet::instance->irq();
    }
}

// void LPC17XX_Ethernet::provide_net(netcore* n)
// {
//     NetworkInterface::provide_net(n);
//     up = false;
//     n->set_interface_status(this, up);
// }

#endif
