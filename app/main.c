/*
 * Copyright (C) 2017 Baptiste CLENET
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @file
 * @brief       OpenThread Ncp test application
 *
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 */

#include <stdio.h>
#include "ot.h"
#include <openthread/udp.h>
#include <openthread/cli.h>
#include <openthread/openthread.h>
#include "periph/gpio.h"
#include "sched.h"
#include "xtimer.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/* LED pin configuration */
#define TX_PIN GPIO_PIN(0, 27) // D1
#define RX_PIN GPIO_PIN(1, 23) // D2
#define MCU_PIN GPIO_PIN(1, 22) // D3
#define WAN_PIN GPIO_PIN(0, 23) // D5

uint16_t myRloc = 0;

#ifdef CPU_DUTYCYCLE_MONITOR
volatile uint64_t cpuOnTime = 0;
volatile uint64_t cpuOffTime = 0;
volatile uint32_t contextSwitchCnt = 0;
volatile uint32_t preemptCnt = 0;
#endif
#ifdef RADIO_DUTYCYCLE_MONITOR
uint64_t radioOnTime = 0;
uint64_t radioOffTime = 0;
#endif

uint32_t packetSuccessCnt = 0;
uint32_t packetFailCnt = 0;
uint32_t packetBusyChannelCnt = 0;
uint32_t broadcastCnt = 0;
uint32_t queueOverflowCnt = 0;

uint32_t totalIpv6MsgCnt = 0;
uint32_t Ipv6TxSuccCnt = 0;
uint32_t Ipv6TxFailCnt = 0;
uint32_t Ipv6RxSuccCnt = 0;
uint32_t Ipv6RxFailCnt = 0;

uint16_t nextHopRloc = 0;
uint8_t borderRouterLC = 0;
uint8_t borderRouterRC = 0;
uint32_t borderRouteChangeCnt = 0;
uint32_t routeChangeCnt = 0;

uint32_t pollMsgCnt = 0;
uint32_t mleMsgCnt = 0;

uint32_t mleRouterMsgCnt = 0;
uint32_t addrMsgCnt = 0;
uint32_t netdataMsgCnt = 0;

uint32_t meshcopMsgCnt = 0;
uint32_t tmfMsgCnt = 0;

uint32_t totalSerialMsgCnt = 0;

#define BLINK_INTERVAL 1000000ul
#define PAYLOAD_SIZE (89)

static uint8_t source = OPENTHREAD_SOURCE;
static char buf[PAYLOAD_SIZE];

static otUdpSocket mSocket;
static otMessageInfo messageInfo;
static otMessage *message = NULL;

void wdt_clear(void) {
    volatile uint8_t* wdt_clear = (volatile uint8_t*) 0x40001008;
    *wdt_clear = 0xA5;
}

void wdt_setup(void) {
    /* Enable the bus for WDT and PAC0. */
    volatile uint32_t* pm_apbamask = (volatile uint32_t*) 0x40000418;
    *pm_apbamask = 0x0000007f;

    /* Setup GCLK_WDT at (32 kHz) / (2 ^ (7 + 1)) = 128 Hz. */
    GCLK->GENDIV.reg  = (GCLK_GENDIV_ID(5)  | GCLK_GENDIV_DIV(7));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_ID(5) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIVSEL |
                         GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_SRC_OSCULP32K);
    while (GCLK->STATUS.bit.SYNCBUSY);

    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_GEN(5) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_WDT_Val));
    while (GCLK->STATUS.bit.SYNCBUSY);

    volatile uint8_t* wdt_status = (volatile uint8_t*) 0x40001007;
    volatile uint8_t* wdt_config = (volatile uint8_t*) 0x40001001;
    //volatile uint8_t* wdt_ctrl = (volatile uint8_t*) 0x40001000;

    while (*wdt_status);
    /* Set up the WDT to reset after 4096 cycles (32 s), if not cleared. */
    *wdt_config = 0x09;
    while (*wdt_status);
    /* Enable the WDT. */
    //*wdt_ctrl = 0x02;
    //while (*wdt_status);
}

void br_on_tx(void) {
    gpio_toggle(TX_PIN);
}

void br_on_rx(void) {
    gpio_toggle(RX_PIN);
}

/* Possible values fow wan_status */
#define FULLOFF  1
#define FULLON  2
#define BLINKING1  3
#define BLINKING2  4
#define BLINKING3  5
volatile int wan_status = FULLOFF;

/* Maximum time since last heartbeat before LEDs start blinking. */
#define MAX_HB_TIME 1600000u
volatile uint64_t last_hb = 0;

void br_on_serial(void) {
    //gpio_toggle(WAN_PIN);
    //wdt_clear();
}

void on_udp_heartbeat(void* aContext, otMessage* aMessage, const otMessageInfo* aMessageInfo) {
    (void) aContext;
    (void) aMessageInfo;

    if (aMessage == NULL) {
        return;
    }

    uint16_t offset = otMessageGetOffset(aMessage);
    uint16_t length = otMessageGetLength(aMessage);
    if (offset == length) {
        wan_status = FULLOFF;
    } else {
        char firstbyte;
        otMessageRead(aMessage, offset, &firstbyte, 1);
        wan_status = (int) firstbyte;
        last_hb = xtimer_now_usec64();
    }
}

void fill_buffer(void);

void openthread_lock_coarse_mutex(void);
void openthread_unlock_coarse_mutex(void);

int main(void) {
    /* Set up the watchdog before anything else */
    wdt_setup();

    /* Initialize gpio pins */
    gpio_init(TX_PIN, GPIO_OUT);
    gpio_init(RX_PIN, GPIO_OUT);
    gpio_init(MCU_PIN, GPIO_OUT);
    gpio_init(WAN_PIN, GPIO_OUT);
    gpio_clear(TX_PIN);
    gpio_clear(RX_PIN);
    gpio_set(MCU_PIN);
    gpio_clear(WAN_PIN);

    /* Run wpantund to interact with NCP */
    DEBUG("This is a TCP-capable Border Router\n");

    /* Wait for OpenThread. */
    xtimer_usleep(100000U);

    openthread_lock_coarse_mutex();

    /* Prepare for sending/receiving UDP packets */
    otError error;
    {
        otSockAddr addr;
        memset(&addr.mAddress, 0x00, sizeof(addr.mAddress)); // Unspecifed
        addr.mPort = 12345;
        addr.mScopeId = 0; // All interfaces
        otUdpOpen(openthread_get_instance(), &mSocket, on_udp_heartbeat, NULL);
        otUdpBind(&mSocket, &addr);
    }

    /* Get my IP address. */
    const otIp6Address* address = otThreadGetMeshLocalEid(openthread_get_instance());

    memset(&messageInfo, 0x00, sizeof(messageInfo));
    memcpy(&messageInfo.mPeerAddr, address, sizeof(messageInfo.mPeerAddr));
    //otIp6AddressFromString("fdde:ad00:beef:0000:e9f0:45bc:c507:6f0e", &messageInfo.mPeerAddr);
    messageInfo.mPeerPort = 54321;
    messageInfo.mInterfaceId = 1;

    {
        otNetifAddress addr;
        memset(&addr, 0x00, sizeof(addr));

        // Copy the first eight bytes of the address
        memcpy(&addr.mAddress.mFields.m8[0], &address->mFields.m8[0], 8);
        memset(&addr.mAddress.mFields.m8[8], 0x00, 7);
        addr.mAddress.mFields.m8[15] = 1;
        addr.mPrefixLength = 128;
        addr.mValid = true;
        error = otIp6AddUnicastAddress(openthread_get_instance(), &addr);
        printf("Adding unicast address: %d\n", (int) error);
    }

    openthread_unlock_coarse_mutex();

    /* Clear the heartbeat buffer. */
    for (int i = 0; i < PAYLOAD_SIZE; i++) {
        buf[i] = 0x0;
    }

    int count = 0;
    for (;;) {
        /* Execute the following once every 100 ms */
        xtimer_usleep(100000U);

        count++;
        int interval = count & 0xf;

        openthread_lock_coarse_mutex();

        if (xtimer_now_usec64() - last_hb < MAX_HB_TIME) {
            /* Heartbeats are OK; clear watchdog */
            wdt_clear();

            /* Handle MCU LED */
            gpio_set(MCU_PIN);

            /* Handle WAN LED */
            switch(wan_status) {
            case BLINKING1:
                if (interval == 0) {
                    gpio_set(WAN_PIN);
                } else {
                    gpio_clear(WAN_PIN);
                }
                break;
            case BLINKING2:
                if (interval == 0 || interval == 2) {
                    gpio_set(WAN_PIN);
                } else {
                    gpio_clear(WAN_PIN);
                }
                break;
            case BLINKING3:
                if (interval == 0 || interval == 2 || interval == 4) {
                    gpio_set(WAN_PIN);
                } else {
                    gpio_clear(WAN_PIN);
                }
                break;
            case FULLON:
                gpio_set(WAN_PIN);
                break;
            case FULLOFF:
            default:
                gpio_clear(WAN_PIN);
                break;
            }
        } else {
            if ((count & 0x8) == 0) {
                gpio_set(MCU_PIN);
            } else {
                gpio_clear(MCU_PIN);
            }
            gpio_clear(WAN_PIN);
        }

        if ((count & 0x7) == 0) {
            /* Send a UDP packet, as a heartbeat to the Raspberry Pi. */
            message = otUdpNewMessage(openthread_get_instance(), true);
            if (message == NULL) {
                DEBUG("error in new message\n");
            } else {
                error = otMessageSetLength(message, PAYLOAD_SIZE);
                if (error != OT_ERROR_NONE) {
                    DEBUG("error in set length\n");
                    otMessageFree(message);
                } else {
                    fill_buffer();
                    otMessageWrite(message, 0, buf, PAYLOAD_SIZE);

                    DEBUG("\n[Main] Tx UDP packet %u\n", buf[3]*256+buf[4]);
                    error = otUdpSend(&mSocket, message, &messageInfo);
                    if (error != OT_ERROR_NONE) {
                        DEBUG("error in udp send\n");
                        otMessageFree(message);
                    }
                }
            }
        }

        openthread_unlock_coarse_mutex();
    }
    /* should be never reached */
    return 0;
}

void fill_buffer(void) {
    /* Identity */
    buf[0] = source;
    buf[2] = myRloc & 0xff;
    buf[1] = (myRloc >> 8) & 0xff;

    /* Sequence Number */
    buf[4]++;
    if (buf[4] == 0) {
        buf[3]++;
    }

#ifdef CPU_DUTYCYCLE_MONITOR
    /* context switch */
    buf[8] = contextSwitchCnt & 0xff;
    buf[7] = (contextSwitchCnt >> 8) & 0xff;
    buf[6] = (contextSwitchCnt >> 16) & 0xff;
    buf[5] = (contextSwitchCnt >> 24) & 0xff;

    /* context switch */
    buf[12] = preemptCnt & 0xff;
    buf[11] = (preemptCnt >> 8) & 0xff;
    buf[10] = (preemptCnt >> 16) & 0xff;
    buf[9] = (preemptCnt >> 24) & 0xff;

    uint16_t cpuDutycycle = (uint16_t) (10000 * cpuOnTime / (cpuOnTime + cpuOffTime));
    buf[14] = cpuDutycycle & 0xff;
    buf[13] = (cpuDutycycle >> 8) & 0xff;
#endif
#ifdef RADIO_DUTYCYCLE_MONITOR
    /* Radio duty-cycle */
    uint16_t radioDutycycle = (uint16_t) (10000 * radioOnTime / (radioOnTime + radioOffTime));
    buf[16] = radioDutycycle & 0xff;
    buf[15] = (radioDutycycle >> 8) & 0xff;
#endif
    /* Link Performance */
    buf[20] = packetSuccessCnt & 0xff;
    buf[19] = (packetSuccessCnt >> 8) & 0xff;
    buf[18] = (packetSuccessCnt >> 16) & 0xff;
    buf[17] = (packetSuccessCnt >> 24) & 0xff;

    buf[24] = packetBusyChannelCnt & 0xff;
    buf[23] = (packetBusyChannelCnt >> 8) & 0xff;
    buf[22] = (packetBusyChannelCnt >> 16) & 0xff;
    buf[21] = (packetBusyChannelCnt >> 24) & 0xff;

    buf[28] = packetFailCnt & 0xff;
    buf[27] = (packetFailCnt >> 8) & 0xff;
    buf[26] = (packetFailCnt >> 16) & 0xff;
    buf[25] = (packetFailCnt >> 24) & 0xff;

    buf[32] = broadcastCnt & 0xff;
    buf[31] = (broadcastCnt >> 8) & 0xff;
    buf[30] = (broadcastCnt >> 16) & 0xff;
    buf[29] = (broadcastCnt >> 24) & 0xff;

    /* Queue Overflow */
    buf[36] = queueOverflowCnt & 0xff;
    buf[35] = (queueOverflowCnt >> 8) & 0xff;
    buf[34] = (queueOverflowCnt >> 16) & 0xff;
    buf[33] = (queueOverflowCnt >> 24) & 0xff;

    /* Ipv6 Overhead */
    buf[40] = Ipv6TxSuccCnt & 0xff;
    buf[39] = (Ipv6TxSuccCnt >> 8) & 0xff;
    buf[38] = (Ipv6TxSuccCnt >> 16) & 0xff;
    buf[37] = (Ipv6TxSuccCnt >> 24) & 0xff;

    buf[44] = Ipv6TxFailCnt & 0xff;
    buf[43] = (Ipv6TxFailCnt >> 8) & 0xff;
    buf[42] = (Ipv6TxFailCnt >> 16) & 0xff;
    buf[41] = (Ipv6TxFailCnt >> 24) & 0xff;

    buf[48] = Ipv6RxSuccCnt & 0xff;
    buf[47] = (Ipv6RxSuccCnt >> 8) & 0xff;
    buf[46] = (Ipv6RxSuccCnt >> 16) & 0xff;
    buf[45] = (Ipv6RxSuccCnt >> 24) & 0xff;

    buf[52] = Ipv6RxFailCnt & 0xff;
    buf[51] = (Ipv6RxFailCnt >> 8) & 0xff;
    buf[50] = (Ipv6RxFailCnt >> 16) & 0xff;
    buf[49] = (Ipv6RxFailCnt >> 24) & 0xff;

    /* Route toward the BR */
    buf[54] = nextHopRloc & 0xff;
    buf[53] = (nextHopRloc >> 8) & 0xff;

    buf[55] = borderRouterLC;
    buf[56] = borderRouterRC;

    buf[60] = borderRouteChangeCnt & 0xff;
    buf[59] = (borderRouteChangeCnt >> 8) & 0xff;
    buf[58] = (borderRouteChangeCnt >> 16) & 0xff;
    buf[57] = (borderRouteChangeCnt >> 24) & 0xff;

    buf[64] = routeChangeCnt & 0xff;
    buf[63] = (routeChangeCnt >> 8) & 0xff;
    buf[62] = (routeChangeCnt >> 16) & 0xff;
    buf[61] = (routeChangeCnt >> 24) & 0xff;

    /* Msg Overhead */
    buf[68] = pollMsgCnt & 0xff;
    buf[67] = (pollMsgCnt >> 8) & 0xff;
    buf[66] = (pollMsgCnt >> 16) & 0xff;
    buf[65] = (pollMsgCnt >> 24) & 0xff;

    buf[72] = mleMsgCnt & 0xff;
    buf[71] = (mleMsgCnt >> 8) & 0xff;
    buf[70] = (mleMsgCnt >> 16) & 0xff;
    buf[69] = (mleMsgCnt >> 24) & 0xff;

    buf[76] = mleRouterMsgCnt & 0xff;
    buf[75] = (mleRouterMsgCnt >> 8) & 0xff;
    buf[74] = (mleRouterMsgCnt >> 16) & 0xff;
    buf[73] = (mleRouterMsgCnt >> 24) & 0xff;

    buf[80] = addrMsgCnt & 0xff;
    buf[79] = (addrMsgCnt >> 8) & 0xff;
    buf[78] = (addrMsgCnt >> 16) & 0xff;
    buf[77] = (addrMsgCnt >> 24) & 0xff;

    buf[84] = netdataMsgCnt & 0xff;
    buf[83] = (netdataMsgCnt >> 8) & 0xff;
    buf[82] = (netdataMsgCnt >> 16) & 0xff;
    buf[81] = (netdataMsgCnt >> 24) & 0xff;

    buf[76] = meshcopMsgCnt & 0xff;
    buf[75] = (meshcopMsgCnt >> 8) & 0xff;
    buf[74] = (meshcopMsgCnt >> 16) & 0xff;
    buf[73] = (meshcopMsgCnt >> 24) & 0xff;

    buf[76] = tmfMsgCnt & 0xff;
    buf[75] = (tmfMsgCnt >> 8) & 0xff;
    buf[74] = (tmfMsgCnt >> 16) & 0xff;
    buf[73] = (tmfMsgCnt >> 24) & 0xff;

    buf[88] = totalSerialMsgCnt & 0xff;
    buf[87] = (totalSerialMsgCnt >> 8) & 0xff;
    buf[86] = (totalSerialMsgCnt >> 16) & 0xff;
    buf[85] = (totalSerialMsgCnt >> 24) & 0xff;
}
