#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <config_options.h>

void Anchor_INIT(void);
double* Multi_Ranging(char tagID, uint8_t dist);

extern void test_run_info(unsigned char *data);

static dwt_config_t config = {
        5,               /* Channel number. */
        DWT_PLEN_128,    /* Preamble length. Used in TX only. */
        DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
        DWT_BR_6M8,      /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        DWT_PHRRATE_STD, /* PHY header rate. */
        (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        DWT_STS_MODE_OFF, /* STS disabled */
        DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
        DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. */
#ifdef STM32F429xx
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#endif //STM32F429xx
#ifdef NRF52840_XXAA
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#endif //NRF52840_XXAA

/* Receive response timeout. */
#ifdef STM32F429xx
#define RESP_RX_TIMEOUT_UUS 210
#endif //STM32F429xx
#ifdef NRF52840_XXAA
#define RESP_RX_TIMEOUT_UUS 400
#endif //NRF52840_XXAA

/* Saving Ranging results */
#define TAG_nb 4 //number of TAG used for Ranging
static double dist_result[TAG_nb];

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. */
extern dwt_txconfig_t txconfig_options;


int ss_twr_anchor(void)
{

	/*DW configuration*/
	Anchor_INIT();

	/* SS_TWR Ranging Loop start*/
    while (1)
    {
    	/* Ranging process for Tag ID : 1 */
    	Multi_Ranging('1',0);
    	//double dist_TAG1 = dist_result[0];

    	/* Ranging process for Tag ID : 2 */
    	Multi_Ranging('2',1);
    	//double dist_TAG2 = dist_result[1];

    	/* Ranging process for Tag ID : 3 */
    	Multi_Ranging('3',2);
    	//double dist_TAG3 = dist_result[2];

        /* A delay between ranging exchanges. */
         Sleep(RNG_DELAY_MS);
    }

}

void Anchor_INIT(void){
	/*DW configuration start*/
    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);

    while (!dwt_checkidlerc())
    { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        while (1)
        { };
    }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    if(dwt_configure(&config))
    {
        while (1)
        { };
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
	/*Configuration end*/
}

double* Multi_Ranging(char tagID, uint8_t dist){

	double tof;
	double distance;

	uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'G', tagID, 0xE0, 0, 0};
	uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'G', tagID, 'T', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };

    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        uint32_t frame_len;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32_t rtd_init, rtd_resp;
                float clockOffsetRatio ;

                poll_tx_ts = dwt_readtxtimestamplo32();
                resp_rx_ts = dwt_readrxtimestamplo32();

                clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                rtd_init = resp_rx_ts - poll_tx_ts;
                rtd_resp = resp_tx_ts - poll_rx_ts;

                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;
                dist_result[dist] = distance;

                /* USB_Serial communication for printing each TAG's distance result */
            	snprintf(dist_result,100,"TAG ID : %d, distance : %.3f\n",dist, dist_result[dist]);
            	test_run_info((unsigned char *) dist_result);
            }
        }
    }
    else
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
    return dist_result;

    /* inter-TAG delay(=Ranging round time) */
    Sleep(100); //10ms
}
