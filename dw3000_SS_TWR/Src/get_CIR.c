
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>

void get_CIR_INIT(void);
void extract_data(void);
void send_realCIR(void);

extern void test_run_info(unsigned char *data);

/* Communication configuration. */
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
    DWT_STS_MODE_OFF, /* Cipher disabled */
    DWT_STS_LEN_64,/* Cipher length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Constants */
#define ACCUM_DATA_LEN (3 * 2 * (3 + 3) + 1)

/* struct */
static dwt_get_CIR getCIR;
static dwt_rxdiag_t rxdiag;

/* Variables*/
static uint32_t status_reg = 0;
static uint16_t frame_len = 0;
static dwt_deviceentcnts_t event_cnt;
static uint8_t rx_buffer[FRAME_LEN_MAX];
static uint8_t accum_data[ACCUM_DATA_LEN];
static uint8_t CIR_data[6097];
static int32_t real_CIR[1016];

int get_CIR(void)
{
    /* DW IC configuration*/
    get_CIR_INIT();
    dwt_configeventcounters(1);
    dwt_configciadiag(1);

    /* Loop for receiving frames. */
	while (1) {

		int i;
		/* Clear local RX buffer and accumulator values */
		for (i = 0; i < FRAME_LEN_MAX; i++) {
			rx_buffer[i] = 0;
		}
		for (i = 0; i < ACCUM_DATA_LEN; i++) {
			accum_data[i] = 0;
		}
		memset(&getCIR, 0, sizeof(getCIR));

		/* Activate reception immediately. See NOTE 4 below. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		/* Poll until a frame is properly received or an error/timeout occurs. See NOTE 5 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
		{};

		/*Convert CIR data to real_CIR data, then send to serial monitor*/
		send_realCIR();

		/* Read event counters. See NOTE 7. */
		dwt_readeventcounters(&event_cnt);

		dwt_configciadiag(&real_CIR);
	}
}

void get_CIR_INIT(void){

	port_set_dw_ic_spi_fastrate();
	reset_DWIC();
	Sleep(2);

	while (!dwt_checkidlerc())
	{};

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
		while (1) {};
	}

	/* Configure DW IC. */
	if (dwt_configure(&config))
	{
		while (1)
		{};
	}
}

void extract_data(void){
	   dwt_readdiagnostics(&getCIR);
	   uint16_t fp_int = getCIR.ipatovFpIndex >> 6;
	   uint32_t fa_1 = getCIR.ipatovF1;
	   uint32_t fa_2 = getCIR.ipatovF2;
	   uint32_t fa_3 = getCIR.ipatovF3;
	   uint16_t cir_power = getCIR.ipatovPower;
	   uint16_t cir_peak1 = getCIR.ipatovPeak >> 21;
	   uint32_t cir_peak2 = getCIR.ipatovPeak & 0x1fffff;
}

void send_realCIR(void) {
	if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
		/* Clear good RX frame event in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

		/* A frame has been received, copy it to our local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
		if (frame_len <= FRAME_LEN_MAX) {
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/*if received, extract CIR and other data*/
		extract_data();

		dwt_readaccdata(&CIR_data, 6097, 0);
		int j = 0;
		for (int i = 1; i < 6097; i += 6) {
			real_CIR[j] = ((CIR_data[i + 2]) << 16 | (CIR_data[i + 1]) << 8 | (CIR_data[i]));
			if (real_CIR[j] & 0x00800000) {
				real_CIR[j] |= 0xff000000;
			}
			snprintf(real_CIR, sizeof(real_CIR), "%d,", real_CIR[j]);
			test_run_info((unsigned char *) real_CIR);
			j++;
			if (j%1015==0){
				CDC_Transmit_FS((uint8_t*)"\r\n",2);
			}
		}
	}

	else {
		/* Clear RX error events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
	}
	/* Read event counters. See NOTE 7. */
	dwt_readeventcounters(&event_cnt);
}


/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Accumulator values are complex numbers: one 24-bit integer for real part and one 24-bit value for imaginary part, for each sample. In this
 *    example, we chose to read 3 values below the first path index and 3 values above. It must be noted that the first byte read when accessing the
 *    accumulator memory is always garbage and must be discarded, that is why the data length to read is increased by one byte here.
 * 3. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *    thereafter.
 * 4. Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *    optimize system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 5. We use polled mode of operation here to keep the example as simple as possible, but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW IC User Manual for more details on "interrupts".
 * 6. Here we chose to read only a few values around the first path index but it is possible and can be useful to get all accumulator values, using
 *    the relevant offset and length parameters. Reading the whole accumulator will require 4064 bytes of memory. First path value gotten from
 *    dwt_readdiagnostics is a 10.6 bits fixed point value calculated by the DW IC. By dividing this value by 64, we end up with the integer part of
 *    it. This value can be used to access the accumulator samples around the calculated first path index as it is done here.
 * 7. Event counters are never reset in this example but this can be done by re-enabling them (i.e. calling again dwt_configeventcounters with
 *    "enable" parameter set).
 * 8. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW IC API Guide for more details on the DW IC driver functions.
 ****************************************************************************************************************************************************/

