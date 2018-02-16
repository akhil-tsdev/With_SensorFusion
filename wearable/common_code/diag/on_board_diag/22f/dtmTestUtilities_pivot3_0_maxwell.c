/*******************************************************************************
 * Standard C Include Files
 ******************************************************************************/
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * SDK Include Files
 ******************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_slave_driver.h"
#include "board.h"
#include "lsm6dsl_firmware.h"
#include "mpu9250_maxwell.h"
#include "mpu9250_freq.h"
#include "ts_rtc.h"

/* USB */
#include "dtm_common_types.h"

#include "fsl_debug_console.h"

#include "crc16.h"
#include "dtmTestUtilities_pivot3_0_maxwell.h"
#include "fsl_dspi_master_driver.h"
#include "main_22f_on_board_diag_maxwell.h"

/*******************************************************************************
 * Freescale Open Source Sensor Fusion (OSSF) Include Files
 ******************************************************************************/
#include "build.h"
#include "tasks.h"
#include "magnetic.h"
/*******************************************************************************
 * TuringSense Include Files
 ******************************************************************************/
#include "common_types.h"
#include "common_err.h"
#include "sat_module.h"
#include "ts_fusion.h"
#include "cbuf.h"

#define MAX(a,b) ((a>b)?(a):(b))
#define SPI_COMM_LENGTH MAX(sizeof(sat_to_nordic_dtm_t),sizeof(nordic_to_sat_dtm_t))


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static uint8_t spi_tx_arr[SPI_COMM_LENGTH] = {0};
static uint8_t spi_rx_arr[SPI_COMM_LENGTH] = {0};

typedef struct {
	int16_t mag_x;		// Magnetometer's x-axis value
	int16_t mag_y;		// Magnetometer's y-axis value
	int16_t mag_z;		// Magnetometer's z-axis value
} magTmp_t;

magTmp_t tmpMag[MAX_SENSORS_IN_CS] = {0};

/* SPI related */
typedef union {
	sat_mosi_packet_t packet;
	uint8_t bytes[/*sizeof(sat_mosi_packet_t)*/SPI_COMM_LENGTH];
} receivedCommand_t;
typedef union dataToSend_ {
	sat_miso_packet_t data;
	uint8_t bytes[sizeof(sat_miso_packet_t)];
} dataToSend_t;

dataToSend_t dataToSend;

static uint32_t sRandomSeed;
uint16_t user_input;

void initRandom()
{
	sRandomSeed = 0x02;
}


uint32_t getRandom(uint8_t is_prbs9)
{
	uint8_t a = (is_prbs9 ? 6 : 14);
	uint32_t new_v = (((sRandomSeed < a) ^ (sRandomSeed >> (a-1))) & 0x01);
	sRandomSeed = ((sRandomSeed << 16) | new_v) | sRandomSeed >> 16 ;
	return sRandomSeed;
}

// ************************************************************ //
// ************************* OPTIONS ************************** //
void setNordicParameters(int flag)
{
	if ((flag == 0) || (flag == 2))
	{
		debug_printf("S|	DTM frequency in MHz (0-83) [actual value %u]: \r\n",spi_tx.dtm_tx_freq);
		debug_printf("E|\r\n");
		debug_scanf("%u", &user_input);

		if (user_input > DTM_FREQ_MAX) {
			spi_tx.dtm_tx_freq = DTM_FREQ_MAX;
			debug_printf("	You entered %u, defaulting to %u\r\n", user_input, spi_tx.dtm_tx_freq);
		} else {
			spi_tx.dtm_tx_freq = user_input;
			debug_printf("	%u\r\n", spi_tx.dtm_tx_freq);
		}
		debug_printf("S|	Transmit power (dBm) [actual value %u]: \r\n",spi_tx.dtm_power);
		debug_printf("S|	 	0: +4dBm   DEFAULT\r\n");
		debug_printf("S|	 	1: 0dBm\r\n");
		debug_printf("S|	 	2: -4dBm\r\n");
		debug_printf("S|	 	3: -8dBm\r\n");
		debug_printf("S|	 	4: -12dBm\r\n");
		debug_printf("S| 		5: -16dBm\r\n");
		debug_printf("S|	 	6: -20dBm\r\n");
		debug_printf("S| 		7: -30dBm\r\n");
		debug_printf("S|	DTM power: \r\n");

		debug_scanf("%u", &user_input);

		if (user_input > DTM_ENUM_POWER_MAX) {
			spi_tx.dtm_power = DTM_ENUM_POWER_MAX;
			debug_printf("	You entered %u, defaulting to %u\r\n", user_input, spi_tx.dtm_power);
		} else {
			spi_tx.dtm_power = user_input;
			debug_printf("	%u\r\n", spi_tx.dtm_power);
		}
		debug_printf("E|\r\n");
	}

	if ((flag==1)||(flag==2))
	{
		debug_printf("S|	DTM packet length in bytes (1-252) [actual value %u]: \r\n",spi_tx.dtm_tx_pkt_len);
		debug_printf("E|\r\n");
		debug_scanf("	%u", &user_input);

		if (user_input > DTM_PKT_LEN_MAX) {
			spi_tx.dtm_tx_pkt_len = DTM_PKT_LEN_MAX;
			debug_printf("	You entered %u, defaulting to %u\r\n", user_input, spi_tx.dtm_tx_pkt_len);
		} else if (user_input < DTM_PKT_LEN_MIN) {
			spi_tx.dtm_tx_pkt_len = DTM_PKT_LEN_MIN;
			debug_printf("	You entered %u, defaulting to %u\r\n", user_input, spi_tx.dtm_tx_pkt_len);
		} else {
			spi_tx.dtm_tx_pkt_len = user_input;
			debug_printf("	%u\r\n", spi_tx.dtm_tx_pkt_len);
		}

		debug_printf("S|	DTM transmit interval in ms (0 to 65535) [actual value %u]: \r\n",spi_tx.tx_interval);
		debug_scanf("%u", &user_input);
		debug_printf("E|\r\n");

		if (user_input > UINT16_MAX) {
			spi_tx.tx_interval = UINT16_MAX;
			debug_printf("	You entered %u, defaulting to %u\r\n", user_input, spi_tx.tx_interval);
		} else {
			spi_tx.tx_interval = user_input;
			debug_printf("	%u\r\n", spi_tx.tx_interval);
		}

		debug_printf("S|	Packet Type (0-3) [actual value %u]: \r\n",spi_tx.dtm_pkt_type);
		debug_printf("S| 		0: prbs9\r\n");
		debug_printf("S| 		1: prbs15\r\n");
		debug_printf("S| 		2: 11110000\r\n");
		debug_printf("S| 		3: 01010101\r\n");
		debug_printf("S| 		4: Unmodulated\r\n");
		debug_printf("S| 		5: 10101010\r\n");
		debug_printf("S|	Packet Type: \r\n");

		debug_scanf("%u", &user_input);

		if (user_input > DTM_PKT_TYPE_MAX) {
			spi_tx.dtm_pkt_type = DTM_PKT_TYPE_MAX;
			debug_printf("You entered %u, defaulting to %u\r\n", user_input, spi_tx.dtm_pkt_type);
		} else {
			spi_tx.dtm_pkt_type = user_input;
			debug_printf("%u\r\n", spi_tx.dtm_pkt_type);
		}
		debug_printf("E|\r\n");
	}
}

void spiResetParams()
{
	spi_tx.dtm_tx_freq = DTM_FREQ_DEFAULT;

	spi_tx.dtm_power = DTM_ENUM_POWER_DEFAULT;

	spi_tx.dtm_pkt_type = DTM_PKT_TYPE_PRBS15;

	spi_tx.dtm_tx_pkt_len = DTM_PKT_LEN_MAX;

	// milliseconds
	spi_tx.tx_interval = GET_DTM_TX_INTERVAL_US(1);
}

void spiSendUntilAcked() {
    dspi_status_t 			dspiResult;
    uint32_t 				wordsTransfer = 0;
    static uint16_t			last_crc16_sent = 0;
    uint16_t				generated_crc16, temp_crc16;
    bool					need_spi_resend = false;

	/* Generate CRC16 */
	spi_tx.crc16 = 0;
	generated_crc16 = crc16_compute((uint8_t*)&spi_tx, sizeof(spi_tx), 0);
	spi_tx.crc16 = last_crc16_sent = generated_crc16;

	do {
		/* 06/17/17 cwati size of spi tx & rx should be the same for reliability
		 * in the SPI transfer.
		 */
		memcpy(&spi_tx_arr, &spi_tx, sizeof(spi_tx));
		memcpy(&spi_rx_arr, &spi_rx, sizeof(spi_rx));

		dspiResult = DSPI_DRV_SlaveTransfer(knRF51822SpiInstance,
											(uint8_t*)&spi_tx_arr,
											(uint8_t*)&spi_rx_arr,
											SPI_COMM_LENGTH);

		if (dspiResult != kStatus_DSPI_Success)
		{
			debug_printf("\r\n ERROR: Can not start SPI to Nordic \r\n");
			blinkLEDerror(redLed, 4);
		};

		/* Check SPI response */
		while (DSPI_DRV_SlaveGetTransferStatus(knRF51822SpiInstance, &wordsTransfer)== kStatus_DSPI_Busy) {}

		memcpy(&spi_rx, &spi_rx_arr, sizeof(spi_rx));

		/* Check CRC16 */
		temp_crc16 = spi_rx.crc16;
		spi_rx.crc16 = 0;

		generated_crc16 = crc16_compute ((uint8_t*) &spi_rx, sizeof(spi_rx), 0);
		if (generated_crc16 == temp_crc16) {
//				debug_printf("\r\nGot clean crc16 from Nordic DEBUG: 0x%x\n", generated_crc16rated_crc16);

			/* Check last CRC16 received by Nordic */
			if (last_crc16_sent != spi_rx.last_crc16_recvd) {
//					debug_printf("\r\nGot crc16 from Nordic DEBUG last crc sent: 0x%x crc received: 0x%x\n",
//							last_crc16_sent, spi_rx.last_crc16_recvd);
				need_spi_resend = true;
			} else {
//					debug_printf("\r\nGot Right crc16 from Nordic DEBUG: 0x%x Exiting...\n", spi_rx.last_crc16_recvd);
				debug_printf("\r\nCommand has been successfully requested to the RF Chip. "
							 "\r\nRF chip will continue transmitting unless you enter new command...\n");
				debug_printf("E|\r\n");
				need_spi_resend = false;
			}
		} else {
//				debug_printf("\r\nGot bad crc16 from Nordic DEBUG: 0x%x Expected:0x%x\n", temp_crc16, generated_crc16);
			need_spi_resend = true;
		}

		if (!need_spi_resend) {
				break;
		}

	} while(1);


}
int spiSendReceive()
{

	uint32_t wordsTransfer = 0;
	dspi_status_t dspiResult;
	int cntBusyState = 0;

	/* 06/17/17 cwati size of spi tx & rx should be the same for reliability
	* in the SPI transfer.
	*/

	memcpy(&spi_tx_arr, &spi_tx, sizeof(spi_tx));
	memcpy(&spi_rx_arr, &spi_rx, sizeof(spi_rx));

	// ************* TRASFER DATA WITH SPI **************************** //
	// **************************************************************** //
	dspiResult = DSPI_DRV_SlaveTransfer(knRF51822SpiInstance,
										(uint8_t*)&spi_tx_arr,
										(uint8_t*)&spi_rx_arr,
										SPI_COMM_LENGTH);

	// **************************************************************** //
	// **************************************************************** //


	if (dspiResult != kStatus_DSPI_Success)
	{
		debug_printf("ERROR: Can not start SPI to Nordic \r\n");
		blinkLEDerror(redLed, 4);
	};

	/* Check SPI response */
	while (DSPI_DRV_SlaveGetTransferStatus(knRF51822SpiInstance, &wordsTransfer)== kStatus_DSPI_Busy) {

		//debug_printf("\r\nnwait status SPI busy...\n");
		cntBusyState++;
	}

	memcpy(&spi_rx, &spi_rx_arr, sizeof(spi_rx));

	return cntBusyState;

}

// ************************************************************ //
// ***************** TEST SPI COMMUNICATION ******************* //
int spiTest(parameters_t parameters)
{
	sensor_data_t raw_data;			// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t sensor_record;// Timestamp + sensor data (raw + sensor-fusion output)
	uint32_t current_ts10, time_now;
	uint32_t results[NR_STATISTIC_ELEMENTS]={0};
	bool is_sleeping = false, retbool;


    static uint16_t			last_crc16_sent = 0;
    uint16_t				generated_crc16, temp_crc16;
    bool					need_spi_resend = false, first=true;
    uint16_t				user_input;

    // SPI init
    nrf51822_init();
    nrf51822_swdio_reset();

    results[POS_NR_WRREADCICLE]=parameters.spi_numberOfTests;

    debug_printf(" ************ RESULTS SPI 22f TO NORDIC TESTING ************\r\n");

	do {
		initRandom();

		/* Generate CRC16 */
		spi_tx.crc16 = 0;
		sRandomSeed=k22f_get_rtc();
		spi_tx.randomFrom22F=getRandom(false);//getRandom(true); //generate random number
		generated_crc16 = crc16_compute((uint8_t*)&spi_tx, sizeof(spi_tx), 0);
		spi_tx.crc16 = last_crc16_sent = generated_crc16;

		results[POS_NR_PACKAGESENT]++;

		bool firstCicle=true;

		do {

			results[POS_NR_ITERACTIONS]++;

			//results initialization
			results[POS_NR_ATTEMPTCICL]=0;

			// Send and receive data through the SPI to nordic
			results[POS_NR_SPBUSYSTATE] += spiSendReceive();

			/* Check CRC16 */
			temp_crc16 = spi_rx.crc16;
			spi_rx.crc16 = 0;

			generated_crc16 = crc16_compute ((uint8_t*) &spi_rx, sizeof(spi_rx), 0);



			 if (generated_crc16 == temp_crc16) {
				 // debug_printf("\r\nGot clean crc16 from Nordic DEBUG: 0x%x\n", generated_crc16rated_crc16);

				 // Check last CRC16 received by Nordic
				 if (last_crc16_sent != spi_rx.last_crc16_recvd) {
						if(!firstCicle)
			                   results[POS_NR_CRC16LASTFL_TOT]++;

						if(parameters.log_enableDetail>1)
			                   if(!firstCicle)
			                        debug_printf("\r\n LAST CRC CK FAILED value=%d RECEIVED value=%d",last_crc16_sent,spi_rx.last_crc16_recvd);

			                        need_spi_resend = true;
				 }
				 else if(spi_tx.randomFrom22F == spi_rx.randomFrom22F)
				 {
			            // debug_printf("\r\nGot Right crc16 from Nordic DEBUG: 0x%x Exiting...\n", spi_rx.last_crc16_recvd);
						// debug_printf("\r\nCommand has been successfully requested to the RF Chip. RF chip will continue transmitting unless you enter new command...\n");
						//if(parameters.log_enableDetail>0)
			            //  debug_printf("Sent %d - Rcvd %d \r\n",spi_tx.randomFrom22F,spi_rx.randomFrom22F);
			            need_spi_resend = false;

			            results[POS_NR_ITERACTIONS_TOT] += results[POS_NR_ITERACTIONS];
			            results[POS_NR_SPBUSYSTATE_TOT] += results[POS_NR_SPBUSYSTATE];
			            results[POS_NR_PACKAGERECE]++;
			            results[POS_NR_ITERACTIONS]=0;
			            results[POS_NR_SPBUSYSTATE]=0;

			            if(parameters.log_enableDetail>1)
							debug_printf("\r\n SENT value=%d RECEIVED value=%d [%d|%d]",spi_tx.randomFrom22F,spi_rx.randomFrom22F,results[POS_NR_ITERACTIONS],results[POS_NR_SPBUSYSTATE]);

				 }
				 else
						results[POS_NR_ERRORTRANSM]++;

			 }
			 else if(results[POS_NR_ATTEMPTCICL] > parameters.spi_attemptLimit)
			      {
			           	results[POS_NR_PACKAGELOST]++;
			            	break;
			      }
			      else
			      {
			            //debug_printf("\r\nGot bad crc16 from Nordic DEBUG: 0x%x Expected:0x%x\n", temp_crc16, generated_crc16);
			            results[POS_NR_CRC16FAILED]++;
			            need_spi_resend = true;
			      }

			            //debug_printf("\r\nGot bad crc16 from Nordic DEBUG: 0x%x Expected:0x%x\n", temp_crc16, generated_crc16);

			if (!need_spi_resend) {
						break;
			}

			firstCicle=false;

		} while(1);

		parameters.spi_numberOfTests--;
	} while(parameters.spi_numberOfTests>0);

	if(parameters.log_enableDetail>0){
		debug_printf(" ************ RESULTS SPI TESTING ************\r\n");
		debug_printf(" * Number of packets sent: =%d\r\n", results[POS_NR_PACKAGESENT] );
		debug_printf(" * Number of packets received =%d\r\n", results[POS_NR_PACKAGERECE] );
		debug_printf(" * Number of lost packets: =%d\r\n", results[POS_NR_PACKAGELOST] );
		debug_printf(" * Number of CRC failed =%d\r\n", results[POS_NR_CRC16FAILED] );
		debug_printf(" * Number of LAST CRC failed =%d\r\n", results[POS_NR_CRC16LASTFL_TOT] );
		debug_printf(" * Number of compromise package: =%d\r\n", results[POS_NR_ERRORTRANSM] );
		debug_printf(" * Average number attempts per shipment (consider 2 cycle for packet): =%d\r\n", (uint)((float)results[POS_NR_ITERACTIONS_TOT]/(float)results[POS_NR_WRREADCICLE]));
		debug_printf(" * Average busy state per shipment: =%d\r\n", (uint)((float)results[POS_NR_SPBUSYSTATE_TOT]/(float)results[POS_NR_WRREADCICLE]));
	}

	if(results[POS_NR_CRC16FAILED]>parameters.spi_max_crc_fail)
		debug_printf("[SPI2TN:FAILED]\r\n");
	else
		debug_printf("[SPI2TN:OK]\r\n");
}

// ************************************************************ //
// ************** TEST SPIsensor COMMUNICATION **************** //

#define BUFFER_SIZE 65
static uint8_t mts_buffer[BUFFER_SIZE];
static uint8_t stm_buffer[BUFFER_SIZE];
static void mpu9250_write_reg(uint8_t reg_addr, uint8_t data)
{
	dspi_status_t Error;

	mts_buffer[0] = reg_addr;
	mts_buffer[1] = data;

	Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance,
											& mpu9250_dspi_device,
											mts_buffer,
											stm_buffer,
											2,
											1000);
}
static void mpu9250_read_regs(uint8_t reg_addr, size_t count, uint8_t *data)
{
	dspi_status_t Error;
	mts_buffer[0] = 0x80 | reg_addr;
	memset(& mts_buffer[1], 0, count);  // not really necessary
	memset(stm_buffer, 0, count+1); // also not really necessary
	Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance,
											& mpu9250_dspi_device,
											mts_buffer,
											stm_buffer,
											count + 1,
											1000);
	memcpy(data, & stm_buffer[1], count);
}

static uint8_t mpu9250_read_reg(uint8_t reg_addr)
{
	uint8_t data;

	mpu9250_read_regs(reg_addr, 1, & data);
	return data;
}

void sensorDataIntegrityTest_spi(parameters_t parameters, bool* pass)
{
	uint8_t readV=0, writeV;
#ifndef WRITEVCNT
#define WRITEVCNT 6
#endif
	uint8_t writeVarr[WRITEVCNT] = {0xAA, 0x55, 0xF0, 0x0F, 0x00, 0xFF};
	uint8_t writeVcnt;
	uint32_t testCnt = 2000;
	uint8_t max_retry = 8;

	if(parameters.log_enableDetail>0) {
		debug_printf("\r\n ************ SPI TEST Sensor %u (CS %u) ************\r\n", mpu9250_num, valid_cs[mpu9250_num]);
	}
	mpu9250_start();

	uint32_t nFail=0;		/* Test value read is not with test value written */
	uint32_t majorFail = 0; /* Failed to recover initial value because write fails */
	uint32_t totalCheck = 0;/* Total number of write run */

	uint8_t addresses[] = {MPU9250_REG_XA_OFFSET_H,MPU9250_REG_XA_OFFSET_L,
						MPU9250_REG_YA_OFFSET_H,MPU9250_REG_YA_OFFSET_L,
						MPU9250_REG_ZA_OFFSET_H,MPU9250_REG_ZA_OFFSET_L,
						MPU9250_REG_XG_OFFSET_H,MPU9250_REG_XG_OFFSET_L,
						MPU9250_REG_YG_OFFSET_H,MPU9250_REG_YG_OFFSET_L,
						MPU9250_REG_ZG_OFFSET_H,MPU9250_REG_ZG_OFFSET_L};
	uint8_t addressesLength=12;
	uint8_t address;
	uint8_t initialValues[addressesLength];

	for(int i=0;i<addressesLength;i++)
	{
		address = addresses[i];
		mpu9250_read_regs(address, 1, & readV);
		initialValues[i] = readV;
	}

	parameters.spiSensors_numberOfTests = testCnt;
	writeVcnt = 0;
	do{
		 address = addresses[(parameters.spiSensors_numberOfTests)%12];

		 writeV = writeVarr[writeVcnt % WRITEVCNT];

		 if(parameters.log_enableDetail>1) {
			 debug_printf(" *  Write: =%d ... \r\n", writeV);
		 }

		 mpu9250_write_reg(address, writeV);
		 mpu9250_read_regs(address, 1, & readV);
		 testCnt++;

		 if(parameters.log_enableDetail>1) {
			 debug_printf("read: =%d ->\r\n", readV);
		 }

		 /* Calculate failure */
		 if(readV==writeV) {
			 if(parameters.log_enableDetail>1) {
				 debug_printf(" OK\r\n", writeV);
			 }
		 } else{
			 nFail++;

			 if(parameters.log_enableDetail>1) {
				 debug_printf(" FAILED!\r\n", writeV);
			 }
		 }

		 parameters.spiSensors_numberOfTests--;
		 writeVcnt++;
	} while(parameters.spiSensors_numberOfTests>0);



	if(parameters.log_enableDetail>0){
		debug_printf(" *  register value BEFORE UPDATE\r\n");
		for(int i=0;i<addressesLength;i++){
			address = addresses[i];
			debug_printf(" *  register %u=%d\r\n", addresses[i], initialValues[i]);
		}
		debug_printf(" -\r\n");
	}

	if(parameters.log_enableDetail>0){
		int i;
		debug_printf(" *  register value AFTER UPDATE\r\n");
		for(i=0;i<addressesLength;i++){
			address = addresses[i];
			mpu9250_read_regs(address, 1, & readV);
			debug_printf(" *  register %u=%d ... \r\n", address,readV);
		}
	}

	// restore the proper offset values
	for(int i=0;i<addressesLength;i++) {
		mpu9250_write_reg(addresses[i], initialValues[i]);
	}

	if(parameters.log_enableDetail>0){
		debug_printf(" -\r\n");
		debug_printf(" *  RESTORED register values\r\n");
	}

	for(int i=0;i<addressesLength;i++){
		address = addresses[i];
		mpu9250_read_regs(address, 1, & readV);
		testCnt++;

		max_retry = 8;
		while ((readV != initialValues[i]) && (max_retry--)) {
			nFail++;
			mpu9250_write_reg(addresses[i], initialValues[i]);
			mpu9250_read_regs(address, 1, & readV);
		}
		max_retry = 8; /* Restore value for printf later */
		/* Still not the same */
		if (readV != initialValues[i]) {
			majorFail++;
		}
		if(parameters.log_enableDetail>0){
			debug_printf(" *  register %u=%d ... \r\n", address,readV);
		}
	}

	mpu9250_stop();

	 if(majorFail || nFail) {
		 if(parameters.log_enableDetail>0) {
			 debug_printf("\n\r[SPI2TI:FAILED]\tFailed test: %u Major failure: %u",
				 nFail, majorFail);
		 }
		 *pass = 0;
	 } else {
		 if(parameters.log_enableDetail>0) {
			 debug_printf("\n\r[SPI2TI:OK]");
		 }
		 *pass = 1;
	 }
}

int sensorDataIntegrityTest_magReadWrite(parameters_t parameters, bool* pass)
{
	uint32_t c = 0, N = 5000;
	uint8_t data[MPU9250_REG_COUNT];
	uint32_t 	lastRead = 0;
	uint32_t nFail = 0;

	debug_printf("\n\r ************ MAG READ/WRITE TEST Sensor %u (CS %u) ************\r\n",
			mpu9250_num, valid_cs[mpu9250_num]);

	while (c < N)
	{
		if (c % 500 == 0) {
			debug_printf("\n\rReading %u times...", c);
		}
		while((k22f_get_rtc()-lastRead) < 5 /* MAG is 100 Hz */) {
			; //do-nothing while loop
		}
		lastRead = k22f_get_rtc();

		mpu9250_read_regs(MPU9250_FIRST_REG, MPU9250_REG_COUNT, data);

		if ((data[AK8963_REG_OFFSET(AK8963_REG_WAI)] != 0x48) || (data[AK8963_REG_OFFSET(AK8963_REG_DEV_INFO)] != 0x9a)) {
			nFail++;
			debug_printf("\n\rnFail: %u WAI: 0x%x INFO: 0x%x", nFail, data[AK8963_REG_OFFSET(AK8963_REG_WAI)],
					data[AK8963_REG_OFFSET(AK8963_REG_DEV_INFO)]);
		}
		c++;
	}

	if(nFail) {
		debug_printf("\n\r[MAG:FAILED]\tFailed test: %u out %u", nFail, N);
		*pass = 0;
	} else {
		debug_printf("\n\r[MAG:OK]");
		*pass = 1;
	}

	mpu9250_stop();

}

int sensorDataIntegrityTest(parameters_t parameters, bool* pass) {
	if (parameters.log_enableDetail > 0) {
		debug_printf("\r\n ************ SENSOR DATA INTEGRITY Individual TESTING Sensor %u (CS %u) ************\r\n",
			mpu9250_num, valid_cs[mpu9250_num]);
	}
	sensorDataIntegrityTest_spi(parameters, pass);
}



int spiSensorRegisterWriteTest(parameters_t parameters, bool* pass)
{
	uint8_t  writeV=0, readV=0;
	debug_printf("\r\n ************ RESULTS SPI SENSORS TESTING ************\r\n");

	mpu9250_start();
	uint nFail=0;
	uint8_t addresses[] = {MPU9250_REG_XA_OFFSET_H,MPU9250_REG_XA_OFFSET_L,
						MPU9250_REG_YA_OFFSET_H,MPU9250_REG_YA_OFFSET_L,
						MPU9250_REG_ZA_OFFSET_H,MPU9250_REG_ZA_OFFSET_L,
						MPU9250_REG_XG_OFFSET_H,MPU9250_REG_XG_OFFSET_L,
						MPU9250_REG_YG_OFFSET_H,MPU9250_REG_YG_OFFSET_L,
						MPU9250_REG_ZG_OFFSET_H,MPU9250_REG_ZG_OFFSET_L};
	uint8_t addressesLength=12;
	uint8_t address;
	uint8_t initialValues[addressesLength];

	for(int i=0;i<addressesLength;i++)
	{
		address = addresses[i];
		mpu9250_read_regs(address, 1, & readV);
		initialValues[i] = readV;
	}


	if(parameters.log_enableDetail>0){
		int i;
		debug_printf(" *  register value BEFORE UPDATE\r\n");
		for(i=0;i<addressesLength;i++){
			address = addresses[i];
			mpu9250_read_regs(address, 1, & readV);
			debug_printf(" *  register %u=%d\r\n", address,readV);
		}
		debug_printf(" -\r\n");
	}
	OSA_TimeDelay(300); /* ms */

	do{
		 address = addresses[(parameters.spiSensors_numberOfTests)%12];

		 writeV=(parameters.spiSensors_numberOfTests)%256;
		 if(parameters.log_enableDetail>1)
			 debug_printf(" *  Write: =%d ... \r\n", writeV);

		 mpu9250_write_reg(address, writeV);
		 mpu9250_read_regs(address, 1, & readV);

		 if(parameters.log_enableDetail>1)
			 debug_printf("read: =%d ->\r\n", readV);

		 if(parameters.log_enableDetail>1){
			 if(readV==writeV)
				 debug_printf(" OK\r\n", writeV);
			 else{
				 debug_printf(" FAILED!\r\n", writeV);
				 nFail++;
			 }
		 }

		 parameters.spiSensors_numberOfTests--;
	} while(parameters.spiSensors_numberOfTests>0);


	if(parameters.log_enableDetail>0){
		int i;
		debug_printf(" - \r\n");
		debug_printf(" *  register value AFTER UPDATE\r\n");
		for(i=0;i<addressesLength;i++){
			address = addresses[i];
			mpu9250_read_regs(address, 1, & readV);
			debug_printf(" *  register %u=%d ... \r\n", address,readV);
		}
	}



	// restore the proper offset values
	for(int i=0;i<addressesLength;i++)
		mpu9250_write_reg(addresses[i], initialValues[i]);

	if(parameters.log_enableDetail>0){
		int i;
		debug_printf(" -\r\n");
		debug_printf(" *  Restored register values\r\n");
		for(i=0;i<addressesLength;i++){
			address = addresses[i];
			mpu9250_read_regs(address, 1, & readV);
			debug_printf(" *  register %u=%d ... \r\n", address,readV);
		}
	}

	mpu9250_stop();

	 if(nFail > parameters.spiSensor_max_WR_fail) {
		 debug_printf("[SPI2TI:FAILED]\r\n");
		 *pass = 0;
	 } else {
		 debug_printf("[SPI2TI:OK]\r\n");
		 *pass = 1;
	 }
}

/*******************************************************************************
 * Read Sensor raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample_basic(uint8_t sensor_num, sensor_data_t *raw_data) {
	bool ret;
	uint32_t time_start, time_now;

	//CF need in case SAT have more sensors. GYR freq=200Hz, but MAG freq=100Hz
	raw_data->mag[0] = tmpMag[sensor_num].mag_x;
	raw_data->mag[1] = tmpMag[sensor_num].mag_y;
	raw_data->mag[2] = tmpMag[sensor_num].mag_z;

	ret = lsm6dsl_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,raw_data);

	tmpMag[sensor_num].mag_x = raw_data->mag[0];
	tmpMag[sensor_num].mag_y = raw_data->mag[1];
	tmpMag[sensor_num].mag_z = raw_data->mag[2];

	return ret;
}

//bool mag_sensor_data_equal (sensor_data_t old, sensor_data_t new) {
//	for (uint8_t qq = 0; qq < 3; qq++) {
////		if (old.accel[qq] != new.accel[qq]) {
////			return false;
////		}
////		if (old.gyro[qq] != new.gyro[qq]) {
////			return false;
////		}
//		if (old.mag[qq] != new.mag[qq]) {
//			return false;
//		}
//	}
//
//	return true;
//}
//
//void readMagData(uint32_t* magvalid, uint32_t* totalread)
//{
//	uint32_t 	c = 0;
//	uint32_t 	N = 2000;
//	uint32_t 	offset = 50;
//	uint32_t 	lastRead = 0;
//	uint8_t		sensor_num;
//	bool		retbool;
//	sensor_data_t raw_data = {0};
//	uint8_t 	mag_data[10];
//	bool 		ret;
//	uint32_t 	new_data_percentage;
//	const uint32_t	min_new_data_percentage = 45;
//
//	sensor_data_t last_raw_data[MAX_SENSORS_IN_CS];
//	bool		first_time[MAX_SENSORS_IN_CS] = {true, true, true, true, true};
//	static uint32_t same_data_cnt[MAX_SENSORS_IN_CS] = {0};
//	static uint32_t no_data_cnt[MAX_SENSORS_IN_CS] = {0};
//	static uint32_t new_data_cnt[MAX_SENSORS_IN_CS] = {0};
//
//	turnOnLED(turqoiseLed);
//
//	debug_printf("\n\r...");
//
//	while (c < N+offset)
//	{
//
//		if (c % 100 == 0) {
//			debug_printf("...");
//		}
//		for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++)
//		{
//			while((k22f_get_rtc()-lastRead) < 5) {
//				; //do-nothing while loop
//			}
//			lastRead = k22f_get_rtc();
//
//			mpu9250_enable_sensor(sensor_num);
//			retbool = get_raw_data_sample_basic(&raw_data);
//
//			if (first_time[sensor_num]) {
//				last_raw_data[sensor_num] = raw_data;
//				first_time[sensor_num] = false;
//				same_data_cnt[sensor_num] = 0;
//				new_data_cnt[sensor_num] = 0;
//				no_data_cnt[sensor_num] = 0;
//			} else {
//
//				if (raw_data.valid_sensors & SENSOR_MAG) {
//					if (mag_sensor_data_equal(last_raw_data[sensor_num], raw_data)) {
//						same_data_cnt[sensor_num]++;
//					} else {
//						new_data_cnt[sensor_num]++;
//					}
//				} else {
//					no_data_cnt[sensor_num]++;
//				}
//
//				last_raw_data[sensor_num] = raw_data;
//			}
//			if (c > offset)
//			{
//				(*totalread)++;
//				if (raw_data.valid_sensors & SENSOR_MAG) {
//					(*magvalid)++;
//				}
//			}
//		}
//
//		c++;
//	}
//
//	debug_printf("\n\rSENSOR(cs)\t|No Data |New Data |Stale Data|NewData%% |PASS/FAIL");
//	debug_printf("\n\r----------------------------------------------------------------------");
//
//	for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
//		uint32_t fail_uint, total_read;
//		total_read = no_data_cnt[sensor_num] + same_data_cnt[sensor_num] + new_data_cnt[sensor_num];
//
//		debug_printf("\n\r%u(%u)\t\t|%-8u|%-9u|%-10u|", sensor_num, valid_cs[sensor_num],
//				no_data_cnt[sensor_num],
//				new_data_cnt[sensor_num],
//				same_data_cnt[sensor_num]);
//
//		new_data_percentage = (new_data_cnt[sensor_num] * 100) / total_read;
//		if (new_data_percentage >= min_new_data_percentage) {
//			debug_printf("%-9u|PASS", new_data_percentage);
//		} else {
//			debug_printf("%-9u|FAIL", new_data_percentage);
//		}
//	}
//
//	debug_printf("\n");
//
//	turnOnLED(greenLed);
//	OSA_TimeDelay(500);
//
//}
//
//void computeAggregateStdVector(float meanACC[][3], float meanGYR[][3], float meanMAG[][3], float stdACC[][3], float stdGYR[][3], float stdMAG[][3],
//				uint32_t* magvalid, uint32_t* totalread)
//{
//	int 		c = 0;
//	int 		N = 200;
//	int 		offset = 50;
//	uint32_t 	lastRead = 0;
//	uint8_t		sensor_num;
//	bool		retbool;
//	sensor_data_t raw_data = {0};
//	bool ret;
//
//	sensor_data_t last_raw_data;
//	bool		first_time = true;
//	static uint32_t same_data_cnt = 0;
//
//	turnOnLED(turqoiseLed);
//
//	while (c < N+offset)
//	{
//
//		for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++)
//		{
//			while((k22f_get_rtc()-lastRead) < 5) {
//				; //do-nothing while loop
//			}
//			lastRead = k22f_get_rtc();
//
//			mpu9250_enable_sensor(sensor_num);
//			retbool = get_raw_data_sample_basic(&raw_data);
//
//			if (first_time) {
//				last_raw_data = raw_data;
//				first_time = false;
//			} else {
//				if (mag_sensor_data_equal(last_raw_data, raw_data)) {
//					same_data_cnt++;
//				}
//				last_raw_data = raw_data;
//			}
//			if (c > offset)
//			{
//				(*totalread)++;
//				if (raw_data.valid_sensors & SENSOR_MAG) {
//					(*magvalid)++;
//				}
//				for(int k = 0; k < 3; k++)
//				{
//					stdACC[sensor_num][k] += pow(( (float) raw_data.accel[k])*MPU9250_GPERCOUNT - meanACC[sensor_num][k],2)/(float)N;
//					stdGYR[sensor_num][k] += pow(( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT - meanGYR[sensor_num][k],2)/(float)N;
//					stdMAG[sensor_num][k] += pow(( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT - meanMAG[sensor_num][k],2)/(float)N;
//				}
//			}
//
//		}
//
//		c++;
//	}
//
//	for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++)
//	{
//		for(int k = 0; k < 3; k++)
//		{
//			stdACC[sensor_num][k] = sqrt(stdACC[sensor_num][k]);
//			stdGYR[sensor_num][k] = sqrt(stdGYR[sensor_num][k]);
//			stdMAG[sensor_num][k] = sqrt(stdMAG[sensor_num][k]);
//		}
//	}
//
////	debug_printf("\n\rSame data cnt: %u", same_data_cnt);
//	turnOnLED(greenLed);
//	OSA_TimeDelay(500);
//}
//
//
//void computeAggregateMinMaxVector(float minACC[][3], float minGYR[][3], float minMAG[][3],float maxACC[][3], float maxGYR[][3], float maxMAG[][3],
//		 	 	 	 	 uint32_t* magvalid, uint32_t* totalread)
//{
//	int 		c = 0;
//	int 		N = 200;
//	int 		offset = 50;
//	uint32_t 	lastRead = {0};
//	uint8_t		sensor_num;
//	bool		retbool;
//	sensor_data_t raw_data = {0};
//	bool ret;
//
//	sensor_data_t last_raw_data;
//	bool		first_time = true;
//	static uint32_t same_data_cnt = 0;
//
//	turnOnLED(turqoiseLed);
//
//
//	while (c < N+offset)
//	{
//
//		for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++)
//		{
//			while((k22f_get_rtc()-lastRead) < 5) {
//				; //do-nothing while loop
//			}
//			lastRead = k22f_get_rtc();
//
//			mpu9250_enable_sensor(sensor_num);
//			retbool = get_raw_data_sample_basic(&raw_data);
//
//			if (first_time) {
//				last_raw_data = raw_data;
//				first_time = false;
//			} else {
//				if (mag_sensor_data_equal(last_raw_data, raw_data)) {
//					same_data_cnt++;
//				}
//				last_raw_data = raw_data;
//			}
//			if (c > offset)
//			{
//				(*totalread)++;
//				if (raw_data.valid_sensors & SENSOR_MAG) {
//					(*magvalid)++;
//				}
//				for(int k = 0; k < 3; k++)
//				{
//					minACC[sensor_num][k] = fmin(minACC[sensor_num][k],( (float) raw_data.accel[k])*MPU9250_GPERCOUNT);
//					minGYR[sensor_num][k] = fmin(minGYR[sensor_num][k],( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT);
//					minMAG[sensor_num][k] = fmin(minMAG[sensor_num][k],( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT);
//					maxACC[sensor_num][k] = fmax(maxACC[sensor_num][k],( (float) raw_data.accel[k])*MPU9250_GPERCOUNT);
//					maxGYR[sensor_num][k] = fmax(maxGYR[sensor_num][k],( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT);
//					maxMAG[sensor_num][k] = fmax(maxMAG[sensor_num][k],( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT);
//				}
//			}
//
//		}
//
//		c++;
//	}
//
////	debug_printf("\n\rSame data cnt: %u", same_data_cnt);
//	turnOnLED(greenLed);
//	OSA_TimeDelay(500);
//}

void computeAverageVector(uint8_t sensor_num, float meanACC[3], float meanGYR[3], float meanMAG[3])
{
	int c = 0;
	int N = 200;
	int offset = 50;

	sensor_data_t raw_data;
	sensor_record_t data;

	bool ret;

	float coeffACC = MPU9250_GPERCOUNT/(float)N;
	float coeffGYR = MPU9250_DEGPERSECPERCOUNT/(float)N;
	float coeffMAG = MPU9250_UTPERCOUNT/(float)N;

	turnOnLED(turqoiseLed);

	while (c < N+offset)
	{
		OSA_TimeDelay(10);

		get_raw_data_sample_basic(sensor_num, &raw_data);

		if (c > offset)
		{
			for(int k = 0; k < 3; k++)
			{
				meanACC[k] += ( (float) raw_data.accel[k])*coeffACC;
				meanGYR[k] += ( (float) raw_data.gyro[k])*coeffGYR;
				meanMAG[k] += ( (float) raw_data.mag[k])*coeffMAG;
			}
		}

		c++;

	}

	turnOnLED(greenLed);
	OSA_TimeDelay(500);

}


void computeStdVector(uint8_t sensor_num, float meanACC[3], float meanGYR[3], float meanMAG[3], float stdACC[3], float stdGYR[3], float stdMAG[3])
{
	int c = 0;
	int N = 200;
	int offset = 50;
	sensor_data_t raw_data;

	turnOnLED(turqoiseLed);

	while (c < N+offset)
	{
		OSA_TimeDelay(10);

		get_raw_data_sample_basic(sensor_num, &raw_data);

		if (c > offset)
		{
			for(int k = 0; k < 3; k++)
			{
				stdACC[k] += pow(( (float) raw_data.accel[k])*MPU9250_GPERCOUNT - meanACC[k],2)/(float)N;
				stdGYR[k] += pow(( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT - meanGYR[k],2)/(float)N;
				stdMAG[k] += pow(( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT - meanMAG[k],2)/(float)N;
			}
		}

		c++;

	}

	for(int k = 0; k < 3; k++)
	{
		stdACC[k] = sqrt(stdACC[k]);
		stdGYR[k] = sqrt(stdGYR[k]);
		stdMAG[k] = sqrt(stdMAG[k]);
	}

	turnOnLED(greenLed);
	OSA_TimeDelay(500);


}


void computeMinMaxVector(uint8_t sensor_num, float minACC[3], float minGYR[3], float minMAG[3],float maxACC[3], float maxGYR[3], float maxMAG[3])
{
	int c = 0;
	int N = 200;
	int offset = 50;


	sensor_data_t raw_data;

	bool ret;

	turnOnLED(turqoiseLed);

	while (c < N+offset)
	{
		OSA_TimeDelay(10);

		get_raw_data_sample_basic(sensor_num,&raw_data);

		if(c > offset)
		{
			for(int k = 0; k < 3; k++)

			{
				minACC[k] = fmin(minACC[k],( (float) raw_data.accel[k])*MPU9250_GPERCOUNT);
				minGYR[k] = fmin(minGYR[k],( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT);
				minMAG[k] = fmin(minMAG[k],( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT);

				maxACC[k] = fmax(maxACC[k],( (float) raw_data.accel[k])*MPU9250_GPERCOUNT);
				maxGYR[k] = fmax(maxGYR[k],( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT);
				maxMAG[k] = fmax(maxMAG[k],( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT);
			}
		}

		c++;
	}

	turnOnLED(greenLed);
	OSA_TimeDelay(500);

}

bool checkInitialGyrData(float m, float M, float std)
{
	return !((M > 10)|(m < -10)|(std > 1));
}

bool checkInitialAccData(float m, float M, float std)
{
	return !((M > 2)|(m < -2)|(std > 0.1));
}

bool checkInitialMagData(float m, float M, float std)
{
	return !((M > 500)|(m < -500)|(std > 5));
}

bool tsSelftest()
{
	bool retboolG;
	int c = 0;
	int N = 500;
	uint8_t sensor_num;

	float MAG_RANGE = 4800;

	sensor_data_t raw_data;

	bool goodAcc[4];
	bool goodGyr[4];
	bool goodMag[4];

	//mpu9250_init();
	//mpu9250_start();

	float meanACC[3], meanGYR[3], meanMAG[3];
	float stdACC[3], stdGYR[3], stdMAG[3];
	float minACC[3], minGYR[3], minMAG[3];
	float maxACC[3], maxGYR[3], maxMAG[3];

	for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {

		// Must set the correct CS to select the correct Sensor
		sat_enable_sensor(sensor_num, get_uart_num(sensor_num));

		// Initialize Values
		for (uint8_t qq = 0; qq < 3; qq++) {
			meanACC[qq] = 0;
			meanGYR[qq] = 0;
			meanMAG[qq] = 0;

			stdACC[qq] = 0;
			stdGYR[qq] = 0;
			stdMAG[qq] = 0;

			minACC[qq] = ACCEL_RANGE_G;
			minGYR[qq] = GYRO_RANGE_DPS;
			minMAG[qq] = MAG_RANGE;

			maxACC[qq] = -ACCEL_RANGE_G;
			maxGYR[qq] = -GYRO_RANGE_DPS;
			maxMAG[qq] = -MAG_RANGE;
		}

		computeAverageVector(sensor_num,meanACC,meanGYR,meanMAG);

		computeMinMaxVector(sensor_num,minACC,minGYR,minMAG,maxACC,maxGYR,maxMAG);

		computeStdVector(sensor_num,meanACC,meanGYR,meanMAG,stdACC,stdGYR,stdMAG);

		//Thresholding
		for (int k = 0; k < 3; k++)
		{

			goodAcc[k] = checkInitialAccData(minACC[k],maxACC[k],stdACC[k]);

			goodGyr[k] = checkInitialGyrData(minGYR[k],maxGYR[k],stdGYR[k]);

			goodMag[k] = checkInitialMagData(minMAG[k],maxMAG[k],stdMAG[k]);

		}

		goodAcc[4] = goodAcc[0]&goodAcc[1]&goodAcc[2];
		goodGyr[4] = goodGyr[0]&goodGyr[1]&goodGyr[2];
		goodMag[4] = goodMag[0]&goodMag[1]&goodMag[2];


		if (!(goodAcc[4]&goodGyr[4]&goodMag[4]))
		{
			turnOnLED(redLed);
			OSA_TimeDelay(3000);
		} else {
			turnOnLED(blueLed);
			OSA_TimeDelay(3000);
			turnOnLED(greenLed);
		}
	}

}

/* WARNING: Make sure you call set_enable_sensor for the sensor
 * before you call this function!
 */
bool tsSelftestIndividual(uint8_t sensor_num)
{
	bool retboolG;
	int c = 0;
	int N = 500;
	bool ret;
	float MAG_RANGE = 4800;

	sensor_data_t raw_data;

	bool goodAcc[4];
	bool goodGyr[4];
	bool goodMag[4];

	float meanACC[3], meanGYR[3], meanMAG[3];
	float stdACC[3], stdGYR[3], stdMAG[3];
	float minACC[3], minGYR[3], minMAG[3];
	float maxACC[3], maxGYR[3], maxMAG[3];

	// Initialize Values
	for (uint8_t qq = 0; qq < 3; qq++) {
		meanACC[qq] = 0;
		meanGYR[qq] = 0;
		meanMAG[qq] = 0;

		stdACC[qq] = 0;
		stdGYR[qq] = 0;
		stdMAG[qq] = 0;

		minACC[qq] = ACCEL_RANGE_G;
		minGYR[qq] = GYRO_RANGE_DPS;
		minMAG[qq] = MAG_RANGE;

		maxACC[qq] = -ACCEL_RANGE_G;
		maxGYR[qq] = -GYRO_RANGE_DPS;
		maxMAG[qq] = -MAG_RANGE;
	}

	computeAverageVector(sensor_num,meanACC,meanGYR,meanMAG);

	computeMinMaxVector(sensor_num,minACC,minGYR,minMAG,maxACC,maxGYR,maxMAG);

	computeStdVector(sensor_num,meanACC,meanGYR,meanMAG,stdACC,stdGYR,stdMAG);

	//Thresholding
	for (int k = 0; k < 3; k++)
	{

		goodAcc[k] = checkInitialAccData(minACC[k],maxACC[k],stdACC[k]);

		goodGyr[k] = checkInitialGyrData(minGYR[k],maxGYR[k],stdGYR[k]);

		goodMag[k] = checkInitialMagData(minMAG[k],maxMAG[k],stdMAG[k]);

	}

	goodAcc[4] = goodAcc[0]&goodAcc[1]&goodAcc[2];
	goodGyr[4] = goodGyr[0]&goodGyr[1]&goodGyr[2];
	goodMag[4] = goodMag[0]&goodMag[1]&goodMag[2];


	if (!(goodAcc[4]&goodGyr[4]&goodMag[4]))
	{
		ret = false;
		turnOnLED(redLed);
		OSA_TimeDelay(3000);
	} else {
		ret = true;
		turnOnLED(blueLed);
		OSA_TimeDelay(3000);
		turnOnLED(greenLed);
	}

	return ret;
}

//bool checkMagOutput(parameters_t parameters)
//{
//	uint32_t	magvalid, totalread;
//	uint8_t		sensor_num;
//
//	for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++)
//	{
//		mpu9250_enable_sensor(sensor_num);
//		mpu9250_start();
//	}
//
//	/* Data collection */
//	magvalid = 0;
//	totalread = 0;
//	readMagData(&magvalid, &totalread);
//}

//bool checkInitialSensorData(parameters_t parameters, bool quiet, bool* accpass, bool* gyrpass, bool* magpass)
//{
//	bool retboolG;
//	int c = 0;
//	int N = 500;
//
//	uint32_t magvalid;
//	uint32_t totalread;
//
//	float MAG_RANGE = 4800;
//
//	sensor_data_t raw_data;
//
//	bool goodAcc[4];
//	bool goodGyr[4];
//	bool goodMag[4];
//
//	mpu9250_start();
//
//	float meanACC[3] = {0.0f,0.0f,0.0f};
//	float meanGYR[3] = {0.0f,0.0f,0.0f};
//	float meanMAG[3] = {0.0f,0.0f,0.0f};
//
//	float stdACC[3] = {0.0f,0.0f,0.0f};
//	float stdGYR[3] = {0.0f,0.0f,0.0f};
//	float stdMAG[3] = {0.0f,0.0f,0.0f};
//
//	float minACC[3] = {ACCEL_RANGE_G,ACCEL_RANGE_G,ACCEL_RANGE_G};
//	float minGYR[3] = {GYRO_RANGE_DPS,GYRO_RANGE_DPS,GYRO_RANGE_DPS};
//	float minMAG[3] = {MAG_RANGE,MAG_RANGE,MAG_RANGE};
//
//	float maxACC[3] = {-ACCEL_RANGE_G,-ACCEL_RANGE_G,-ACCEL_RANGE_G};
//	float maxGYR[3] = {-GYRO_RANGE_DPS,-GYRO_RANGE_DPS,-GYRO_RANGE_DPS};
//	float maxMAG[3] = {-MAG_RANGE,-MAG_RANGE,-MAG_RANGE};
//
//	magvalid = 0;
//	totalread = 0;
//	debug_printf("\n\r");
//	if (parameters.log_enableDetail > 0) {
//		debug_printf("\n\rAverage...");
//	}
//	computeAverageVector(meanACC,meanGYR,meanMAG,&magvalid,&totalread);
//	if (parameters.log_enableDetail > 0) {
//		debug_printf("done\r\n");
//	}
//	if (parameters.log_enableDetail > 0) {
//		debug_printf("\n\rMinMax...");
//	}
//	computeMinMaxVector(minACC,minGYR,minMAG,maxACC,maxGYR,maxMAG,&magvalid,&totalread);
//	if (parameters.log_enableDetail > 0) {
//		debug_printf("done\r\n");
//	}
//
//	if (parameters.log_enableDetail > 0) {
//		debug_printf("\n\rStd...");
//	}
//	computeStdVector(meanACC,meanGYR,meanMAG,stdACC,stdGYR,stdMAG,&magvalid,&totalread);
//	if (parameters.log_enableDetail > 0) {
//		debug_printf("done\r\n");
//	}
//
//	if (!quiet) {
//		debug_printf("\n\r************ RESULTS SENSOR SCREENING ************\r\n");
//	}
//	if (parameters.log_enableDetail > 1)
//	{
//		debug_printf("\t MIN\tMEAN\tMAX\tSTD\r\n");
//		debug_printf("ACCx:\t %d\t%d\t%d\t%d\r\n",(int)(minACC[0]*1000),(int)(meanACC[0]*1000),(int)(maxACC[0]*1000),(int)(stdACC[0]*1000));
//		debug_printf("ACCy:\t %d\t%d\t%d\t%d\r\n",(int)(minACC[1]*1000),(int)(meanACC[1]*1000),(int)(maxACC[1]*1000),(int)(stdACC[1]*1000));
//		debug_printf("ACCz:\t %d\t%d\t%d\t%d\r\n",(int)(minACC[2]*1000),(int)(meanACC[2]*1000),(int)(maxACC[2]*1000),(int)(stdACC[2]*1000));
//
//		debug_printf("GYRx:\t %d\t%d\t%d\t%d\r\n",(int)(minGYR[0]*1000),(int)(meanGYR[0]*1000),(int)(maxGYR[0]*1000),(int)(stdGYR[0]*1000));
//		debug_printf("GYRy:\t %d\t%d\t%d\t%d\r\n",(int)(minGYR[1]*1000),(int)(meanGYR[1]*1000),(int)(maxGYR[1]*1000),(int)(stdGYR[1]*1000));
//		debug_printf("GYRz:\t %d\t%d\t%d\t%d\r\n",(int)(minGYR[2]*1000),(int)(meanGYR[2]*1000),(int)(maxGYR[2]*1000),(int)(stdGYR[2]*1000));
//
//		debug_printf("MAGx:\t %d\t%d\t%d\t%d\r\n",(int)(minMAG[0]*1000),(int)(meanMAG[0]*1000),(int)(maxMAG[0]*1000),(int)(stdMAG[0]*1000));
//		debug_printf("MAGy:\t %d\t%d\t%d\t%d\r\n",(int)(minMAG[1]*1000),(int)(meanMAG[1]*1000),(int)(maxMAG[1]*1000),(int)(stdMAG[1]*1000));
//		debug_printf("MAGz:\t %d\t%d\t%d\t%d\r\n",(int)(minMAG[2]*1000),(int)(meanMAG[2]*1000),(int)(maxMAG[2]*1000),(int)(stdMAG[2]*1000));
//	}
//
//	mpu9250_stop();
//
//	//TODO thresholding the mean* vectors
//	for (int k = 0; k < 3; k++)
//	{
//
//		goodAcc[k] = checkInitialAccData(minACC[k],maxACC[k],stdACC[k]);
//
//		goodGyr[k] = checkInitialGyrData(minGYR[k],maxGYR[k],stdGYR[k]);
//
//		goodMag[k] = checkInitialMagData(minMAG[k],maxMAG[k],stdMAG[k]);
//
//	}
//
//	goodAcc[3] = goodAcc[0]&goodAcc[1]&goodAcc[2];
//	goodGyr[3] = goodGyr[0]&goodGyr[1]&goodGyr[2];
//	goodMag[3] = goodMag[0]&goodMag[1]&goodMag[2];
//
//	if (parameters.log_enableDetail > 0)
//	{
//		debug_printf("Accelerometer test...");
//		if (goodAcc[3])
//			(debug_printf("OK!\r\n"));
//		else
//			(debug_printf("NO!\r\n"));
//
//		debug_printf("Gyroscope test...");
//		if (goodGyr[3])
//			(debug_printf("OK!\r\n"));
//		else
//			(debug_printf("NO!\r\n"));
//
//		debug_printf("Magnetometer test...");
//
//		if (goodMag[3])
//			(debug_printf("OK!\r\n"));
//		else
//			(debug_printf("NO!\r\n"));
//	}
//
//
//	*accpass = goodAcc[3];
//	*gyrpass = goodGyr[3];
//	*magpass = goodMag[3];
//	if (!quiet) {
//		if (goodAcc[3]&goodGyr[3]&goodMag[3]) {
//			debug_printf("[SENSOR:PASS]\r\n");
//		} else {
//			debug_printf("[SENSOR:FAIL]\r\n");
//		}
//	}
//}

void showResultTag(){
	debug_printf("****************** OUTPUT CODES ******************\r\n");
	debug_printf("  SPI2TN: test 1.A - 22F<>Nordic SPI test\r\n");
	debug_printf("  SPI2TI: test 1.B - 22F<>InvenSense SPI test\r\n");
	debug_printf("  SENSCR: test 1.C - sensor screening\r\n");
	debug_printf("  SATDTM: DTM FW version\r\n");
	debug_printf("  SATIDX: calculated SAT ID\r\n");

}


