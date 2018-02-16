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
#include "mpu9250_firmware.h"
#include "mpu9250_freq.h"
#include "ts_rtc.h"

/* USB */
#include "dtm_common_types.h"

#include "fsl_debug_console.h"

#include "crc16.h"
#include "dtmTestUtilities.h"
#include "fsl_dspi_master_driver.h"


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
#include "main_22f_on_board_diag.h"

#define MAX(a,b) ((a>b)?(a):(b))
#define SPI_COMM_LENGTH MAX(sizeof(sat_to_nordic_dtm_t),sizeof(nordic_to_sat_dtm_t))


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static uint8_t spi_tx_arr[SPI_COMM_LENGTH] = {0};
static uint8_t spi_rx_arr[SPI_COMM_LENGTH] = {0};

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
static const dspi_device_t mpu9250_dspi_device =
{
	.bitsPerSec = 1000000L,
	.dataBusConfig.bitsPerFrame = 8,
	.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
	.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge,
	.dataBusConfig.direction = kDspiMsbFirst,
};
#define BUFFER_SIZE 65
static uint8_t mts_buffer[BUFFER_SIZE];
static uint8_t stm_buffer[BUFFER_SIZE];
static void mpu9250_write_reg(uint8_t reg_addr, uint8_t data)
{
	dspi_status_t Error;

	mts_buffer[0] = reg_addr;
	mts_buffer[1] = data;

	Error = DSPI_DRV_MasterTransferBlocking(kMpu9250SpiInstance,
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
	Error = DSPI_DRV_MasterTransferBlocking(kMpu9250SpiInstance,
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

int spiSensorRegisterWriteTest(parameters_t parameters)
{
	uint8_t  writeV=0, readV=0;
	debug_printf(" ************ RESULTS SPI SENSORS TESTING ************\r\n");

	mpu9250_init();
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

	 if(nFail > parameters.spiSensor_max_WR_fail)
		 debug_printf("[SPI2TI:FAILED]\r\n");
	 else
		 debug_printf("[SPI2TI:OK]\r\n");
}

// ************************************************************ //
// **************** SELF TEST IMPLEMENTATION ***************** //
static bool get_raw_data_sample_basic(sensor_data_t *raw_data) {
	bool ret;

	while (!mpu9250_data_avail())
		;
	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,
			raw_data);

	return ret;
}


void computeAverageVector(float meanACC[3], float meanGYR[3], float meanMAG[3])
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

		get_raw_data_sample_basic(&raw_data);

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


void computeStdVector(float meanACC[3], float meanGYR[3], float meanMAG[3], float stdACC[3], float stdGYR[3], float stdMAG[3])
{
	int c = 0;
	int N = 200;
	int offset = 50;
	sensor_data_t raw_data;

	turnOnLED(turqoiseLed);

	while (c < N+offset)
	{
		OSA_TimeDelay(10);

		get_raw_data_sample_basic(&raw_data);

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


void computeMinMaxVector(float minACC[3], float minGYR[3], float minMAG[3],float maxACC[3], float maxGYR[3], float maxMAG[3])
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

		get_raw_data_sample_basic(&raw_data);

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

bool checkInitialSensorData(parameters_t parameters)
{
	bool retboolG;
	int c = 0;
	int N = 500;

	float MAG_RANGE = 4800;

	sensor_data_t raw_data;

	bool goodAcc[4];
	bool goodGyr[4];
	bool goodMag[4];

	mpu9250_init();
//
//	uint8_t data[MPU9250_REG_COUNT];
//	mpu9250_read_regs(MPU9250_FIRST_REG, MPU9250_SELF_TEST_REG_COUNT, data);
//	for (uint8_t qq = 0; qq < MPU9250_REG_COUNT; qq++) {
//		printf("\n\r0 Sensor data[%u]: %u", qq, data[qq]);
//	}
	mpu9250_start();
//
//	mpu9250_read_regs(MPU9250_FIRST_REG, MPU9250_SELF_TEST_REG_COUNT, data);
//	for (uint8_t qq = 0; qq < MPU9250_REG_COUNT; qq++) {
//		printf("\n\r1 Sensor data[%u]: %u", qq, data[qq]);
//	}

	float meanACC[3] = {0.0f,0.0f,0.0f};
	float meanGYR[3] = {0.0f,0.0f,0.0f};
	float meanMAG[3] = {0.0f,0.0f,0.0f};

	float stdACC[3] = {0.0f,0.0f,0.0f};
	float stdGYR[3] = {0.0f,0.0f,0.0f};
	float stdMAG[3] = {0.0f,0.0f,0.0f};

	float minACC[3] = {ACCEL_RANGE_G,ACCEL_RANGE_G,ACCEL_RANGE_G};
	float minGYR[3] = {GYRO_RANGE_DPS,GYRO_RANGE_DPS,GYRO_RANGE_DPS};
	float minMAG[3] = {MAG_RANGE,MAG_RANGE,MAG_RANGE};

	float maxACC[3] = {-ACCEL_RANGE_G,-ACCEL_RANGE_G,-ACCEL_RANGE_G};
	float maxGYR[3] = {-GYRO_RANGE_DPS,-GYRO_RANGE_DPS,-GYRO_RANGE_DPS};
	float maxMAG[3] = {-MAG_RANGE,-MAG_RANGE,-MAG_RANGE};

	debug_printf("Average...");
	computeAverageVector(meanACC,meanGYR,meanMAG);
	debug_printf("done\r\n");

	debug_printf("MinMax...");
	computeMinMaxVector(minACC,minGYR,minMAG,maxACC,maxGYR,maxMAG);
	debug_printf("done\r\n");

	debug_printf("Std...");
	computeStdVector(meanACC,meanGYR,meanMAG,stdACC,stdGYR,stdMAG);
	debug_printf("done\r\n");

	debug_printf("************ RESULTS SENSOR SCREENING ************\r\n");
	if (parameters.log_enableDetail > 1)
	{
		debug_printf("\t MIN\tMEAN\tMAX\tSTD\r\n");
		debug_printf("ACCx:\t %d\t%d\t%d\t%d\r\n",(int)(minACC[0]*1000),(int)(meanACC[0]*1000),(int)(maxACC[0]*1000),(int)(stdACC[0]*1000));
		debug_printf("ACCy:\t %d\t%d\t%d\t%d\r\n",(int)(minACC[1]*1000),(int)(meanACC[1]*1000),(int)(maxACC[1]*1000),(int)(stdACC[1]*1000));
		debug_printf("ACCz:\t %d\t%d\t%d\t%d\r\n",(int)(minACC[2]*1000),(int)(meanACC[2]*1000),(int)(maxACC[2]*1000),(int)(stdACC[2]*1000));

		debug_printf("GYRx:\t %d\t%d\t%d\t%d\r\n",(int)(minGYR[0]*1000),(int)(meanGYR[0]*1000),(int)(maxGYR[0]*1000),(int)(stdGYR[0]*1000));
		debug_printf("GYRy:\t %d\t%d\t%d\t%d\r\n",(int)(minGYR[1]*1000),(int)(meanGYR[1]*1000),(int)(maxGYR[1]*1000),(int)(stdGYR[1]*1000));
		debug_printf("GYRz:\t %d\t%d\t%d\t%d\r\n",(int)(minGYR[2]*1000),(int)(meanGYR[2]*1000),(int)(maxGYR[2]*1000),(int)(stdGYR[2]*1000));

		debug_printf("MAGx:\t %d\t%d\t%d\t%d\r\n",(int)(minMAG[0]*1000),(int)(meanMAG[0]*1000),(int)(maxMAG[0]*1000),(int)(stdMAG[0]*1000));
		debug_printf("MAGy:\t %d\t%d\t%d\t%d\r\n",(int)(minMAG[1]*1000),(int)(meanMAG[1]*1000),(int)(maxMAG[1]*1000),(int)(stdMAG[1]*1000));
		debug_printf("MAGz:\t %d\t%d\t%d\t%d\r\n",(int)(minMAG[2]*1000),(int)(meanMAG[2]*1000),(int)(maxMAG[2]*1000),(int)(stdMAG[2]*1000));
	}

	mpu9250_stop();

	//TODO thresholding the mean* vectors
	for (int k = 0; k < 3; k++)
	{

		goodAcc[k] = checkInitialAccData(minACC[k],maxACC[k],stdACC[k]);

		goodGyr[k] = checkInitialGyrData(minGYR[k],maxGYR[k],stdGYR[k]);

		goodMag[k] = checkInitialMagData(minMAG[k],maxMAG[k],stdMAG[k]);

	}

	goodAcc[4] = goodAcc[0]&goodAcc[1]&goodAcc[2];
	goodGyr[4] = goodGyr[0]&goodGyr[1]&goodGyr[2];
	goodMag[4] = goodMag[0]&goodMag[1]&goodMag[2];

	if (parameters.log_enableDetail > 0)
	{
		debug_printf("Accelerometer test...");
		if (goodAcc[4])
			(debug_printf("OK!\r\n"));
		else
			(debug_printf("NO!\r\n"));

		debug_printf("Gyroscope test...");
		if (goodGyr[4])
			(debug_printf("OK!\r\n"));
		else
			(debug_printf("NO!\r\n"));

		debug_printf("Magnetometer test...");

		if (goodMag[4])
			(debug_printf("OK!\r\n"));
		else
			(debug_printf("NO!\r\n"));
	}


	if (goodAcc[4]&goodGyr[4]&goodMag[4])
		debug_printf("[SENSCR:OK]\r\n");
	else
		debug_printf("[SENSCR:FAILED]\r\n");
}

void showResultTag(){
	debug_printf("****************** OUTPUT CODES ******************\r\n");
	debug_printf("  SPI2TN: test 1.A - 22F<>Nordic SPI test\r\n");
	debug_printf("  SPI2TI: test 1.B - 22F<>InvenSense SPI test\r\n");
	debug_printf("  SENSCR: test 1.C - sensor screening\r\n");
	debug_printf("  SATDTM: DTM FW version\r\n");
	debug_printf("  SATIDX: calculated SAT ID\r\n");

}


