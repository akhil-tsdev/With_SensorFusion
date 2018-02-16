/*
 * dtmTestUtilities_pivot3_0_maxwell.h
 *
 *  Created on: 2 août 2016
 *      Author: cdoix
 */


#ifndef SOURCES_PERIPHERAL_DTMTESTUTILITIES_H_
#define SOURCES_PERIPHERAL_DTMTESTUTILITIES_H_


#define POS_NR_ITERACTIONS 0
#define POS_NR_SPBUSYSTATE 1
#define POS_NR_WRREADCICLE 2  //number of packages to send
#define POS_NR_PACKAGESENT 3  //number of package sent
#define POS_NR_ATTEMPTCICL 4  //attempt for package
#define POS_NR_PACKAGERECE 5  //number of package received
#define POS_NR_PACKAGELOST 6  //number of package lost
#define POS_NR_ERRORTRANSM 7  //number of compromise package
#define POS_NR_ITERACTIONS_TOT 8 //as 0 total
#define POS_NR_SPBUSYSTATE_TOT 9 //as 1 total
#define POS_NR_CRC16FAILED 10  //CRC FAILED
#define POS_NR_CRC16LASTFL_TOT 11  //LAST CRC FAILED
#define NR_STATISTIC_ELEMENTS 12


extern sat_to_nordic_dtm_t spi_tx;
extern nordic_to_sat_dtm_t spi_rx;

int spiSendReceive();

void spiResetParams();

int spiTest(parameters_t parameters);

bool checkMagOutput(parameters_t parameters);
void setNordicParameters(int flag);
int spiTest(parameters_t parameters);
void showResultTag();
int spiSensorRegisterWriteTest(parameters_t parameters, bool* pass);
void sensorDataIntegrityTest_spi(parameters_t parameters, bool* pass);
void spiSendUntilAcked();
void initSensors();
bool tsSelftestIndividual(uint8_t sensor_num);

#endif /* SOURCES_PERIPHERAL_DTMTESTUTILITIES_H_ */
