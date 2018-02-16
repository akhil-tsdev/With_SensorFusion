/*
 * mpu9250.c
 *
 *  Created on: Jan 24, 2015
 *      Author: cwati
 */
#include "MPU9250_hw_if.h"
#include "MPU9250_SPI.h"

/* CWATI specific */
extern uint32_t cwati_spi_send(uint8_t writeAddr); //CWATI SUPER BAD NEED TO REPLACE LATER

//cwati static volatile uint8_t spi_rx_ready_flag=0;
mpu9250_spi spi;
#ifndef CWATI
#define CWATI(s, ...) printf("%s:%d cwati" s "\n", __FUNCTION__,__LINE__, __VA_ARGS__)
#endif /* CWATI */


int cwati() {
	printf("CWATI: this will be library \n");
}

uint32_t mpu9250_spi_WriteReg(uint8_t WriteAddr, uint8_t WriteData )
{
    uint32_t data;

    mpu9250_spi_select();
    mpu9250_spi_send(WriteAddr);
    data = mpu9250_spi_send(WriteData);
    mpu9250_spi_deselect();
    //cwati wait_us(50);
    OSA_TimeDelay(0.050); /* in ms */
    return data;
}

uint32_t mpu9250_spi_send(uint8_t data) {
    return(cwati_spi_send(data));
}

uint32_t  mpu9250_spi_ReadReg( uint8_t WriteAddr, uint8_t WriteData )
{
    return mpu9250_spi_WriteReg( WriteAddr | MPU9250_READ_FLAG, WriteData);
}

void mpu9250_spi_ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint32_t Bytes )
{
    uint32_t  i = 0;
//cwati not sure if select/deselect is needed
    mpu9250_spi_select();
//    mpu9250_spi_send(ReadAddr | MPU9250_READ_FLAG);
    cwati_spi_send(ReadAddr | MPU9250_READ_FLAG);
    for(i=0; i<Bytes; i++)
        ReadBuf[i] = cwati_spi_send(0x00);
    mpu9250_spi_deselect();
    //cwati wait_us(50);
    OSA_TimeDelay(0.050); /* in ms */
}

uint8_t mpu9250_spi_init(int sample_rate_div,int low_pass_filter)
{
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        {0x80, MPUREG_PWR_MGMT_1},     // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
        {low_pass_filter, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
        {0x08, MPUREG_ACCEL_CONFIG},   // +-4G
        {0x09, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, MPUREG_INT_PIN_CFG},    //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz

        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

    };

    /*
     * mpu9250_spi_set_mode(mode);
     * mpu9250_spi_set_bits(bits);
     * mpu9250_spi_set_freq(freq);
     * CWATI TODO let's not get too fancy now
     */
//    spi.mode = 	2;
//    spi.bits = 	8;
//    spi.frequency = 1000000;

    for(i=0; i<MPU_InitRegNum; i++) {
        mpu9250_spi_WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        //cwati wait(0.001);  //I2C must slow down the write speed, otherwise it won't work
    	OSA_TimeDelay(1); /* in ms */
    }

    //cwati to do, make numbers prettier
    mpu9250_set_acc_scale(2);
    mpu9250_set_gyro_scale(250);

    //AK8963_calib_Magnetometer();  //Can't load this function here , strange problem?

    return mpu9250_ok;
}

uint32_t mpu9250_set_acc_scale(int scale){
    uint32_t temp_scale;
    mpu9250_spi_WriteReg(MPUREG_ACCEL_CONFIG, scale);

    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;
    }

    temp_scale=mpu9250_spi_WriteReg(MPUREG_ACCEL_CONFIG|MPU9250_READ_FLAG, 0x00);
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;
    }
    return temp_scale;
}


uint32_t mpu9250_set_gyro_scale(int scale){
    uint32_t temp_scale;
    mpu9250_spi_WriteReg(MPUREG_GYRO_CONFIG, scale);
    switch (scale){
        case BITS_FS_250DPS:
            gyro_divider=131;
        break;
        case BITS_FS_500DPS:
            gyro_divider=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_divider=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_divider=16.4;
        break;
    }
    temp_scale=mpu9250_spi_WriteReg(MPUREG_GYRO_CONFIG|MPU9250_READ_FLAG, 0x00);
    switch (temp_scale){
        case BITS_FS_250DPS:
            temp_scale=250;
        break;
        case BITS_FS_500DPS:
            temp_scale=500;
        break;
        case BITS_FS_1000DPS:
            temp_scale=1000;
        break;
        case BITS_FS_2000DPS:
            temp_scale=2000;
        break;
    }
    return temp_scale;
}

uint32_t mpu9250_whoami(){
    uint32_t response;
    response=mpu9250_spi_WriteReg(MPUREG_WHOAMI|MPU9250_READ_FLAG, 0x00);
    return response;
}

void mpu9250_spi_read_acc()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    mpu9250_spi_ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }
}

void mpu9250_read_gyr()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    mpu9250_spi_ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyroscope_data[i]=data/gyro_divider;
    }

}

void mpu9250_spi_read_temp(){
    uint8_t response[2];
    int16_t bit_data;
    float data;
    mpu9250_spi_ReadRegs(MPUREG_TEMP_OUT_H,response,2);

    bit_data=((int16_t)response[0]<<8)|response[1];
    data=(float)bit_data;
    Temperature=(data/340)+36.53;
    mpu9250_spi_deselect();
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
void mpu9250_spi_calib_acc()
{
    uint8_t response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=mpu9250_spi_WriteReg(MPUREG_ACCEL_CONFIG|MPU9250_READ_FLAG, 0x00);
    mpu9250_set_acc_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    mpu9250_spi_ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0]=((response[0]&11100000)>>3)|((response[3]&00110000)>>4);
    calib_data[1]=((response[1]&11100000)>>3)|((response[3]&00001100)>>2);
    calib_data[2]=((response[2]&11100000)>>3)|((response[3]&00000011));

    mpu9250_set_acc_scale(temp_scale);
}

uint8_t mpu9250_AK8963_whoami(){
    uint8_t response;
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|MPU9250_READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    //cwati wait(0.001);
    OSA_Time_Delay(1);
    response=mpu9250_spi_WriteReg(MPUREG_EXT_SENS_DATA_00|MPU9250_READ_FLAG, 0x00);    //Read I2C
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C

    return response;
}
void AK8963_calib_Magnetometer(){
    uint8_t response[3];
    float data;
    int i;

    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|MPU9250_READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    //cwati wait(0.001);
    OSA_Time_Delay(1);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|MPU9250_READ_FLAG, 0x00);    //Read I2C
    mpu9250_spi_ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);

    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
    for(i=0; i<3; i++) {
        data=response[i];
        Magnetometer_ASA[i]=((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
}
void AK8963_read_Magnetometer(){
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;

    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|MPU9250_READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer

    //cwati wait(0.001);
    OSA_Time_Delay(1);
    mpu9250_spi_ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        Magnetometer[i]=data*Magnetometer_ASA[i];
    }
}
void mpu9250_spi_read_all(){
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;

    //Send I2C command at first
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|MPU9250_READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    mpu9250_spi_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    //wait(0.001);
    mpu9250_spi_ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
    //Get accelerometer value
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }
    //Get temperature
    bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
    data=(float)bit_data;
    Temperature=((data-21)/333.87)+21;
    //Get gyroscop value
    for(i=4; i<7; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyroscope_data[i-4]=data/gyro_divider;
    }
    //Get Magnetometer value
    for(i=7; i<10; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        Magnetometer[i-7]=data*Magnetometer_ASA[i-7];
    }
}

void mpu9250_spi_select() {
	printf("cwati PCS=%d ",CS_RD());
	CS_WR(0);
	printf("PCS now=%d",CS_RD());
}

void mpu9250_spi_deselect() {
	CS_WR(1);
}

