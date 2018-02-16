#include "adc_ts.h"




/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ADC_X                   ADC0
#define CHANNEL_0				(0U)
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool conversionCompleted = false; /*! Conversion is completed Flag */


static bool firstTimeCalibration = true;
void adc_init_1(void) {
  bool conversionCompleted = 0; //0 is OK
  //initializzation pg 793
   ADC1_CFG1 = ADC_CFG1_MODE(0x3);
   ADC1_CFG1 |= ADC_CFG1_ADIV(0x3); //pg793->765(ADIV) -- 11 The divide ratio is 8 and the clock rate is (input clock)/8.

   ADC1_CFG1 |= (0x1 << ADC_CFG1_ADLPC_SHIFT) & ADC_CFG1_ADLPC_MASK; //pg793->765(ADLPC) -- 1 Low-power configuration. The power is reduced at the expense of maximum clock speed.

   ADC1_CFG1 |= (0x1 << ADC_CFG1_ADLSMP_SHIFT) & ADC_CFG1_ADLSMP_MASK; //Sample time configuration: 1 Long sample time. [new]

   //calibration
   ADC1_SC3 = ADC_SC3_AVGS(0x3); //and SC3[AVGS]=11 Hardware Average Select: 11 32 samples averaged.
   ADC1_SC3 |= (0x1 << ADC_SC3_AVGE_SHIFT) & ADC_SC3_AVGE_MASK; //SC3[AVGE]=1
   ADC1_SC3 |= ADC_SC3_AVGS_MASK | ADC_SC3_AVGS(0x3); //SC3[AVGE]=1 and SC3[AVGS]=11

   ADC1_SC2 |= (0x0 << ADC_SC2_ADTRG_SHIFT) & ADC_SC2_ADTRG_MASK; //pg794->770(ADLPC) -- Conversion Trigger Select: 0 Software trigger selected.
   ADC1_SC3 |= (0x0 << ADC_SC3_CAL_SHIFT) & ADC_SC3_CAL_MASK; //clean register
   ADC1_SC3 |= (0x1 << ADC_SC3_CAL_SHIFT) & ADC_SC3_CAL_MASK; //pg794->771(ADLPC) -- Begins the calibration sequence when set NOTE! CALF is Calibration Failed Flag

   //wait for calibrazion
   while(!(ADC1_SC1A & ADC_SC1_COCO_MASK) && (ADC_SC3_CALF_MASK & ADC1_SC3)) { //pg794->762(ADLPC) -- 1 Conversion is completed.
   }

   conversionCompleted = ADC1_SC3 & ADC_SC3_CALF_MASK;

   //gain calibration values
   int16_t gainPlus = (ADC1_CLP0+ADC1_CLP1+ADC1_CLP2+ADC1_CLP3+ADC1_CLP4+ADC1_CLPS);
   gainPlus /= 2;
   gainPlus = gainPlus | (0x1 << 15); //MSB (more significant bit) setted
   ADC1_PG = gainPlus; //Store the value in the plus-side gain calibration register PG.

   int16_t gainMinus = (ADC1_CLM0+ADC1_CLM1+ADC1_CLM2+ADC1_CLM3+ADC1_CLM4+ADC1_CLMS);
   gainMinus /= 2;
   gainMinus = gainMinus | (0x1 << 15); //MSB (more significant bit) setted
   ADC1_MG = gainMinus; //Store the value in the plus-side gain calibration register PG.

   //enable DIFF option
   ADC1_SC1A |= (0x1 << ADC_SC1_DIFF_SHIFT) & ADC_SC1_DIFF_MASK; //pg 113

   //SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK ;

}


uint32_t adc_read_1(uint8_t channel) {
   uint32_t tmp = SIM_SCGC6;
   uint32_t result;

   //uint32_t SIM_SCGC3_tmp = SIM_SCGC3;
   //uint32_t ADC1_CFG1_tmp = ADC1_CFG1;
   //uint32_t ADC1_SC3_tmp = ADC1_SC3;
   //uint32_t ADC1_SC2_tmp = ADC1_SC2;

   SIM_SCGC6 |= SIM_SCGC6_ADC1_MASK ;

   if(true){
     adc_init_1();
     firstTimeCalibration = false;
   }
   ADC1_SC1A = channel;

   while(!(ADC1_SC1A & ADC_SC1_COCO_MASK)) {
   }
   result = ADC1_RA;
   SIM_SCGC6 = tmp;
   //DelayFunction();

   //SIM_SCGC3 = SIM_SCGC3_tmp;
   //ADC1_CFG1 = ADC1_CFG1_tmp;
   //ADC1_SC3 = ADC1_SC3_tmp;
   //ADC1_SC2 = ADC1_SC2_tmp;

   return result;
}

static bool readBatteryFirstCall = true;
float readBatteryApplyFilter(uint32_t res16){

       //calculate multiplicativeVREF
       if(true){
         float VREFH = (float)adc_read_1(29); //-VREFH (Diff)
         multiplicativeVREF  = (VREFH/1.75);
         readBatteryFirstCall = false;
       }

       	avgResult2[avgResult1_posX%NUMSIMPLE] = multiplicativeVREF;
	float divider=0, result1=0;
	for(int i=0;i<NUMSIMPLE;i++){
		if(avgResult2[i]!=0){
			result1 += avgResult2[i];
			divider+=1;
		}
	}
        multiplicativeVREF = result1 / divider;

	avgResult1[avgResult1_posX%NUMSIMPLE] = (((float)res16)); //1850*4.1f*1.78615 +
	avgResult1_posX++;
	divider=0, result1=0;
	for(int i=0;i<NUMSIMPLE;i++){
		if(avgResult1[i]!=0){
			result1 += avgResult1[i];
			divider+=1;
		}
	}
	result1 = 3.709f * (result1 / (multiplicativeVREF * (!divider?1:(float)divider)));

	return result1;
}
