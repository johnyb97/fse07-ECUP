#include "ADC.h"
#include "flash.h"
#include "test_sensors_connection.h"

typedef struct {
  uint16_t press_f;
  uint16_t press_r;
  uint16_t apps1;
  uint16_t apps2;
  uint16_t brk1;
  uint16_t brk2;
} ADC_measurements_t;

volatile ADC_measurements_t adc_measurement1;// namerene raw hodnoty
volatile uint16_t adc_measurement2[2];// namerene raw hodnoty
volatile uint16_t adc_measurement3[2];// namerene raw hodnoty

static const uint16_t reference_val = 1000; 
#define PERCENT(percent_) (reference_val*(percent_)/100)
static const uint16_t low_deadzone_threshold = 100; 
static const uint16_t min_acc_presed_threshold = 50; 
static const uint16_t high_deadzone_threshold = 1100;
static const uint16_t FT_can_timeout = 200; //for how long is FT as True because of one error
uint32_t FT_can_countdown; //timeout set after FT found 

#define LOW_DEADZONE(threshold_) ((threshold_) * (reference_val - low_deadzone_threshold)/reference_val) //minimal threshold for pedal to react else pedal in 0
#define HIGH_DEADZONE(threshold_) ((threshold_) * (high_deadzone_threshold)/reference_val) //maximal threshold for pedal to react else pedal in 0

volatile static uint32_t sens_ACC1; //acc pedal sensor 1
volatile static int sens_ACC1_valid;
volatile static int ACC1_raw;

volatile static uint32_t sens_ACC2;  //acc pedal sensor 2
volatile static int sens_ACC2_valid;
volatile static int ACC2_raw;

volatile uint16_t Vol_IN; //vstupni napeti
volatile uint16_t Vol1_out;
volatile uint16_t VOI;

static uint64_t ATPS_countdown; //countdown for ATPS pausability (if brake presed and accelerator is presed more than 25% for at least 200ms than accelerator = 0)
static volatile uint32_t break_press [2];
static const int ATPS_TIMEOUT = 200;
//static uint16_t reakup[2]; //možna bude fungovat za rok???

static Calib_t calibrate;

static uint16_t max_acc_pedal [2] = {1050,1200};
static uint16_t min_acc_pedal [2] = {600,770};

static uint16_t max_breake_pedal [2] = {750,900};
static uint16_t min_breake_pedal [2]= {345,311};	

static uint16_t calib_offset = 10; // calibration offset

static ECUP_Status_t ECUP_stat;
static ECUP_Pedals_t ECUP_ACC;
static ECUP_Pressure_t ECUP_BREAKE;
static ECUP_DiagPressure_t ECUPdiag2;
static ECUP_DiagPos_t ECUPdiag1;
static ECUP_REQCalibPedal_t ECUP_CALIB;
static int send;


/*initialization functions*/

void init_measurements(){
	ATPS_countdown = 0;
	ECUP_stat.APPS_Plausible = 0;
	ECUP_stat.BPPC_Latch = 0;
	ECUP_stat.BrakeActive =0;
	ECUP_stat.BrakeActive_BSPD=0;
	ECUP_stat.FT_ANY=0;
	ECUP_stat.SDC_BOTS=0;
	ECUP_ACC.SEQ = 0;
	FT_can_countdown = 0;
}
void calib_init(){
	load_flash(&calibrate);
	if (calibrate.magic == 0x06c4951e ){
		if(calibrate.validity [0] == 1){
			min_acc_pedal [0]= calibrate.min_val_acc[0];
			min_acc_pedal [1]= calibrate.min_val_acc[1];
		}
		if (calibrate.validity[1] == 1){
			max_acc_pedal [0] = calibrate.max_val_acc[0];
			max_acc_pedal [1] = calibrate.max_val_acc[1];
		}
		if (calibrate.validity[2] == 1){
		min_breake_pedal[0] = calibrate.min_val_breake[0];
		min_breake_pedal[1] = calibrate.min_val_breake [1];
		}
		if (calibrate.validity[3] == 1){
		max_breake_pedal[0] = calibrate.max_val_breake[0];
		max_breake_pedal[1] = calibrate.max_val_breake[1];
		}
	}
}

void start_ADC(ADC_HandleTypeDef* ADC_handle,uint8_t witch){
	switch (witch){
		case 1:
			HAL_ADC_Start_DMA(ADC_handle,(uint32_t*)&adc_measurement1,6); //spusteni DMA ukladani do pameti...uklada se do adc_measurement...presny pocet upresnit
		break;
		case 2:
			HAL_ADC_Start_DMA(ADC_handle,(uint32_t*)adc_measurement2,2); //spusteni DMA ukladani do pameti...uklada se do adc_measurement...presny pocet upresnit
		break;
		case 3:
			HAL_ADC_Start_DMA(ADC_handle,(uint32_t*)adc_measurement3,2); //spusteni DMA ukladani do pameti...uklada se do adc_measurement...presny pocet upresnit
		break;
		default:
			break;
	}
}

/*end of initialization functions */



/*debug part*/

void BLIK_LED(GPIO_TypeDef * port,uint16_t pin){
		HAL_GPIO_TogglePin(port,pin);  
}

/*end of debug part*/

/*
void ERROR_check_pressure(){//not ready yet at all
	//ECUP_ACC.Brake_pos = (sens_Brake_Preasure1+sens_Brake_Preasure2)/2;
	if (ECUP_ACC.Brake_pos > 10){ // zapinam brzdici svetla pri procentnim zmacknuti
		ECUP_stat.BrakeActive = 1;
		ECUP_stat.BrakeActive_BSPD = 1;
		
	}
	else {ECUP_stat.BrakeActive = 0;
		ECUP_stat.BrakeActive_BSPD = 0;}
	if(plauss == 1 && ECUP_ACC.APPS_pos < 50 &&  ECUP_BREAKE.BrakeF < 1000 && ECUP_BREAKE.BrakeR < 1000 ){ // podminky pro zruseni plausability erroru
		plauss = 0;
	}
			
	if(((ECUP_BREAKE.BrakeF > 1000 ||ECUP_BREAKE.BrakeR > 1000 ) && ECUP_ACC.APPS_pos > 250  &&  (int)last_time>APPS_DEADTIME)|| plauss == 1){ //testovani plausability brzd a plynu podminky brzda > 10 bar, a plyn vice nez 25 %
			ECUP_stat.BPPC_Latch = 1;
			ECUP_stat.FT_ANY = 1;
			//ECUP_ACC.APPS_pos = 0;
			plauss = 1;
		} 
	else{
		ECUP_stat.BPPC_Latch = 0;
		last_time =  HAL_GetTick();
	}
}
*/


/*
void ERROR_check_acc_and_percent_transfer(){ // checkovani plausability plynu
	int sensor_difference = sens_ACC1-sens_ACC2; // vypocet zda li rozdil neni vetsi nez 50
	if (ECUP_ACC.Brake_pos > 50 ){  //kontrola zda li neni zmackla brzda
		together = 0; // odkomentovat po otestovani brzdy
	}
	if (sensor_difference<50 && sensor_difference>-50
		&& adc_measurement1.apps1 <= max_control_acc[0]
		&& min_control_acc[0] <= adc_measurement1.apps1
		&& adc_measurement1.apps2 <= max_control_acc[1]
		&& min_control_acc[1] < adc_measurement1.apps2){ // kontrola zda li odchylka mereni pedalu neni vetsi nez 5% a porovnaní s max a min hodnotou 
		ECUP_ACC.FT_APPS_pos=0;
		ECUP_stat.APPS_Plausible = 0;

	}
	else{
		ECUP_ACC.APPS_pos = 0;
		ECUP_stat.APPS_Plausible = 1;

		test_conection(aw_3_Pin,aw_3_GPIO_Port,&ECUP_ACC.FT_APPS_pos,&ECUP_ACC.FT_APPS_pos,&ECUP_stat);//melo by testovat pripojeni senzoru
		test_conection(AW_4_Pin,AW_4_GPIO_Port,&ECUP_ACC.FT_APPS_pos,&ECUP_ACC.FT_APPS_pos,&ECUP_stat);//melo by testovat pripojeni senzoru
	}
}
*/

static int apply_calibration(int raw_value, int min, int max) {
  // TODO: deadzone, a tyhle veci -> Done i hope 
	// TODO: funguje jen pro max > min. nutno overit ze smysl senzoru je kladny -> JAJA it is, pokud to nějaký genius nastavi obracene pocítí můj hněv 
	// TODO: pred volanim funkce musi byt zkontrolovano ze min != max -> prečo? však vracis nulu
	if (FT_can_countdown>0){
		if (FT_can_countdown < HAL_GetTick()){
				ECUP_stat.FT_ANY = 0;
		}
	}
	if(max <= min){
		return 0;
	}
	if (raw_value < min) {
		if (raw_value < LOW_DEADZONE(min)){
			ECUP_stat.FT_ANY = 1;
			FT_can_countdown = HAL_GetTick()+ FT_can_timeout;
		}
		return 0;
	}
	else if (raw_value > max) {
		if (raw_value < HIGH_DEADZONE(max)){
			return reference_val;
		}else{
			ECUP_stat.FT_ANY = 1;
			FT_can_countdown = HAL_GetTick()+ FT_can_timeout;
			return 0;
		}
	}
	else {
		return ( raw_value - min)*PERCENT(100)/(max - min); // senzor plyn
	}
}

static int brake_pressure_raw_to_kPa(int bp_3v3, int correction_offset, int* bp_kPa_out) {
	static const int adcref_mV = 3300;

	// resistor divider values
	const int div_high = 22;
	const int div_low = 33;

	// compensate for the resistive divider
	int bp_actual = bp_3v3 * (div_high + div_low) / div_low;
	int bp_mV = adcref_mV * bp_actual / 4095 + correction_offset;
	int bp_kPa = 100 + (bp_mV - 500) * 10 / 4;

	if (bp_mV < 460) {
		// sensor failure/disconnect
      	*bp_kPa_out = 0;
      	return 0;
	}

	// TODO: low-pass filter

  	// TODO: otestovat! 4.7.
  
	*bp_kPa_out = bp_kPa;
  	return 1;
}
void Calibrate_pedals(ECUP_REQCalibPedal_t *ECUP_CALIB){
	switch (ECUP_CALIB->which){
		case ECUP_CAL_PedalIndex_None://ECUP_CAL_PedalIndex_None
		break;

		case ECUP_CAL_PedalIndex_AppsMin://ECUP_CAL_PedalIndex_AppsMin
				min_acc_pedal[0] = adc_measurement1.apps1+calib_offset;
				min_acc_pedal[1] = adc_measurement1.apps2+calib_offset;
				calibrate.min_val_acc[0] = min_acc_pedal[0];
				calibrate.min_val_acc[1] = min_acc_pedal[1];
				calibrate.validity[0] = 1;
		break;

		case ECUP_CAL_PedalIndex_AppsMax://ECUP_CAL_PedalIndex_AppsMax
				max_acc_pedal[0] = adc_measurement1.apps1-calib_offset;
				max_acc_pedal[1] = adc_measurement1.apps2- calib_offset;
				calibrate.max_val_acc[0] = max_acc_pedal[0];
				calibrate.max_val_acc[1] = max_acc_pedal[1];
				calibrate.validity[1] = 1;
		break;

		case ECUP_CAL_PedalIndex_BrakeMin://ECUP_CAL_PedalIndex_BrakeMin
				min_breake_pedal[0] =ECUPdiag2.BrakePressFRaw-calib_offset;
				min_breake_pedal[1] = ECUPdiag2.BrakePressRRaw-calib_offset;
				calibrate.min_val_breake[0] = min_breake_pedal[0];
				calibrate.min_val_breake[1] = min_breake_pedal[1];
				calibrate.validity[2] = 1;
		break;
		case  ECUP_CAL_PedalIndex_BrakeMax:// ECUP_CAL_PedalIndex_BrakeMax	
				max_breake_pedal[0] = ECUPdiag2.BrakePressFRaw+calib_offset;
				max_breake_pedal[1] = ECUPdiag2.BrakePressRRaw+calib_offset;
				calibrate.max_val_breake[0] = max_breake_pedal[0];
				calibrate.max_val_breake[1] = max_breake_pedal[1];
				calibrate.validity[3] = 1;
		break;
		case ECUP_CAL_PedalIndex_RegenMax:
			//reakup[0] = adc_measurement1.press_f;  //možna bude fungovat za rok???
			//reakup[1] = adc_measurement1.press_r;  //možna bude fungovat za rok???
		default:
			break;
		}
}

static void update_pedal_measurements(int PRESS_F_raw, int PRESS_R_raw, int ACC1_raw, int ACC2_raw, int BRK1_raw, int BRK2_raw) {
 // TODO: projit vsechny faulty definovane v CANdb a overit ze se skutecne kontroluji tady v kodu
	int pressure_F_valid;
	int pressure_R_valid;
	static int pressure_F_kPa;
	static int pressure_R_kPa;
  ECUPdiag1.Acc1_raw = ACC1_raw;
	ECUPdiag1.Acc2_raw = ACC2_raw;
	//ECUPdiag1.BPPC_raw = 
	ECUPdiag2.BrakePressFRaw = PRESS_F_raw;
	ECUPdiag2.BrakePressRRaw = PRESS_R_raw;

	ECUP_stat.FT_ANY = 0;

	// Pedal processing

    // Pedal processing -- brake pressure
	pressure_F_valid = brake_pressure_raw_to_kPa(PRESS_F_raw, 100, &pressure_F_kPa);
	pressure_R_valid = brake_pressure_raw_to_kPa(PRESS_R_raw, 100, &pressure_R_kPa);

	ECUP_BREAKE.BrakeF = pressure_F_kPa;
	ECUP_BREAKE.BrakeR = pressure_R_kPa;
    // brakeActive threshold
  	const int brakeActive_low = 240;
  	const int brakeActive_high = 300;
  
	if ((pressure_R_valid)&&(pressure_F_valid)) {
		if (pressure_R_kPa > brakeActive_high){
			ECUP_stat.BrakeActive = 1;
		}else if (pressure_R_kPa < brakeActive_low){
			ECUP_stat.BrakeActive = 0;
		}
	}
	else{
		ECUP_stat.BrakeActive = 0;
	}
	HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,(GPIO_PinState)ECUP_stat.BrakeActive);

  	// Pedal processing -- APPS calibration
    // Convert raw values to percentages
	sens_ACC1 = apply_calibration(ACC1_raw, min_acc_pedal[0], max_acc_pedal[0]);
	sens_ACC2 = apply_calibration(ACC2_raw, min_acc_pedal[1], max_acc_pedal[1]);

	// Check plausibility
    const int plausibility_threshold = PERCENT(5);
    int APPS_plausible = (abs((int)sens_ACC1 - (int)sens_ACC2) <= plausibility_threshold);

    if (APPS_plausible) {
      ECUP_ACC.APPS_pos = (sens_ACC1+sens_ACC2)/2;
      ECUP_ACC.FT_APPS_pos = 0;
      ECUP_stat.APPS_Plausible = 1;
    }
	else {
    	ECUP_ACC.APPS_pos=0;
      ECUP_ACC.FT_APPS_pos = 1;
    	ECUP_stat.APPS_Plausible = 0;
  	}
	if(ECUP_ACC.APPS_pos <= min_acc_presed_threshold){
		ECUP_ACC.APPS_pos = 0;
	}
		
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, (GPIO_PinState)APPS_plausible);
  // starsi (overeny) kod pro BPPC (soft-bspd) kdyby byl potreba
	// EV2.5 Torque Encoder / Brake Pedal Plausibility Check
		/*if(s_brakeActive && (ECUP_ACC.APPS_pos > PERCENT(25))) // 25 % accelerator limit
		{
			if(!apps_dead_flag && !s_throttleBlocked)
			{
				apps_lasttime = HAL_GetTick(); // Deadtime is used for driving experience
				apps_dead_flag = true;
			}
			else
			{
				if(((int)HAL_GetTick() - (int)apps_lasttime) > APPS_DEADTIME)
				{
					apps_dead_flag = false;
					s_throttleBlocked = true;
				}
			}
		}
		else apps_dead_flag = false;

	if (s_throttleBlocked) {
		if (ECUP_ACC.APPS_pos < PERCENT(5)) // 5 % accelerator limit
			s_throttleBlocked = false;
		else
			ECUP_ACC.APPS_pos = 0;
	}*/
  
  
    // TODO: jak je zajistena podminka 5% ??
	if (ECUP_stat.BrakeActive && (ECUP_ACC.APPS_pos > PERCENT(25))){
		if (ATPS_countdown == 0){
			ATPS_countdown = HAL_GetTick()+ATPS_TIMEOUT;
		}
	}else{
      if (ECUP_ACC.APPS_pos < PERCENT(5)){
        	ATPS_countdown = 0;
					ECUP_stat.BPPC_Latch = 0;
      }
	}
	if(ATPS_countdown > 0){
		if(ATPS_countdown < HAL_GetTick()){
			ECUP_ACC.APPS_pos = 0;
			ECUP_stat.BPPC_Latch = 1;
		}
	}
	int bspd = HAL_GPIO_ReadPin(BSPD_In_GPIO_Port,BSPD_In_Pin);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, (GPIO_PinState)bspd);
	
  // this won't work until we have the sensors!!!!!!
	ECUP_ACC.Brake_pos = 0;

	/*
	if (break_press[0] < 500){//predpriprava vypoctu tlaku
		if (break_press[0] < 10){
			test_conection(AW_1_Pin, AW_1_GPIO_Port,&ECUP_BREAKE.FT_BrakeF,&ECUP_ACC.FT_Brake_Pos,&ECUP_stat);
		}
		break_press[0] = 0;
	}
	else {
		break_press[0] = break_press[0]-500; // mereni od 0,5 - 4,5 V
	}
	if (break_press[1] < 500) {
		break_press[1] = 0;
		if (break_press[1] < 10){
			test_conection(AW_1_Pin, AW_1_GPIO_Port,&ECUP_BREAKE.FT_BrakeF,&ECUP_ACC.FT_Brake_Pos,&ECUP_stat);
		}
	}
	else{
			break_press[1] = break_press[1]-500;
	}
	
	sens_Brake_Preasure1 = 1000*(adc_measurement1.press_f-min_breake_pedal[0])/(max_breake_pedal[0]-min_breake_pedal[0]); // senzor brzda procenta z 
	sens_Brake_Preasure2 = 1000*(adc_measurement1.press_r-min_breake_pedal[1])/(max_breake_pedal[0]-min_breake_pedal[0]); // senzor brzda procenta
	if(sens_Brake_Preasure1>min_breake_pedal[0]  && sens_Brake_Preasure2> min_breake_pedal[1] ){
			if (sens_Brake_Preasure1 > 1000){
			sens_Brake_Preasure1 = 1000;
		}
		if (sens_Brake_Preasure2 > 1000){
			sens_Brake_Preasure2 = 1000;
		}
	}
	else{
		if (sens_Brake_Preasure1 > 1000){
			sens_Brake_Preasure1 = 0;
		}
		if (sens_Brake_Preasure2 > 1000){
			sens_Brake_Preasure2 = 0;
		}
	}
	*/
	
	VOI = adc_measurement1.brk2*reference_val/4095; //
	//voltage_to_current();
	//ERROR_check_acc_and_percent_transfer();
	//ERROR_check_pressure();
}

static void update_voltage_measurements(int Vol_IN_raw, int Vol1_out_raw) {
  	Vol_IN = Vol_IN_raw*reference_val/4095; //
	Vol1_out = Vol1_out_raw*reference_val/4095; //overit
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)// ADC calback
{
	update_pedal_measurements(adc_measurement1.press_f, adc_measurement1.press_r, adc_measurement1.apps1, adc_measurement1.apps2, adc_measurement1.brk1, adc_measurement1.brk2);
	
  update_voltage_measurements(adc_measurement1.brk1, adc_measurement2[1]);
}
 
void Send_can_messagess(CAN_HandleTypeDef *hcan){
	if(ECUP_Status_need_to_send()){
		ECUP_stat.SDC_BOTS = !HAL_GPIO_ReadPin(mSDC_BOTS_GPIO_Port,mSDC_BOTS_Pin );
		ECUP_send_Status_s(&ECUP_stat);		
	}
	if(ECUP_Pedals_need_to_send()){
	  ECUP_send_Pedals_s(&ECUP_ACC);	
		++send;
		++ ECUP_ACC.SEQ;
	}
	if (ECUP_Pressure_need_to_send()){
		ECUP_send_Pressure_s(&ECUP_BREAKE);
	}
	if (ECUP_DiagPos_need_to_send()){
		ECUP_send_DiagPos_s(&ECUPdiag1);
	}
	if (ECUP_DiagPressure_need_to_send()){
		ECUP_send_DiagPressure_s(&ECUPdiag2);
	}
}

void Pedals_process(CAN_HandleTypeDef *hcan){

	ECUPdiag1.Acc1_raw = adc_measurement1.apps1; 
	ECUPdiag1.Acc2_raw = adc_measurement1.apps2;
	ECUPdiag2.BrakePressFRaw = adc_measurement1.press_f;
	ECUPdiag2.BrakePressRRaw = adc_measurement1.press_r;
	if (ECUP_get_REQCalibPedal(&ECUP_CALIB) & CAN_MSG_PENDING){ // dojdou hodnoty pro kalibraci
		BLIK_LED(LED_1_GPIO_Port,LED_1_Pin);
		calibrate.magic = 0x06c4951e;
		Calibrate_pedals(&ECUP_CALIB);
		write_flash(&calibrate);
  }
	Send_can_messagess(hcan);
}
void stop_newADC(ADC_HandleTypeDef* ADC_handle){
	HAL_ADC_Stop_DMA(ADC_handle); //ukončení DMA ukládání do paměti 
}

