#ifndef CAN_ECUP_H
#define CAN_ECUP_H

#include <eforce/can.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { ECUP_Status_id = 0x040 };
enum { ECUP_Status_timeout = 500 };
enum { ECUP_Pedals_id = 0x141 };
enum { ECUP_Pedals_timeout = 50 };
enum { ECUP_Pressure_id = 0x142 };
enum { ECUP_REQCalibPedal_id = 0x44D };
enum { ECUP_DiagPos_id = 0x54E };
enum { ECUP_DiagPos_timeout = 250 };
enum { ECUP_DiagPressure_id = 0x54F };
enum { ECUP_DiagPressure_timeout = 250 };

enum ECUP_CAL_PedalIndex {
    /* None */
    ECUP_CAL_PedalIndex_None = 0,
    /* Minimum apps position */
    ECUP_CAL_PedalIndex_AppsMin = 1,
    /* Maximum apps position */
    ECUP_CAL_PedalIndex_AppsMax = 2,
    /* Minimum brake position */
    ECUP_CAL_PedalIndex_BrakeMin = 3,
    /* Maximum brake position */
    ECUP_CAL_PedalIndex_BrakeMax = 4,
    /* Maximum regenerative position */
    ECUP_CAL_PedalIndex_RegenMax = 5,
};

enum ECUP_CalibrationIndex {
    /* FIX THIS SHIT */
    ECUP_CalibrationIndex_dummy = 1,
};

/*
 * ECUP Status message
 */
typedef struct ECUP_Status_t {
	/* SDC is powered after BOTS */
	uint8_t	SDC_BOTS;

	/* If any error is present */
	uint8_t	FT_ANY;

	/* Accelerator pedals plausible */
	uint8_t	APPS_Plausible;

	/* Brake Pedal Plausibility Check (software bspd) */
	uint8_t	BPPC_Latch;

	/* Brake active (Signal mainly for brakelight) */
	uint8_t	BrakeActive;

	/* Brake active for the purposes of BSPD */
	uint8_t	BrakeActive_BSPD;
} ECUP_Status_t;

/*
 * Pedal values and valid flags
 */
typedef struct ECUP_Pedals_t {
	/* Normalized accelerator value */
	uint16_t	APPS_pos;

	/* Normalized brake encoder value */
	uint16_t	Brake_pos;

	/* There is some error with sensor or calibration */
	uint8_t	FT_APPS_pos;

	/* There is some error with sensor or calibration */
	uint8_t	FT_Brake_Pos;

	/* Message counter for safety */
	uint8_t	SEQ;
} ECUP_Pedals_t;

/*
 * Brake pressures sensor
 */
typedef struct ECUP_Pressure_t {
	/* Brake front pressure */
	uint16_t	BrakeF;

	/* Brake rear pressure */
	uint16_t	BrakeR;

	/* Fault of brake pressure sensor front */
	uint8_t	FT_BrakeF;

	/* Fault of brake pressure sensor rear */
	uint8_t	FT_BrakeR;
} ECUP_Pressure_t;

/*
 * Request calibration.
 * Assign pedal max/min to its' current position.
 */
typedef struct ECUP_REQCalibPedal_t {
	/* Calibration index to set */
	enum ECUP_CAL_PedalIndex	which;
} ECUP_REQCalibPedal_t;

/*
 * Raw diagnostic positional data of pedals
 */
typedef struct ECUP_DiagPos_t {
	/* Raw read value of accelerator position sensor 1 */
	uint16_t	Acc1_raw;

	/* Raw read value of accelerator position sensor 2 */
	uint16_t	Acc2_raw;

	/* Raw read value of brake position sensor */
	uint16_t	BrkPos_raw;

	/* Raw calculated value of brake pedal plausability check (BPPC) */
	uint16_t	BPPC_raw;
} ECUP_DiagPos_t;

/*
 * Raw sensor values
 */
typedef struct ECUP_DiagPressure_t {
	/* Front brake sensor raw data from ADC */
	uint16_t	BrakePressFRaw;

	/* Rear brake sensor raw data from ADC */
	uint16_t	BrakePressRRaw;
} ECUP_DiagPressure_t;

void candbInit(void);

int ECUP_send_Status_s(const ECUP_Status_t* data);
int ECUP_send_Status(uint8_t SDC_BOTS, uint8_t FT_ANY, uint8_t APPS_Plausible, uint8_t BPPC_Latch, uint8_t BrakeActive, uint8_t BrakeActive_BSPD);
int ECUP_Status_need_to_send(void);

int ECUP_send_Pedals_s(const ECUP_Pedals_t* data);
int ECUP_send_Pedals(uint16_t APPS_pos, uint16_t Brake_pos, uint8_t FT_APPS_pos, uint8_t FT_Brake_Pos, uint8_t SEQ);
int ECUP_Pedals_need_to_send(void);

int ECUP_send_Pressure_s(const ECUP_Pressure_t* data);
int ECUP_send_Pressure(uint16_t BrakeF, uint16_t BrakeR, uint8_t FT_BrakeF, uint8_t FT_BrakeR);
int ECUP_Pressure_need_to_send(void);

int ECUP_decode_REQCalibPedal_s(const uint8_t* bytes, size_t length, ECUP_REQCalibPedal_t* data_out);
int ECUP_decode_REQCalibPedal(const uint8_t* bytes, size_t length, enum ECUP_CAL_PedalIndex* which_out);
int ECUP_get_REQCalibPedal(ECUP_REQCalibPedal_t* data_out);
void ECUP_REQCalibPedal_on_receive(int (*callback)(ECUP_REQCalibPedal_t* data));

int ECUP_send_DiagPos_s(const ECUP_DiagPos_t* data);
int ECUP_send_DiagPos(uint16_t Acc1_raw, uint16_t Acc2_raw, uint16_t BrkPos_raw, uint16_t BPPC_raw);
int ECUP_DiagPos_need_to_send(void);

int ECUP_send_DiagPressure_s(const ECUP_DiagPressure_t* data);
int ECUP_send_DiagPressure(uint16_t BrakePressFRaw, uint16_t BrakePressRRaw);
int ECUP_DiagPressure_need_to_send(void);

#ifdef __cplusplus
}
#endif

#endif
