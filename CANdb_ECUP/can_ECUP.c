#include "can_ECUP.h"
#include <string.h>

int32_t ECUP_Status_last_sent;
int32_t ECUP_Pedals_last_sent;
int32_t ECUP_Pressure_last_sent;
CAN_msg_status_t ECUP_REQCalibPedal_status;
ECUP_REQCalibPedal_t ECUP_REQCalibPedal_data;
int32_t ECUP_DiagPos_last_sent;
int32_t ECUP_DiagPressure_last_sent;

void candbInit(void) {
    ECUP_Status_last_sent = -1;
    ECUP_Pedals_last_sent = -1;
    ECUP_Pressure_last_sent = -1;
    canInitMsgStatus(&ECUP_REQCalibPedal_status, -1);
    ECUP_DiagPos_last_sent = -1;
    ECUP_DiagPressure_last_sent = -1;
}

int ECUP_send_Status_s(const ECUP_Status_t* data) {
    uint8_t buffer[2];
    buffer[0] = (data->SDC_BOTS ? 1 : 0);
    buffer[1] = (data->FT_ANY ? 1 : 0) | (data->APPS_Plausible ? 2 : 0) | (data->BPPC_Latch ? 4 : 0) | (data->BrakeActive ? 8 : 0) | (data->BrakeActive_BSPD ? 16 : 0);
    ECUP_Status_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_Status_id, buffer, sizeof(buffer));
}

int ECUP_send_Status(uint8_t SDC_BOTS, uint8_t FT_ANY, uint8_t APPS_Plausible, uint8_t BPPC_Latch, uint8_t BrakeActive, uint8_t BrakeActive_BSPD) {
    uint8_t buffer[2];
    buffer[0] = (SDC_BOTS ? 1 : 0);
    buffer[1] = (FT_ANY ? 1 : 0) | (APPS_Plausible ? 2 : 0) | (BPPC_Latch ? 4 : 0) | (BrakeActive ? 8 : 0) | (BrakeActive_BSPD ? 16 : 0);
    ECUP_Status_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_Status_id, buffer, sizeof(buffer));
}

int ECUP_Status_need_to_send(void) {
    return (ECUP_Status_last_sent == -1) || (txGetTimeMillis() >= ECUP_Status_last_sent + 100);
}

int ECUP_send_Pedals_s(const ECUP_Pedals_t* data) {
    uint8_t buffer[6];
    buffer[0] = data->APPS_pos;
    buffer[1] = (data->APPS_pos >> 8);
    buffer[2] = data->Brake_pos;
    buffer[3] = (data->Brake_pos >> 8);
    buffer[4] = (data->FT_APPS_pos ? 1 : 0) | (data->FT_Brake_Pos ? 2 : 0);
    buffer[5] = ((data->SEQ & 0x0F) << 4);
    ECUP_Pedals_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_Pedals_id, buffer, sizeof(buffer));
}

int ECUP_send_Pedals(uint16_t APPS_pos, uint16_t Brake_pos, uint8_t FT_APPS_pos, uint8_t FT_Brake_Pos, uint8_t SEQ) {
    uint8_t buffer[6];
    buffer[0] = APPS_pos;
    buffer[1] = (APPS_pos >> 8);
    buffer[2] = Brake_pos;
    buffer[3] = (Brake_pos >> 8);
    buffer[4] = (FT_APPS_pos ? 1 : 0) | (FT_Brake_Pos ? 2 : 0);
    buffer[5] = ((SEQ & 0x0F) << 4);
    ECUP_Pedals_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_Pedals_id, buffer, sizeof(buffer));
}

int ECUP_Pedals_need_to_send(void) {
    return (ECUP_Pedals_last_sent == -1) || (txGetTimeMillis() >= ECUP_Pedals_last_sent + 10);
}

int ECUP_send_Pressure_s(const ECUP_Pressure_t* data) {
    uint8_t buffer[5];
    buffer[0] = data->BrakeF;
    buffer[1] = (data->BrakeF >> 8);
    buffer[2] = data->BrakeR;
    buffer[3] = (data->BrakeR >> 8);
    buffer[4] = (data->FT_BrakeF ? 1 : 0) | (data->FT_BrakeR ? 2 : 0);
    ECUP_Pressure_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_Pressure_id, buffer, sizeof(buffer));
}

int ECUP_send_Pressure(uint16_t BrakeF, uint16_t BrakeR, uint8_t FT_BrakeF, uint8_t FT_BrakeR) {
    uint8_t buffer[5];
    buffer[0] = BrakeF;
    buffer[1] = (BrakeF >> 8);
    buffer[2] = BrakeR;
    buffer[3] = (BrakeR >> 8);
    buffer[4] = (FT_BrakeF ? 1 : 0) | (FT_BrakeR ? 2 : 0);
    ECUP_Pressure_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_Pressure_id, buffer, sizeof(buffer));
}

int ECUP_Pressure_need_to_send(void) {
    return (ECUP_Pressure_last_sent == -1) || (txGetTimeMillis() >= ECUP_Pressure_last_sent + 10);
}

int ECUP_decode_REQCalibPedal_s(const uint8_t* bytes, size_t length, ECUP_REQCalibPedal_t* data_out) {
    if (length < 1)
        return 0;

    data_out->which = (enum ECUP_CAL_PedalIndex) ((bytes[0] & 0x0F));
    return 1;
}

int ECUP_decode_REQCalibPedal(const uint8_t* bytes, size_t length, enum ECUP_CAL_PedalIndex* which_out) {
    if (length < 1)
        return 0;

    *which_out = (enum ECUP_CAL_PedalIndex) ((bytes[0] & 0x0F));
    return 1;
}

int ECUP_get_REQCalibPedal(ECUP_REQCalibPedal_t* data_out) {
    if (!(ECUP_REQCalibPedal_status.flags & CAN_MSG_RECEIVED))
        return 0;

    if (data_out)
        memcpy(data_out, &ECUP_REQCalibPedal_data, sizeof(ECUP_REQCalibPedal_t));

    int flags = ECUP_REQCalibPedal_status.flags;
    ECUP_REQCalibPedal_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUP_REQCalibPedal_on_receive(int (*callback)(ECUP_REQCalibPedal_t* data)) {
    ECUP_REQCalibPedal_status.on_receive = (void (*)(void)) callback;
}

int ECUP_send_DiagPos_s(const ECUP_DiagPos_t* data) {
    uint8_t buffer[8];
    buffer[0] = data->Acc1_raw;
    buffer[1] = (data->Acc1_raw >> 8);
    buffer[2] = data->Acc2_raw;
    buffer[3] = (data->Acc2_raw >> 8);
    buffer[4] = data->BrkPos_raw;
    buffer[5] = (data->BrkPos_raw >> 8);
    buffer[6] = data->BPPC_raw;
    buffer[7] = (data->BPPC_raw >> 8);
    ECUP_DiagPos_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_DiagPos_id, buffer, sizeof(buffer));
}

int ECUP_send_DiagPos(uint16_t Acc1_raw, uint16_t Acc2_raw, uint16_t BrkPos_raw, uint16_t BPPC_raw) {
    uint8_t buffer[8];
    buffer[0] = Acc1_raw;
    buffer[1] = (Acc1_raw >> 8);
    buffer[2] = Acc2_raw;
    buffer[3] = (Acc2_raw >> 8);
    buffer[4] = BrkPos_raw;
    buffer[5] = (BrkPos_raw >> 8);
    buffer[6] = BPPC_raw;
    buffer[7] = (BPPC_raw >> 8);
    ECUP_DiagPos_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_DiagPos_id, buffer, sizeof(buffer));
}

int ECUP_DiagPos_need_to_send(void) {
    return (ECUP_DiagPos_last_sent == -1) || (txGetTimeMillis() >= ECUP_DiagPos_last_sent + 50);
}

int ECUP_send_DiagPressure_s(const ECUP_DiagPressure_t* data) {
    uint8_t buffer[4];
    buffer[0] = data->BrakePressFRaw;
    buffer[1] = (data->BrakePressFRaw >> 8);
    buffer[2] = data->BrakePressRRaw;
    buffer[3] = (data->BrakePressRRaw >> 8);
    ECUP_DiagPressure_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_DiagPressure_id, buffer, sizeof(buffer));
}

int ECUP_send_DiagPressure(uint16_t BrakePressFRaw, uint16_t BrakePressRRaw) {
    uint8_t buffer[4];
    buffer[0] = BrakePressFRaw;
    buffer[1] = (BrakePressFRaw >> 8);
    buffer[2] = BrakePressRRaw;
    buffer[3] = (BrakePressRRaw >> 8);
    ECUP_DiagPressure_last_sent = txGetTimeMillis();
    return txSendCANMessage(ECUP_DiagPressure_id, buffer, sizeof(buffer));
}

int ECUP_DiagPressure_need_to_send(void) {
    return (ECUP_DiagPressure_last_sent == -1) || (txGetTimeMillis() >= ECUP_DiagPressure_last_sent + 50);
}

void candbHandleMessage(uint32_t timestamp, CAN_ID_t id, const uint8_t* payload, size_t payload_length) {
    switch (id) {
    case ECUP_REQCalibPedal_id: {
        if (!ECUP_decode_REQCalibPedal_s(payload, payload_length, &ECUP_REQCalibPedal_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUP_REQCalibPedal_status, timestamp);

        if (ECUP_REQCalibPedal_status.on_receive)
            ((int (*)(ECUP_REQCalibPedal_t*)) ECUP_REQCalibPedal_status.on_receive)(&ECUP_REQCalibPedal_data);

        break;
    }
    }
}
