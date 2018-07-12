/*
 * eForce Libs - Tx
 * 2016 Martin Cejp
 */

#ifndef EFORCE_LIBS_CAN
#define EFORCE_LIBS_CAN

#include "tx.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { CAN_MESSAGE_SIZE = 8 };

struct CAN_msg_header {
	uint32_t timestamp;
	CAN_ID_t id;
	uint16_t length;
};

typedef struct {
	uint32_t flags;
	int32_t timeout;
	int32_t timestamp;

	void (*on_receive)(void);
} CAN_msg_status_t;

enum {
	CAN_MSG_PENDING = 1,
	CAN_MSG_RECEIVED = 2,
	CAN_MSG_MISSED = 4,
};

void canInitMsgStatus(CAN_msg_status_t*, int timeout);
void canUpdateMsgStatusOnReceive(CAN_msg_status_t*, uint32_t timestamp);

#ifdef __cplusplus
}
#endif

#endif
