/*
 * eForce Libs - Tx
 * 2016 Martin Cejp
 */

#ifndef EFORCE_LIBS_TX
#define EFORCE_LIBS_TX

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STD_ID(sid_) (sid_)
#define EXT_ID(eid_) (0x80000000L | (eid_))

#define IS_EXT_ID(id_) ((id_) & 0x80000000L)
#define IS_STD_ID(id_) (!IS_EXT_ID(id_))

typedef uint16_t CAN_ID_t;

enum {
	TX_RECV_BUFFER_OVERFLOW = (1<<0),
	TX_SEND_BUFFER_OVERFLOW = (1<<1),
	TX_UNHANDLED_MESSAGE = (1<<2),
	TX_IO_ERROR = (1<<3),
	TX_NOT_IMPLEMENTED = (1<<4),
	TX_BAD_IRQ = (1<<5),
};

void txInit(void);
int txReceiveCANMessage(CAN_ID_t sid, const void* data, size_t length);
void txProcess(void);

/* define by library user */
uint32_t txGetTimeMillis(void);
int txHandleCANMessage(uint32_t timestamp, CAN_ID_t id, const void* data, size_t length);
int txSendCANMessage(CAN_ID_t id, const void* data, size_t length);
/* can't touch this */
void txEnterDownloaderSTM32F1(void);

#ifdef STM32F1
#include <stm32f1xx.h>

void canInitPeripheral(CAN_TypeDef* instance, int loopback, int prescaler, int bs1, int bs2, int irqn);
int canSendMessage(CAN_TypeDef* instance, CAN_ID_t id, const void* data, size_t length);
int canReceiveMessage(CAN_TypeDef* instance);
int canTryReceive(CAN_TypeDef* instance, CAN_ID_t* id_out, void* data_out, size_t* length_out);
int canInitFiltersSTD(CAN_TypeDef* instance, const uint16_t* sids, size_t count);
#endif

#ifdef STM32F2
#include <stm32f2xx.h>

void canInitPeripheral(CAN_TypeDef* instance, int loopback, int prescaler, int bs1, int bs2, int irqn);
int canSendMessage(CAN_TypeDef* instance, CAN_ID_t id, const void* data, size_t length);
int canReceiveMessage(CAN_TypeDef* instance);
int canTryReceive(CAN_TypeDef* instance, CAN_ID_t* id_out, void* data_out, size_t* length_out);
int canInitFiltersSTD(CAN_TypeDef* instance, const uint16_t* sids, size_t count);
#endif

#ifdef STM32F3
#include <stm32f3xx.h>

void canInitPeripheral(CAN_TypeDef* instance, int loopback, int prescaler, int bs1, int bs2, int irqn);
int canSendMessage(CAN_TypeDef* instance, CAN_ID_t id, const void* data, size_t length);
int canReceiveMessage(CAN_TypeDef* instance);
int canTryReceive(CAN_TypeDef* instance, CAN_ID_t* id_out, void* data_out, size_t* length_out);
int canInitFiltersSTD(CAN_TypeDef* instance, const uint16_t* sids, size_t count);
#endif

#ifdef __cplusplus
}
#endif

#endif
