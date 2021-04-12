#include "decoder/IbusDecoder.h"
#include <stdint.h>


typedef enum {
    IBD_STATE_UNKNOWN = 0,
    IBD_STATE_IDLE,
    IBD_STATE_INIT,
    IBD_STATE_SYNC,
    IBD_STATE_COLLECT,
    IBD_STATE_PARSE,
    IBD_STATE_ERROR
} Ibd_State_t;

void Ibd_Init(void)
{

}

void Ibd_Update(void)
{

}
