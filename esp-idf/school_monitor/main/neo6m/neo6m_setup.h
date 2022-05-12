#ifndef NEO6M_SETUP_H
#define NEO6M_SETUP_H

#include "global_data.h"

#define TX_PIN          32
#define RX_PIN          33
#define RD_BUF_SIZE     1024
#define PATTERN_CHR_NUM (1)

void neo6m_task ( void *pvParameters );

#endif /* NEO6M_SETUP_H */
