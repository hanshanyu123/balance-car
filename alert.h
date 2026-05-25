#ifndef __ALERT_H
#define __ALERT_H

#include "main.h"

typedef enum {
    ALERT_EVENT_POINT = 0
} Alert_Event;

void Alert_Init(void);
void Alert_Task(void);
void Alert_Request(Alert_Event event);

#endif
