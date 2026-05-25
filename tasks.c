#include "tasks.h"
#include "sensor.h"
#include "task2.h"
#include "task3.h"
#include "task4.h"
#include "alert.h"

static Task2_State last_task2_state = TASK2_STATE_IDLE;

static void Tasks_AlertOnPointEvent(void)
{
    Task2_State task2_state = Task2_GetState();

    if (task2_state != last_task2_state) {
        if (task2_state == TASK2_STATE_FINISHED) Alert_Request(ALERT_EVENT_POINT);
        last_task2_state = task2_state;
    }

    if (Task3_GetPointEvent() != TASK3_POINT_NONE) {
        Alert_Request(ALERT_EVENT_POINT);
        Task3_ClearPointEvent();
    }

    if (Task4_GetPointEvent() != TASK4_POINT_NONE) {
        Alert_Request(ALERT_EVENT_POINT);
        Task4_ClearPointEvent();
    }
}

void Tasks_Init(void)
{
    Sensor_Init();
    Task2_Init();
    Task3_Init();
    Task4_Init();
    last_task2_state = Task2_GetState();
}

void Tasks_Task(void)
{
    Sensor_JC();
    Task2_UpdateSensor(Sensor_GetBits(), Sensor_GetError(), Sensor_IsValid());
    Task3_UpdateSensor(Sensor_GetBits(), Sensor_GetError(), Sensor_IsValid());
    Task4_UpdateSensor(Sensor_GetBits(), Sensor_GetError(), Sensor_IsValid());
    Task2_Task();
    Task3_Task();
    Task4_Task();
    Tasks_AlertOnPointEvent();
}
