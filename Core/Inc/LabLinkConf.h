// Configuration Parameters
#define MAX_USERS 31
#define SSID ""
#define PASSWORD ""
#define PORT 80

// States for State Machine
typedef enum SM_States {
	INIT = 0,
	CONNECTED,
	QUEUE_EMPTY,
	QUEUE_VALUES
} State_t;

enum FreeRTOS_State_Priority {
	PRIORITY_BASE = 0,
	PRIORITY_LOW,
	PRIORITY_MED,
	PRIORITY_HIGH
};
