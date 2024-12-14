// Configuration Parameters
#define MAX_USERS 31

// States for State Machine
typedef enum SM_States {
	WEBSERVER = 0,
} State_t;

enum FreeRTOS_State_Priority {
	PRIORITY_BASE = 0,
	PRIORITY_LOW,
	PRIORITY_MED,
	PRIORITY_HIGH
};

typedef uint8_t Hand;

