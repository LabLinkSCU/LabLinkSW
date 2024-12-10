// Configuration Parameters
#define MAX_USERS 31

// States for State Machine
typedef enum SM_States {
		INIT_SEARCH = 0,
		HOST_SETUP,
		HOST_NORMAL_OP,
		CLIENT_PAIR,
		CLIENT_STANDBY,
		CLIENT_NORMAL_OP,
} State_t;

enum FreeRTOS_State_Priority {
	PRIORITY_BASE = 0,
	PRIORITY_LOW,
	PRIORITY_MED,
	PRIORITY_HIGH
};

typedef uint8_t Hand;

