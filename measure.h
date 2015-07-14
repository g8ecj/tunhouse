



// what sensors we have
#define NUMSENSORS  3
#define SENSOR_LOW  0
#define SENSOR_HIGH 1
#define SENSOR_OUT  2

// order here is important as its used in the UI 
#define NUMINDEX    3
#define TINDEX_MIN  0
#define TINDEX_NOW  1
#define TINDEX_MAX  2

// indexes into the open/close window limits array
#define NUMLIMIT    2
#define LIMIT_UP    0
#define LIMIT_DN    1

extern int16_t gValues[NUMSENSORS][NUMINDEX]; // current, max and min temperatures for each sensor
extern int16_t gLimits[NUMSENSORS][NUMLIMIT]; // upper and lower limits for driving window motors
extern int16_t gBattery;
extern uint8_t idmap[6];



void measure_init (void);
int8_t getlims (uint8_t sensor, int16_t * now, int16_t * up, int16_t * down);
void run_measure (void);


