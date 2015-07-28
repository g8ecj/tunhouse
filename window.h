

//states
#define MANOPENING   0
#define MANCLOSING   1
#define MANOPEN      2
#define MANCLOSED    3
#define WINCLOSED    4
#define WINOPENING   5
#define WINOPEN      6
#define WINCLOSING   7


// events
#define TEMPGREATER     0       // temperature greater than upper limit
#define TEMPLESSER      1       // temp less than lower limit
#define MANUALOPEN      2       // request for manual open
#define MANUALCLOSE     3       // request for manual close
#define MANUALCANCEL    4       // cancel a current window movement or timer
#define TIMEOUT         5       // timeout event

// default timer to 30seconds which suits the top windows
#define RUNVALUE 30
// manual operation locks out the sensors for this long
#define LOCKOUTVALUE 1800
// how long before we cancel (determines how long the msg stays on the display)
#define CANCELVALUE 3



extern int16_t gWinState[];

void window_init (void);
void run_windows (void);
void wintimer (void);

void windowopen (int8_t sensor);
void windowclose (int8_t sensor);
void windowcan (int8_t sensor);
uint8_t windowopening (uint8_t sensor);
uint8_t windowclosing (uint8_t sensor);
uint8_t windowcanceling (uint8_t sensor);

