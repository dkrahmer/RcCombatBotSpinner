#define L_FWD_PIN  2	// H-bridge left motor forward
#define L_REV_PIN  4	// H-bridge left motor reverse

#define R_FWD_PIN  7	// H-bridge right motor forward
#define R_REV_PIN  8	// H-bridge right motor reverse

#define PPM_IN_PIN A0
#define NUM_CHANNELS 12

#define WHACKER_SERVO_OUT_PIN 5
#define SPINNER_SERVO_OUT_PIN 6 

#define CHANNEL_SPINNER 1	// throttle stick
#define CHANNEL_WHACKER 2	// Rudder (L stick side-to-side)

#define CHANNEL_THROTTLE 3
#define CHANNEL_STEERING 4

#define CHANNEL_SPEED_PERCENT 5
#define CHANNEL_INVERT 6

#define CHANNEL_WEDGE_MODE 7

#define MAX_SERVO_VALUE 180
#define SPINNER_RAMP_UP_MS 2000

const uint8_t INDICATOR_LED_PIN = 13;

struct WheelPower
{
	int Left;
	int Right;
};