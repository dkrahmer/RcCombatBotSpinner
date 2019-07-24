//#define DEBUG 1
//#define IS_PPM  // comment this define for serial input from satellite receiver

#include "configuration.h"
#include "RcSatelliteReceiver\RcSatelliteReceiver.h"
#include <SoftPWM.h>
#include <Servo.h>

#ifdef IS_PPM
#define MIN_CHANNEL_VAL 840
#define MAX_CHANNEL_VAL 1650
#else
#define MIN_CHANNEL_VAL 342
#define MAX_CHANNEL_VAL 1706
#endif // IS_PPM

RcSatelliteReceiver Receiver;

int _channels[NUM_CHANNELS];
int _ppmSkipCount = 5;
bool _isInitialized = false;
Servo _whackerServo;
Servo _spinnerServo;

double _spinnerRampUpPerMicro = (double)MAX_SERVO_VALUE / (double)(SPINNER_RAMP_UP_MS * 1000);
double _spinnerRampedPosition = 0;
unsigned long _spinnerLastMicros = micros();

void setup()
{
	Serial.begin(115200);

	InitGPIO();

	// RunTestSequence();
}

void loop() {
#ifdef IS_PPM
	GetPpmChannels();
#else
	GetSatelliteChannels();
#endif // IS_PPM

	/*
	#ifdef DEBUG
		for (int channelNum = 1; channelNum <= NUM_CHANNELS; channelNum++)
		{
			Serial.print("  ");
			Serial.print(channelNum);
			Serial.print(": ");
			Serial.print(getChannel(channelNum));
		}
		Serial.println();
	#endif
	*/

	int throttlePosition = GetNormalizedStickPosition(getChannel(CHANNEL_THROTTLE), -127, 127, 6);
	int steeringPosition = GetNormalizedStickPosition(getChannel(CHANNEL_STEERING), -127, 127, 6);
	int whackerPosition = GetNormalizedStickPosition(getChannel(CHANNEL_WHACKER), 0, MAX_SERVO_VALUE, 6);
	int spinnerPosition = GetNormalizedStickPosition(getChannel(CHANNEL_SPINNER), 0, MAX_SERVO_VALUE, 0);
	double speedRatio = (double)GetNormalizedStickPosition(getChannel(CHANNEL_SPEED_PERCENT), 0, 100, 0) / 100.0;
	bool isWedgeMode = GetNormalizedStickPosition(getChannel(CHANNEL_WEDGE_MODE), 0, 100, 0) > 60 ? true : false;
	bool isInverted = GetNormalizedStickPosition(getChannel(CHANNEL_INVERT), 0, 100, 0) > 60 ? true : false;

	// wait for first good signal from the receiver
	if (!_isInitialized)
	{
		if (throttlePosition != 0 || steeringPosition != 0 || spinnerPosition != 0)
		{
#ifdef DEBUG
			Serial.println("Not initialized! Throttle, steering and spinner must be 0 to initialize.");
#endif
			return;
		}
#ifdef DEBUG
		Serial.println("Initialize. Throttle and steering are both 0.");
#endif
		_isInitialized = true;
	}

	if (isWedgeMode)
	{
		steeringPosition = -steeringPosition;
		isInverted = !isInverted;
	}

	WheelPower wheelPower = GetWheelPower(steeringPosition, throttlePosition);
	if (speedRatio < 1)
	{
		wheelPower.Left = (double)wheelPower.Left * speedRatio;
		wheelPower.Right = (double)wheelPower.Right * speedRatio;
	}

	spinnerPosition = getRampedPosition(spinnerPosition, _spinnerRampUpPerMicro, &_spinnerRampedPosition, &_spinnerLastMicros);

	if (isInverted)
	{
		// invert wheel directions
		int temp = wheelPower.Left;
		wheelPower.Left = -wheelPower.Right;
		wheelPower.Right = -temp;

		// invert swiping direction
		whackerPosition = MAX_SERVO_VALUE - whackerPosition;
	}

	// Make it so...
	if (wheelPower.Left == 0 && wheelPower.Right == 0)
	{
		// Brake
		SoftPWMSet(L_FWD_PIN, 255);
		SoftPWMSet(L_REV_PIN, 255);
		SoftPWMSet(R_FWD_PIN, 255);
		SoftPWMSet(R_REV_PIN, 255);
	}
	else
	{
		SoftPWMSet(L_FWD_PIN, wheelPower.Left > 0 ? Bind(abs(wheelPower.Left) * 2 + 1, 0, 255) : LOW);
		SoftPWMSet(L_REV_PIN, wheelPower.Left < 0 ? Bind(abs(wheelPower.Left) * 2 + 1, 0, 255) : LOW);
		SoftPWMSet(R_FWD_PIN, wheelPower.Right > 0 ? Bind(abs(wheelPower.Right) * 2 + 1, 0, 255) : LOW);
		SoftPWMSet(R_REV_PIN, wheelPower.Right < 0 ? Bind(abs(wheelPower.Right) * 2 + 1, 0, 255) : LOW);
	}

	_whackerServo.write(whackerPosition);
	_spinnerServo.write(spinnerPosition);

#ifdef DEBUG
	Serial.print("wheelPower.Left: ");
	Serial.print(wheelPower.Left);
	Serial.print("    wheelPower.Right: ");
	Serial.print(wheelPower.Right);
	Serial.println();
#endif

	digitalWrite(INDICATOR_LED_PIN, HIGH);
}

int getRampedPosition(int targetPosition, double rampUpPerMicro, double* rampedPosition, unsigned long* lastMicros)
{
	unsigned long prevLastMicros = *lastMicros;
	*lastMicros = micros();
	if (targetPosition <= *rampedPosition)
	{ // Allow immediate reduction (no ramp down)
		*rampedPosition = targetPosition;
	}
	else
	{
		unsigned long elapsedMicros = *lastMicros - prevLastMicros;
		*rampedPosition += (double)elapsedMicros * rampUpPerMicro;
		if (targetPosition > *rampedPosition)
		{
			*rampedPosition = targetPosition;
		}
	}

	return *rampedPosition;
}

#ifdef IS_PPM
void GetPpmChannels()
{
	unsigned int syncPulse;
	unsigned int syncPulseFailures = 0;

	// wait until the synchronize pulse arrives
	do
	{
		syncPulse = pulseIn(PPM_IN_PIN, HIGH);

		if (syncPulse > 3000 && syncPulse < 50000)	// anything ins this range is a PPM sync pulse
			break;

		syncPulseFailures++;

		if (syncPulseFailures >= 30)
		{
			digitalWrite(INDICATOR_LED_PIN, LOW);
#ifdef DEBUG
			Serial.println("No sync pulse detected (no signal)");
#endif		
			syncPulseFailures = 0;
		}
	} while (true);

	if (_ppmSkipCount > 0)
	{
		// skip the first few PPM frames to clear the pipe of any junk data
#ifdef DEBUG
		Serial.println("Skipping PPM frame...");
#endif
		_ppmSkipCount--;
		return;
	}

	// Read the pulses for each channel
	for (int i = 1; i <= NUM_CHANNELS; i++)
	{
		int channel = mapPpmChannel(i);

		setChannel(channel, pulseIn(PPM_IN_PIN, HIGH));
	}
}

int mapPpmChannel(int ppmChannel)
{
	// PPM sends the first 3 channels out of order so we need to map them
	if (ppmChannel == 1)
		return 2;
	if (ppmChannel == 2)
		return 3;
	if (ppmChannel == 3)
		return 1;

	return ppmChannel;
}

#else
void GetSatelliteChannels()
{
	Receiver.readChannelValues();

	for (int channel = 1; channel <= NUM_CHANNELS; channel++)
	{
		setChannel(channel, Receiver.getChannel(channel));
	}
}
#endif // IS_PPM

int GetNormalizedStickPosition(int absoluteChannelValue, int minValue, int maxValue, int centerJitterPercent)
{
	if (absoluteChannelValue <= MIN_CHANNEL_VAL)
		return minValue; // make sure we don't go past the min value

	if (absoluteChannelValue >= MAX_CHANNEL_VAL)
		return maxValue;	// make sure we don't go past the max value

	int zeroBased = absoluteChannelValue - MIN_CHANNEL_VAL;
	double percent = ((double)zeroBased) / ((double)(MAX_CHANNEL_VAL - MIN_CHANNEL_VAL));

	// prevent jitter at rest
	if (abs(percent - .50) <= ((double)centerJitterPercent / 100.0))
		percent = .50;

	int delta = maxValue - minValue;
	return minValue + (delta * percent);
}

int getChannel(int channel)
{
	return _channels[channel - 1];
}

void setChannel(int channel, int value)
{
	_channels[channel - 1] = value;
}

// Differential Steering Joystick Algorithm
// ========================================
//   by Calvin Hass
//   https://www.impulseadventure.com/elec/
//
// Converts a single dual-axis joystick into a differential
// drive motor control, with support for both drive, turn
// and pivot operations.
// Original code from: https://www.impulseadventure.com/elec/robot-differential-steering.html
// INPUTS
//     nJoyX;              // Joystick X input                     (-128..+127)
//     nJoyY;              // Joystick Y input                     (-128..+127)
WheelPower GetWheelPower(int nJoyX, int nJoyY)
{
	// OUTPUTS
	int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
	int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

	// CONFIG
	// - fPivYLimt  : The threshold at which the pivot action starts
	//                This threshold is measured in units on the Y-axis
	//                away from the X-axis (Y=0). A greater value will assign
	//                more of the joystick's range to pivot actions.
	//                Allowable range: (0..+127)
	float fPivYLimit = nJoyY > 0 ? 80.0 : 50.0;

	// TEMP VARIABLES
	float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
	float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
	int     nPivSpeed;      // Pivot Speed                          (-128..+127)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

	// Calculate Drive Turn output due to Joystick X input
	if (nJoyY >= 0) {
		// Forward
		nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
		nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
	}
	else {
		// Reverse
		nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
		nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
	}

	// Scale Drive output due to Joystick Y input (throttle)
	nMotPremixL = nMotPremixL * nJoyY / 128.0;
	nMotPremixR = nMotPremixR * nJoyY / 128.0;

	// Now calculate pivot amount
	// - Strength of pivot (nPivSpeed) based on Joystick X input
	// - Blending of pivot vs drive (fPivScale) based on Joystick Y input
	nPivSpeed = nJoyX;

	fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

	// Calculate final mix of Drive and Pivot
	nMotMixL = (1.0 - fPivScale)*nMotPremixL + fPivScale * (nPivSpeed);
	nMotMixR = (1.0 - fPivScale)*nMotPremixR + fPivScale * (-nPivSpeed);

	// Make reversing into a turn feel more natural
	if (nJoyY < -fPivYLimit)
	{
		int temp = nMotMixR;
		nMotMixR = nMotMixL;
		nMotMixL = temp;
	}

	WheelPower wheelPower;
	wheelPower.Left = nMotMixL;
	wheelPower.Right = nMotMixR;

	return wheelPower;
}

int Bind(int value, int min, int max)
{
	if (value < min)
		return min;

	if (value > max)
		return max;

	return value;
}


int GetHighestValue(int value1, int value2)
{
	return (value1 > value2) ? value1 : value2;
}

// motor control
void SetMotorspeed(int speedLeft, int speedRight)
{
	//analogWrite(L_SPEED_PIN, speedLeft);
	//analogWrite(R_SPEED_PIN, speedRight);
}

void GoStop(int milliseconds = 0)
{
	digitalWrite(L_FWD_PIN, LOW);
	digitalWrite(L_REV_PIN, LOW);
	digitalWrite(R_FWD_PIN, LOW);
	digitalWrite(R_REV_PIN, LOW);
	SetMotorspeed(0, 0);
	delay(milliseconds);
}

// initialize Pins
void InitGPIO()
{
	SoftPWMBegin();
	SoftPWMSet(L_FWD_PIN, 0);
	SoftPWMSet(L_REV_PIN, 0);
	SoftPWMSet(R_FWD_PIN, 0);
	SoftPWMSet(R_REV_PIN, 0);

#ifdef IS_PPM
	pinMode(PPM_IN_PIN, INPUT);
#endif

	_whackerServo.attach(WHACKER_SERVO_OUT_PIN);
	_spinnerServo.attach(SPINNER_SERVO_OUT_PIN);

	GoStop();
}

void RunTestSequence()
{
	// Test sequence
	digitalWrite(L_FWD_PIN, 255);
	delay(1000);
	GoStop(500);

	digitalWrite(L_REV_PIN, 255);
	delay(1000);
	GoStop(500);

	digitalWrite(R_FWD_PIN, 255);
	delay(1000);
	GoStop(500);

	digitalWrite(R_REV_PIN, 255);
	delay(1000);

	GoStop(0);
}
