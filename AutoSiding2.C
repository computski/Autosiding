
#include <xc.h>
#include <stdint.h>

/*
Auto siding controller 
v1.0 2015-01-15
v2.0 2015-01-18 first working version, does not use interrupts
v2.1 2015-01-19 WDT enabled and overcurrent protection
v2.3 2015-01-21 fixed SKIP and modified overcurrent to use counter
v2.4 2015-01-22 better noise immunity for sensors

v2.5 2015-02-28 realised PNP open drian drive was not going to work
V2.6 2015-03-01 SET FOR NPN transistors, advise use of darlington TIP122

2015-06 hardware note: even with darlington transistors the solenoids would not deploy fully on a 15v supply
and a fat 4700 uF cap did not help either.  The solution was to parallel a 0.68R resistor with the 3R3 existing current
detector one.  Clearly the 3R3 dropped too much voltage during the pulse.  0.68R is still large enough to cause a current trip
if >1A is flowing when it shouldn't so protection is not compromised.

When two sets of points need to change, the system fires one then the other.  The reduces the current load compared to firing 4
solenoids at once


DEVICE      16F84
OSCILLATOR  4MHz external
COMPILER	XC8
*/


#define _XTAL_FREQ    4000000

/*instruction cycle is 1MHz.  TMR0 prescale is 64, giving 15khz ints, then we count through 157 giving 100Hz ints
IMPORTANT: The 16F84 seems to have a problem with TMR0 ints.  Will NOT use any ints in this code.
*/


//C:/Program%20Files/Microchip/xc8/v1.11/docs/chips/16f84.html
#pragma config FOSC = XT, WDTE = ON, CP = OFF, PWRTE = OFF

/*
System boots with SIDING1 set to normal.
In normal operation a loco entering a siding will cause a trigger event from one of the two sensors at each end
of the siding.  The first event will 'arm' the system, the second trigger at the opposite end must be seen within 10 seconds else the siding will not activate
if we DO see a second event, the solenoids are fired cycling through to the next siding and disconnecting power to the loco thus stopping it.

To skip a siding, tie one of its sensors to ground.  The user can also hit a push button to cycle the siding

Holding the push button for 3 seconds will set siding 1 to normal and lock it there.  To unlock, hold button for 3 secs again

The solenoids are driven by a 0.5 second pulse, active high.  

Note: The sensors are not particularly sensitive, you need to glue a small square of reflective tinfoil to the bottom of the loco.  Even if the wagons also are
picked up going over the first sensor, no problem, because system will be looking for the second sensor.

You can run a train in from either end of the siding, but the loco (with reflective foil) must be at the head of the train

Since a loco will be depowered when on-top of a sensor, immediately on cycling to a new siding that holds a parked loco, we will see an 'arm' event
however, since the train is leaving the siding, the arm event will time out after 10s and that siding will remain open.  Remember that the system will
be looking for a trigger from the opposite end of the siding and this will not occur because the train is moving AWAY from that sensor.  Therefore
a timeout will occur and the siding will remain open.  Thus the train can run 1 circuit of the track, return to the siding and trigger a cycle of the
siding to the next.

INDICATOR lamp on RB7 shows 
1 breif flash: siding 1 active
2 brief flashes: siding 2 active
3 brief flashes: siding 3 active
50% duty flash: armed, looking for second trigger
solid on: locked to siding 1
on, brief off flash: overcurrent fault condition


HARDWARE NOTES
The IR LED drive: pin is active high and intended to drive a transistor to achieve a 60mA pulse per IR LED.  The LEDs are wired in serial with a resistor so they
all can be illuminated with 60mA from the 20v rail.  Together they drop about 7.2v for 6 of them.

Solenoid drive: Originally I used TIP31 NPN power transistors.  These have an hfe of about 150, and I put 100 ohm base resistors on them.  However they did not reliably
drive the solenoids because the PIC cannot source sufficient current into the base.  PIC can source about 20mA and we need about 40mA of base drive to saturate the
transistor.  Use TIP120 or TIP122 instead.  These are darlingon power transistors with an hfe over 1000.   PNP transistors cannot be used because the PIC outputs cannot
be made true open drain.  They have ESD diodes clamping the rails, and with a PNP emitter at 20v, the ESD diode would cause the base to be at 5v and permanently biased on.
Note also that 15v is not enough to trigger a pair of solenoids.  Use 20v instead.  Powering two solenoids at once will see about 1.8A drawn.

The solenoid pairs are staggered, such that the first pair of turnouts fire, then the second rather than all 4 together.

Current limter:  A 3R3 resitor is in series with the 20v supply. The low side of this feeds a PNP current sense transistor which pulls a 1k res to 20v, this res is
connected into RB0 and clamped to the 5v rail.   RB0 also has a 1k res into an NPN sink transistor to drive the IR leds.   At the end of every LED pulse, the system flips
RB0 to an input and looks for a high condition, which signifies current flowing.  This is used as the basis of the fault condition current trip. During normal operation,
this flag is masked out for 2 seconds during a solenoid drive event.  if current is seen outside of this event, it is a fault condition and we tristate the outputs 
to protect the solenoid coils.  Its easy to burn out a coil in only 10 seconds.

PROTECTION:  WDT enabled.  needs a WDT prescalar because the timeout might go as low as 8mS.

*/


//OPTIONAL COMPILE SWITCHES
#define	SKIP
#define PUSHBUTTON
#define NOISE2_no
#define NOISE1
#define OPENDRAIN_no

//DECLARE functions
void sendPulse(uint8_t s);




/*
Create shadow register for GPIO.  Needed because of read-modify-write operations.
usage is sGPIO.port to refer to entire port or sGPIO.RA0 etc for individual bits
*/
volatile union {
uint8_t		port;
struct {
	unsigned	RA0	:1;   //pin [18] solenoid 1a
	unsigned	RA1	:1;   //pin [17] solenoid 1b
	unsigned	RA2	:1;   //pin [1] solenoid 2a
	unsigned	RA3	:1;   //pin [2] solenoid 2b
	unsigned	RA4	:1;   //pin [3] open drain only 
	unsigned	RA5	:1;   //not implemented 
	unsigned	RA6	:1;   //not implemented
	unsigned	RA7	:1;   //not implemented
	};
} sPORTA;

volatile union {
uint8_t		port;
struct {
	unsigned	RB0	:1;   //pin [6] spare
	unsigned	RB1	:1;   //pin [7] Sensor 1
	unsigned	RB2	:1;   //pin [8] sensor 2
	unsigned	RB3	:1;   //pin [9] sensor 3
	unsigned	RB4	:1;   //pin [10] sensor 4
	unsigned	RB5	:1;   //pin [11] sensor 5
	unsigned	RB6	:1;   //pin [12] sensor 6
	unsigned	RB7	:1;   //pin [13] not implemented
	};
} sPORTB;


//port READS
#define sensor1 PORTBbits.RB1
#define sensor2 PORTBbits.RB2
#define sensor3 PORTBbits.RB3
#define sensor4 PORTBbits.RB4
#define sensor5 PORTBbits.RB5
#define sensor6 PORTBbits.RB6
#define button	PORTAbits.RA4

//port WRITES, all to shadow ports
#define LED    			sPORTB.RB0
#define	indicator		sPORTB.RB7
#define solenoid1normal sPORTA.RA0
#define solenoid1thrown sPORTA.RA1
#define solenoid2normal sPORTA.RA2
#define solenoid2thrown sPORTA.RA3

#define currentSenseTris	TRISB0    //odd but its not TRISBbits.RB0
#define currentSensor		PORTBbits.RB0
uint8_t	currentTripTimer;


#ifdef NOISE2
//sensor bit patterns to detect
#define armed	0b11111111
#define skipped	0b01010101
#endif

//prior scheme
#ifdef NOISE1
#define armed	0b01010101
#define skipped	0
#endif




/*the status of which siding is open, we rotate through 1 to 2*/
enum stateCONTROL
{
SIDING1_NORMAL,
SIDING1_ARM,
SIDING2_NORMAL,
SIDING2_ARM,
SIDING3_NORMAL,
SIDING3_ARM,
SIDING1_LOCK_NORMAL,
FAULT
};

uint8_t stateEngine;


enum sidingCONTROL
{
SIDING_DONE,
SIDING1_STEP1,
SIDING1_STEP2,
SIDING2_STEP1,
SIDING2_STEP2,
SIDING3_STEP1,
SIDING3_STEP2
};

uint8_t stateSiding;


//display the value of the stateEngine with various flash combinations held in IND[]
uint8_t ticks;
const uint8_t IND[] = {0b10000000,0b11110000,0b10100000,0b11110000,0b10101000,0b11110000,0b11111111,0b11111110};
uint8_t indicatorPointer=0;

//TIMER VALUES
//10mS timebase values
#define T_ARMED 	1500  //sensor armed timeout
#define T_DEBOUNCE	5  //switch debounce
#define T_LONGPUSH	300	//long-push timer trigger
#define	T_PULSE		60  //solenoid pulse

//100mS timebase values
#define T_CURRENT	25  //overcurrent trip




enum buttonSTATE
{
BTN_WAIT_UP,
BTN_UP,
BTN_DOWN,
BTN_PUSH,
BTN_LONG_PUSH
};

uint8_t stateButton;



/*there are 6 opto sensors, one at each end of the three sidings
these are the shift registers*/
uint8_t S1_1reg;
uint8_t S1_2reg;
uint8_t S2_1reg;
uint8_t S2_2reg;
uint8_t S3_1reg;
uint8_t S3_2reg;

//which sensor was first to trigger, A end or B end
bit	sensorAB;


//all timing is based on a 10mS program loop, need 16 bit counters for delays over 2sec
uint8_t pulseTimeout;
uint8_t debounceTimeout;
unsigned int armTimeout;
unsigned int longPushTimeout;

bit	dopulse;



void main(void)
{
//BOOT sequence

//set WPU on all of port b
OPTION_REGbits.nRBPU=0;

//set inputs and outputs
TRISA=0b11110000;  //all outputs except RA4
TRISB=0b01111110   ;  //RB0 is the LED drive, RB7 the indicator, rest are inputs


//set prescalar divide 64
OPTION_REGbits.T0CS=0;  //use instruction clock
//OPTION_REGbits.PSA=0;  //prescalar to TMR0
//OPTION_REGbits.PS = 0b101;  //divide 64

OPTION_REGbits.PSA=1;  //prescalar to WDT
OPTION_REGbits.PS = 0b010;  //divide 8

//note WDT has a notional timeout of 18mS but this can drop to 8mS when using a 5v supply.  To be safe, we will use a postscalar on the WDT

//PROBLEM WITH TMR0 INTS, DO NOT USE
//enable TMR0 and interrupts
INTCONbits.T0IE=0;  //disabled
di();

stateEngine=SIDING1_NORMAL;
stateButton=BTN_WAIT_UP;
dopulse=1;  //fire solenoids on startup
pulseTimeout=0;//will reset all solenoids


/*since INTS don't work, we will use a fixed delay to generate 10ms delay, after which the rest of the code will execute
10mS @ fsOsc 4MHz is 10,000 instructions 
*/


//PROGRAM LOOP
while(1){

//assert ports, this is the ONLY place this happens
PORTA = sPORTA.port;
PORTB = sPORTB.port;
CLRWDT();

//this controls the countdown timers, because rest of prog loop executes probably in less than 1mS
__delay_ms(10);
CLRWDT();


//led status indicator, flashes variously to indicate state.  set in the main state engine section below
ticks++;
if (ticks>=10)
{//this code runs every 100mS
ticks=0;
indicatorPointer--;
uint8_t i;
//point to the part of the bit pattern we need
i = 1<<(indicatorPointer & 0b00000111);
//AND with the relevant bit pattern itself
i = i & IND[stateEngine];
//drive the indicator bit
indicator = (i==0)?0:1;

//current sensing
currentSenseTris = 1; //set as input
NOP();
NOP();
NOP();
//allow a few uS for things to settle, now read the pin
if (currentSensor==1)
{
	currentTripTimer++;
	if (currentTripTimer>T_CURRENT)
	{//error condition, shut down outputs by taking all of PORTA tristate
	TRISA=0xFF;
	stateEngine=FAULT;
	}
}
else
{
	currentTripTimer=0;
}

currentSenseTris = 0; //set as output again
}


//per loop (per 10ms) button processing
//remember &,| bitwise whereas &&, || logical
#ifdef PUSHBUTTON
switch (stateButton){
	case BTN_WAIT_UP:
		//button released
		if (button==1){stateButton=BTN_UP;}
		break;	

	case BTN_UP:
		//look for down, and seed counters
		if (button==0){
			debounceTimeout=T_DEBOUNCE;  //50mS
			longPushTimeout=T_LONGPUSH; //approx 3 sec
			stateButton=BTN_DOWN;
		}		
		break;

	case BTN_DOWN:
		debounceTimeout-=(debounceTimeout>0)?1:0;
		longPushTimeout-=(longPushTimeout>0)?1:0;
		/*if button held longer than debouceTimout but less than longPushTimout then declare a push
		 if button released before either timeout declare BTN_UP
		*/		
		
		//btn still down, longPushTimeout exceed
		if ((button==0) && (longPushTimeout==0))
		{//btn still down, declare long event
			stateButton=BTN_LONG_PUSH;
			break;
		}
		
		//btn now up, but was debounceTimeout exceeded?
		if (button==1){
			if (debounceTimeout==0)
				{//yes
				stateButton=BTN_PUSH;
				}
				else
				{//no
				stateButton=BTN_UP;
				}
		}
		break;	

	//case BTN_PUSH:
	//case BTN_LONG_PUSH:
	//these button states are processed later

}; //end switch
#endif


//also per loop, sample sensors and then toggle LED drive
S1_1reg = S1_1reg<<1;
S1_2reg = S1_2reg<<1;
S2_1reg = S2_1reg<<1;
S2_2reg = S2_2reg<<1;
S3_1reg = S3_1reg<<1;
S3_2reg = S3_2reg<<1;

//for SKIP, do not invert sensor reading, i.e. no detect is all 1s not all zeros
//NOISE: I found that the kitchen halogens cause false triggers.  these have a strong IR signal at 50Hz
//means we need to XOR the drive status with the sensor
//a valid detect signal is now all 1s and a valid skip signal is 0101010

S1_1reg += (sensor1==1)?1:0;
S1_2reg += (sensor2==1)?1:0;
S2_1reg += (sensor3==1)?1:0;
S2_2reg += (sensor4==1)?1:0;
S3_1reg += (sensor5==1)?1:0;
S3_2reg += (sensor6==1)?1:0;

#ifdef NOISE2
uint8_t j=0;
j+= (LED==1)?1:0;
S1_1reg ^=j;
S1_2reg ^=j;
S2_1reg ^=j;
S2_2reg ^=j;
S3_1reg ^=j;
S3_2reg ^=j;
#endif



armTimeout-=(armTimeout>0)?1:0;

//toggle LED drive once sampling completed above
LED =~LED;

//clear solenoid pulses once completed, but don't disturb LED pulses
pulseTimeout-=(pulseTimeout>0)?1:0;
if (pulseTimeout==0){
	solenoid1normal =0;
	solenoid1thrown =0;
	solenoid2normal =0;
	solenoid2thrown =0;
;}




/*
routes are;
solenoid1normal, solenoid2 normal  is siding 1
solenoid1thrown, solendoid2 thrown  is siding 2
solendoi1thrown, solendoid2 normal  is siding 3

IMPORTANT: the solenoid pulses are sent when we first enter the SIDINGx_NORMAL state because we may advance to this state through
arming, through button push or through skip

IMPORTANT: any button push events will only take effect once system has dealt with any arm conditions, because during arm there is clearly
a train moving into a siding and we don't wish to action a point change until it has stopped

Note: bitwise |,&   logical ||, &&
*/


switch (stateEngine){
	case SIDING1_NORMAL:
		sendPulse(1);
		//arm condition?  

//for noise immunity, a sequence 01010101 is only valid if current state of LED (as just toggled) is 1
		if ((S1_1reg==armed)||(S1_2reg==armed)){
		#ifdef NOISE1
			if (LED==0){break;}
		#endif
		stateEngine=SIDING1_ARM;
		armTimeout=T_ARMED;  //5 sec
		//which sensor triggered first?
		sensorAB = (S1_1reg==armed)?1:0;
		}
		break;

	case SIDING1_ARM:
		//If we accepted the same end trigger, we'd immediately re-trigger as the loco is still overhead
		//so we need to see the opposite end trigger within 5 sec
		//Note any arm condition will clear a pending button event
		stateButton=BTN_WAIT_UP;
	
		if (armTimeout==0){
		//fail, revert to normal route
		stateEngine=SIDING1_NORMAL;
		break;
		}
		//A triggered first, need to see B
		if ((sensorAB) && (S1_2reg==armed))
		{dopulse=1;
		stateEngine=SIDING2_NORMAL;
		break;}
		//B triggered first, need to see A
		if ((!sensorAB) && (S1_1reg==armed))
		{dopulse=1;
		stateEngine=SIDING2_NORMAL;
		}
		//if don't see right thing exit anyway
		break;

	case SIDING2_NORMAL:
		#ifdef SKIP
		if (S2_1reg==skipped){
			//skip siding 2
			stateEngine=SIDING3_NORMAL;
			dopulse=1;
			break;
		}
		#endif
		sendPulse(2);
		//arm condition?
		if ((S2_1reg==armed)||(S2_2reg==armed)){
		#ifdef NOISE1
			if (LED==0){break;}
		#endif
		stateEngine=SIDING2_ARM;
		armTimeout=T_ARMED;  //5 sec
		//which sensor triggered first?
		sensorAB = (S2_1reg==armed)?1:0;
		}
		break;
		
	case SIDING2_ARM:
		//If we accepted the same end trigger, we'd immediately re-trigger as the loco is still overhead
		//so we need to see the opposite end trigger within 5 sec
		//Note any arm condition will clear a pending button event
		stateButton=BTN_WAIT_UP;
			
		if (armTimeout==0){
			//fail
			stateEngine=SIDING2_NORMAL;
			dopulse=1;
			break;
		}
		//A triggered first, need to see B
		if ((sensorAB) && (S2_2reg==armed))
		{dopulse=1;
		stateEngine=SIDING3_NORMAL;
		break;}
		//B triggered first, need to see A
		if ((!sensorAB) && (S2_1reg==armed))
		{dopulse=1;
		stateEngine=SIDING3_NORMAL;
		}
		//if don't see right thing exit anyway
		break;
	
	case SIDING3_NORMAL:
		#ifdef SKIP
			if (S3_1reg==skipped){
			//skip siding 3
			stateEngine=SIDING1_NORMAL;
			dopulse=1;
			break;
		}
		#endif

		sendPulse(3);
		//arm condition?
		if ((S3_1reg==armed)||(S3_2reg==armed)){
		#ifdef NOISE1
			if (LED==0){break;}
		#endif	
			stateEngine=SIDING3_ARM;
			armTimeout=T_ARMED;  //5 sec
			//which sensor triggered first?
			sensorAB = (S3_1reg==armed)?1:0;
			}
			break;
	case SIDING3_ARM:
		//If we accepted the same end trigger, we'd immediately re-trigger as the loco is still overhead
		//so we need to see the opposite end trigger within 5 sec
		//Note any arm condition will clear a pending button event
		stateButton=BTN_WAIT_UP;
	
		if (armTimeout==0){
		//fail
		stateEngine=SIDING3_NORMAL;
		break;
		}
		//note, you cannot skip siding1
		
		//A triggered first, need to see B
		if ((sensorAB) && (S3_2reg==armed))
		{dopulse=1;
		stateEngine=SIDING1_NORMAL;
		break;}
		//B triggered first, need to see A
		if ((!sensorAB) && (S3_1reg==armed))
		{dopulse=1;
		stateEngine=SIDING1_NORMAL;
		}
		//if don't see right thing exit anyway
		break;

	case FAULT:
		//do nothing,we can only recover by cycling the power
		NOP();
		break;

	//case SIDING1_LOCK_NORMAL:
	//handled in the button processing below

	};


#ifdef PUSHBUTTON
//button processing.  At this point in the code, the system would have recognised any ARM events
//so the buttons will only apply to stable _NORMAL states

if ((stateButton==BTN_PUSH)||(stateButton==BTN_LONG_PUSH))
{
		
switch (stateEngine){
	case SIDING1_NORMAL:
		if (stateButton==BTN_PUSH)
			{
			stateEngine=SIDING2_NORMAL;
			break;
			}
		if (stateButton==BTN_LONG_PUSH)
			{
			stateEngine=SIDING1_LOCK_NORMAL;
			}			
		break;

	case SIDING2_NORMAL:
		if (stateButton==BTN_PUSH)
			{
			stateEngine=SIDING3_NORMAL;
			break;
			}
		if (stateButton==BTN_LONG_PUSH)
			{
			stateEngine=SIDING1_LOCK_NORMAL;
			}			
		break;

	case SIDING3_NORMAL:
		if (stateButton==BTN_PUSH)
			{
			stateEngine=SIDING1_NORMAL;
			break;
			}
		if (stateButton==BTN_LONG_PUSH)
			{
			stateEngine=SIDING1_LOCK_NORMAL;
			}			
		break;

	case SIDING1_LOCK_NORMAL:
		if (stateButton==BTN_LONG_PUSH)
		{
		stateEngine=SIDING1_NORMAL;
		}
		break;
	};

dopulse=1;
stateButton=BTN_WAIT_UP;

}//end push
#endif


}//end prog loop
}//end main


/*Interrupt routine. 
NOT USED, SEEMS TO BE A PROBLEM WITH INTS ON THE 16F84
*/
interrupt ISR(void)
{
if (INTCONbits.T0IF==1){
INTCONbits.T0IF==0;
}

}



/*pulse solenoids to achieve routing into siding 1,2 or 3
configs are;
siding 1:  sol1=n sol2=t
siding 2:  sol1=t sol2=t
siding 3:  sol1=t sol2=n

call sendPulse with the siding of choice in variable s being 1,2 or 3.  the doPulse flag has to be set beforehand

the routine has an internal state engine and will fire all 4 solenoids as first pair, second pair rather than all 4 at once.  Once it has fired the second pair
it will clear the doPulse flag

*/

void sendPulse(uint8_t s){
//call with s=1,2,3
//routine will take that as an intiator if doPulse=1


//else we are following through with the second step
switch(stateSiding){

case SIDING1_STEP2:
	//wait for first step to finish, then action this one
	if (pulseTimeout!=0){return;}
	pulseTimeout=T_PULSE;
	dopulse=0;
	stateSiding=SIDING_DONE;
	solenoid2thrown=1;   //second solenoid step
	break;

case SIDING2_STEP2:
	if (pulseTimeout!=0){return;}
	pulseTimeout=T_PULSE;
	dopulse=0;
	stateSiding=SIDING_DONE;
	solenoid2thrown=1;   //second solenoid step
	break;	

case SIDING3_STEP2:
	if (pulseTimeout!=0){return;}
	pulseTimeout=T_PULSE;
	dopulse=0;
	stateSiding=SIDING_DONE;
	solenoid2normal=1;
	break;

default:
	//initiation (or SIDING_DONE)
	if (dopulse==0){return;}
	pulseTimeout=T_PULSE;

	switch (s){
	case 1:
			solenoid1normal=1;   //active high
			stateSiding=SIDING1_STEP2;
			break;
	case 2:
			solenoid1thrown=1;   //active high
			stateSiding=SIDING2_STEP2;
			break;
	case 3:
			solenoid1thrown=1;   //active high
			stateSiding=SIDING3_STEP2;
			break;
	}
	return;  //iniation complete
}

}


/*
*/




