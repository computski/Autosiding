
#include <xc.h>
#include <stdint.h>

//version marker in EEPROM space
eeprom char ver[]="AutoSiding v4.0 2020-06-07";
//this was a straight port to PICF88 and two relay outputs added to RB6+7

/*
Usage: 
Unit boots in auto mode with siding 1 active.  sidings advance automatically when sensors are triggered in sequence. 
A short button push in auto-mode will also advance the siding.
A long button push (3 sec) will lock a particular siding open.  Another long push will take you back to auto mode on that siding.
A short push in lock-mode will advance to the next siding, keeping that locked open.


Auto siding controller 
v1.0 2015-01-15
v2.0 2015-01-18 first working version, does not use interrupts
v2.1 2015-01-19 WDT enabled and overcurrent protection
v2.3 2015-01-21 fixed SKIP and modified overcurrent to use counter
v2.4 2015-01-22 better noise immunity for sensors

v2.5 2015-02-28 realised PNP open drian drive was not going to work
V2.6 2015-03-01 SET FOR NPN transistors, advise use of darlington TIP122
v3.0 2015-09-19 Circuit modified to use capacitor discharge at 2 x DCC track voltage through a doubler
v3.1 2015-10-04 changed firing order of solenoids for S3.  A long button push will now lock the current siding, and a second long push will
unlock and keep that same siding selected (does not reset to siding 1).
v3.2 2015-10-05 bug fix, siding lock did not unlock.
v3.4 2015-10-06 added a blanking period when activating a siding to allow loco to exit in either direction without causing an arm event.  Previously, if you reversed loco
direction, it would immediately arm at the sensor it was sitting on, then trigger the opposite end trigger and cause the siding to advance whilst the loco was mid exit.	
v3.5 2015-10-08 fix for manual mode advance
v4.0 2020-06-07 re-written for PICF88 and driving two relays to power track

2015-09-20 hardware. 
Fed from the DCC supply via a 33R resistor.
Using a half wave rectified supply into a 7805 regulator for the logic.
Also running a voltage doubler into 2200 uF CDU.

Some CDU designs have a darlington biased on from the supply via a base resistor, its emitter charges the discharge capacitors.  The discharge cap pathway is via
a diode, and on the cathode side, a link is taken back to the darlington base.  When in discharge cycle, the base is held low by the solenoid thus removing the charge
supply.  This will lose about 1.5v with the transistor and extra diode.  In my design I have a 33R charge resistor, and when the solenoid path is closed, the reactance of
the 100uF cap in series is near zero meaning the full 14v line voltage appears across the 33R giving a peak current of approx 400mA.  This is present only for a fraction
of a second prior to the solenoid disconnecting.  We then see a peak whilst the CDU recharges but this is less than 0.5s.  The resistor is a 5W device and gets slightly
warm.

The IR LEDs also draw current from the half wave supply, approx 30mA.  drop on the supply resistor will be max 1v if using the 33R.

The current monitoring resistor is now removed.  In this version, virtual current monitoring is employed; the code indpendently looks at the output states
and declares a fault if they have been active for too long.  Hardware based monitoring correctly showed current was flowing, but we could only manipulate 
RA<0-3> to respond.  So we might as well monitor sPORT.port RA<0-3> in an independent software process and use this as the trip.  The risk of coil burnout is now
much lower anyway due to the 33R resistor in the supply.  Max current for a stuck-on condition is about 400mA.

The IR LED drive: pin is active high and intended to drive a current-sink transistor to achieve a 60mA pulse per IR LED.  The LEDs are wired in serial with a resistor so they
all can be illuminated with 60mA from the 14v rail.  Together they drop about 7.2v for 6 of them.


2014-08 hardware.
Original design used 20v DC as the supply.  Had a 3R3 current sense res in the path, later dropped this to 0.68R. Regular TIP31s needed approx 40mA drive and the
PIC does not have this.  Moved to using TIP122s.  Found even these darlingtons were not able to reliably fire solenoid pairs.
This is why I moved to using a voltage doubling CDU.

Original current sense hardware:  A 3R3 resitor is in series with the 20v supply. The low side of this feeds a PNP current sense transistor which pulls a 1k res to 20v, this res is
connected into RB0 and clamped to the 5v rail.   RB0 also has a 1k res into an NPN sink transistor to drive the IR leds.   At the end of every LED pulse, the system flips
RB0 to an input and looks for a high condition, which signifies current flowing.  This is used as the basis of the fault condition current trip. During normal operation,
this flag is masked out for 2 seconds during a solenoid drive event.  if current is seen outside of this event, it is a fault condition and we tristate the outputs 
to protect the solenoid coils.  Its easy to burn out a coil in only 10 seconds as the DC supply can push 4A through it.


CODE TIDY UP: code here compiles to a full 1k.  Could abstract part of the siding state changes to a sub
void sidingStateProcess(current siding, next siding, sensor A, sensor B, sensorAB, mode)
hmmm.  modify sendPulse? this is generic but perhaps we should add an extra state of SIDING1_PULSE.  this would send the pulse, but it needs to know where to 
go next depending on mode.




DEVICE      16F88
OSCILLATOR  4MHz internal
COMPILER	XC8
*/


#define _XTAL_FREQ    4000000

/*instruction cycle is 1MHz.  TMR0 prescale is 64, giving 15khz ints, then we count through 157 giving 100Hz ints
IMPORTANT: The 16F84 seems to have a problem with TMR0 ints.  Will NOT use any ints in this code.
*/


//C:\Program Files (x86)\Microchip\xc8\v1.36\docs\chips/16f88.html
#pragma config FOSC = INTOSCIO, WDTE = ON, CP = OFF, PWRTE = OFF, MCLRE=OFF, IESO=OFF, FCMEN=OFF, BOREN=OFF, LVP = OFF, CCPMX=RB0
#pragma warning disable 359


/*
System boots with a 2 second delay this allows the capacitors to charge before any action is taken (were getting 2A current trips prior)
then resets with SIDING1 set to normal.

Each siding has a sensor at each end.  In normal operation, a loco entering the siding will cause a trigger event in one of the sensors.  This 'arms' the
system which then expects a second trigger from the other sensor within T_ARMED (15 sec).  If we see a second event, the solenoids are fired in sequence.

The solenoids fire but have to wait T_RECHARGE before the second solenoid firing can take place.  If a second trigger event is not seen the system will timeout and
be looking for a new arm condition on that same siding.

When the first solenoid fires, it disconnects the DCC signal from that siding thus stopping the train.

To skip a siding, tie one of its sensors to ground.  The user can also hit a push button to cycle the siding

Holding the push button for 3 seconds will lock the current siding.  To unlock, hold button for 3 secs again

The solenoids are driven by a T_PULSE (0.2 second) pulse active high.  

Note: The sensors are not particularly sensitive, you may need to stick a small square of reflective white paper/tinfoil to the bottom of the loco.  Even if the wagons also are
picked up going over the first sensor, no problem, because system will be looking for the second sensor.

You can run a train in from either end of the siding, but the loco (with reflective square) must be at the head of the train

Since a loco will be depowered when on-top of a sensor, immediately on cycling to a new siding that holds a parked loco, we will see an 'arm' event.  This is not
a problem since the train is leaving the siding; the arm event will time out after 10s and that siding will remain open.  The system always looks for a trigger event from
opposite end of the sensor pair compared to the arm event.  This way, a train can leave the siding and run 1 circuit of the track, return to the siding and trigger a 
cycle of the siding to the next.

INDICATOR lamp on RB7 shows 
1 breif flash: siding 1 active
2 brief flashes: siding 2 active
3 brief flashes: siding 3 active
50% duty flash: armed, looking for second trigger
solid on: locked to siding 1
on, brief off flash: overcurrent fault condition



PROTECTION:  WDT enabled.  needs a WDT prescalar because the timeout might go as low as 8mS.

2020-06-05 no more outputs available.  want to drive two relays to switch power to the sidings rather than relying on the turnouts themselves  or attaching micro-switches to them.
Could we use a Nano instead?  this has D0-D13 as digital pins (D0+1 are used for serial) and 7 analog pins but you could use analogue read on the photo transistors in the IR sensor.
so easily enough pins to run the whole show.  But hassle to rebuild the whole unit around a nano.

2020-06-07 re-written for PIC16F88
we can now drive relays with the 2 pins saved from external crystal.  These will switch power into the sidings rather than relying on the frog rail contacts.
I have a couple of dpdt 5v coil relays which should do the job.
we get RA5,6,7 by using this PIC, however pin [4]=RA5 is tied high as its the MCLR pin

TIMING of the relays versus the solenoid pulses is not critical, because the relays just provide a backup of the current path through the point-rail in question to the siding.
If a loco has a lot inertial programmed and a keep alive, there is a danger it will run-on with power-loss and fowl the points.
Once option is to implement ABC, which is asymetric ddc-initiated braking.  One rail section is isolated and fed via say 3 diode drops.  An ABC feature decoder can detect this
and will apply brakes to stop in a preset distance.   None of my decoders have this option.

http://www.wiringfordcc.com/switches.htm


TRANSPONDER idea.  Fit an IR LED to the loco which can be programmed with its DCC address.  this continuously transmits the address.  when a loco enters a section its address is detected
by an IR phototransistor and the controller can send an eSTOP signal.  Does rail com achieve the same?


FUTURES: implement a dcc accessory decoder.  this will allow the sidings to map to say 5 and 6.  but there will be a disconnect between originally set route and activity on 
auto-route itself.

*/


//OPTIONAL COMPILE SWITCHES
#define	SKIP
#define PUSHBUTTON
#define NOISE2_no
#define NOISE1
#define OPENDRAIN_no
#define	VIRTUAL_CURRENT_MONITOR

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
	unsigned	RA4	:1;   //pin [3] open drain only.  button
	unsigned	RA5	:1;   //pin [4] = MCLR on 16F84 
	unsigned	RA6	:1;   //pin [15] = OSC2 on 16F84
	unsigned	RA7	:1;   //pin [16] = OSC1 on 11684
	};
} sPORTA;

volatile union {
uint8_t		port;
struct {
	unsigned	RB0	:1;   //pin [6] LED, also INT pin so future use for DCC decode
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
uint8_t	currentTripTimer;  //100mS clock

//2020-06-07 define relays
#define relay1	sPORTA.RA6
#define relay2	sPORTA.RA7


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




/*the status of which siding is open, we rotate through 1 to 2
These enumerate the corresponding IND values too
*/

enum stateCONTROL
{
SIDING1_NORMAL,
SIDING1_ARM,
SIDING2_NORMAL,
SIDING2_ARM,
SIDING3_NORMAL,
SIDING3_ARM,
SIDING1_LOCK,
SIDING2_LOCK,
SIDING3_LOCK,
FAULT,
SIDING_BOOT
};

uint8_t stateEngine;
//display the value of the stateEngine with various flash combinations held in IND[]
const uint8_t IND[] = {0b10000000,0b11110000,0b10100000,0b11110000,0b10101000,0b11110000,0b01111111,0b01011111,0b01010111,0b11111111,0x00};
uint8_t indicatorPointer=0;

//2015-09-17 siding control modified.  there is no need to always fire both pairs
//only siding 1 needs two steps (first to take 1 to main, then a delay and siding 2 going open
//next transition is just siding 2 going closed to point at 3
//final transition is siding 1 going back to main, followed by siding 2 going open to reset.
//added a 2 second BOOT delay to allow the CDU to reach voltage on boot

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


uint8_t ticks;


//TIMER VALUES
//10mS timebase values
#define T_ARMED 	1500  //sensor armed timeout
#define T_DEBOUNCE	5  //switch debounce
#define T_LONGPUSH	300	//long-push timer trigger
#define	T_PULSE		20  //was 60, now reduced to 20. solenoid pulse
#define T_BLANKING	200	//sensor blanking period as 2 sec, long enough for loco to clear sensor

//100mS timebase values
#define T_CURRENT	25  //overcurrent trip
#define T_RECHARGE	20	//Capacitor recharge period
#define T_BOOT		30  //boot wait period



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


//these items, timing is based on a 10mS clock, need 16 bit counters for delays over 2sec
uint8_t pulseTimeout;
uint8_t debounceTimeout;
unsigned int armTimeout;
unsigned int longPushTimeout;
unsigned int sensorBlankingTimeout; //2015-10-06 added a blanking period for the sensors


//these items, timing based on 100ms clock
uint8_t rechargeTimeout;






//2015-10-04 used to restore state after a siding lock
uint8_t lastState;
bit	dopulse;

//++++++++++++++++++++++++++++++++++++++
/*DCC decode related*/

volatile unsigned int	shiftRMB;  //16 right most bits
volatile unsigned long long shiftLMB;   //32 left most bits
volatile unsigned int	packetRMB;
volatile unsigned long long packetLMB;


volatile bit packetRX;  //received a possible packet
#define DCCin	RB0
uint8_t data0,data1,data2,data3,data4;

bit	packet4byte;
bit	packetGood; //packet verified
bit lastPacketWasReset;
 

enum sDCC{
DCC_WAIT,   //wait for a packet
DCC_RESET,  //saw reset
DCC_SERVICE,  //service mode
DCC_ACCESSORY  //accessory function
};

uint8_t stateDCC=DCC_WAIT;

/*decoder CVs*/

eeprom uint8_t CV513=0x01; //decoder address LSB
eeprom uint8_t CV514=0x99;
eeprom uint8_t CV515; //on-time f1
eeprom uint8_t CV516; //on-time f2
eeprom uint8_t CV517; //on-time f3
eeprom uint8_t CV518; //on-time f4
eeprom uint8_t CV519;
eeprom uint8_t CV520;
eeprom uint8_t CV521=0b111; //decoder address msb
eeprom uint8_t CV522;
eeprom uint8_t CV523;
eeprom uint8_t CV524;
eeprom uint8_t CV525;
eeprom uint8_t CV526;
eeprom uint8_t CV527; //decoder lock
eeprom uint8_t CV528; //decoder lock

/*might as well just reserve a block of 16 and then map cv513 to cv1 etc to make life easy
we have ample eeprom space*/








void DCCdecode(void);

//+++++++++++++++++++++++++++++++++++++++




//SIZE: this compiles into 1193  which is only a couple of hundred bytes too large for the PIC'84





void main(void)
{

DCCdecode();








//BOOT sequence, use internal osc at 4MHz
OSCCONbits.SCS=0b10;
OSCCONbits.IRCF=0b110;

//set WPU on all of port b
OPTION_REGbits.nRBPU=0;

//set inputs and outputs
TRISA=0b00110000;  //all outputs except RA4,5
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
INTCONbits.TMR0IE=0;  //disabled
di();

stateEngine=SIDING_BOOT;
stateButton=BTN_WAIT_UP;
stateSiding=SIDING1_NORMAL;
dopulse=0;  //DO NOT fire solenoids on startup
pulseTimeout=0;//will reset all solenoids
rechargeTimeout=T_BOOT;


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

rechargeTimeout-=(rechargeTimeout>0)?1:0;

indicatorPointer--;
uint8_t i;
//point to the part of the bit pattern we need
i = 1<<(indicatorPointer & 0b00000111);
//AND with the relevant bit pattern itself
i = i & IND[stateEngine];
//drive the indicator bit
indicator = (i==0)?0:1;


#ifdef VIRTUAL_CURRENT_MONITOR
//expect all solendoid outputs to be a zero value, do this by reading the shadow register. 
//The real ports are tied to transistor bases and will likely read as zeros

if((sPORTA.port & 0b00001111)==0){
	//happy, all solenoid drive pins on RA<0-3> are inactive zero
	currentTripTimer=0;
}  
else
{
	currentTripTimer++;
	if (currentTripTimer>T_CURRENT)
	{//error condition, shut down outputs by taking all of PORTA tristate
	TRISA=0xFF;
	stateEngine=FAULT;
	}
}


#else
//conventional hardware based current sensing
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
#endif

}  //end of 100mS block


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
sensorBlankingTimeout-=(sensorBlankingTimeout>0)?1:0;


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

//2015-09-20 if pulseTimeout is 1 then we've had a pulse event and need to trigger a recharge event
//this is the ONLY place the rechargeTimeout is set
if (pulseTimeout==1){rechargeTimeout=T_RECHARGE;}




/*
routes are;
[1] solenoid1 normal, solenoid2 normal  is siding 1
[2] solenoid1 thrown, solendoid2 thrown  is siding 2
[3] solendoi1 thrown, solendoid2 normal  is siding 3

IMPORTANT: the solenoid pulses are sent when we first enter the SIDINGx_NORMAL state because we may advance to this state through
arming, through button push or through skip

IMPORTANT: any button push events will only take effect once system has dealt with any arm conditions, because during arm there is clearly
a train moving into a siding and we don't wish to action a point change until it has stopped

Note: bitwise |,&   logical ||, &&
*/


//2015-10-06 added sensorBlankingTimeout,  which prevents an armed condition happening
//if a siding is selected, as a result of arm-fire or user selecting via pushbutton when in auto mode, we need to activate a blanking period

switch (stateEngine){
	case SIDING_BOOT:
		//2015-09-17 need to wait 2 sec
		if (rechargeTimeout==0){
			dopulse=1;
			stateEngine=SIDING1_NORMAL;
	}
	break;

	case SIDING1_LOCK:
		sendPulse(1);			
		break;

	case SIDING1_NORMAL:
		sendPulse(1);
		//arm condition?  

//for noise immunity, a sequence 01010101 is only valid if current state of LED (as just toggled) is 1
		if ((S1_1reg==armed)||(S1_2reg==armed)){
		if (sensorBlankingTimeout>0) break;
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
		sensorBlankingTimeout=T_BLANKING;
		break;}
		//B triggered first, need to see A
		if ((!sensorAB) && (S1_1reg==armed))
		{dopulse=1;
		stateEngine=SIDING2_NORMAL;
		sensorBlankingTimeout=T_BLANKING;
		}
		//if don't see right thing exit anyway
		break;

	
	case SIDING2_LOCK:
		sendPulse(2);			
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
		if (sensorBlankingTimeout>0) break;
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
		sensorBlankingTimeout=T_BLANKING;
		break;}
		//B triggered first, need to see A
		if ((!sensorAB) && (S2_1reg==armed))
		{dopulse=1;
		stateEngine=SIDING3_NORMAL;
		sensorBlankingTimeout=T_BLANKING;
		}
		//if don't see right thing exit anyway
		break;

	
	case SIDING3_LOCK:
		sendPulse(3);			
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
		if (sensorBlankingTimeout>0) break;
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
		sensorBlankingTimeout=T_BLANKING;
		break;}
		//B triggered first, need to see A
		if ((!sensorAB) && (S3_1reg==armed))
		{dopulse=1;
		stateEngine=SIDING1_NORMAL;
		sensorBlankingTimeout=T_BLANKING;
		}
		//if don't see right thing exit anyway
		break;

	case FAULT:
		//do nothing,we can only recover by cycling the power
		NOP();
		break;

		};


#ifdef PUSHBUTTON
//button processing.  At this point in the code, the system would have recognised any ARM events
//so the buttons will only apply to stable _NORMAL states


if (stateButton==BTN_LONG_PUSH)
{
switch (stateEngine){
	case SIDING1_NORMAL:
			lastState = stateEngine;	
			stateEngine=SIDING1_LOCK;
			break;

	case SIDING2_NORMAL:
			lastState = stateEngine;	
			stateEngine=SIDING2_LOCK;
			break;

	case SIDING3_NORMAL:
			lastState = stateEngine;	
			stateEngine=SIDING3_LOCK;
			break;

	case SIDING1_LOCK:
	case SIDING2_LOCK:
	case SIDING3_LOCK:
			stateEngine=lastState;	
			break;
}
//no pulse is required
stateButton=BTN_WAIT_UP;
}

//in both auto and lock modes, a short button push will advance to next siding
if (stateButton==BTN_PUSH)
{
//2015-10-07 bug fix, set dopulse before calling sendPulse() below
	
switch (stateEngine){

	case SIDING1_NORMAL:
			stateEngine=SIDING2_NORMAL;
			sensorBlankingTimeout=T_BLANKING;
			break;
		
	case SIDING1_LOCK:
			stateEngine=SIDING2_LOCK;
			break;

	case SIDING2_NORMAL:
			stateEngine=SIDING3_NORMAL;
			sensorBlankingTimeout=T_BLANKING;
			break;
		
	case SIDING2_LOCK:
			stateEngine=SIDING3_LOCK;
			break;

	case SIDING3_NORMAL:
			stateEngine=SIDING1_NORMAL;
			sensorBlankingTimeout=T_BLANKING;
			break;
		
	case SIDING3_LOCK:
			stateEngine=SIDING1_LOCK;
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
if (TMR0IF==1){
TMR0IF=0;
}


/*DCC bit decoder*/
if (INTF){
	/*we've detected an edge, INTON<2> will indicate 1 for TMR0 timeout, i.e. T0IF is set.  DCC 1s have 58uS half cycles (max 64), DCC0s have half cycles notionally longer than
	100uS, but a min of 90uS  so if we detect a timeout at 80uS allowing for 20 instr to this point then we have a ZERO bit.*/
	TMR0 = 96;  //set 80uS, which is 160 counts. load with 256-160 = 96
	/*if we were timing low part, resolve data bit*/
	INTEDG = ~INTEDG;

	if (INTEDG==1){
	/* Effectively processing on what was the falling edge
	rotate LMB and RMB and rotate in new bit.  Doing this in ASM is 13 instr versus 55 from compiled C */
#asm
	/*clear carry. If TMR2IF is clear then set carry for a 1 bit
	both STATUS and INTCON registers appear in all banks, PIR1 only appears in bank 0
	asm is 13 instr versus 55 from compiled C*/
	BCF	(STATUS),0     //assume a zero bit
//	BANKSEL(PIR1)
//	BTFSS (PIR1),1   //did TMR2 timeout?
	BTFSS (INTCON),2   //did TMR0 timeout?
	BSF (STATUS),0     //no, so was a 1 detected
	BANKSEL(_shiftRMB)
	RLF (_shiftRMB)
	RLF (_shiftRMB+1)
	BANKSEL(_shiftLMB)
	RLF (_shiftLMB)
	RLF (_shiftLMB+1)
	RLF (_shiftLMB+2)
	RLF (_shiftLMB+3)
#endasm
	/*approx 50 instr to this point*/

		/*preamble and first 3 zero markers good?  If yes we have a packet for processing. another 47 instr 
		if we don't stop looking for bits, then the main routine has until the next packet detected, approx 40 bits of average length 160uS= 6400 uS to process 
		the existing packet before the packet buffer is over written.
		This test is approx 30 instr.  Copying to packetXMB is another 20 instr
		so most ints will see a longest cost path of 70 instr	
		*/
		//if ((shiftLMB & 0xFFF00804) == 0xFFE00000){
		if ((shiftLMB & 0xFFE01008) == 0xFFC00000){
			packetLMB=shiftLMB;
			packetRMB=shiftRMB;
			packetRX=1;
		}
	}
	/*on way out reset int flags*/
	TMR0IF=0;
	INTF=0;
}



}//end ISR



/*pulse solenoids to achieve routing into siding 1,2 or 3
firing orders are below, with the second solenoid being fired after the recharge period;
siding 1:  sol1=n sol2=t (s2/3 isolated, but s2 is preselected)
siding 2:  sol1=t sol2=t (s1 isolated first, s2 stays as is)
siding 3:  sol2=n sol1=t (s2 needs to snap to 3, s1 stays as is)

call sendPulse with the siding of choice in variable s being 1,2 or 3.  the doPulse flag has to be set beforehand

the routine has an internal state engine and will fire all 4 solenoids as first pair, second pair rather than all 4 at once.  Once it has fired the second pair
it will clear the doPulse flag

2020-06-07 added code to set relay states.
thrown condition of a solenoid will activate its relay.
TIMING is not critical, because our relay is simply acting as an additional current path to the point-rail selector, and each point rail will energise a separate
track section. if the relay cuts out later than the point rail breaks, then the 'old' section is energised just fractionally longer than it would have been.
if the relay cuts-in fractionally sooner, no problem, it means the yet-to-close point rail is energised through the relay, but when it closes the polarity will be correct.
What is IMPORTANT is we use dpdt relays, because we need one set of contacts for each point-rail.  This is NOT the same as power-routing a frog.  We actually need the entire
siding to be de/energised based on the point position. this is what stops the train (provided it does not have so much programmed inertia it rolls through).


*/

void sendPulse(uint8_t s){
//call with s=1,2,3
//routine will take that as an intiator if doPulse=1
//pulseTimeout and rechargeTimeout both have to be zero before the state engine will take action.
//otherwise it will do nothing

//Note that rechargeTimeout is set in the main loop as pulseTimeout decays

//exit now if neither of these has decayed to zero
if (pulseTimeout!=0){return;}
if (rechargeTimeout!=0){return;}

switch(stateSiding){

case SIDING1_STEP2:
	//wait for first step to finish, then action the second
	pulseTimeout=T_PULSE;
	stateSiding=SIDING_DONE;
	solenoid2thrown=1;   //second solenoid step
	relay2=1;
	dopulse=0;
	break;

case SIDING2_STEP2:
	//2015-10-04 s2 fired second. its optional
	pulseTimeout=T_PULSE;
	stateSiding=SIDING_DONE;
	solenoid2thrown=1;   //second solenoid step
	relay2=1;
	dopulse=0;	
	break;	

case SIDING3_STEP2:
	//2015-10-04 s1 fired second. its optional
	pulseTimeout=T_PULSE;
	stateSiding=SIDING_DONE;
	solenoid1thrown=1;
	relay1=1;
	dopulse=0;
	break;

default:
	//initiation (i.e. selecting route 1,2 or 3)
	//also we are 'listening' here if stateSiding=DONE
	//we will only initate if dopulse=1

	if (dopulse!=1){return;}
	//initiate a pulse, per specified route
	pulseTimeout=T_PULSE;

	switch (s){
	case 1:
			solenoid1normal=1;   //active high
			relay1=0;
			stateSiding=SIDING1_STEP2;
			break;
	case 2:
			//2015-10-04 order is correct s1 isolated first
			solenoid1thrown=1;   //active high
			relay1=1;
			stateSiding=SIDING2_STEP2;
			break;
	case 3:
			//2015-10-04 s2 needs to be isolated first
			solenoid2normal=1;   //active high
			relay2=0;
			stateSiding=SIDING3_STEP2;
			break;
	}
	return;  //iniation complete
}

}





/*************************************************************
DCC decode routine

3 byte packet
1111111111 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1

4 byte packet
1111111111 0 CCCCCCCC 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1

so a 4 byte packed is detected by a 0 in separator pos 3
3 byte packet is preamble 0 A 0 D 0 E 1
4 byte packet is premable 0 A 0 C 0 D 0 E 1 so is detected by a 0 after the 3rd byte

we can just apply one mask to find a valid 3 or 4 stream in the LMB register
11111111111000000001000000001000 = FFE01008   mask
11111111110000000000000000000000 = FFC00000  expected
we don't check any of the bits in RMB until later, so above assumes its unlikely we will detect preamble and
three of the zero markers by mistake

to find individual bytes within a long long or a long
http://www.microchip.com/forums/m160493.aspx you might be able to access parts of the 32bit int this way

*/ 

void DCCdecode(void){
packetGood=0;
packetRX=0;

/*test, do we see preamble?  ,, 
//FFC0 pre only OK
// pre and a zero FFE0 OK
// FFE0 1000 pre and two zeros FAIL

http://www.dccwiki.com/Digital_packet   address is MSB last
1111111111 0 0AAAAAAA 0 01DUSSSS 0 EEEEEEEE 1
1111111111100000000100000000000 

http://www.opendcc.de/elektronik/opendcc/opendcc_sw_lenz_e.html
1s consist of two 58uS transitions, zeros consist of two >95 transitions thus total len is >116

NMRA general packet format
http://www.nmra.org/sites/default/files/s-92-2004-07.pdf

A decoder reset packet is 3 bytes, all zero
Service mode can be entered within 20mS, if that is the intent

Basic accessory packet, 9bit address, however it effectively goes up in 8s because CDDD.  D<0> selects which of the 'pair' of devices at the sub-address D<1-2> is being
addressed, and C indicates whether we are activating it or deactivating it (0). AAA of the 2nd byte are the MSB of the 9 bit address and are two's complemented.
Therefore, turnouts 0-3 are actually all on the same A-9-bit address, and are distinguished by D<1-2>.

{preamble}  0  10AAAAAA  0  1AAACDDD  0  EEEEEEEE  1


0xFFD54DD7 = 11111111110101010100110111010111
0x7400 = 111010000000000
FFE01008 =   1111111111 1 00000000 1 00000000 1 000
FFC00000 =   1111111111 0 00000000 0 00000000 0 000?


THIS ROUTINE is non destructive on the bit shift registers, however it will be confused if the shift registers change whilst its examining them
to stop this happening, we'd need to do all this processing immediately after an int.  if we leave it too late, an int will likely hit during this decode
routine

its approx 220 instr, which is 44uS so we are running short on int time during a 1 to process it.

previous approach was to just alter the last bit of shiftRMB in the int, return and ask the main loop to process it.  however this still has problems if the main loop
was doing something else and does not pick up the bit in time... means we'd have an overrun situation and lost bits.

YES:  One fix is to use the int to also check the preamble + first 3 zero markers.  If this is good, copy the shift buffers to a mirror buffer, and flag to the main
loop that a packet was found which needs processing, meanwhile the int carries on shifting.  3/4 byte detection can happen on the buffer.
*/



//preamble and first 3 zero markers good?
if ((packetLMB & 0xFFE01008) ==0xFFC00000)
{
	/* check for 3 or 4 byte packet */
	//3 byte packets:
	//1111111111 0 AAAAAAAA 0 DDDDDDDD 0 EEE  EEEEE 10000000000
	//1111111111 0 CCCCCCCC 0 AAAAAAAA 0 DDD  DDDDD 0 EEEEEEEE 1x

//1111111111 0 CCCCCCCC 0 AAAAAAAA 0 DDD      DDDDD 0 EEEEEEEE 1x


	if ((packetRMB & 0x400) == 0x400)
	{//3 byte packet
	data1= packetLMB >>13;
	data2 = packetLMB >>4;
	data3 = packetLMB <<5;  //in theory only picks up LSD
	data3+ = packetRMB>>11;
	data4=0;	
	packet4byte=0;

	}
else
	{//4 byte
	packet4byte=1;
	data1= packetLMB >>13;
	data2 = packetLMB >>4;
	data3 = packetLMB <<5;  //in theory only picks up LSD
	data3+ = packetRMB>>11;
	data4=  packetRMB>>2;
	}

/* XOR checksum */
uint8_t x = data1;
x ^= data2;
x ^= data3;
x ^= data4;
if (x==0){packetGood=1;}
}

return;
}

