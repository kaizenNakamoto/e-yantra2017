/*********************************************************************************************************/



//                                         	  FILE LEVEL COMMENTS                                        //



/*********************************************************************************************************/

/*

* Team ID:			Harvester Bot-eYRC#492

* Authors' List:    Karthik S Nayak
                    Kiran R S
					Chinmai R
					Mithun M R
s
* Filename: 		eYRC-HB-492.c

* Theme: 			Harvester Bot(eYRC 2017)

* Functions: 		Indicator Functions: buzzer_pin_config(), buzzer_on(), lcd_port_config(), led_pin_config(),

					buzzer_off(), buzz(), led_off()



					ADC Functions: adc_pin_config(), adc_init(), ADC_Conversion(unsigned char),

					print_sensor(char, char, unsigned char)



					Timer Functions: timer1_init(), timer3_init(), timer4_init(), timer5_init(), start_timer4(),

					stop_timer4(), ISR(TIMER3_OVF_vect), ISR(TIMER4_OVF_vect)



					Motion Functions: motion_pin_config(), left_encoder_pin_config(), right_encoder_pin_config(),

					left_position_encoder_interrupt_init(), right_position_encoder_interrupt_init(), ISR(INT4_vect),

					ISR(INT5_vect), velocity(unsigned char, unsigned char), motion_set(unsigned char), forward(), 

					back(), left(), right(), soft_left(), soft_left_2(), soft_right(), soft_right_2(), stop(),

					angle_rotate(unsigned int), linear_distance_mm(unsigned int), forward_mm(unsigned int),

					back_mm(unsigned int), left_degrees(unsigned int), right_degrees(unsigned int) 

					soft_left_degrees(unsigned int), soft_right_degrees(unsigned int), soft_left_2_degrees(unsigned int),

					soft_right_2_degrees(unsigned int)



					Servomotor Functions: servo1_pin_config(), servo2_pin_config(), servo3_pin_config(),

					servo_1(unsigned char), servo_2(unsigned char), servo_3(unsigned char), servo_1_free(), servo_2_free(),

					servo_3_free(), gripper_open(), gripper_close(), arm_rotate(), arm_initial(), cycle(), door_open(), door_close()

					
					Line Follower Functions: read_line(), display_line(), enable_flags(), clear_count(), search_line(),

					line_conditions(), follow_line(), line_follower(), correct_left(int), correct_right(int), correct_right2(int), correct_left2(int)
					
					
					Path Traversal Functions: setb(), set_dir(), set_dir2(char), set_dir3(char), assignTree(), flood_fill(), flood_fillz(int, int), flood(int, int), pathchk(),
					
					align(), trace(), align2() 


					Fruit Pluck and Drop Mechanism Functions: pluckDrop(int), setDrop(int)

					
					Initiation Functions: port_init(), init_devices()


					Main Function: main()
					
*/
					
/*********************************************************************************************************/



//                                         HEADERS AND DECLARATIONS                                      //



/*********************************************************************************************************/

//define constants
#define F_CPU 14745600
#define maxs 49

//Header files
# include <avr/io.h> 
#include <avr/interrupt.h> 
#include <util/delay.h> 
#include <stdio.h> 
# include <math.h> //included to support power function
#include "lcd.c"

//Function Prototypes
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void stop_timer4();
void read_line();
void line_conditions();
void flood_fill();
void flood_fillz();
void set_dir();
void set_dir2(char algdir);
void set_dir3(char algdir);
void handle_nodes();
void setb();
void pluckDrop(int Count);
void setDrop(int node1);
void uturn();
void align2(int pdxx,int pdyy);
int flood(int x, int y);
void pathchk(int ex, int ey);
void align();
void trace();
int main();
/*********************************************************************************************************/



//                                             GLOBAL VARIABLES                                          //



/*********************************************************************************************************/

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
volatile unsigned char Left_white_line = 0;
volatile unsigned char Center_white_line = 0;
volatile unsigned char Right_white_line = 0;

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

volatile int line_cond = 0;
volatile int en_left = 0, en_right = 0, en_node = 1;
volatile int left_flag = 0, right_flag = 0, node_flag;
volatile int left_turn = 0, right_turn = 0;
volatile int t4_flag = 0, node = 0;
volatile int t4_sec = 0;
volatile int t4_count;
int master_flag=1;

int stacks[maxs];
int path[maxs];
int c = 0, x, y, cnt = 0;
int m = 7, n = 7, ll = 0;
int start=0,end=48;
int i, j, min = 999, ex = 0, md=0,ey = 0,mjj=0,we=0, top = 0, d = 0, sdd=0,q = 0, px = 0, py = 0;
int nx = 0, ny = 0,sx=0,sy=0,node1=0,Count=0,fx=0,fy=0,tx=0,ty=0 ,eex, eey, ndx = 0, ndy = 0, z = 0,qw=0,pxx=0,pyy=0,pdxx=0,pdyy=0,ndxx=-1,ndyy=0;
int cot = 0, pdx = -1, pdy = 0,stopCount=0;//pdx is prev x direction pdy is previous y direction.
char dir[maxs];
char hardc[16]={'d','r','l','l','d','r','l','l','d','r','l','l','d','r','l','l'};
char master_dir[200];
int a[7][7],b[7][7]={
{100,100,100,100,100,100,100},
{100,100,100,100,100,100,100},
{100,100,100,100,100,100,100},
{100,100,100,100,100,100,100},
{100,100,100,100,100,100,100},
{100,100,100,100,100,100,100},
{100,100,100,100,100,100,100}};
int tree[3]={9,29,18};
int depZone[12]={37, 38, 44, 45,35, 36, 42, 43,40, 41, 47, 48};
int deps[4]={0,38,36,41};


/*********************************************************************************************************/



//                                                INDICATORS                                             //



/*********************************************************************************************************/

/*

* Function Name: 	lcd_port_config

* Input: 			None

* Output: 			None

* Logic: 			Function to configure LCD port (PORT C)

* Example Call:		lcd_port_config();

*/
void lcd_port_config(void)
 {
    DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
    PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/*

* Function Name: 	buzzer_pin_config

* Input: 			None

* Output: 			None

* Logic: 			Function to initialize Buzzer connected to PORTC 3

* Example Call:		buzzer_pin_config();

*/
void buzzer_pin_config(void) 
{
	DDRC = DDRC | 0x08; //Setting PORTC 3 as output
	PORTC = PORTC & 0xF7; //Setting PORTC 3 logic low to turnoff buzzer
}

/*

* Function Name: 	buzzer_on

* Input: 			None

* Output: 			None

* Logic: 			Function to switch buzzer on, PORTC 3 pin is set high

* Example Call:		buzzer_on();

*/
void buzzer_on(void) 
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

/*

* Function Name: 	buzzer_off

* Input: 			None

* Output: 			None

* Logic: 			Function to switch buzzer off, PORTC 3 pin is set low

* Example Call:		buzzer_off();

*/
void buzzer_off(void) 
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

/*********************************************************************************************************/



//                                               ADC FUNCTIONS                                           //



/*********************************************************************************************************/

/*

* Function Name: 	adc_pin_config

* Input: 			None

* Output: 			None

* Logic: 			PORTS F AND K are configured to enable ADC conversion

* Example Call:		adc_pin_config ();

*/
void adc_pin_config(void)
{
    DDRF = 0x00;
    PORTF = 0x00;
    DDRK = 0x00;
    PORTK = 0x00;
}

/*

* Function Name: 	adc_init

* Input: 			None

* Output: 			None

* Logic: 			The registers required for ADC conversion are configured

* Example Call:		adc_init();

*/
void adc_init() 
{
	ADCSRA = 0x00;
	ADCSRB = 0x00; //MUX5 = 0
	ADMUX = 0x20; //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86; //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*

* Function Name: 	ADC_Conversion	

* Input: 			Ch (unsigned char) -> Channel Number



					ADC CH.     PORT        Sensor

                  	0 			PF0 		Battery Voltage

                  	1 			PF1 		White line sensor 3

                  	2 			PF2 		White line sensor 2

                  	3 			PF3 		White line sensor 1

                  	4 			PF4 		IR Proximity analog sensor 1

                  	5 			PF5 		IR Proximity analog sensor 2

                  	6 			PF6 		IR Proximity analog sensor 3

                  	7 			PF7 		IR Proximity analog sensor 4

                  	8 			PK0 		IR Proximity analog sensor 5

                  	9 			PK1			Sharp IR range sensor 1

                  	10 			PK2 		Sharp IR range sensor 2

                  	11 			PK3 		Sharp IR range sensor 3

                  	12 			PK4 		Sharp IR range sensor 4

                  	13 			PK5 		Sharp IR range sensor 5



* Output: 			a (unsigned char) -> Returns corresponding ADC value

* Logic: 			Reads data from the sensors and performs analog to digital

 					conversion by using two registers ADCSRA and ADCSRB

* Example Call:		value = ADC_Conversion(11);

*/
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if (Ch > 7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX = 0x20 | Ch;
	ADCSRA = ADCSRA | 0x40; //Set start conversion bit
	while ((ADCSRA & 0x10) == 0); //Wait for conversion to complete
	a = ADCH;
	ADCSRA = ADCSRA | 0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/*

* Function Name: 	print_sensor	

* Input: 			row (char) -> LCD row

					column(char) -> LCD column

					channel (unsigned char) -> ADC Channel Number

* Output: 			None

* Logic: 			Function To Print Sensor Values At Desired Row And Column Location on LCD

* Example Call:		print_sensor(1, 1, 11);

*/
void print_sensor(char row, char coloumn, unsigned char channel)
 {

	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/*********************************************************************************************************/



//                                            TIMER FUNCTIONS                                            //



/*********************************************************************************************************/

/*

* Function Name: 	timer1_init	

* Input: 			None

* Output: 			None

* Logic: 			Function to initiate timer 1 for Servo control

					- TIMER1 initialization in 10 bit fast PWM mode  

					- Pre-scaler:256

					- WGM: 7) PWM 10bit fast, TOP=0x03FF

					- Actual value: 52.25Hz 

* Example Call:		timer1_init();

*/
void timer1_init()
{
	TCCR1A = 0X00;
	ICR1 = 1023;
	TCNT1H = 0XFC;
	TCNT1L = 0X01;
	OCR1AH = 0x03;	//Output compare Register high value for servo 1
	OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
	OCR1BH = 0x03;	//Output compare Register high value for servo 2
	OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
	OCR1CH = 0x03;	//Output compare Register high value for servo 3
	OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
	ICR1H  = 0x03;
	ICR1L  = 0xFF;
	TCCR1A = 0XAB;
	TCCR1B = 0X0C;
	TCCR1C = 0x00;
}

/*

* Function Name: 	timer3_init	

* Input: 			None

* Output: 			None

* Logic: 			Function to initiate timer 3 for Line Sensor Readings

					- TIMER3 initialize -> pre-scaler:256

					- WGM: 0) Normal, TOP=0xFFFF

					- Desired value: 100Hz

					- Actual value:  100.000Hz (0.0%)

* Example Call:		timer3_init();

*/
void timer3_init(void)
{
	TCCR3B = 0x00; //stop
	TCNT3 = 0xFDC0; // 0.01s
	OCR3AH = 0x00; //Output Compare Register (OCR)- Not used
	OCR3AL = 0x00; //Output Compare Register (OCR)- Not used
	OCR3BH = 0x00; //Output Compare Register (OCR)- Not used
	OCR3BL = 0x00; //Output Compare Register (OCR)- Not used
	OCR3CH = 0x00; //Output Compare Register (OCR)- Not used
	OCR3CL = 0x00; //Output Compare Register (OCR)- Not used
	ICR3H = 0x00; //Input Capture Register (ICR)- Not used
	ICR3L = 0x00; //Input Capture Register (ICR)- Not used
	TCCR3A = 0x00;
	TCCR3C = 0x00;
	TCCR3B = 0x04; //Pre-scaler 256 1-0-0
}

/*

* Function Name: 	timer4_init	

* Input: 			None

* Output: 			None

* Logic: 			Function to initiate timer 4 for Timing Operations

					- TIMER4 initialize -> Pre-scaler:256

					- WGM: 0 Normal, TOP=0xFFFF

					- Desired value: 10Hz

					- Actual value:  10.000Hz (0.0%)

* Example Call:		timer4_init();

*/
void timer4_init(void) 
{
	TCCR4B = 0x00; //stop
	TCNT4 = 0xE980; // 0.1s
	OCR4AH = 0x00; //Output Compare Register (OCR)- Not used
	OCR4AL = 0x00; //Output Compare Register (OCR)- Not used
	OCR4BH = 0x00; //Output Compare Register (OCR)- Not used
	OCR4BL = 0x00; //Output Compare Register (OCR)- Not used
	OCR4CH = 0x00; //Output Compare Register (OCR)- Not used
	OCR4CL = 0x00; //Output Compare Register (OCR)- Not used
	ICR4H = 0x00; //Input Capture Register (ICR)- Not used
	ICR4L = 0x00; //Input Capture Register (ICR)- Not used
	TCCR4A = 0x00;
	TCCR4C = 0x00;
	TCCR4B = 0x04; //Pre-scaler 256 1-0-0
}

/*

* Function Name: 	timer5_init	

* Input: 			None

* Output: 			None

* Logic: 			Function to initiate timer 5

					- Timer 5 initialized in PWM mode for velocity control

					- Pre-scaler:256

					- PWM 8bit fast, TOP=0x00FF

					- Timer Frequency:225.000Hz

* Example Call:		timer5_init();

*/
void timer5_init()
{
    TCCR5B = 0x00; //Stop
    TCNT5H = 0xFF; //Counter higher 8-bit value to which OCR5xH value is compared with
    TCNT5L = 0x01; //Counter lower 8-bit value to which OCR5xH value is compared with
    OCR5AH = 0x00; //Output compare register high value for Left Motor
    OCR5AL = 0xFF; //Output compare register low value for Left Motor
    OCR5BH = 0x00; //Output compare register high value for Right Motor
    OCR5BL = 0xFF; //Output compare register low value for Right Motor
    OCR5CH = 0x00; //Output compare register high value for Motor C1
    OCR5CL = 0xFF; //Output compare register low value for Motor C1
    TCCR5A = 0xA9;
    /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

    TCCR5B = 0x0B; //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*********************************************************************************************************/



//                                           MOTION FUNCTIONS                                            //



/*********************************************************************************************************/

/*

* Function Name: 	motion_pin_config

* Input: 			None

* Output: 			None

* Logic: 			Function to configure PORT A and PORT L to enable robot's motion

* Example Call:		motion_pin_config();

*/
void motion_pin_config(void)
 {
    DDRA = DDRA | 0x0F;
    PORTA = PORTA & 0xF0;
    DDRL = DDRL | 0x18; //Setting PL3 and PL4 pins as output for PWM generation
    PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*

* Function Name: 	left_encoder_pin_config

* Input: 			None

* Output: 			None

* Logic: 			Function to configure INT4 (PORTE 4) pin as input for the left position encoder

* Example Call:		left_encoder_pin_config();

*/
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*

* Function Name: 	right_encoder_pin_config

* Input: 			None

* Output: 			None

* Logic: 			Function to configure INT5 (PORTE 5) pin as input for the right position encoder

* Example Call:		right_encoder_pin_config();

*/
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/*

* Function Name: 	velocity

* Input: 			left_motor (unsigned char) -> Left motor velocity

                    right_motor (unsigned char) -> Right motor velocity

* Output: 			None

* Logic: 			PWM is used. 255 (0xFF)is set as the maximum value of the timer. When the count of the timer exceeds

255 the timer overflows. The TCNT and OCR5_n registers are compared and on match the timer overflows.

For 100% Duty cycle, the value is 255. Hence to vary the duty cycle we can give any value in the range 0-255.

* Example Call:		velocity(255,255);

*/
void velocity(unsigned char left_motor, unsigned char right_motor) 
{
    OCR5AL = (unsigned char) left_motor;
    OCR5BL = (unsigned char) right_motor;
}

/*

* Function Name: 	left_position_encoder_interrupt_init

* Input: 			None

* Output: 			None

* Logic: 			Interrupt 4 enable

* Example Call:		left_position_encoder_interrupt_init();

*/
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

/*

* Function Name: 	right_position_encoder_interrupt_init

* Input: 			None

* Output: 			None

* Logic: 			Interrupt 5 enable

* Example Call:		right_position_encoder_interrupt_init();

*/
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

/*

* Function Name: 	ISR

* Input: 			INT5_vect

* Output: 			None

* Logic: 			ISR for right position encoder, increment right shaft position count

* Example Call:		Called automatically by interrupt

*/
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}

/*

* Function Name: 	ISR

* Input: 			INT4_vect

* Output: 			None

* Logic: 			ISR for left position encoder, increment left shaft position count

* Example Call:		Called automatically by interrupt

*/
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

/*

* Function Name: 	motion_set	

* Input: 			Direction (unsigned char) -> Direction HEX

					- 0x06 --> Forward	

                    - 0x09 --> Backward

                    - 0x05 --> Left

                    - 0x0A --> Right

                    - 0x04 --> Soft Left

                    - 0x02 --> Soft right

                    - 0x01 --> Soft Left 2 (Reverse logic of soft left)

                    - 0x08 --> Soft Right 2 (Reverse logic of soft right)

                    - 0x00 --> Stop

* Output: 			None

* Logic: 			Assigns the motor to move in specified direction. The motor consists of two inputs.

					Based on polarity of the inputs, it rotates clockwise or anticlockwise.

* Example Call:		motion_set(0x01);

*/
void motion_set(unsigned char Direction)
 {
    unsigned char PortARestore = 0;

    Direction &= 0x0F; // removing upper nibble for protection
    PortARestore = PORTA; // reading PORTA's original status
    PortARestore &= 0xF0; // making lower direction nibble to 0
    PortARestore |= Direction; // adding lower nibble for forward command and restoring PORTA's status
    PORTA = PortARestore; // executing the command
}

/*

* Function Name: 	forward

* Input: 			None

* Output: 			None

* Logic: 			Both wheels forward

* Example Call:		forward();

*/
void forward(void)
 {
    motion_set(0x06);
}

/*

* Function Name: 	back

* Input: 			None

* Output: 			None

* Logic: 			Both wheels backward

* Example Call:		back();

*/
void back (void)
{
	motion_set(0x09);
}

/*

* Function Name: 	left

* Input: 			None

* Output: 			None

* Logic: 			Left wheel backward, Right wheel forward

* Example Call:		left();

*/
void left (void)
{
	motion_set(0x05);
}

/*

* Function Name: 	right

* Input: 			None

* Output: 			None

* Logic: 			Left wheel forward, Right wheel backward

* Example Call:		right();

*/
void right (void)
{
	motion_set(0x0A);
}

/*

* Function Name: 	soft_left

* Input: 			None

* Output: 			None

* Logic: 			Left wheel stationary, Right wheel forward

* Example Call:		soft_left();

*/
void soft_left (void)
{
	motion_set(0x04);
}

/*

* Function Name: 	soft_right

* Input: 			None

* Output: 			None

* Logic: 			Left wheel forward, Right wheel is stationary

* Example Call:		soft_right();

*/
void soft_right (void) 
{
	motion_set(0x02);
}

/*

* Function Name: 	soft_left_2

* Input: 			None

* Output: 			None

* Logic: 			Left wheel backward, right wheel stationary

* Example Call:		soft_left_2();

*/
void soft_left_2 (void) 
{
	motion_set(0x01);
}

/*

* Function Name: 	soft_right_2

* Input: 			None

* Output: 			None

* Logic: 			Left wheel stationary, Right wheel backward

* Example Call:		soft_right_2();

*/
void soft_right_2 (void)
{
	motion_set(0x08);
}

/*

* Function Name: 	stop

* Input: 			None

* Output: 			None

* Logic: 			Both wheels stop

* Example Call:		stop():

*/
void stop(void)
{
	motion_set(0x00);
}

/*

* Function Name: 	angle_rotate	

* Input: 			Degrees (unsigned int) -> Rotate by specified degrees

* Output: 			None

* Logic: 			The robot moves 4.090 degrees per count. Hence, it calculates required shaft count by 

					dividing by 4.090 for moving by specified degrees.

* Example Call:		angle_rotate(180);

*/
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

/*

* Function Name: 	linear_distance_mm	

* Input: 			DistanceInMM (unsigned int) -> Traverse by specified distance

* Output: 			None

* Logic: 			The robot moves 5.338mm per pulse. Hence,it calculates required shaft count by dividing by 

					5.338 to move the specified distance.

* Example Call:		linear_distance_mm(100);

*/
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

/*

* Function Name: 	forward_mm

* Input: 			DistanceInMM (unsigned int) -> Traverse by specified distance

* Output: 			None

* Logic: 			The robot moves forward for the specified distance

* Example Call:		forward_mm(100);

*/
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

/*

* Function Name: 	back_mm

* Input: 			DistanceInMM (unsigned int) -> Traverse by specified distance

* Output: 			None

* Logic: 			The robot moves backward for the specified distance

* Example Call:		back_mm(100);

*/
void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

/*

* Function Name: 	left_degrees

* Input: 			Degrees (unsigned int) -> Rotate by specified degrees

* Output: 			None

* Logic: 			The robot rotates left for the specified degrees

* Example Call:		left_degrees(90);

*/
void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

/*

* Function Name: 	right_degrees

* Input: 			Degrees (unsigned int) -> Rotate by specified degrees

* Output: 			None

* Logic: 			The robot rotates right for the specified degrees

* Example Call:		right_degrees(90);

*/
void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}

/*

* Function Name: 	soft_left_degrees

* Input: 			Degrees (unsigned int) -> Rotate by specified degrees

* Output: 			None

* Logic: 			The robot takes a soft left for the specified degrees

* Example Call:		soft_left_degrees(90);

*/
void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*

* Function Name: 	soft_right_degrees

* Input: 			Degrees (unsigned int) -> Rotate by specified degrees

* Output: 			None

* Logic: 			The robot takes a soft right for the specified degrees

* Example Call:		soft_right_degrees(90);

*/
void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*

* Function Name: 	soft_left_2_degrees

* Input: 			Degrees (unsigned int) -> Rotate by specified degrees

* Output: 			None

* Logic: 			The robot takes a reverse soft left for the specified degrees

* Example Call:		soft_left_2_degrees(90);

*/
void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*

* Function Name: 	soft_right_2_degrees

* Input: 			Degrees (unsigned int) -> Rotate by specified degrees

* Output: 			None

* Logic: 			The robot takes a reverse soft right for the specified degrees

* Example Call:		soft_right_2_degrees(90);

*/
void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*

* Function Name:   	correct_right	

* Input: 			None			

* Output: 			None	

* Logic:			Keep rotating right till center sensor see line

					Keep rotating right till right sensor see line
					
* Example Call:		correct_right();

*/
void correct_right()
{
	TIMSK3=0x00;
	forward_mm(50);
	_delay_ms(100);
	right_degrees(30);
	right();
	while(1)
	{   
		read_line();
		if (Center_white_line>12)
		{
			stop();
			_delay_ms(100);
			break;
		}
	}
	TIMSK3 = 0x01;
}

/*

* Function Name:   	correct_right2	

* Input: 			None			

* Output: 			None	

* Logic:			Keep rotating right till center sensor see line

					Keep rotating right till right sensor see line
					
* Example Call:		correct_right2();

*/
void correct_right2()
{      
	TIMSK3=0x00;
	back_mm(3);
	_delay_ms(100);
	right_degrees(30);
	right();
	while(1)
	{
		read_line();
		if (Center_white_line>12)
		{
			stop();
			_delay_ms(100);
			break;
		}
	}
	TIMSK3 = 0x01;
}

/*

* Function Name:   	correct_left	

* Input: 			None			

* Output: 			None	

* Logic:			Keep rotating left till center sensor sees line

					Keep rotating left till left sensor sees line

* Example Call:		correct_left();

*/
void correct_left()
{   
	TIMSK3=0x00;
	forward_mm(50);
	_delay_ms(100);
	left_degrees(30);
	left();
	while(1)
	{   
		read_line();
		if (Center_white_line>12)
		{
			stop();
			_delay_ms(100);
			break;
		}
	}
	TIMSK3 = 0x01;
}

/*

* Function Name:   	correct_left2	

* Input: 			None			

* Output: 			None	

* Logic:			Keep rotating right till center sensor see line

					Keep rotating right till right sensor see line
					
* Example Call:		correct_left2();

*/
void correct_left2()
{   
	TIMSK3=0x00;
	back_mm(3);
	_delay_ms(100);
	left_degrees(30);
	left();
	while(1)
	{   read_line();
		if (Center_white_line>12)
		{
			stop();
			_delay_ms(100);
			break;
		}
	}
	TIMSK3 = 0x01;
}

/*

* Function Name:   	uturn	

* Input: 			None			

* Output: 			None	

* Logic:			Take a U-turn
					
* Example Call:		uturn();

*/
void uturn()
{
	right_degrees(180);
}

/*********************************************************************************************************/



//                                       SENSOR READING FUNCTIONS                                        //



/*********************************************************************************************************/



/*

* Function Name:	ISR 	

* Input: 			TIMER3_OVF_vect			

* Output: 			None

* Logic:			- Timer 3 ISR is used to update Line Sensor Readings at 100Hz

					- Based on readings, determine the line condition

* Example Call:		Called automatically by timer interrupt

*/
ISR(TIMER3_OVF_vect)
{
    read_line();
    line_conditions();
    TCNT3 = 0xFDC0;
}

/*

* Function Name: 	ISR	

* Input: 			TIMER4_OVF_vect	

* Output: 			None

* Logic:			- Timer 4 ISR is used for timing operations at a resolution of 10Hz

					- t4_sec is incremented every 0.1 sec until it is equal to required t4_count

* Example Call:		Called automatically by timer interrupt

*/
ISR(TIMER4_OVF_vect)
{
    TCNT4 = 0xE980;
    t4_sec++; //Increment every 0.1 sec

    if (t4_sec == t4_count && t4_flag == 1)
        stop_timer4();
}

/*

* Function Name: 	start_timer4	

* Input: 			count (int) -> Required number of time units in 0.1 sec to count

* Output: 			None

* Logic:			- Enable Timer 4 overflow interrupt

					- Initialize Timer 4 flags (global variables)

* Example Call:		start_timer4(20); -> This will count for 2 sec and stop

*/
void start_timer4(int count)
{
    t4_flag = 1; //Indicate timer 4 is enabled
    t4_sec = 0;
    t4_count = count + 1;
    TIMSK4 = 0x01; //Timer 4 overflow interrupt enable
}

/*

* Function Name: 	stop_timer4	

* Input: 			None

* Output: 			None

* Logic:			- If t4_sec reaches required t4_count, stop timer 4

					- Disable Timer 4 overflow interrupt

					- Also used for turning off buzzer if it is on

* Example Call:		stop_timer4();

*/
void stop_timer4()
{
	t4_flag = 0;
	t4_sec = 0;
	TIMSK4 = 0x00; //Timer4 overflow interrupt disable
}



/**********************************************************************************************************/



//                                         LINE FOLLOWER FUNCTIONS                                         //



/**********************************************************************************************************/



/*

* Function Name: 	read_line

* Input: 			None

* Output: 			None

* Logic:			Read left, center and right white line sensor values and store in global variables

* Example Call:		read_line();

*/
void read_line()
{
    Left_white_line = (int) ADC_Conversion(3); //Getting data of Left WL Sensor
    Center_white_line = (int) ADC_Conversion(2); //Getting data of Center WL Sensor
    Right_white_line = (int) ADC_Conversion(1); //Getting data of Right WL Sensor
}

/*

* Function Name: 	display_line

* Input: 			None

* Output: 			None

* Logic:			Display line sensor readings on LCD

* Example Call:		display_line();

*/
void display_line()
 {
    lcd_print(1,1,Left_white_line,3);
    lcd_print(1,5,Center_white_line,3);
    lcd_print(1,9,Right_white_line,3);
    lcd_print(1,1,line_cond,1);
}

/*

* Function Name: 	clear_count

* Input: 			None

* Output: 			None

* Logic:			Clear all line follower flags and turn/node counts

* Example Call:		clear_count();

*/
void clear_count()
{
    node = 0;
    node_flag = 0;
    left_flag = 0;
    right_flag = 0;
}

/*

* Function Name:	enable_flags 	

* Input: 			l_flag (int) -> 0 - Disable / 1 -> Enable

					n_flag (int) -> 0 - Disable / 1 -> Enable

					r_flag (int) -> 0 - Disable / 1 -> Enable 

* Output: 			None

* Logic:			- Function to enable/disable left/right turn or node detection

* Example Call:		enable_flags(0,1,0); -> Detect only nodes, ignore left and right turns

*/
void enable_flags(int l_flag, int n_flag, int r_flag)
{
    en_left = l_flag;
    en_right = r_flag;
    en_node = n_flag;
}

/*

* Function Name: 	line_conditions	

* Input: 			None

* Output: 			None

* Logic:			- Based on the position of robot on line, the line condition flag is set

					- NOTE: This functions only checks the condition and sets the line follower flags, doesn't follow the line

					- Line following and Line detection are done separately, in order to increase the sampling rate of the 

					  sensors so that the sensors do not miss out nodes and turns when the robot is moving at higher velocities

* Example Call:		line_conditions();	

*/
void line_conditions()
{
	// Nodes Condition
	if (Center_white_line >= 100 || Left_white_line >= 100 || Right_white_line >= 100 || (Center_white_line >=85 && (Left_white_line >= 75 || Right_white_line >= 75))) // 1 1 1
	{
		line_cond = 1;
		node_flag = 1;
	}

	//Straight Line Condition
	else if (Center_white_line >= 11 && Left_white_line < 15 && Right_white_line < 15) // 0 1 0
	{
		line_cond = 2;
	}

	//Straight Line Condition (Robot moved slightly towards right)
	else if (Left_white_line >= 11 && Center_white_line < 15 && Right_white_line < 15) // 1 0 0
	{
		line_cond = 3;
	}

	//Straight Line Condition (Robot moved slightly towards left)
	else if (Right_white_line >= 11 && Center_white_line < 15 && Left_white_line < 15) // 0 0 1
	{
		line_cond = 4;
	}

	//No Line Condition
	else if (Center_white_line < 11 && Left_white_line < 11 && Right_white_line < 11) // 0 0 0
	{
		line_cond = 5;
	}
}
/*

* Function Name: 	buzz

* Input: 			time_delay (int) -> Time duration between buzzer on and off

* Output: 			None

* Logic: 			Buzzer is set to on for required time period and then switched off

* Example Call:		buzz(1000);

*/
void buzz(unsigned long mst)
{
    buzzer_on();
    _delay_ms(50);
    buzzer_off();
}

/*

* Function Name: 	line_follow	

* Input: 			None

* Output: 			None

* Logic:			- Follow the line based on the line conditions and turn flags set

					- Line following and Line detection are done separately, in order to increase the 

					  sampling rate of the sensors so that the sensors do not miss out nodes and turns

					  when the robot is moving at higher velocities

					- To avoid duplicate detection of nodes and turns, after the first detection, timer 4 is enabled for

					  a short duration (0.5 sec). As long as the timer 4 is running, duplicate detections will be discarded.

* Example Call:		line_follow();

*/
void line_follow() 
{
    if (t4_flag == 1) 
	{
        node_flag = 0;
        left_flag = 0;
        right_flag = 0;
    }

    if (node_flag == 1)
	 {
        start_timer4(5);
		buzz(50);
		node++;		
		forward();
		velocity(255,250);
        lcd_print(1, 1, node, 2);
        node_flag = 0;
		handle_nodes();
	 }

    else if (line_cond == 2)
	{
        forward();
        velocity(255, 250);
    } 
	else if (line_cond == 3)
	{
        stop();
        _delay_ms(10);
        forward();
        velocity(175, 255);
    } 
	else if (line_cond == 4)
	{
        stop();
        _delay_ms(10);
        forward();
        velocity(250, 175);
    } 
	else if (line_cond == 5)
	{
        forward();
        velocity(255, 250);
    }
}

/*

* Function Name: 	handle_nodes()

* Input: 			None

* Output: 			None

* Logic: 			To tell the bot what it has to do when it encounters a node
                    master_dir[node] -> 'r' implies the bot has to take right
					master_dir[node] -> 'l' implies the bot has to take left
					master_dir[node] -> 's' implies the bot has to stop and buzz for 500ms and then increment the node number
					master_dir[node] -> 'u' implies the bot has to take a U-turn

* Example Call:		handle_nodes();

*/
void handle_nodes()
{
	if(master_dir[node]=='r')
	{
		correct_right();
	}
	else if(master_dir[node]=='l')
	{
		correct_left();
	}
	else if(master_dir[node]=='u')
	{
		uturn();
	}
	else if(master_dir[node]=='o')
	{
		master_flag=0;
	}
	setDrop(node);
}

/*

* Function Name: 	line_following

* Input: 			None


* Output: 			None

* Logic:			Enabling Timer3 Register to initiate Line following of the Bot

* Example Call:		line_following();

*/
void line_following()
{
    TIMSK3 = 0x01;
}

/*********************************************************************************************************/



//                                          SERVOMOTOR FUNCTIONS                                         //



/*********************************************************************************************************/

/*

* Function Name: 	servo1_pin_config

* Input: 			None

* Output: 			None

* Logic: 			Configure PORTB 5 pin for servo motor 1 operation

* Example Call:		servo1_pin_config();

*/
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*

* Function Name: 	servo2_pin_config

* Input: 			None

* Output: 			None

* Logic: 			Configure PORTB 6 pin for servo motor 2 operation

* Example Call:		servo2_pin_config();

*/
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

/*

* Function Name: 	servo3_pin_config

* Input: 			None

* Output: 			None

* Logic: 			Configure PORTB 7 pin for servo motor 3 operation

* Example Call:		servo3_pin_config();

*/
void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

/*

* Function Name: 	servo_1

* Input: 			degrees (unsigned char) -> The angle by which the servo should rotate

* Output: 			None

* Logic: 			Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees

* Example Call:		servo_1(60);

*/
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}

/*

* Function Name: 	servo_2

* Input: 			degrees (unsigned char) -> The angle by which the servo should rotate

* Output: 			None

* Logic: 			Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees

* Example Call:		servo_2(100);

*/
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

/*

* Function Name: 	servo_3

* Input: 			degrees (unsigned char) -> The angle by which the servo should rotate

* Output: 			None

* Logic: 			Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees

* Example Call:		servo_3(160);

*/
void servo_3(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}

/*

Servo_free functions unlocks the servo motors from any angle and make them free by giving 100% duty cycle at the PWM.

This function can be used to reduce the power consumption of the motor if it is holding load against the gravity.

*/


/*

* Function Name: 	servo_1_free

* Input: 			None

* Output: 			None

* Logic: 			Makes servo 1 free rotating

* Example Call:		servo_1_free():

*/
void servo_1_free (void)
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

/*

* Function Name: 	servo_2_free

* Input: 			None

* Output: 			None

* Logic: 			Makes servo 2 free rotating

* Example Call:		servo_2_free();

*/

void servo_2_free (void)
{
    OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

/*

* Function Name: 	servo_3_free

* Input: 			None

* Output: 			None

* Logic: 			Makes servo 3 free rotating

* Example Call:		servo_3_free();

*/
void servo_3_free (void)
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}

/*

* Function Name:     gripper_open

* Input:             None

*Output:             None

*Logic:              To open the gripper by setting the servo motor rotation to 90 degrees

*Example Call:       gripper_open();

*/
void gripper_open()
{
	servo_2(-90);
	_delay_ms(1000);
}

/*

* Function Name:     gripper_close

* Input:             None

*Output:             None

*Logic:              To close the gripper by setting the servo motor rotation to zero degrees

*Example Call:       gripper_close();

*/
void gripper_close()
{
	servo_2(0);
	_delay_ms(1000);
}

/*

* Function Name:     arm_rotate

* Input:             None

*Output:             None

*Logic:              To rotate the robotic arm to the left using servo motor by 90 degrees

*Example Call:       arm_rotate();

*/
void arm_rotate()
{
	servo_1(90);
	_delay_ms(1000);
}

/*

* Function Name:     arm_initial

* Input:             None

*Output:             None

*Logic:              To set the initial position or the default position of the robotic arm by 
                     setting the servo motor rotation to zero degrees

*Example Call:       arm_initial();

*/
void arm_initial()
{
	servo_1(0);
	_delay_ms(1000);
}

/*

* Function Name:     cycle

* Input:             None

*Output:             None

*Logic:              To complete one cycle of motion of the robotic arm by calling
                     the movement functions

*Example Call:       Calling the movement functions here

*/
void cycle()
{
	arm_initial();
	gripper_open();
	gripper_close();
	arm_rotate();
	gripper_open();
	gripper_close();
	arm_initial();
}

/*

* Function Name:     door_open

* Input:             None

*Output:             None

*Logic:              Make the door open to let the fruits drop into the deposition zone by 
                     setting the servo motor rotation to 180 degrees

*Example Call:       door_open();

*/
void door_open()
{
	servo_3(0);
	_delay_ms(2000);
	
}

/*

* Function Name:     door_close

* Input:             None

*Output:             None

*Logic:              Make the door close to keep the fruits within the box until all fruits are plucked from a tree
                     by setting the servo motor rotation by 180 degrees

*Example Call:       door_close();

*/
void door_close()
{
	servo_3(180);
	_delay_ms(2000);
}

/*********************************************************************************************************/



//                                          PATH TRAVERSAL FUNCTIONS                                       //



/*********************************************************************************************************/

/*

* Function Name:     setb

* Input:             None

*Output:             None

*Logic:

*Example Call:       setb();

*/
void setb()
{
	for(i=0;i<7;i++)
	for(j=0;j<7;j++){
		b[i][j]=a[i][j];
	}
	for(i=0;i<49;i++)
	stacks[i]=0;
	top=0;
	c=0;
	cnt=0;
	min = 999;
}

/*

* Function Name: 	set_dir()

* Input: 			None

* Output: 			None

* Logic: 			To set the direction of the robot according to the shortest path algorithm
                    algdir -> 'r' implies bot has to take right(correct_right)
					algdir -> 'l' implies bot has to take left(correct_left)
					algdir -> 'u' implies bot has to take a U-turn(right_degrees by 180)

* Example Call:		set_dir();

*/
void set_dir()
{
	if(master_dir[0]=='r')
	{
		correct_right();
	}
	else if(master_dir[0]=='l')
	{
		correct_left();
	}
	else if(master_dir[0]=='u')
	{
		uturn();
	}
}

/*

* Function Name: 	set_dir2

* Input: 			algdir(char) -> To take input algidr in order to tell the bot what it has to do by traversing the shortest path

* Output: 			None

* Logic: 			To set the direction of the robot according to the shortest path algorithm
                    algdir -> 'r' implies bot has to take right(correct_right)
					algdir -> 'l' implies bot has to take left(correct_left)
					algdir -> 'u' implies bot has to take a U-turn(right_degrees by 180)
					algdir -> 'o' implies master_flag has to be set to zero

* Example Call:		set_dir2(r);

*/
void set_dir2(char algdir)
{
	if(algdir=='r'||algdir=='R')
	{
		if(master_dir[node-2]=='n'||master_dir[node-2]=='N')
		correct_right();
		else
		correct_right2();
	}
	else if(algdir=='l'||algdir=='L')
	{
		if(master_dir[node-2]=='n'||master_dir[node-2]=='N')
		correct_left();
		else
		correct_left2();
	}
	else if(algdir=='u'||algdir=='U')
	{
		uturn();
	}
	else if(algdir=='o')
	{
		master_flag=0;
	}
}

/*

* Function Name: 	set_dir3

* Input: 			algdir(char) -> To take input algidr in order to tell the bot what it has to do by traversing the shortest path

* Output: 			None

* Logic: 			To set the direction of the robot according to the shortest path algorithm
                    algdir -> 'r' implies bot has to take right(correct_right)
					algdir -> 'l' implies bot has to take left(correct_left)
					algdir -> 'u' implies bot has to take a U-turn(right_degrees by 180)
					algdir -> 'o' implies master_flag has to be set to zero

* Example Call:		set_dir3(r);

*/
void set_dir3(char algdir)
{
	if(algdir=='r'||algdir=='R')
	{
		correct_right();
	}
	else if(algdir=='l'||algdir=='L')
	{
		correct_left();
	}
	else if(algdir=='u'||algdir=='U')
	{
		right_degrees(180);
	}
	else if(algdir=='o')
	{
		master_flag=0;
	}
}
/*

* Function Name:     assignTree

* Input:             None

*Output:             None

*Logic:              To obtain the coordinates of the tree
                     

*Example Call:       assignTree();

*/
void assignTree()
{
 for(i=0;i<3;i++)
 {
	 if(tree[i]!=99)
	 {
		 tx=abs((tree[i]/7)-6);
		 ty=abs((tree[i]%7));
		 b[tx][ty]=0;
	 }
 }
 for(i=0;i<7;i++)
 {
 for(j=0;j<7;j++)
 {
	 a[i][j]=b[i][j];
 }
}
}

/*

* Function Name:     flood_fill

* Input:             None

*Output:             None

*Logic:

*Example Call:       flood_fill();

*/
void flood_fill()
{	
	 
	 for(i=0;i<200;i++)
	 master_dir[i]='o';

	 assignTree();

	 for(mjj=0;mjj<6;mjj++)
	 {
		 if(mjj%2==0)
		 {
			 if(tree[mjj/2]!=99)
			 {
			  end=tree[mjj/2];
			  start=deps[mjj/2];
			 }
			 else
			   continue;
		 }
		 else
		 {
			 if(deps[sdd]!=99)
			 {
				 sdd++;
				 end=deps[sdd];
			 start=(7*abs(pxx-6))+pyy;
			 }
			 else
			   continue;
		 }
		 if(start!=99&&end!=99)
		 {
			 flood_fillz(start,end);
		 }
	 }
}

/*

* Function Name:     flood_fillz

* Input:             start(int) -> Get node number of the starting position of the bot
                     end(int)   -> Get node number of the ending postion of the bot
					 
*Output:             None

*Logic:              Getting coordinates of the starting and ending position of the bot
                     flood(i,j) -> to initiate the floodfill algorithm
					 pathchk(i,j) -> to find the shortest path between the starting point and the ending point
					 align() -> to set the starting direction of the bot(aligning it) at the starting position
                     trace() -> to initiate movement of the bot along the shortest path after the bot has been aligned
					 
*Example Call:       flood_fillz(1,8);

*/
void flood_fillz(int start,int end)
{
	sx=abs((start/7)-6);
	sy=abs(start%7);
	fx=abs((end/7)-6);
	fy=abs((end%7));

	b[sx][sy]=-1;
	b[fx][fy]=-5;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (b[i][j] == -5)
			{
				eex = i;
				eey = j;
			}
		}
	}
	stacks[top] = (eex * 100) + eey;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		if (b[i][j] == -1)
		{
			flood(i, j);
			px = i;
			py = j;
		}
	}
	for (c = 1; c < 49; c++)
	{
		for (i = 0; i < m; i++)
		{
			for (j = 0; j < n; j++)
			if (b[i][j] == c)
			flood(i, j);
		}
	}



	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (b[i][j] == -5)
			{
				eex = i;
				eey = j;
				pathchk(i, j);
			}
		}
	}

	align();
	trace();
	if(mjj%2!=0)
	{
		align2(pdxx,pdyy);
	}
	dir[cnt]='s';
	cnt++;
	if(mjj%2==0){
	  for(i=0;i<16;i++)
	  {
		  dir[cnt]=hardc[i];
		  cnt++;
	  }
	}
	for(i=0;i<cnt;i++)
	{
		master_dir[md]=dir[i];
		md++;
	}
	setb();
}


void setRep(){
volatile int aster;
	if(aster)
		get_bot();
		set_bot();
	for(int i=0;i<100;i++)
		hold();
}
/*

* Function Name:     flood

* Input:             x(int) ->
                     y(int) ->
*Output:             None

*Logic:

*Example Call:       flood(2,5);

*/
int flood(int x, int y) 
{
    if (x < (m - 1) && y < m)
	{
	    if (b[x + 1][y] > c)
	    b[x + 1][y] = c + 1;
    }
    if (x < m && y < (m - 1))
	{
	    if (b[x][y + 1] > c)
	    b[x][y + 1] = c + 1;
    }
    if (x > 0 && y > -1)
	{
	    if (b[x - 1][y] > c)
	    b[x - 1][y] = c + 1;
    }
    if (x > -1 && y > 0)
	{
	    if (b[x][y - 1] > c)
	    b[x][y - 1] = c + 1;
    }
    return 0;
}

/*

* Function Name:     pathchk

* Input:             ex(int) ->
                     ey(int) ->
					 
*Output:             None

*Logic:              The shortest path is being found out using the algorithm here

*Example Call:       pathchk(3,7);

*/
void pathchk(int ex, int ey) 
{
    while (min != -1)
	{
	    x = ex;
	    y = ey;
	    top++;
	    if (x < (m - 1) && y < m)
		{
		    if (b[x + 1][y] < min && b[x + 1][y] != -5 && b[x + 1][y] != 0)
			{
			    ex = x + 1;
			    ey = y;
			    min = b[x + 1][y];
			    d = (100 * ex) + ey;
			    stacks[top] = d;
		    }
	    }
	    if (x < m && y < (m - 1))
		{
		    if (b[x][y + 1] < min && b[x][y + 1] != -5 && b[x][y + 1] != 0)
			{
			    ex = x;
			    ey = y + 1;
			    min = b[x][y + 1];
			    d = (100 * ex) + ey;
			    stacks[top] = d;
		    }
	    }
	    if (x > 0 && y > -1)
		{
		    if (b[x - 1][y] < min && b[x - 1][y] != -5 && b[x - 1][y] != 0)
			{
			    ex = x - 1;
			    ey = y;
			    min = b[x - 1][y];
			    d = (100 * ex) + ey;
			    stacks[top] = d;
		    }
	    }
	    if (x > -1 && y > 0)
		{
		    if (b[x][y - 1] < min && b[x][y - 1] != -5 && b[x][y - 1] != 0)
			{
			    ex = x;
			    ey = y - 1;
			    min = b[x][y - 1];
			    d = (100 * (ex)) + (ey);
			    stacks[top] = d;
		    }
	    }
    }
}

/*

* Function Name:     align

* Input:             None

*Output:             None

*Logic:              To align the bot to a specific direction at the starting point

*Example Call:       align();

*/
void align() 
{
 z = stacks[top - 1];
 nx = z / 100;
 ny = z % 100;
 ndx = nx - px;
 ndy = ny - py;

 if (pdx == ndx && pdy == ndy)
 {
	 dir[cnt] = 'n';
	 cnt++;
 }
 else if ((pdx == 0 && ndx == 0) || (pdy == 0 && ndy == 0))
 {
	 dir[cnt] = 'u';
	 cnt++;
 }

 if (pdx != ndx || pdy != ndy)
 {
	 if (pdx > 0 && pdy == 0)
	 {
		 if (ndx == 0 && ndy > 0)
		 {
			 dir[cnt] = 'l';
			 cnt++;
		 }
		 if (ndx == 0 && ndy < 0)
		 {
			 dir[cnt] = 'r';
			 cnt++;
		 }
	 }

	 if (pdx == 0 && pdy > 0)
	 {
		 if (ndx > 0 && ndy == 0)
		 {
			 dir[cnt] = 'r';
			 cnt++;
		 }
		 if (ndx < 0 && ndy == 0)
		 {
			 dir[cnt] = 'l';
			 cnt++;
		 }
	 }

	 if (pdx < 0 && pdy == 0)
	 {
		 if (ndx == 0 && ndy > 0)
		 {
			 dir[cnt] = 'r';
			 cnt++;
		 }
		 if (ndx == 0 && ndy < 0)
		 {
			 dir[cnt] = 'l';
			 cnt++;
		 }
	 }

	 if (pdx == 0 && pdy < 0)
	 {
		 if (ndx > 0 && ndy == 0)
		 {
			 dir[cnt] = 'l';
			 cnt++;
		 }
		 if (ndx < 0 && ndy == 0)
		 {
			 dir[cnt] = 'r';
			 cnt++;
		 }

	 }

 }
 pdx = ndx;
 pdy = ndy;
}

/*

* Function Name:     trace

* Input:             None

*Output:             None

*Logic:             to make the bot trace the shortest path after the initial alignment

*Example Call:       trace();

*/
void trace()
 {
     z = stacks[top - 1];
     px = z / 100;
     py = z % 100;

     for (i = (top - 2); i > -1; i--)
	 {
	     z = stacks[i];
	     nx = z / 100;
	     ny = z % 100;
	     ndx = nx - px;
	     ndy = ny - py;

	     if (pdx == ndx && pdy == ndy)
		 {
		     dir[cnt] = 'n';
		     cnt++;
	     }
	     else if ((pdx == 0 && ndx == 0) || (pdy == 0 && ndy == 0))
		 {
		     dir[cnt] = 'u';
		     cnt++;
	     }

	     if (pdx != ndx || pdy != ndy)
		 {

		     if (pdx > 0 && pdy == 0)
			 {
			     if (ndx == 0 && ndy > 0)
				 {
				     dir[cnt] = 'l';
				     cnt++;
			     }
			     if (ndx == 0 && ndy < 0)
				 {
				     dir[cnt] = 'r';
				     cnt++;
			     }
		     }

		     if (pdx == 0 && pdy > 0)
			 {
			     if (ndx > 0 && ndy == 0)
				 {
				     dir[cnt] = 'r';
				     cnt++;
			     }
			     if (ndx < 0 && ndy == 0)
				 {
				     dir[cnt] = 'l';
				     cnt++;
			     }
		     }

		     if (pdx < 0 && pdy == 0)
			 {
			     if (ndx == 0 && ndy > 0)
				 {
				     dir[cnt] = 'r';
				     cnt++;
			     }
			     if (ndx == 0 && ndy < 0)
				 {
				     dir[cnt] = 'l';
				     cnt++;
			     }
		     }

		     if (pdx == 0 && pdy < 0)
			 {
			     if (ndx > 0 && ndy == 0)
				 {
				     dir[cnt] = 'l';
				     cnt++;
			     }
			     if (ndx < 0 && ndy == 0)
				 {
				     dir[cnt] = 'r';
				     cnt++;
			     }
		     }
	     }
	     pdxx=ndx;
	     pdyy=ndy;
	     pxx=px;
	     pyy=py;
	     px = nx;
	     py = ny;
	     pdx = ndx;
	     pdy = ndy;
     }
}

/*

* Function Name:     align2

* Input:             pdxx(int) ->
                     pdyy(int) ->
					 
*Output:             None

*Logic:

*Example Call:       align2(3,6);

*/
void align2(int pdxx,int pdyy)
{
	ndxx=-1;
	ndyy=0;
	if (pdxx == ndxx && pdyy == ndyy)
	{
		dir[cnt] = 'N';
		cnt++;
	}
	else if ((pdxx == 0 && ndxx == 0) || (pdyy == 0 && ndyy == 0))
	{
		dir[cnt] = 'U';
		cnt++;
	}
	if (pdxx != ndxx || pdyy != ndyy)
	{
		if (pdxx > 0 && pdyy == 0)
		{
			if (ndxx == 0 && ndyy > 0)
			{
				dir[cnt] = 'L';
				cnt++;
			}
			if (ndxx == 0 && ndyy < 0)
			{
				dir[cnt] = 'R';
				cnt++;
			}
		}
		if (pdxx == 0 && pdyy > 0)
		{
			if (ndxx > 0 && ndyy == 0)
			{
				dir[cnt] = 'R';
				cnt++;
			}
			if (ndxx < 0 && ndyy == 0)
			{
				dir[cnt] = 'L';
				cnt++;
			}
		}
		if (pdxx < 0 && pdyy == 0)
		{
			if (ndxx == 0 && ndyy > 0)
			{
				dir[cnt] = 'R';
				cnt++;
			}
			if (ndxx == 0 && ndyy < 0)
			{
				dir[cnt] = 'L';
				cnt++;
			}
		}
		if (pdxx == 0 && pdyy < 0)
		{
			if (ndxx > 0 && ndyy == 0)
			{
				dir[cnt] = 'L';
				cnt++;
			}
			if (ndxx < 0 && ndyy== 0)
			{
				dir[cnt] = 'R';
				cnt++;
			}
		}
	}
	pdx=-1;
	pdy=0;
}

/*********************************************************************************************************/



//                                          FRUIT PLUCK AND DROP MECHANISM FUNCTIONS                     //



/*********************************************************************************************************/

/*

* Function Name:     pluckDrop

* Input:             Count(int) ->

*Output:             None

*Logic:

*Example Call:       pluckDrop(5);

*/
void pluckDrop(int Count)
{
	if(Count==1)
	{
		// PICK FRUIT:
		node=node+2;
		stop();
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);
		set_dir2(master_dir[node]);
	}
	else if(Count==2)
	{
		// DROP FRUITSTOP:
		stop();
		_delay_ms(450);
		set_dir3(master_dir[node]);
		stop();
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);
		node=node+2;
		set_dir2(master_dir[node]);
	}
	Count=0;
}

/*

* Function Name:     setDrop

* Input:             node1(int) ->

*Output:             None

*Logic:

*Example Call:       setDrop();

*/
void setDrop(int node1)
{
	if(master_dir[node1+1]=='s')
	{   
		if(master_dir[node1]=='N'||master_dir[node1]=='R'||master_dir[node1]=='L'||master_dir[node1]=='U')
		{
			Count=2;
		    pluckDrop(Count);}
		else
		{
			Count=1;
			pluckDrop(Count);
		}
	}
}	

/*********************************************************************************************************/



//                                         INITIATION FUNCTIONS                                          //



/*********************************************************************************************************/

/*

* Function Name: 	port_init()

* Input: 			None

* Output: 			None

* Logic: 			Function to initialize all the ports

* Example Call:		port_init();

*/
void port_init()
{   DDRJ= DDRJ | 0xF7;
	PORTJ=0xFF;
	buzzer_pin_config(); //Buzzer pin configuration
	lcd_port_config();  //LCD pin configuration
	
	adc_pin_config();  //ADC pin configuration
	motion_pin_config(); //Motion pin configuration
	left_encoder_pin_config(); //Left Encoder pin configuration
	right_encoder_pin_config(); //Right Encoder pin configuration
}

/*

* Function Name: 	init_devices

* Input: 			None

* Output: 			None

* Logic: 			Function to initiate all the sensors, actuators and interrupts

* Example Call:		init_devices();

*/
void init_devices(void)
{
	cli(); //Clears the global interrupts
	
	port_init(); //Initiate all ports
	adc_init(); //Initiate ADC registers
	
	//Initiate timers
	timer1_init();
	timer5_init();
	timer3_init();
	timer4_init();
	
	//Initiate position encoder interrupts
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	
	sei(); //Enables the global interrupts
}

/*********************************************************************************************************/



//                                               MAIN FUNCTION                                           //



/*********************************************************************************************************/

/*

* Function Name:     main

* Input:             None

*Output:             None

*Logic:              

*Example Call:       main();

*/
int main()
{
	flood_fill();
	init_devices();
	lcd_set_4bit();
	lcd_init();
	enable_flags(0, 1, 0);
	line_following();
	set_dir();
	while(master_flag)
	{
		lcd_wr_command(0x01);
		display_line();
		line_follow();
		_delay_ms(50);
	}
	stop();
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
}	
