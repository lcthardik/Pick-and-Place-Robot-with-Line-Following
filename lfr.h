//#define F_CPU 16000000
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>
//#include <math.h> //included to support power function
#include "My_servo.h"
#include "sharp.h"

void pick();
void drop();
void init_devices_lfr();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void line_follower();
void calculate_error();
void drive_motors();
unsigned char ADC_Conversion(unsigned char);
//unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
int error=0;

int thresh = 0x28;
int distance_sharp;

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init_lfr()
{
	adc_pin_config();
	motion_pin_config();	
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD


//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
	velocity(210,200);
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}

void back(void)
{
	velocity(210,200);
	motion_set (0x09);
}
void soft_left_turn(void)
{
	velocity(170,230);
	motion_set (0x06);
	//motion_set (0x02);
}
void soft_right_turn(void)
{
	velocity(250,160);
	motion_set (0x06);
	//motion_set (0x04);
}
void left_turn(void)
{
	motion_set (0x0A);
}
void right_turn(void)
{
	motion_set (0x05);
}

void forward_by_node(unsigned char node)
{
	int on_the_node=0;
	
	forward();
	_delay_ms(50);
	while(on_the_node != node)
	{
		line_follower();
		distance_sharp=sharp_sensor_distance();
		if (distance_sharp < 80 )
		{
			stop();
		sound_on();
		break;
		}
		if ((ADC_Conversion(3)> thresh && ADC_Conversion(2)>thresh) || (ADC_Conversion(2)>thresh && ADC_Conversion(1)>thresh))  //node detected
		{
			while((ADC_Conversion(3)> thresh && ADC_Conversion(2)>thresh) || (ADC_Conversion(2)>thresh && ADC_Conversion(1)>thresh))
			{
				forward();
			}
			_delay_ms(10);
			on_the_node++;
			
		}
	}
	forward();
	_delay_ms(250); //axis

}

void right_mudja()
{
	right_turn();
	_delay_ms(200);
	while(ADC_Conversion(1)<thresh)
	{
		right_turn();
	}
	_delay_ms(50);
}

void left_mudja()
{
	left_turn();
	_delay_ms(200);
	while(ADC_Conversion(3)<thresh)
	{
		left_turn();
	}
	_delay_ms(50);
}

void init_devices_lfr (void)  //called in the main 
{
 	cli(); //Clears the global interrupts
	port_init_lfr();
	adc_init();
	timer5_init();
	sei();   //Enables the global interrupts
}



void line_follower()
{
	calculate_error();
	drive_motors();
}

void calculate_error()
{
	if(ADC_Conversion(2)>thresh)
		{
			error=0;
			//forward();
			//velocity(150,150);
		}

		if(ADC_Conversion(3)>thresh)
		{
			error=-1;
			//soft_left_turn();
			//velocity(130,50);
		}

		if(ADC_Conversion(1)>thresh)
		{
			error=1;
			//soft_right_turn();
			//velocity(50,130);
		}

		if((ADC_Conversion(3) < thresh) && (ADC_Conversion(2) < thresh) && (ADC_Conversion(1) < thresh))
		{
			error=error;
		}
}

void drive_motors()
{
	if (error == 0)
	{
		forward();
	}
	else if (error == 1)
	{
		right_turn();
	}
	else if (error == -1)
	{
		left_turn();
	}
}


/*int main()
{
	
	//port_init();
	//forward_by_node(1);
	
	init_devices();
	init_devices_main();
	port_init_main();
	port_init();
	
	servo_1(30);
	servo_1_free();
	servo_2(80);
	_delay_ms(1000);
	servo_2(160);
	_delay_ms(4000);
	servo_2_free();*/
	//velocity(200,200);
	
	/*forward_by_node(1);
	forward_by_node(1);
	right_mudja();
	forward_by_node(1);
	left_mudja();
	
	pick();
	right_mudja();
	forward_by_node(1);
	forward_by_node(1);
	right_mudja();
	forward_by_node(1);
	forward_by_node(1);
	forward_by_node(1);
	
	drop();*//*
	
	forward_by_node(1);
	stop();
	servo_1(60);
	_delay_ms(9000);
	forward_by_node(1);
	right_mudja();
	forward_by_node(1); 
	
	while(1)
	{
		stop();
	}	
	
	//servo_1(30);
	//_delay_ms(1000);
	//servo_1(110);
	//_delay_ms(1000);
	//servo_1_free();
	//pick ();
	//_delay_ms(3000);
	//drop();
	
	
	return 0;
}*/


void pick ()
{
	
	stop();
	_delay_ms(1000);
	back();
	_delay_ms(200);
	stop();
	_delay_ms(1000);
	
	
	
	servo_1(30);   ///uppar
	_delay_ms(1000);
	
	servo_2(100);  //khul
	_delay_ms(1000);
	
	servo_1(110);  //neche
	_delay_ms(1000);
	
	servo_2(160); //band
	_delay_ms(1000);
	//servo_2_free();
	
	servo_1(20);   ///uppar
	_delay_ms(2000);
	
	
	servo_1_free();
	servo_2_free();
	
	
	
}

void drop ()
{
	back();
	_delay_ms(250);
	stop();
	_delay_ms(1000);
	
	servo_1(110);  //neche
	_delay_ms(1000);
	
	servo_2(100);  //khul
	_delay_ms(1000);
	
	servo_1(20);   ///uppar
	_delay_ms(2000);
	
	servo_1_free();
	servo_2_free();
}
