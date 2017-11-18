//==========================================================================
//	Author			: Cytron Technologies Sdn Bhd
//	Project			: Sample code demonstration for Cytron DC Geared Motor with Encoder (MO-SPG-30E-XXXK)
//	Project description	: Demonstrate the usage of the Encoder at the motor
//	Version			: 1.0 Sep 2010  	
//
//==========================================================================


#include<pic.h>
__CONFIG (0x3F32);

#define rs			RC3
#define e			RC5
#define	lcd_data	PORTB
#define	ButA		RA0
#define	ButB		RA1
#define	LEDRED		RA3
#define	LEDGREEN	RA4

#define	IN3			RC4			
#define	IN4			RC7
#define	EN2			CCPR2L

#define	A			RA5
#define	B			RA2


void send_config(unsigned char data);
void delay(unsigned long data);
void send_char(unsigned char data);
void lcd_goto(unsigned char data);
void lcd_clr(void);	
void send_string(const char *s)	;
void run_cw(unsigned char speed);
void run_ccw(unsigned char speed);
void stop (unsigned char data);
void motor_run(unsigned char speed, unsigned long degree);
void motor_run_overshoot(unsigned char speed, unsigned long degree);

bit initA, initB, stopA, stopB,end;
	unsigned long bigdegree;
	unsigned long desire_angle;
	unsigned long smalldegree;
	unsigned long limit;
	unsigned long countAB;
	unsigned long countover;
	unsigned long countstop;
	unsigned char gear_ratio;




void main(void)
{
	TRISA = 0b11100111;					//configure PORTA I/O direction
	TRISB = 0b00000000;					//configure PORTB I/O direction
	TRISC = 0b00000000;	
	
	ADCON1 = 0b10000110;				//configure PORTA as digital I/O
	
	send_config(0b00000001);			//clear display at lcd
	send_config(0b00000010);			//lcd return to home 
	send_config(0b00000110);			//entry mode-cursor increase 1
	send_config(0b00001100);			//display on, cursor off and cursor blink off
	send_config(0b00111000);
	
	CCP2CON = 0b00001100;				
	PR2 = 0xFF;							//PWM Period Setting (4.88KHz)
	T2CON = 0b00000101;					//Timer2 On, prescale 4
	LEDRED=1;
	LEDGREEN=1;

//================================================================
//	By pressing button A, motor will rotate to the desired location 
//	and the overshoot occur on each and every time the motor rotates

//	By pressing on button B, motor will rotates to the desired location
//	with overshoot for the 1st time. If the load and the speed remain
//	the motor will rotate and reach at the specified angle accurately
//	after the 1st rotation.The idea is to pass the overshoot error to
//	the next and compensate it to archieve better precision.
	while(1)
	{
		if (ButA==0)				//button A pressed?
		{
			while(ButA==0);			//waiting for release
			motor_run(254,360);		//run motor at speed 254 for 360 degree
		}
		else if (ButB==0)			//button B pressed?
		{
			while(ButB==0);			//waiting for release
			motor_run_overshoot(254,360);		//run motor at speed 254 for angle 360degree
			lcd_goto(0);					//set lcd cursor to location 0
			send_string("overshoot = ");	//print string
			lcd_goto(12);					//set lcd cursor to location 12
			send_char(countover/10 +0x30);	//print tens digit of the overshoot counter
			lcd_goto(13);					//set lcd cursor to location 13
			send_char(countover%10 +0x30);	//print units digit of the overshoot counter
			delay(50000);
		}
		else
		stop(500);					//motor not running
	}	
}
//==========================================================
//		SAMPLE FUNCTION WITHOUT OVERSHOOT COMPENSATION	
//==========================================================
void motor_run(unsigned char speed, unsigned long degree)
{
	desire_angle = degree;
	//in this case gear ratio is 1:20					
	gear_ratio = 20;
	//convert the desired agle to the total angle at the rear shaft (multiply gear ratio)						
	smalldegree = desire_angle *gear_ratio;
	//12 count per revolution at the rear shaft
	//meaning each count represent 30 degree
	//to calculate how many counts needed for the desire angle
	//we divide total angle at the rear shaft with 30 degree	
	limit = smalldegree/30;

	countAB=0;		//reset the counter
	lcd_clr();		//clear lcd screen
	while(countAB < limit)					//while counter is in the limit
	{										//motor will overshoot because it can not sudden stop
		initA = A;	//get current A value	//overshoot depends on load and speed of the motor
		initB = B;	//get current B value	
		do
		{
			run_cw(speed);					//run motor in clockwise direction with user defined speed
		}while(initA ==A && initB ==B);		//continue run until state change
		countAB++;							//increment counter for each state change
	}
	//once program exit the previous while loop
	//meaning that the motor has reached the desired angle
	stop(1000);					
}
//==========================================================
//		SAMPLE FUNCTION WITH OVERSHOOT COMPENSATION	
//==========================================================
void motor_run_overshoot(unsigned char speed, unsigned long degree)
{
	desire_angle = degree;
	//in this case gear ratio is 1:20					
	gear_ratio = 20;
	//convert the desired agle to the total angle at the rear shaft (multiply gear ratio)						
	smalldegree = desire_angle *gear_ratio;
	//12 count per revolution at the rear shaft
	//meaning each count represent 30 degree
	//to calculate how many counts needed for the desire angle
	//we divide total angle at the rear shaft with 30 degree	
	limit = smalldegree/30;

	countAB=0;		//reset the counter
	lcd_clr();		//clear lcd screen
	while(countAB < (limit-countover+3))	//while counter is in the (limit-previous overshoot +3)
	{										//for the 1st time, the motor will overshoot because PIC
		initA = A;	//get current A value	//has not get the overshoot value to compensate
		initB = B;	//get current B value	
		do
		{
			run_cw(speed);					//run motor in clockwise direction with user defined speed
		}while(initA ==A && initB ==B);		//continue run until state change
		countAB++;							//increment counter for each state change
		stopA=A;							//stopA store A value right after state change
		stopB=B;							//stopB store B value right after state change
	}
	//once program exit the previous while loop
	//meaning that the motor has reached the desired angle
	countover=0;	//reset overshoot counter
	end=0;			//reset end function flag
	while(end==0)	//while the flag is not set
	{
		do
		{
			if(countstop>10000) end=1;	//set end flag if stop counter >10000	
			else
			stop(0);				//brake the motor
			countstop++;			//stop counter increment by 1
		}while(stopA==A && stopB==B && end==0);	//loop if states do not change
		//if states change,meaning overshoot occur
		countover++;	//increment overshoot counter
		countstop=0;	//clear stop counter
		stopA=A;		//stopA store the current A value
		stopB=B;		//stopB store the current B value
	}
						
}
//================================================
//				LCD FUNCTION	
//================================================
void send_config(unsigned char data)		//send lcd configuration 
{
	rs=0;									//set lcd to configuration mode
	lcd_data=data;							//lcd data port = data
	e=1;									//pulse e to confirm the data
	delay(50);
	e=0;
	delay(50);
}

void delay(unsigned long data)			//delay function, the delay time
{										//depend on the given value
	for( ;data>0;data--);
}

void send_char(unsigned char data)		//send lcd character
{
	rs=1;								//set lcd to display mode
	lcd_data=data;						//lcd data port = data
	e=1;								//pulse e to confirm the data
	delay(10);
	e=0;
	delay(10);
}

void lcd_goto(unsigned char data)		//set the location of the lcd cursor
{										//if the given value is (0-15) the 
 	if(data<16)							//cursor will be at the upper line
	{									//if the given value is (20-35) the 
	 	send_config(0x80+data);			//cursor will be at the lower line
	}									//location of the lcd cursor(2X16):
	else								// -----------------------------------------------------
	{									// | |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15| |
	 	data=data-20;					// | |20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35| |
		send_config(0xc0+data);			// -----------------------------------------------------	
	}
}

void lcd_clr(void)						//clear the lcd
{
 	send_config(0x01);
	delay(600);	
}

void send_string(const char *s)			//send a string to display in the lcd
{          
	unsigned char i=0;
  	while (s && *s)send_char (*s++);
}
//==================================================
//				BASIC MOTOR FUNCTION	
//==================================================
void run_cw (unsigned char speed)		//run motor in clockwise direction
{
	IN3 = 1;
	IN4 = 0;
	EN2 = speed;
}

void run_ccw (unsigned char speed)		//run motor in counter clockwise direction
{
	IN3	= 0;
	IN4 = 1;
	EN2 = speed;
}	

void stop (unsigned char data)			//brake/stop the motor motion
{
	IN3 = 1;
	IN4 = 1;
	EN2 = 254;
	delay(data);
}		