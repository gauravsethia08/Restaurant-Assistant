/*
 *
 *
 * Team ID			- #951
 * Author List		- Shruthi J, Rashmi N, Nithyapoornima Moorthy, Viswakiran
 * File Name		- TBT.c
 * Theme			- RESTAURANT ASSISTANT: ORDER TAKING ROBOT
 * Functions		- adc_pin_config, motion_pin_config, port_init, timer5_init, adc_init, ADC_Conversion, Sharp_GP2D12_estimation,
						init_devices, velocity, motion_set, forward, right, left, hard_right, hard_left, take_order, place_order,
						show_order, led_config, red_led_on, blue_led_on, green_led_on, all_led_off, buzzer_pin_config, buzzer_on,
						buzzer_off, print, read, main
						
 * Global Variables	- ADC_Value, Left, Center, Right, thresh, sharp, t[], a[], b[], c[], fa, fb, fc, k, value, node, turn, count, x
 *
 *
 */ 


#define F_CPU 14745600

// Including header files
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"

//Declaring Global Variables
unsigned char ADC_Value;					//To store the value computed by ADC function
unsigned char Left = 0;						//To store the value of the left white line sensor
unsigned char Center = 0;					//To store the value of the middle white line sensor
unsigned char Right = 0;					//To store the value of the right white line sensor
unsigned char thresh = 50;					//Declaring the threshold to differentiate between black line and white area
unsigned char sharp;						//To store the value of the sharp sensor
int t[] = {8, 6, 4, 1, 2, 3, 5, 7, 9};		//Array which has all the table number in the order which robot visits them
int a[9], b[9], c[9];						//Arrays to store the 3 types of Food
int fa = 0, fb = 0, fc = 0, k = 0;			//To store the count of the food ordered and the total tables it have take order from
int value;									//To store the value of distance in mm given by sharp IR sensor
int node = 0;								//To store the number of nodes it have visited, ie, node counter
int turn = 0;								//To store the number of times it have turned
int count = 0;								//To store the count, to know when to turn left
int x = 4;									//Variable used to determine, whether robot is in kitchen or still taking orders


/*
*
* Function Name - adc_pin_config
* Input			- None
* Output		- None
* Logic			- To configure the Analog to Digital Converter 
* Example Call  - adc_pin_config()
*
*/
void adc_pin_config (void)
{
	 DDRF = 0x00; 
	 PORTF = 0x00;
	 DDRK = 0x00;
	 PORTK = 0x00;
}


/*
*
* Function Name - motion_pin_config
* Input			- None
* Output		- None
* Logic			- To configure the motion control/ DC motors
* Example Call  - motion_pin_config()
*
*/
void motion_pin_config (void) 
{
	 DDRA = DDRA | 0x0F;
	 PORTA = PORTA & 0xF0;
	 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}


/*
*
* Function Name - port_init
* Input			- None
* Output		- None
* Logic			- To initialize all the ports of the micro_controller
* Example Call  - port_init()
*
*/
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	buzzer_pin_config();
	led_config();
}


/*
*
* Function Name - timer5_init
* Input			- None
* Output		- None
* Logic			- To initialize the timer5
* Example Call  - timer5_init()
*
*/
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

/*
*
* Function Name - adc_init
* Input			- None
* Output		- None
* Logic			- To initialize the Analog to Digital Converter
* Example Call  - adc_init()
*
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}


/*
*
* Function Name - ADC_Conversion
* Input			- (unsigned char) pin number from which data has to be read and converted
* Output		- (unsigned char) Digital Value of the sensor data
* Logic			- Converting the value from sensor from analog to digital form
* Example Call  - ADC_Conversion(11)
*
*/
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


/*
*
* Function Name - Sharp_GP2D12_estimation
* Input			- (unsigned char) value given by the sensor, which is read from ADC_Conversion function
* Output		- (unsigned int) Distance in millimeter
* Logic			- Converting the ADC value from the sharp sensor to distance in millimeter 
* Example Call  - Sharp_GP2D12_estimation(adc_value)
*
*/
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}


/*
*
* Function Name - init_devices
* Input			- None
* Output		- None
* Logic			- Initialize all the systems 
* Example Call  - init_devices()
*
*/
void init_devices (void)
{
	port_init();
	adc_init();
	timer5_init();
}


/*
*
* Function Name - velocity
* Input			- 2 (unsigned char, unsigned char) speed of the left and right motor respectively
* Output		- None
* Logic			- Sets the speed of the 2 DC motors
* Example Call  - velocity(100,100)
*
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


/*
*
* Function Name - motion_set
* Input			- (unsigned char) Direction in which the robot moves
* Output		- None
* Logic			- function sets the direction of the robot
* Example Call  - motion_set(0x02)
*
*/
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}


/*
*
* Function Name - forward
* Input			- None
* Output		- None
* Logic			- to set the robot to move in forward direction
* Example Call  - forward()
*
*/
void forward(void)
{
	motion_set(0x06);
	velocity(100,110);
}


/*
*
* Function Name - right
* Input			- None
* Output		- None
* Logic			- to set the robot to move in soft right direction
* Example Call  - right()
*
*/
void right(void)
{
	motion_set(0x06);
	velocity(90,30);
}


/*
*
* Function Name - left
* Input			- None
* Output		- None
* Logic			- to set the robot to move in soft left direction
* Example Call  - left()
*
*/
void left(void)
{
	motion_set(0x06);
	velocity(30,100);
}


/*
*
* Function Name - hard_right
* Input			- None
* Output		- None
* Logic			- to set the robot to move in right direction
* Example Call  - hard_right()
*
*/
void hard_right(void)
{
	motion_set(0x0A);
	velocity(150,150);
}


/*
*
* Function Name - hard_left
* Input			- None
* Output		- None
* Logic			- to set the robot to move in left direction
* Example Call  - hard_left()
*
*/
void hard_left(void)
{
	motion_set(0x05);
	velocity(150,150);
}


/*
*
* Function Name - stop
* Input			- None
* Output		- None
* Logic			- to set the robot to stop
* Example Call  - stop()
*
*/
void stop(void)
{
	motion_set(0x00);
	velocity(0,0);
}


/*
*
* Function Name - take_order
* Input			- None
* Output		- None
* Logic			- to take order based on the distance measured, store the order and give the correct indication with LED or buzzer
* Example Call  - take_order()
*
*/
void take_order(void)
{
	sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
	
	//Food A/ Red Flag
	if (value >= 241 && value <= 290)
	{
		a[fa++] = t[k++];
		all_led_off();
		red_led_on();
	}
	
	//Food B/ Green Flag
	else if (value >= 210 && value <= 240)
	{
		b[fb++] = t[k++];
		all_led_off();
		green_led_on();
	}
	
	//Food C / Blue Flag
	else if (value >= 170 && value <= 205)
	{
		c[fc++] = t[k++];
		all_led_off();
		blue_led_on();
	}
	
	//Table Empty
	else
	{
		all_led_off();
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);
	}
}


/*
*
* Function Name - place_order
* Input			- None
* Output		- None
* Logic			- to give the order by beeping the buzzer correct number of times in the correct cooking area
* Example Call  - place_order()
*
*/
void place_order(void)
{
	sharp = ADC_Conversion(11);
	value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calculated in a variable "value".
	
	//Cooking Zone for Food A
	if (value >= 241 && value <= 290)			
	{
		for (int i=0; i<fa; i++)			//Beeping the buzzer, number of times Red Flag was shown
		{
			buzzer_on();
			_delay_ms(500);
			buzzer_off();
			_delay_ms(500);
		}
	}
	
	//Cooking Zone for Food B
	else if (value >= 210 && value <= 240)
	{
		for (int i=0; i<fb; i++)			//Beeping the buzzer, number times green flag was shown
		{
			buzzer_on();
			_delay_ms(500);
			buzzer_off();
			_delay_ms(500);
		}
	}
	
	//Cooking Zone for Food C
	else if (value >= 170 && value <= 205)
	{
		for (int i=0; i<fc; i++)			//Beeping the buzzer, number of times blue flags were shown
		{
			buzzer_on();
			_delay_ms(500);
			buzzer_off();
			_delay_ms(500);
		}
	}
}


/*
*
* Function Name - show_order
* Input			- None
* Output		- None
* Logic			- to show the order for each type of food with table number
* Example Call  - show_order()
*
*/
void show_order(void)
{
	lcd_clear();
	lcd_string(1,1, "A-");
	lcd_string(1,12,"B-");
	lcd_string(2,1,"C-");
	
	//Food A
	if (fa == 0)
	{
		lcd_string(1,3,"NO ORDER");
	}
	else
	{
		for(int i=0;i<fa;i++)
		{
			lcd_numeric_value(1,3+i,a[i],1);
		}
	}
	
	//Food B
	if (fb == 0)
	{
		lcd_string(1,14,"NO ORDER");
	}
	else
	{
		for(int i=0;i<fb;i++)
		{
			lcd_numeric_value(1,14+i,b[i],1);
		}
	}
	
	//Food C
	if (fc == 0)
	{
		lcd_string(2,3,"NO ORDER");
	}
	else
	{
		for(int i=0;i<fc;i++)
		{
			lcd_numeric_value(2,3+i,c[i],1);
		}
	}
	buzzer_on();
}


/*
*
* Function Name - led_config
* Input			- None
* Output		- None
* Logic			- to configure the pins used by LEDs
* Example Call  - led_config()
*
*/
void led_config(void)
{
	DDRH = DDRH | 0xFF;
	PORTH = PORTH | 0x00;
}


/*
*
* Function Name - blue_led_on
* Input			- None
* Output		- None
* Logic			- turn on the blue LED
* Example Call  - blue_led_on()
*
*/
void blue_led_on(void)
{
	PORTH = PORTH & 0x00;
	PORTH = PORTH | 0x20;
}

/*
*
* Function Name - green_led_on
* Input			- None
* Output		- None
* Logic			- turn on the green LED
* Example Call  - green_led_on()
*
*/
void green_led_on(void)
{
	PORTH = PORTH & 0x00;
	PORTH = PORTH | 0x40;
}


/*
*
* Function Name - red_led_on
* Input			- None
* Output		- None
* Logic			- turn on the red LED
* Example Call  - red_led_on()
*
*/
void red_led_on(void)
{
	PORTH = PORTH & 0x00;
	PORTH = PORTH | 0x10;
}


/*
*
* Function Name - all_led_off
* Input			- None
* Output		- None
* Logic			- turning off all LEDs
* Example Call  - all_led_off()
*
*/
void all_led_off(void)
{
	PORTH = PORTH & 0x00;
}


/*
*
* Function Name - buzzer_pin_config
* Input			- None
* Output		- None
* Logic			- to configure the pins used by buzzer
* Example Call  - buzzer_pin_config()
*
*/
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}


/*
*
* Function Name - buzzer_on
* Input			- None
* Output		- None
* Logic			- turn on the buzzer
* Example Call  - buzzer_on()
*
*/
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}


/*
*
* Function Name - buzzer_off
* Input			- None
* Output		- None
* Logic			- turn off the buzzer
* Example Call  - buzzer_off()
*
*/
void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

/*
*
* Function Name - print
* Input			- None
* Output		- None
* Logic			- to print the values of white line sensor on the LCD display
* Example Call  - print()
*
*/
void print(void)
{
	//Printing the sensor value
	lcd_numeric_value(1, 1, Left, 3);		//Left Sensor
	lcd_numeric_value(1, 5, Center, 3);		//Center Sensor
	lcd_numeric_value(1, 9, Right, 3);		//Right Sensor
}


/*
*
* Function Name - read
* Input			- None
* Output		- None
* Logic			- to read the value from white line sensor
* Example Call  - read()
*
*/
void read(void)
{
	//Reading sensor value
	Left = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right = ADC_Conversion(1);	//Getting data of Right WL Sensor
}

//Main Function
int main(void)
{
    //Calling all the init fucntions to initialize and configure all the ports and pins
	init_devices();
    lcd_set_4bit();
    lcd_init();
	
	//Infinite loop to run the code endlessly
	while(1)
    {
        //Reading the white line sensor value
		read();
		
		//Printing the white line sensor value for better Tunning
		print();
		
		//Checking if a node is there or not
		if (Center >= thresh && Left >= thresh && Right >= thresh)
		{
				stop();
				lcd_clear();
				node++;				//Incrementing the node counter
				turn = node/x;		//Setting appropriate value of turn counter
				_delay_ms(1000);
			
				if ((node == 1 && count == 0) || (node == 1 && count == 5))		//If the node is to be followed by a left turn
				{
					//Logic to turn left accurately
					forward();
					_delay_ms(500);
					hard_left();
					_delay_ms(200);
					read();
					while(Center < thresh)
					{
						hard_left();
						read();
					}
					right();
					_delay_ms(100);
					forward();
					_delay_ms(500);
					node = x-1;
				}	
				
				else						//Else turning right or taking an order
				{
					
					if (turn > 0)			//Turning right, since it has completed one side of the restaurant
					{
						//Logic to turn right accurately
						forward();
						_delay_ms(500);
						hard_right();
						_delay_ms(100);
						read();
						while(Center < thresh)
						{
							hard_right();
							read();
						}
						left();
						_delay_ms(100);
						forward();
						_delay_ms(500);
						
						//Changing the value of counters appropirately
						turn = 0;
						node = 0;
						count++;
					}
					
					else 
					{
						
						if (count != 4)
						{
							forward();
							_delay_ms(100);
							
							if (count < 4)			//Taking orders as there are more tables
							{
								stop();
								_delay_ms(100);
								take_order();
								_delay_ms(100);
								forward();
							}
							
							else					//Placing orders as its has entered the kitchen area
							{
								stop();
								_delay_ms(100);
								place_order();
								_delay_ms(100);
								forward();
							}						
							
							forward();
							_delay_ms(500);
						}
												
					}
					
				}
				
				if (count == 4 && node == 1) //Condition where robot moves right to reach the kitchen
				{
					//Logic to turn right accurately
					forward();
					_delay_ms(500);
					hard_right();
					_delay_ms(100);
					read();
					while(Center < thresh)
					{
						hard_right();
						read();
					}
					left();
					_delay_ms(100);
					forward();
					_delay_ms(300);
					
					//Changing the values of counters
					count++;
					node = 0;
					x = 2;
				}	
				
				if (count == 9 && node == 1) //Condition where robot moves left to reach the start point
				{
					//Logic to turn left accurately
					forward();
					_delay_ms(500);
					hard_left();
					_delay_ms(200);
					read();
					while(Center < thresh)
					{
						hard_left();
						read();
					}
					right();
					_delay_ms(100);
					forward();
					_delay_ms(500);
				}
				
				if (count == 10)	//Condition where robot reaches a node and has black line in 4 direction, it goes straight to reach START point
				{
					forward();
					_delay_ms(500);
					count++;
				}
				
				if (count == 11)	//If the robot senses the start node, it shows all the orders program terminates
				{
					stop();
					_delay_ms(100);
					show_order();
					_delay_ms(1000);
					exit(0);
				}
											
		}
		
		//Turning left is left sensor is sensing a black line
		else if (Left >= thresh)
		{
			left();
			read();
			while(Center < thresh || Left >= thresh)
			{
				left();
				_delay_ms(50);
				read();
			}
		}
		
		//Turning right is right sensor is sensing a black line
		else if (Right >= thresh)
		{
			right();
			read();
			while(Center < thresh || Right > thresh)
			{
				right();
				read();
			}
		}
		
		//Moving forward is center and right sensor is sensing a black line
		else if (Center >= thresh || Right >= thresh)
		{
			forward();
			_delay_ms(100);
		}
		
		//Stoping if all the sensor are sensing white area
		else
		{
			stop();
			_delay_ms(100);
		}						
    }
}