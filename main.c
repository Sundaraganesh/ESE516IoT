/**

/ This is the CLI code of group 12: Wait for Waste! 
Members: Sundaraganesh Rengarajan
Members: Sethupriya Menon

 */

//USART source code is from the website: http://asf.atmel.com/docs/3.28.1/samd21/html/asfdoc_sam0_sercom_usart_callback_use_case.html
//I2C is from the website: http://asf.atmel.com/docs/3.24.2/saml21/html/asfdoc_sam0_sercom_i2c_master_basic_use_case.html

//GPIO set: http://asf.atmel.com/docs/3.34.1/samd21/html/asfdoc_sam0_port_basic_use_case.html

#include <asf.h>
#include <usart.h>
#include <stdio.h>
#include <stdio_serial.h>
#include <i2c_master.h>
#include <delay.h>
#include <string.h>
#include "adc_temp.h"


#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- ese516 CLI example --"STRING_EOL \
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

float coarse_temp; /* Coarse value of the temperature - tempC */
float fine_temp;   /* Finer value of the temperature - tempF */

float tempR;       /* Production Room Temperature value read from NVM memory - tempR */
float tempH;	   /* Production Hot Temperature value read from NVM memory - tempH */
float INT1VR;      /* Room temp 2s complement of the internal 1V reference value - INT1VR */
float INT1VH;	   /* Hot temp 2s complement of the internal 1V reference value - INT1VR */
uint16_t ADCR;     /* Production Room Temperature ADC Value read from NVM memory - ADCR */
uint16_t ADCH;     /* Production Hot Temperature ADC Value read from NVM memory - ADCH */

float VADCR;	   /* Room Temperature ADC voltage - VADCR */
float VADCH;	   /* Hot Temperature ADC voltage - VADCH */


//Function declarations

void gpio_set(int);
void gpio_clear(int);
void gpio_get(int);
void configure_port_pins(int);
void adc_get(void);
void ipAddressFinder (void);
void configure_adc_fortemp(void);

//USART
#define MAX_RX_BUF_LENGTH   5
#define EDBG_CDC_MODULE   SERCOM3 
volatile uint8_t rx_buf[MAX_RX_BUF_LENGTH];
struct usart_module usart_instance;
void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);
void configure_usart(void);
void configure_usart_callbacks(void);
void mcuTempSense (void);
uint16_t adc_read_initial(void);
void macAddressFinder (void);


//I2C
#define DATA_LENGTH 5
#define SLAVE_ADDRESS 0X29
//#define TIMEOUT 1000
static uint8_t readBuffer[DATA_LENGTH];
static uint8_t writeBuffer[DATA_LENGTH] = {0x01,0X02,0X03,0X04,0X05};
struct i2c_master_module i2c_master_instance;
void config_i2c_master(void);
struct i2c_master_packet packet=
{
	.address= SLAVE_ADDRESS,
	.data_length = DATA_LENGTH,
	.data        = writeBuffer,
	.ten_bit_address = false,
	.high_speed      = false,
	.hs_master_code  = 0x0,
	
	
};

struct i2c_master_packet read_packet=
{
	.address= SLAVE_ADDRESS,
	.data_length = DATA_LENGTH,
	.data        = readBuffer,
	.ten_bit_address = false,
	.high_speed      = false,
	.hs_master_code  = 0x0,
	
	
};


//GPIO
//void gpio_set(char arg1[],char arg2[]);
//void gpio_clear(char arg1[],char arg2[]);
//void gpio_get(char arg1[],char arg2[]);
//struct port_config config_port_pin;

//Other
void printcommands(void);
void printversionbl(void);
bool processUserInput(void);
void printverapp(void);
void i2cScan(void);
uint16_t read_IRSensor(int readingCount, int readInterval);




/*Functions for USART configuration*/
void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate    = 9600;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;       //	USART_RX_1_TX_0_XCK_1 = (SERCOM_USART_CTRLA_RXPO(1) | SERCOM_USART_CTRLA_TXPO(0)),
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PA20D_SERCOM3_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PA21D_SERCOM3_PAD3;
	config_usart.generator_source = GCLK_GENERATOR_0;
	
	stdio_serial_init(&usart_instance, SERCOM3, &config_usart);

	usart_enable(&usart_instance);
}
void usart_read_callback(struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_instance,
	(uint8_t *)rx_buf, MAX_RX_BUF_LENGTH);
}

void usart_write_callback(struct usart_module *const usart_module)
{
	uint8_t gpio_pin = 5;
	port_pin_toggle_output_level(gpio_pin);
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}



/*Functions for I2C*/
void config_i2c_master(void)
{
	struct i2c_master_config config_i2c_master;
	
	i2c_master_get_config_defaults(&config_i2c_master);
	
	config_i2c_master.buffer_timeout =65535;
	config_i2c_master.pinmux_pad0       = PINMUX_PA08D_SERCOM2_PAD0;
	config_i2c_master.pinmux_pad1       = PINMUX_PA09D_SERCOM2_PAD1;
	config_i2c_master.generator_source  = GCLK_GENERATOR_0;
	while(i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master)!=STATUS_OK);
	i2c_master_enable(&i2c_master_instance);
	
	
}

/*Function for help*/
void printHelpCommands()
{
	printf("\nHere are some example commands\n");
	printf("help		:Prints all the available commands and its function\n");
	printf("ver_bl		:Prints the bootloader firmware version\n");
	printf("ver_app		: Prints the application code firmware version\n");
	printf("mac		:returns the mac address of the device\n");
	printf("ip		:returns the ip address of the device in the standard format\n");
	printf("mcu_temp	:Prints the temperature of the mcu from the on-board temp sensor in celsius\n");
	printf("i2c_scan	:Prints out a list of addresses for devices on the I2C bus after scanning through all possible 7-bit combinations\n");
	printf("gpio_set [port] [pin number]			:Set a GPIO pin to high\n");
	printf("gpio_clear  [port] [pin number]			: Set a GPIO pin to low\n");
	printf("gpio_get [port] [pin number]			:Get state of specified GPIO pin\n");
	printf("read_<sensor name> [readings] [interval_ms]	:Prints a number of readings at the given interval\n");
	printf("adc_get [port] [pin number]			:Get the ADC value of the given pin\n");
}

 void printVersion()
 {
	printf("The Boot-loader Version is 1.4.88\n");
 }

 void printAppVersion()
 {
	printf("The Application Code Firmware Version is 1.2.77\n");
 }


/* User input Processing*/
#define CR '\r'
#define LF '\n'
#define BS '\b'
#define NULLCHAR '\0'
#define SPACE ' '

#define MAX_RX_BUFFER_LENGTH   100
volatile uint8_t rx_buffer[MAX_RX_BUF_LENGTH];
uint8_t numberCharsRead;

/*Process user input*/
bool processUserInput(void)
{
	char singleCharInput;
	volatile enum status_code uartReadCode = usart_read_buffer_wait(&usart_instance, &singleCharInput, 1);
	if(STATUS_OK != uartReadCode)
	{
		return false;
	}
	if(STATUS_OK == uartReadCode)
	{
		volatile enum status_code uartWriteCode = usart_write_buffer_wait(&usart_instance, &singleCharInput, 1);
	}
	switch (singleCharInput)
	{
		case CR:
		case LF:
		/// On carriage return (CR) or line feed (LF), the user has hit enter and it's time to process their command.
		/// Remember to null terminate your strings!  Otherwise, you could keep reading throughout memory.
		rx_buffer[numberCharsRead] = NULLCHAR;
		if(numberCharsRead > 0)
		{
			numberCharsRead = 0;
			return true;
		}
		break;
		
		case BS:
		/// User input a backspace -- remove the character
		numberCharsRead--;
		rx_buffer[numberCharsRead] = NULLCHAR;
		
		
		break;

		default:
		
		if( numberCharsRead < MAX_RX_BUFFER_LENGTH )
		{
			rx_buffer[numberCharsRead++] = singleCharInput;
		}
		rx_buffer[numberCharsRead] = NULLCHAR;  
		break;
	}
	return false;
	
	
}


void processCommand(void)
{
	printf("\r\nYour entered command was: ");
	printf(rx_buffer);
	printf("\r\n");
	#define MAX_ARGS 4	
	
	#define DELIMITER " "
	int16_t nargs = 0;
	char *midStringPtr;
	char *arguments[MAX_ARGS];
	arguments[nargs++] = rx_buffer;
	midStringPtr = strpbrk(rx_buffer, DELIMITER);
	while (midStringPtr != NULL) 
	{
		arguments[nargs] = midStringPtr+1;
		
		*midStringPtr = '\0';
		midStringPtr = strpbrk(arguments[nargs], DELIMITER);
		nargs++;
	}

	int8_t command_okay = 0;
	if(0==strcmp("help", rx_buffer))
	{
		command_okay = 1;
		printHelpCommands();
		
	}

	else if(0==strcmp("ver_bl", rx_buffer))
	{	
		command_okay = 1;
		printVersion();
	}

	else if (0==strcmp("ver_app", rx_buffer))
	{
		command_okay = 1;
		printAppVersion();
		
	}
	else if(0==strcmp("gpio_set\0", arguments[0]))
	{
		command_okay = 1;

		// For the moment, We are comparing for two pins
		if(0==strcmp("23", arguments[2]))
		{
			gpio_set(1);
			//printf("Going for pin 23");
		}
		if(0==strcmp("17", arguments[2]))
		{
			gpio_set(0);
		}		
		
	}
	else if(0==strcmp("gpio_clear\0", arguments[0]))
	{
		command_okay = 1;
	
		// For the moment, We are comparing for two pins
		if(0==strcmp("23", arguments[2]))
		{
			gpio_clear(1);
			//printf("Going for pin 23");
		}
		if(0==strcmp("17", arguments[2]))
		{
			gpio_clear(0);
		}		
		
	}
	else if(0==strcmp("gpio_get\0", arguments[0]))
	{
		command_okay = 1;
		printf("before strcpy\n");
		
		// For the moment, We are comparing for two pins
		if(0==strcmp("23", arguments[2]))
		{
			gpio_get(1);
			//printf("Going for pin 23");
		}
		if(0==strcmp("17", arguments[2]))
		{
			gpio_get(0);
		}		
		
	}
	
	else if(0==strcmp("i2c_scan", arguments[0]))
	{
		command_okay = 1;
		i2cScan();
	}

	else if(0==strcmp("adc_get", arguments[0]))
	{
		command_okay = 1;
		if(strcmp(arguments[1],"A") || atoi(arguments[2])!=2)
		{
			printf("Wrong arguments\n");
			command_okay = 0;
		}
		else
		{
			adc_get();
		}
	
	}
		
	else if(0==strcmp("read_IRSensor", arguments[0]))
	{
		command_okay = 1;
		
		uint16_t ADCValue;
		
		if(atoi(arguments[1]) < 0) 
		{
			printf("Wrong arguments for count\n");
			command_okay =0;
		}
		
		if(atoi(arguments[2]) < 0) 
		{
			printf("Wrong arguments for Interval\n");
		}
		
		else
		{
		
			int readingCount = atoi(arguments[1]);
			int readInterval = atoi(arguments[2]);
			printf("The current value of readingCount %d and readInterval is %d\n", readingCount, readInterval);
			ADCValue = read_IRSensor( readingCount, readInterval);
		}
		//printf("The average value of the read_IRSensor is %d\n", ADCValue);
			
	}

	else if(0==strcmp("mcu_temp\0", arguments[0]))
	{
		command_okay = 1;
		mcuTempSense ();
				
	}
	
	else if(0==strcmp("mac\0", arguments[0]))
	{
		command_okay = 1;
		macAddressFinder();
		
	}
	
	else if(0==strcmp("ip\0", arguments[0]))
	{
		command_okay = 1;
		ipAddressFinder();
		
	}
	
	else if(0==strcmp("waitforwaste\0", arguments[0]))
	{
		uint16_t levelOfWaste = 0;
		command_okay = 1;
		
		printf("Please wait for the waste to fill\n");
		while(1)
		{
			levelOfWaste = read_IRSensor(100,100);
			if(levelOfWaste> 2000 && levelOfWaste < 2400); break;
		}
		
		printf("The dumpster is full!!!!!");
		gpio_set(1);
		
		
		
	}
	
	if (!command_okay) 
		printf("ERROR: Unknown command\r\n");
	
	
	printf("\r\n\r\n");
	
}

//ADC functions

void configure_adc(void);
uint16_t adc_mainfunction(void);

struct adc_module adc_instance;

void configure_adc(void)
{
	struct adc_config config_adc;
	
	adc_get_config_defaults(&config_adc);
	config_adc.reference = ADC_REFERENCE_AREFA;

	
	#if (SAMC21)
	adc_init(&adc_instance, ADC1, &config_adc);
	#else
	adc_init(&adc_instance, ADC, &config_adc);
	#endif
	
	adc_enable(&adc_instance);
}

uint16_t adc_mainfunction(void)
{
	configure_adc();
	
	adc_start_conversion(&adc_instance);
	
	uint16_t result;

	do 
	{
		// Do nothing!
	} while (adc_read(&adc_instance, &result) == STATUS_BUSY);
	
	//while (1) {	printf("ADC Result is: %d\n",result);
	adc_disable(&adc_instance); //MCU Temp Sensor

		//break;
	return result;
	//}
}


//Functions 

struct port_config config_port_pin;

void configure_port_pins(int option)
{
	if (option == 1) //PA23
	{
		port_get_config_defaults(&config_port_pin);
		config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
		//config_port_pin.input_pull = PORT_PIN_PULL_UP;
		port_pin_set_config(PIN_PA23, &config_port_pin);
	}
	else //PA11
	{
		port_get_config_defaults(&config_port_pin);
		config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
		//config_port_pin.input_pull = PORT_PIN_PULL_UP;
		port_pin_set_config(PIN_PA11, &config_port_pin);
	}

	
}

void gpio_set(int option)
{
	configure_port_pins(option);
	printf("The option is %d", option);
	
	//printf("The command received is %s",portPin);
	
	bool pin_state = true;
	if(option ==1)
	{
		port_pin_set_output_level(PIN_PA23, !pin_state);
		printf("The pin is set to 1");
	}
	else
	{
		port_pin_set_output_level(PIN_PA17, !pin_state);
		printf("The pin is set to 1");
	}
}

void gpio_clear(int option)
{
	configure_port_pins(option);
	
	bool pin_state = false;

	if(option ==1)
	{
		port_pin_set_output_level(PIN_PA23, !pin_state); // Took ! to work for the logic of internal LED
		printf("The pin is set to 0");
	}
	else
	{
		port_pin_set_output_level(PIN_PA17, !pin_state); // Took ! to work for the logic of internal LED
		printf("The pin is set to 0");
	}

}

void gpio_get(int option)
{
	bool pinOutput;
	configure_port_pins(option);
	
	if(option ==1)
	{
		pinOutput = port_pin_get_output_level(PIN_PA23);
		printf("The output pin value is %d\n", !pinOutput);
	}
	else
	{
		pinOutput = port_pin_get_output_level(PIN_PA17);
		printf("The output pin value is %d\n", !pinOutput);
	}
}

void macAddressFinder (void)
{
	printf("The device mac address is ABCDEFGHIJKL\n");
}


void ipAddressFinder (void)
{
	printf("The device IP address is AA:BB:CC:DD:EE:FF\n");
}


uint16_t read_IRSensor(int readingCount, int readInterval)
{
	
	//printf(" Inside the function the current value of readingCount %d and readInterval is %d\n", readingCount, readInterval);

	uint16_t adcValue = 0;
	
	for(int i =0; i<readingCount; i++)
	{
		//printf("Count is %d\n",i);
		uint16_t adcValueCurrent = adc_mainfunction();
		adcValue = adcValue + adcValueCurrent;
		delay_ms(readInterval);
		printf("Reading count is %d and the value is %d\n",i, adcValueCurrent);
	}
	uint16_t movingAverage = adcValue/readingCount;
	return movingAverage;
}


void adc_get(void)
{
	int adcValue = 	adc_mainfunction();
	printf("The ADC value is %d", adcValue);
}

void mcuTempSense (void)
{
	//**************************************MCU temp sensor

	configure_adc_fortemp();
	
	load_calibration_data();
	
	uint16_t raw_temp;
	raw_temp = adc_read_initial();
	
	int temperature = calculate_temperature(raw_temp);

	printf("\nThe MCU temperature is = %d degree C", temperature);

	adc_disable(&adc_instance);
	
	//************************************End MCU Temp sensor

}

//Temp
//double at30tse_read_temperature	(void);

//I2C scan

void i2cScan(void)
{
	int addressLoop;
	enum status_code i2c_status;
	
	//printf("Going into the main loop\n");
	
	for(addressLoop=0; addressLoop <128; addressLoop++)
	{
		//SLAVE_ADDRESS = addressloop;
		
		//printf("Scanning for the slave address %d\n", addressLoop);
		
		packet.address     = addressLoop;
		packet.data_length = 1;
		writeBuffer[0]     = 0xC0;
		packet.data        = writeBuffer;
		
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
		//printf("I2C status is %d", (int)i2c_status);
		if( i2c_status == STATUS_OK ){ i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &packet); }
		i2c_master_send_stop(&i2c_master_instance);
		
		if(i2c_status == STATUS_OK) printf("The slave address is %x\n",addressLoop);
		
	}
}
//**********************************MCU Temp Sensor
int tempstatus = 1;
//int temperature;


//! [module_inst]
struct adc_module adc_instance;
//! [module_inst]

//! [setup]
void configure_adc_fortemp(void)
{
	//! [setup_config]
	struct adc_config config_adc;
	//! [setup_config]
	//! [setup_config_defaults]
	adc_get_config_defaults(&config_adc);
	//! [setup_config_defaults]
//conf_adc.clock_source = GCLK_GENERATOR_1;
config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
config_adc.reference = ADC_REFERENCE_INT1V;
config_adc.positive_input = ADC_POSITIVE_INPUT_TEMP;
config_adc.negative_input = ADC_NEGATIVE_INPUT_GND;
config_adc.sample_length = ADC_TEMP_SAMPLE_LENGTH;

system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE);
	//! [setup_set_config]
	#if (SAMC21)
	tempstatus = adc_init(&adc_instance, ADC1, &config_adc);
	#else
	tempstatus = adc_init(&adc_instance, ADC, &config_adc);
	#endif
	//! [setup_set_config]
ADC->AVGCTRL.reg = ADC_AVGCTRL_ADJRES(2) | ADC_AVGCTRL_SAMPLENUM_4;
	//! [setup_enable]
	adc_enable(&adc_instance);
	//! [setup_enable]
}
//! [setup]
uint16_t adc_read_initial(void)
{
	uint16_t adc_readvalue = 0;

	adc_start_conversion(&adc_instance);
	while((adc_get_status(&adc_instance) & ADC_STATUS_RESULT_READY) != 1);

	adc_read(&adc_instance, &adc_readvalue);

	return adc_readvalue;
}
void load_calibration_data(void)
{
	volatile uint32_t val1;				/* Temperature Log Row Content first 32 bits */
	volatile uint32_t val2;				/* Temperature Log Row Content another 32 bits */
	uint8_t room_temp_val_int;			/* Integer part of room temperature in ?C */
	uint8_t room_temp_val_dec;			/* Decimal part of room temperature in ?C */
	uint8_t hot_temp_val_int;			/* Integer part of hot temperature in ?C */
	uint8_t hot_temp_val_dec;			/* Decimal part of hot temperature in ?C */
	int8_t room_int1v_val;				/* internal 1V reference drift at room temperature */
	int8_t hot_int1v_val;				/* internal 1V reference drift at hot temperature*/
	
	uint32_t *temp_log_row_ptr = (uint32_t *)NVMCTRL_TEMP_LOG;
	
	val1 = *temp_log_row_ptr;
	temp_log_row_ptr++;
	val2 = *temp_log_row_ptr;
	
	room_temp_val_int = (uint8_t)((val1 & NVMCTRL_FUSES_ROOM_TEMP_VAL_INT_Msk) >> NVMCTRL_FUSES_ROOM_TEMP_VAL_INT_Pos);
	
	room_temp_val_dec = (uint8_t)((val1 & NVMCTRL_FUSES_ROOM_TEMP_VAL_DEC_Msk) >> NVMCTRL_FUSES_ROOM_TEMP_VAL_DEC_Pos);
	
	hot_temp_val_int = (uint8_t)((val1 & NVMCTRL_FUSES_HOT_TEMP_VAL_INT_Msk) >> NVMCTRL_FUSES_HOT_TEMP_VAL_INT_Pos);
	
	hot_temp_val_dec = (uint8_t)((val1 & NVMCTRL_FUSES_HOT_TEMP_VAL_DEC_Msk) >> NVMCTRL_FUSES_HOT_TEMP_VAL_DEC_Pos);
	
	room_int1v_val = (int8_t)((val1 & NVMCTRL_FUSES_ROOM_INT1V_VAL_Msk) >> NVMCTRL_FUSES_ROOM_INT1V_VAL_Pos);
	
	hot_int1v_val = (int8_t)((val2 & NVMCTRL_FUSES_HOT_INT1V_VAL_Msk) >> NVMCTRL_FUSES_HOT_INT1V_VAL_Pos);
	
	ADCR = (uint16_t)((val2 & NVMCTRL_FUSES_ROOM_ADC_VAL_Msk) >> NVMCTRL_FUSES_ROOM_ADC_VAL_Pos);
	
	ADCH = (uint16_t)((val2 & NVMCTRL_FUSES_HOT_ADC_VAL_Msk) >> NVMCTRL_FUSES_HOT_ADC_VAL_Pos);
	
	tempR = room_temp_val_int + convert_dec_to_frac(room_temp_val_dec);
	
	tempH = hot_temp_val_int + convert_dec_to_frac(hot_temp_val_dec);
	
	INT1VR = 1 - ((float)room_int1v_val/INT1V_DIVIDER_1000);
	
	INT1VH = 1 - ((float)hot_int1v_val/INT1V_DIVIDER_1000);
	
	VADCR = ((float)ADCR * INT1VR)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
	
	VADCH = ((float)ADCH * INT1VH)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
}

float convert_dec_to_frac(uint8_t val)
{
	if (val < 10)
	{
		return ((float)val/10.0);
	}
	
	else if (val <100)
	{
		return ((float)val/100.0);
	}
	
	else
	{
		return ((float)val/1000.0);
	}
}

float calculate_temperature(uint16_t raw_code)
{
	float VADC;      /* Voltage calculation using ADC result for Coarse Temp calculation */
	float VADCM;     /* Voltage calculation using ADC result for Fine Temp calculation. */
	float INT1VM;    /* Voltage calculation for reality INT1V value during the ADC conversion */
	
	VADC = ((float)raw_code * INT1V_VALUE_FLOAT)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
	
	/* Coarse Temp Calculation by assume INT1V=1V for this ADC conversion */
	coarse_temp = tempR + (((tempH - tempR)/(VADCH - VADCR)) * (VADC - VADCR));
	
	/* Calculation to find the real INT1V value during the ADC conversion */
	INT1VM = INT1VR + (((INT1VH - INT1VR) * (coarse_temp - tempR))/(tempH - tempR));
	
	VADCM = ((float)raw_code * INT1VM)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
	
	/* Fine Temp Calculation by replace INT1V=1V by INT1V = INT1Vm for ADC conversion */
	fine_temp = tempR + (((tempH - tempR)/(VADCH - VADCR)) * (VADCM - VADCR));
	
	return fine_temp;
}


//***************************************MCU Temp Sensor
int main (void)
{
	/* Board initialize*/
	system_init();
	system_interrupt_enable_global();

	/*Initialize USART*/
	configure_usart();
	configure_usart_callbacks();
	delay_init();
	


	printf("Hello!!!!!!!!\n");
	printf("\r\n\r\n");
	printf(STRING_HEADER);
	
	/*Initialize I2C*/
	
	
	config_i2c_master();
	enum status_code status;
	
	
	
	packet.address     = SLAVE_ADDRESS;
	packet.data_length = 1;
	writeBuffer[0]          = 0xC0;
	packet.data        =writeBuffer;
	status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
	if( status == STATUS_OK ){ status = i2c_master_read_packet_wait(&i2c_master_instance, &read_packet); }
	i2c_master_send_stop(&i2c_master_instance);
	  
	writeBuffer[0]          = 0xC1;
	packet.data        =writeBuffer;
	status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
	if( status == STATUS_OK ){ status = i2c_master_read_packet_wait(&i2c_master_instance, &read_packet); }
	i2c_master_send_stop(&i2c_master_instance);
	  
	writeBuffer[0]          = 0xC2;
	packet.data        =writeBuffer;
	status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
	if( status == STATUS_OK ){ status = i2c_master_read_packet_wait(&i2c_master_instance, &read_packet); }
	i2c_master_send_stop(&i2c_master_instance);
	  
	writeBuffer[0]          = 0x51;
	packet.data        =writeBuffer;
	status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
	if( status == STATUS_OK ){ status = i2c_master_read_packet_wait(&i2c_master_instance, &read_packet); }
	i2c_master_send_stop(&i2c_master_instance);
	  
	writeBuffer[0]          = 0x61;
	packet.data        =writeBuffer;
	status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
	if( status == STATUS_OK ){ status = i2c_master_read_packet_wait(&i2c_master_instance, &read_packet); }
	i2c_master_send_stop(&i2c_master_instance);
	
	/* I2C initialization ends*/

	
	while (1)
	  {
		  bool commandEntered = processUserInput();
		  if(commandEntered)
		  {
			  processCommand();
			  
		  }
	  }

	return 0;
			
}
