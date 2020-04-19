/**
* @file     Fan_Controller.ino
* @brief    Fan controller consist of piloting two PWM fan to extract and filter hazardous smoke
* @author   Morike Traore
* @date     November 2019
* 
* 
*/

/**********************************************
 * Macro definitions
 *********************************************/
#define UART_DEBUG          1 // Set to 0 to desactivate debug

#define FAN_1_PWM_PIN	      9
#define	FAN_2_PWM_PIN	      10

#define FAN_1_TCH_PIN       0 //INT0
#define FAN_2_TCH_PIN	      1 //INT1

#define	POT_1_ADC_PIN	      15
#define	POT_2_ADC_PIN	      14

#define	FAN_POLES		        4
#define FAN_MIN_SPEED       30

#ifdef UART_DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
#endif

/**********************************************
 * Global variable definitions
 *********************************************/

uint16_t rawPot1 = 0;  
uint16_t rawPot2 = 0; 

uint8_t fan1Setpoint = 0;
uint8_t fan2Setpoint = 0;

uint16_t fan1Speed = 0;
uint16_t fan2Speed = 0;

volatile uint16_t rpsCountFan1 = 0; 
volatile uint16_t rpsCountFan2 = 0;

unsigned long prevMillis; 

/*********************************************************/
void setup() {
  
  /* Initialize UART communication at 9600 bauds */
	Serial.begin(9600);
 
  /* GPIO Configuration */
	pinMode(FAN_1_PWM_PIN,	OUTPUT);
	pinMode(FAN_2_PWM_PIN,	OUTPUT);
	
	pinMode(POT_1_ADC_PIN, INPUT); 
	pinMode(POT_2_ADC_PIN, INPUT);

  /* Interrupt Configuration */
	attachInterrupt(FAN_1_TCH_PIN, rpmFan1, CHANGE);
	attachInterrupt(FAN_2_TCH_PIN, rpmFan2 , CHANGE);
	
}

void loop() {

  /* Read control potentiometer */ 
  rawPot1 = analogRead(POT_1_ADC_PIN);
  rawPot2 = analogRead(POT_2_ADC_PIN); 

  /* Convert raw potentiometer values to PWM setpoint */
  fan1Setpoint = map(rawPot1, 0, 1023, FAN_MIN_SPEED, 255);
  fan2Setpoint = map(rawPot2, 0, 1023, FAN_MIN_SPEED, 255);

  /* Write PWM value */
  analogWrite(FAN_1_PWM_PIN, fan1Setpoint); 
  analogWrite(FAN_2_PWM_PIN, fan2Setpoint);

  /* Refresh debug values at 10Hz */
  if (millis() - prevMillis >= 100) {

    /* Read PWM fan speed */
  	fan1Speed = GetFanRPM(rpsCountFan1);
  	fan2Speed = GetFanRPM(rpsCountFan2);

    /* Clear pulse counter */
  	rpsCountFan1 = 0; 
  	rpsCountFan2 = 0;

  	prevMillis = millis();
    
    /****************************************************************************************/
    
    /* Debug interface */
    #if UART_DEBUG
    
      /* Clear debug screen (use Tera Term for this feature) */
      DEBUG_PRINT("\x1b[2J\x1b[;H");
      
      DEBUG_PRINT("OUTER FAN SETPOINT\t:\t"), DEBUG_PRINT(rawPot1), DEBUG_PRINT("\t"), DEBUG_PRINTLN(fan1Setpoint);
      DEBUG_PRINT("INNER FAN SETPOINT\t:\t"), DEBUG_PRINT(rawPot2), DEBUG_PRINT("\t"), DEBUG_PRINTLN(fan2Setpoint);
      
      DEBUG_PRINT("OUTER FAN SPEED\t:\t"), DEBUG_PRINT(fan1Speed), DEBUG_PRINTLN("\tRPM");
      DEBUG_PRINT("INNER FAN SPEED\t:\t"), DEBUG_PRINT(fan2Speed), DEBUG_PRINTLN("\tRPM");
      
    #endif

    /****************************************************************************************/
  }
  
  
}

/***************************************************************************************************************************/


void rpmFan1 (void) {
	/* Record tach pulse from FAN 1 */
	rpsCountFan1++;

}

void rpmFan2 (void) {
	/* Record tach pulse from FAN 1 */
	rpsCountFan2++;

}

/**
    Encodes a single digit of a POSTNET "A" bar code.

    @param    count, number of pulse during 100ms
    @return   Revolution per minute 
*/
uint16_t GetFanRPM(uint16_t count) {
	
	uint16_t rpm; 
	
	rpm = ((count * 10) * 60) / FAN_POLES; 
	
	return rpm;
}
