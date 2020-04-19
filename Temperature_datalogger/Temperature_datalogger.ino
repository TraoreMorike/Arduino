/**********************************************
 * Macro definitions
 *********************************************/
#define UART_DEBUG          1 // Set to 0 to desactivate debug

#define R1  100000
#define C1  0.7203283552e-3 
#define C2  2.171656865e-4 
#define C3  0.8706070062e-7

#define THERMISTOR1_PIN  0

#ifdef UART_DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
#endif


/**********************************************
 * Global variable definitions
 *********************************************/
uint32_t  counter = 0; 

float temperature = 0;
 /*********************************************************/
void setup() {
  
  /* Initialize UART communication at 9600 bauds */
  Serial.begin(9600);

  /* Initialize OpenLog communication at 9600 bauds */
  Serial1.begin(115200);

}

/*********************************************************/
void loop() {

  getTemperature(analogRead(THERMISTOR1_PIN), &temperature);
  
  counter++;

  /* Debug interface */
    #if UART_DEBUG
    
      /* Clear debug screen (use Tera Term for this feature) */
      //DEBUG_PRINT("\x1b[2J\x1b[;H");
      
      DEBUG_PRINT(counter), DEBUG_PRINT("\t"), DEBUG_PRINTLN(temperature);
      Serial1.print(counter), Serial1.print("\t"), Serial1.println(temperature); 
      
    #endif

  delay(500);

}

/*********************************************************/
void getTemperature(int adcResult, float* temperature) {
  
  float logR2, R2;
  
  R2 = R1 * (1024.0/(float)adcResult - 1.0);

  logR2 = log(R2);
        
  *temperature =  (1.0 / (C1 + C2*logR2 + C3*logR2*logR2*logR2));
        
  *temperature = *temperature - 273.15;
}
