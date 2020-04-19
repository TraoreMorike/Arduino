/**********************************************
 * Macro definitions
 *********************************************/
#define UART_DEBUG          1 // Set to 0 to desactivate debug

#define R1  100000
#define C1  0.7203283552e-3 
#define C2  2.171656865e-4 
#define C3  0.8706070062e-7

#define ALPHA                       0.9

#define THERMISTOR1_PIN  0

#ifdef UART_DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
#endif

#define HEATING_CONTROL_PIN   9
#define THERMISTOR1_PIN       0
#define POT_PIN               1

/**********************************************
 * Global variable definitions
 *********************************************/
int rawPot = 0;

double temperature = 0;

unsigned long lastTime;

double Input;
double prevInput;
double Output;
double Setpoint;
double errSum;
double lastErr;
double kp, ki, kd;

/*********************************************************/
void setup() {
  
  /* Initialize UART communication at 9600 bauds */
  Serial.begin(9600);
  
  pinMode(HEATING_CONTROL_PIN,  OUTPUT);

  kp = 90;
  ki = 0.0015;
  kd = 0.0060;

}

/*********************************************************/
void loop() {
  
  rawPot = analogRead(POT_PIN);

  Setpoint = map(rawPot, 0, 1023, 45, 70);

  getTemperature(analogRead(THERMISTOR1_PIN), &temperature);
  
  prevInput = Input;
  
  Input = (1-ALPHA) * prevInput + (temperature * ALPHA);
  
  Compute();

  Output = constraints(Output,  0,  75);

  analogWrite(HEATING_CONTROL_PIN, Output);

  /* Debug interface */
    #if UART_DEBUG
    
      /* Clear debug screen (use Tera Term for this feature) */
      //DEBUG_PRINT("\x1b[2J\x1b[;H");
      
     DEBUG_PRINT(Setpoint), DEBUG_PRINT('\t'), DEBUG_PRINT(Input), DEBUG_PRINT("\t"), DEBUG_PRINTLN(Output);
   
    #endif

}

/*********************************************************/
void getTemperature(int adcResult, double* temperature) {
  
  float logR2, R2;
  
  R2 = R1 * (1024.0/(float)adcResult - 1.0);

  logR2 = log(R2);
        
  *temperature =  (1.0 / (C1 + C2*logR2 + C3*logR2*logR2*logR2));
        
  *temperature = *temperature - 273.15;
}

/*********************************************************/
void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
  
   /*Compute all the working error variables*/
   
   /* Calculate the error, P factor */
   double error = Setpoint - Input;
   
   /* Calculate the integrate, I factor */
   errSum += (error * timeChange);
   //errSum += (error * 200);
   
   /* Calculate the derivate, D factor*/
   double dErr = (error - lastErr) / timeChange;
   //double dErr = (error - lastErr) / 200;
   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
}

/*********************************************************/
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}

int constraints(int x, int min, int max) {
  return (x<=min) ? min : (x>=max) ? max : x;
}
