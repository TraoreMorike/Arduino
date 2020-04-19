/**
* @file     UAV_Anti_Collision_Lighting.ino
* @brief    Fan controller consist of piloting two PWM fan to extract and filter hazardous smoke
* @author   Morike Traore
* @date     November 2019
* 
* 
*/

/**********************************************
 * Macro definitions
 *********************************************/
#define UART_DEBUG                  1 // Set to 0 to desactivate debug

#define BEACON_LIGHTS_PIN           3
#define WING_STROBE_LIGHTS_PIN      5   //8
#define TAIL_STROBE_LIGHT_PIN       6

#define NAV_GREEN_LIGHT_PIN         9   //7
#define NAV_RED_LIGHT_PIN           10  //6 on test board
#define NAV_WHITE_LIGHT_PIN         12

#define PWM_CMD_PIN                 7

#define TIMING_CLOCK                500
#define TAIL_TIME_ON                50
#define TAIL_TIME_OFF               90
#define BEACON_TIME_ON              150
#define TAIL_WING_STROBE_TIME_ON    200

#define ALPHA                       0.25

#ifdef UART_DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
#endif

/**********************************************
 * Global variable definitions
 *********************************************/
uint8_t clockState = 1;
uint8_t lock = 1;
uint8_t lock1 = 0;
uint8_t timeout = 0;

uint8_t strobeLightsState = 0; 
uint8_t navLightState = 0; 
uint8_t beaconLightState = 0;


unsigned long T0 = 0;
unsigned long T1 = 0;

unsigned long rawPwmInput = 0;
unsigned long filteredPwmInput = 0;
unsigned long prevFilteredPwmInput = 0;

/*********************************************************/
void setup() {

  Serial.begin(9600); 
  
  /* GPIO Configuration */
  pinMode(PWM_CMD_PIN,              INPUT);
  
  pinMode(BEACON_LIGHTS_PIN,        OUTPUT);
  pinMode(WING_STROBE_LIGHTS_PIN,   OUTPUT);
  pinMode(TAIL_STROBE_LIGHT_PIN,    OUTPUT);
  
  pinMode(NAV_GREEN_LIGHT_PIN,      OUTPUT);
  pinMode(NAV_RED_LIGHT_PIN,        OUTPUT);
  pinMode(NAV_WHITE_LIGHT_PIN,      OUTPUT);

  digitalWrite(BEACON_LIGHTS_PIN,       LOW);
  digitalWrite(WING_STROBE_LIGHTS_PIN,  LOW);
  digitalWrite(TAIL_STROBE_LIGHT_PIN,   LOW);
  
  digitalWrite(NAV_GREEN_LIGHT_PIN,     HIGH);
  digitalWrite(NAV_RED_LIGHT_PIN,       HIGH);
  digitalWrite(NAV_WHITE_LIGHT_PIN,     HIGH);

}

/*********************************************************/
void loop() {

    rawPwmInput = pulseIn(PWM_CMD_PIN, HIGH);

    prevFilteredPwmInput = filteredPwmInput;
    
    filteredPwmInput = (1-ALPHA) * prevFilteredPwmInput + (rawPwmInput * ALPHA);
    
   
   if (filteredPwmInput >= 980) {
       beaconLightState = 1;
   } else {
       beaconLightState = 0;
   }

   if (filteredPwmInput >= 1470) {
       navLightState = 1;
       digitalWrite(NAV_GREEN_LIGHT_PIN,     HIGH);
       digitalWrite(NAV_RED_LIGHT_PIN,       HIGH);
       digitalWrite(NAV_WHITE_LIGHT_PIN,     HIGH);
   } else {
       navLightState = 0;
       digitalWrite(NAV_GREEN_LIGHT_PIN,     LOW);
       digitalWrite(NAV_RED_LIGHT_PIN,       LOW);
       digitalWrite(NAV_WHITE_LIGHT_PIN,     LOW);
   }
   
   if (filteredPwmInput >= 1920) {
       strobeLightsState = 1;
   } else {
       strobeLightsState = 0;
       digitalWrite(WING_STROBE_LIGHTS_PIN, LOW);
   }
   
   /* 1Hz Clock used to synchronize LED lighting */ 
   if (millis() - T0 >= TIMING_CLOCK) {
    
    //Serial.println(filteredPwmInput);
    /* This variable change its state every 500ms */
    clockState = !clockState;

    /* Save T0 */
    T0 = millis();
    
  }

  /* At clockState TRUE Turn on the beacon until elapsed time - T0 is less than BEACON_TIME_ON */ 
  if ((clockState) && (millis() - T0 <= BEACON_TIME_ON))   {
    digitalWrite(BEACON_LIGHTS_PIN, HIGH);

  } else {
    /* 1Hz Clock used to synchronize LED lighting */
    digitalWrite(BEACON_LIGHTS_PIN, LOW);
  }

  /* At clockState FALSE Turn on the tail light until elapsed time - T0 is less than TAIL_WING_STROBE_TIME_ON */ 
  if ((!clockState) && (millis() - T0 <= TAIL_WING_STROBE_TIME_ON))   {
    digitalWrite(TAIL_STROBE_LIGHT_PIN, HIGH);
  
    if (lock && strobeLightsState) {

        /* At clockState FALSE Turn on the tail light until elapsed time - T0 is less than TAIL_WING_STROBE_TIME_ON */
        if (millis() - T0 <= TAIL_TIME_ON) {
          
          /* Turn On strobe lights */
          digitalWrite(WING_STROBE_LIGHTS_PIN, HIGH);

          /* Save T1 */
          T1 = millis();
        } else {
          if (!lock1) {
            digitalWrite(WING_STROBE_LIGHTS_PIN, LOW); // probleme ici
          }
        }

        /* Wait until elapsed time - t1 exceed strobe T_OFF */
        if (millis() - T1 >= TAIL_TIME_OFF) {
          /* Trigger a timeout */
          timeout = 1;
        }
        
       
        if (timeout) {
          
          if (millis() - T1 <= TAIL_TIME_ON + TAIL_TIME_OFF - 5) {
            digitalWrite(WING_STROBE_LIGHTS_PIN, HIGH);
            lock1 = 1;
          } else {
            digitalWrite(WING_STROBE_LIGHTS_PIN, LOW);
            /* Reset timeout */
            timeout = 0;
            lock = 0;
            lock1 = 0; 
          }
        } 
    }
    
    } else {
    /* Turn off TAIL  light */
    digitalWrite(TAIL_STROBE_LIGHT_PIN, LOW);
    lock = 1;
    }

    /* Debug interface */
    #if UART_DEBUG
    
      /* Clear debug screen (use Tera Term for this feature) */
      //DEBUG_PRINT("\x1b[2J\x1b[;H");
      
      DEBUG_PRINT(rawPwmInput),DEBUG_PRINT("\t"), DEBUG_PRINTLN(filteredPwmInput);
      
      
      
      
    #endif
}
