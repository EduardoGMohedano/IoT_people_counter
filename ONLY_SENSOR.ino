#include "image_proc.h"
#include <Melopero_AMG8833.h>
Melopero_AMG8833 amg;

#define   DEBUG   1
//#define   SIMPLE_COUNT    1
#define   AVERAGE_TEMP_FROM_MATRIX      1
//#define   AVERAGE_TEMP_FROM_THERMISTOR    1

//The high and low temperature thresholds. If a temperature that exceeds 
//these values is detected an interrupt will be triggered. The temperatures
//are expressed in Celsius degrees.
float highThreshold = 25.0;
float lowThreshold = 18;

volatile bool intReceived = false;
const byte interruptPin = 3;
#define LED_PERSON_ENTERED  2
#define LED_PERSON_LEFT     4

#define SENSOR_SIZE_PIXELS    (8)
#define IMAGE_SIZE_UP_FACTOR  (2)
#define NEW_IMAGE_SIZE        (SENSOR_SIZE_PIXELS * IMAGE_SIZE_UP_FACTOR)


//Structs to get person counter
typedef struct{
  int previous_state; //This states contain the number of persons in each zone
  int current_state;
}Zones;

Zones zone_1{0,0};
Zones zone_2{0,0};

uint16_t person_counter = 0;
float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];

//The interrupt servire routine. This function will be called each time the 
//interrupt is triggered.
void onInterrupt(){
  intReceived = true;
}

/***************************Function definitions*************************/

//Will help to determine the number of persons in the temperature matrix
void get_person_in_zones(bool pixels[8][8]){
  //Depending on height param, we will count the 1's in both zones and count people
  int ones = 0;
  //Analyzing zone 1
  for(int i=0; i<4; i++){
    for(int j=0; j<8; j++){
        if ( pixels[i][j] == 1) ones++;
      }  
  }
  zone_1.previous_state = zone_1.current_state;
  zone_1.current_state = ones/2;    //En vez de dividir entre dos, podemos hacer pruebas y ajustar con la altura incluso crear una mascara para saber cuantas personas hay

  ones = 0;
  //Analyzing zone 2
  for(int i=4; i<8; i++){
    for(int j=0; j<8; j++){
        if ( pixels[i][j] == 1) ones++;
      }  
  }
  zone_2.previous_state = zone_2.current_state;
  zone_2.current_state = ones/2;    //En vez de dividir entre dos, podemos hacer pruebas y ajustar con la altura

  #ifdef DEBUG
      Serial.print("Pixels in Zone 1 is:");
      Serial.println(zone_1.current_state);
      Serial.print("Pixels in Zone 2 is:");
      Serial.println(zone_2.current_state);

  #endif
  
}

void get_person_in_zones_with_max(float pixels[8][8]){
  //Depending on height param, we will count the 1's in both zones and count people
  int ones = 0;
  float average = 0.0;
  
  #ifdef   AVERAGE_TEMP_FROM_MATRIX
      //Getting average of first zone
      for(int i = 0; i < 4; i++){
        for(int j = 0; j < 8; j++){
          average += pixels[i][j];
        }
      }
      average/=32;
  #elif   AVERAGE_TEMP_FROM_THERMISTOR
      amg.updateThermistorTemperature();       //Update AMGs thermistor temperature and print it.
      average = amg.thermistorTemperature;
  #endif
  
  //Analyzing zone 1
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 8; j++){
      if ( pixels[i][j] > average) ones++;
     }
  }
  zone_1.previous_state = zone_1.current_state;
  zone_1.current_state = ones;    //Contando los maximos de temperatura

  ones = 0;
  average = 0.0;
  #ifdef   AVERAGE_TEMP_FROM_MATRIX
      //Getting average of first zone
      for(int i = 4; i < 8; i++){
        for(int j = 0; j < 8; j++){
          average += pixels[i][j];
        }
      }
      average/=32;
  #elif   AVERAGE_TEMP_FROM_THERMISTOR
      amg.updateThermistorTemperature();       //Update AMGs thermistor temperature and print it.
      average = amg.thermistorTemperature;
  #endif
  
  //Analyzing zone 2
  for(int i = 4; i < 8; i++){
    for(int j = 0; j < 8; j++){
      if ( pixels[i][j] > average) ones++;
    }
  }
  zone_2.previous_state = zone_2.current_state;
  zone_2.current_state = ones;   
}

void count_person(){
  //Moving from zone 1 to zone 2 means leaving the room
  if(  zone_2.current_state >  zone_2.previous_state ){
    if (  zone_1.current_state <  zone_1.previous_state  ){
      person_counter++;
      #ifdef DEBUG
          digitalWrite(LED_PERSON_ENTERED, HIGH);
          delay(100);
          digitalWrite(LED_PERSON_ENTERED, LOW);
          delay(50);
          digitalWrite(LED_PERSON_ENTERED, HIGH);
          delay(100);
          digitalWrite(LED_PERSON_ENTERED, LOW);
      #endif
    }
  }

  //Moving from zone 2 to zone 1 means getting into the room
  if(  zone_2.current_state <  zone_2.previous_state ){
    if (  zone_1.current_state >  zone_1.previous_state  ){
      person_counter--;
      #ifdef DEBUG
          digitalWrite(LED_PERSON_LEFT, HIGH);
          delay(100);
          digitalWrite(LED_PERSON_LEFT, LOW);
          delay(50);
          digitalWrite(LED_PERSON_LEFT, HIGH);
          delay(100);
          digitalWrite(LED_PERSON_LEFT, LOW);
      #endif
    }
  }
  Serial.print("Person counter is:");
  Serial.println(person_counter);
}




void setup() {
  Serial.begin(115200);

  // initializing I2C to use default address AMG8833_I2C_ADDRESS_B and Wire (I2C-0):
  Wire.begin();
  amg.initI2C();
  // To use Wire1 (I2C-1):
  // Wire1.begin();
  // amg.initI2C(AMG8833_I2C_ADDRESS_B, Wire1);

  //Reset the device flags and settings and read the returned status code.
  Serial.print("Resetting amg ... ");
  int statusCode = amg.resetFlagsAndSettings();
  Serial.println(amg.getErrorDescription(statusCode));   //Check if there were any problems.

  statusCode = amg.setFPSMode(FPS_MODE::FPS_10);        //Setting the FPS_MODE this can be: FPS_MODE::FPS_10 or FPS_MODE::FPS_1
  Serial.println(amg.getErrorDescription(statusCode));

  statusCode = amg.setInterruptThreshold(lowThreshold, highThreshold);    //Setting the threshold values
  Serial.println(amg.getErrorDescription(statusCode));
  
  statusCode = amg.enableInterrupt();      //Enable the interrupt.
  Serial.println(amg.getErrorDescription(statusCode));

  //Configuring the interrupt pin to listen for the interrupt.
  pinMode(interruptPin, INPUT_PULLUP);
  //Attaching our function onInterrupt to the interrupt. 
  attachInterrupt(digitalPinToInterrupt(interruptPin), onInterrupt, CHANGE);

  /*LEDs Config*/
  pinMode(LED_PERSON_ENTERED, OUTPUT);
  pinMode(LED_PERSON_LEFT, OUTPUT);

  digitalWrite(LED_PERSON_ENTERED, LOW);
  digitalWrite(LED_PERSON_LEFT, LOW);
}

void loop() {
  

  //Check if an interrupt occurred.
  //The interrupt occurred flag gets set by the interrupt service routine
  //each time an interrupt is triggered.
  if (intReceived){
    //Get pesons count on the zones
    #ifdef SIMPLE_COUNT
        amg.updateInterruptMatrix();
        get_person_in_zones(amg.interruptMatrix);
    //or
    #else
        amg.updatePixelMatrix();
        //get_person_in_zones_with_max(amg.pixelMatrix);
    #endif

    image_t src, dst; 
    src.pixels = (uint32_t*)amg.pixelMatrix;         //Asigning pointer to the begining of the matrix, now because of the typecasting some information from decimals is being lost multiply by 10 all the pixels to avoid it
    src.w = SENSOR_SIZE_PIXELS;
    src.h = SENSOR_SIZE_PIXELS;

    //Destiny image struct init
    dst.pixels = new uint32_t(NEW_IMAGE_SIZE* NEW_IMAGE_SIZE);
    dst.w = NEW_IMAGE_SIZE;
    dst.h = NEW_IMAGE_SIZE;

    scale(&src, &dst, (float) NEW_IMAGE_SIZE, (float) NEW_IMAGE_SIZE);
    //count_person();
  
    #ifdef DEBUG
    //Print out the interrupt matrix.
    amg.updateThermistorTemperature();                                      //Update AMGs thermistor temperature and print it.
    Serial.print("**** interrupt received! **** \t at Temperature: ");
    Serial.print(amg.thermistorTemperature);
    Serial.println("°C");
    //Serial.println("Interrupt Matrix: ");
    //amg.updateInterruptMatrix();
    for (int x = 1; x <= (NEW_IMAGE_SIZE * NEW_IMAGE_SIZE); x++){
      Serial.print( dst.pixels[x-1]);
      Serial.print(", ");
      if( x%NEW_IMAGE_SIZE == 0 ) Serial.println();
    }
    #endif
    delete dst.pixels;
    intReceived = false;
  }

  //delay(100);
}
