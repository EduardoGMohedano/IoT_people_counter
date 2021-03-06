#include "image_proc.h"
#include <Melopero_AMG8833.h>
Melopero_AMG8833 amg;

/* Hardware connection must be as follows: 
 *  AMG sensor
 *  I2C SDA       GPIO21 
 *  I2C SCL       GPIO22
 *  AMG INT PIN   GPIO3
 * 
 *  Indicator LEDs
 *  LED GREEN     GPIO2   INDICATES A PERSON IS GETTING IN THE ROOM
 *  LED RED       GPIO4   INDICATES A PERSON IS LEAVING THE ROOM
 */


#define   DEBUG   1                         //DEBUG MACRO
//#define   SIMPLE_COUNT    1               //ALGORITHM TO BE EXECUTED MACRO
#define   AVERAGE_TEMP_FROM_MATRIX      1   //ALGORITHM TO BE EXECUTED MACRO, IN THIS CASE WE TAKE THE AVERAGE TEMPERATURE OF SCALED IMAGE
//#define   AVERAGE_TEMP_FROM_THERMISTOR    1

//The high and low temperature thresholds. If a temperature that exceeds these values is detected an interrupt will be triggered. The temperatures
//are expressed in Celsius degrees.
float highThreshold = 25.0;
float lowThreshold = 18;

volatile bool intReceived = false;
const byte interruptPin = 3;
#define LED_PERSON_ENTERED  2           //Macros for LED number pins that will show whether a person is leaving or getting into a room
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
int interrupt_no;


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

//void get_person_in_zones_with_max(float pixels[8][8]){
void get_person_in_zones_with_max(uint32_t* pixels, uint8_t width, uint8_t height){
  //Depending on height param, we will count the 1's in both zones and count people
  int ones = 0;
  static float average = 0.0;
  static float average2 = 0.0;
  
  #ifdef   AVERAGE_TEMP_FROM_MATRIX
      if( interrupt_no == 1 ){
          average = 0.0;
        //Getting average of first zone
        for(int i = 0; i < (width/2); i++){
          for(int j = 0; j < height; j++){
            average += *(pixels+(width*i)+j);
            
          }
        }
        average/=(width*height/2);
        Serial.print("*Average Temperature is: ");
        Serial.println(average);
      }
  #elif   AVERAGE_TEMP_FROM_THERMISTOR
      amg.updateThermistorTemperature();       //Update AMGs thermistor temperature and print it.
      average = amg.thermistorTemperature;
  #endif
  
  //Analyzing zone 1
  for(int i = 0; i < (width/2); i++){
    for(int j = 0; j < height; j++){
      if ( *(pixels+(width*i)+j) > average) ones++;
     }
  }
  zone_1.previous_state = zone_1.current_state;
  zone_1.current_state = ones/2;    //Counting values which are above average temperature 

  ones = 0;
  #ifdef   AVERAGE_TEMP_FROM_MATRIX
      if( interrupt_no == 1 ){
         average2 = 0.0;
        //Getting average of second zone
        for(int i = (width/2); i < width; i++){
          for(int j = 0; j < height; j++){
            average2 += *(pixels+(width*i)+j);
          }
        }
        average2/=(width*height/2);
      }
  #elif   AVERAGE_TEMP_FROM_THERMISTOR
      amg.updateThermistorTemperature();       //Update AMGs thermistor temperature and print it.
      average2 = amg.thermistorTemperature;
  #endif
  
  //Analyzing zone 2
    for(int i = (width/2); i < width; i++){
      for(int j = 0; j < height; j++){
        if ( *(pixels+(width*i)+j) > average2) ones++;
      }
    }
  zone_2.previous_state = zone_2.current_state;
  zone_2.current_state = ones/2;   
}

void count_person(){
  //Moving from zone 1 to zone 2 means leaving the room
  if(  zone_2.current_state >  zone_2.previous_state ){
    if (  zone_1.current_state <  zone_1.previous_state  ){
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

  //Moving from zone 2 to zone 1 means getting into the room
  if(  zone_2.current_state <  zone_2.previous_state ){
    if (  zone_1.current_state >  zone_1.previous_state  ){
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

  interrupt_no = 0;
    
}

void core0(void* args){
  while(1){
  }
}


void loop() {
    //Check if an interrupt occurred.
  //The interrupt occurred flag gets set by the interrupt service routine
  //each time an interrupt is triggered.
  if (intReceived){

    /* Below definitons are executed to get an interpolated temperature image
     * the dimensions are determined by IMAGE_SIZE_UP_FACTOR macro
     */
    interrupt_no < 2 ? interrupt_no++ : interrupt_no = 1;
    amg.updatePixelMatrix(); // This might be executed later
    uint32_t pixelptr[SENSOR_SIZE_PIXELS*SENSOR_SIZE_PIXELS];
    for(int i = 0; i < SENSOR_SIZE_PIXELS; i++){
      for(int j = 0; j < SENSOR_SIZE_PIXELS; j++){
          pixelptr[(8*i)+j] = (uint32_t) amg.pixelMatrix[i][j];
      }
    }

    image_t src, dst; 
    src.pixels = pixelptr;         //Asigning pointer to the begining of the matrix, now because of the typecasting some information from decimals is being lost multiply by 10 all the pixels to avoid it
    src.w = SENSOR_SIZE_PIXELS;
    src.h = SENSOR_SIZE_PIXELS;

    //Destiny image struct init
    //dst.pixels = new uint32_t[NEW_IMAGE_SIZE* NEW_IMAGE_SIZE]{};
    dst.pixels = (uint32_t*) malloc(NEW_IMAGE_SIZE* NEW_IMAGE_SIZE*4);
    //dst.pixels = (uint32_t*) heap_caps_malloc(NEW_IMAGE_SIZE* NEW_IMAGE_SIZE*4 , MALLOC_CAP_32BIT);
    dst.w = NEW_IMAGE_SIZE;
    dst.h = NEW_IMAGE_SIZE;

    //Getting interpolated image in dst 
    scale(&src, &dst, (float) IMAGE_SIZE_UP_FACTOR, (float) IMAGE_SIZE_UP_FACTOR);
    
    //Getting persons count on the zones 1 and 2
    #ifdef SIMPLE_COUNT
        amg.updateInterruptMatrix();
        get_person_in_zones(amg.interruptMatrix);
    //or
    #else
        //amg.updatePixelMatrix();
        get_person_in_zones_with_max(dst.pixels, dst.w, dst.h);
    #endif
    
    count_person();
  
    #ifdef DEBUG
    //Print out the interrupt matrix.
    amg.updateThermistorTemperature();                                      //Update AMGs thermistor temperature and print it.
    Serial.print("**** interrupt received! **** \t at Temperature: ");
    Serial.print(amg.thermistorTemperature);
    Serial.println("??C");
    //Serial.println("Interrupt Matrix: ");
    //amg.updateInterruptMatrix();
    for (int x = 1; x <= (NEW_IMAGE_SIZE * NEW_IMAGE_SIZE); x++){
      Serial.print( dst.pixels[x-1]);
      Serial.print(", ");
      if( x%NEW_IMAGE_SIZE == 0 ) Serial.println();
    }
    #endif

    //free(dst.pixels);
    //delete dst.pixels;
    intReceived = false;
  }
}
