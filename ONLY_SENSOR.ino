#include "image_proc.h"
#include <Melopero_AMG8833.h>
#include <ArduinoQueue.h>
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
//#define   AVERAGE_TEMP_FROM_MATRIX      1   //ALGORITHM TO BE EXECUTED MACRO, IN THIS CASE WE TAKE THE AVERAGE TEMPERATURE OF SCALED IMAGE, to check this algo , i might muust update number of interruptions
#define   AVERAGE_TEMP_FROM_THERMISTOR    1   //Triggering pixels will be compared with temperature taken from thermistor
#define   NUMBER_OF_ZONES  3


//The high and low temperature thresholds. If a temperature that exceeds these values is detected an interrupt will be triggered. The temperatures
//are expressed in Celsius degrees.
float highThreshold = 24.0;
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
  int zone_1; //This states contain the number of persons in each zone
  int zone_2;
}Zones;

ArduinoQueue<Zones> zones_interrupt_Queue(NUMBER_OF_ZONES);
uint16_t person_counter = 0;
int interrupt_no;


/***************************Function definitions*************************/

//The interrupt service routine will be called each time the interrupt is triggered.
void onInterrupt(){
  intReceived = true;
}

//void get_person_in_zones_with_max(float pixels[8][8]){
void get_person_in_zones_with_max(uint32_t* pixels, uint8_t width, uint8_t height){
  //Depending on height param, we will count the 1's in both zones and count people
  int ones = 0;
  static float average = 0.0;
  static float average2 = 0.0;
  Zones zone_temp={0,0};
  
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
      //amg.updateThermistorTemperature();       //Update AMGs thermistor temperature and print it.
      //average = amg.thermistorTemperature - 2.0;
      average = highThreshold - 2.0;
  #endif
  
  //Analyzing zone 1
  for(int i = 0; i < (width/2); i++){
    for(int j = 0; j < height; j++){
      if ( *(pixels+(width*i)+j) > average) ones++;
     }
  }

  zone_temp.zone_1 = ones;    //Getting maximum values

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
      /*amg.updateThermistorTemperature();       //This measurement is done only once at the beginning of the function
      average2 = amg.thermistorTemperature; */   
      average2 = average;
  #endif
  
  //Analyzing zone 2
    for(int i = (width/2); i < width; i++){
      for(int j = 0; j < height; j++){
        if ( *(pixels+(width*i)+j) > average2) ones++;
      }
    }  
    zone_temp.zone_2 = ones;

    if( zones_interrupt_Queue.isFull() ){
        zones_interrupt_Queue.dequeue();    ///If queue is full, dequeue all elements to enqueue the newest one
        zones_interrupt_Queue.dequeue();
        zones_interrupt_Queue.dequeue();
    }
    zones_interrupt_Queue.enqueue(zone_temp);
}

//Count persons using 3 phase algorithm, that means we will need at least three triggering interrupts before this function starts counting persons
void count_person(){
  if( !zones_interrupt_Queue.isFull() ){
    return;   //If we don't have at least the three zones detected then don't count persons
  }
  //Moving from zone 1 to zone 2 means leaving the room
  int temp = zones_interrupt_Queue.getHead().zone_1;
  int temp2 = zones_interrupt_Queue.getHead().zone_2;
  zones_interrupt_Queue.dequeue();    //dequeue to access the 2 most recent zone structs

  if( zones_interrupt_Queue.getHead().zone_1 <= temp  ){
    if(  zones_interrupt_Queue.getHead().zone_2 > temp2 ){
      if ( zones_interrupt_Queue.getTail().zone_2  <= zones_interrupt_Queue.getHead().zone_2   ){
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
  }

  //Moving from zone 2 to zone 1 means getting into the room
  if ( zones_interrupt_Queue.getHead().zone_2 <= temp2 ){
    if(  zones_interrupt_Queue.getHead().zone_1 > temp ){
      if (  zones_interrupt_Queue.getTail().zone_1  <= zones_interrupt_Queue.getHead().zone_1  ){
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


void loop() {
  //Checking whether an interrupt has occurred.
  if (intReceived){

    /* Below definitons are executed to get an interpolated temperature image
     * the dimensions are determined by IMAGE_SIZE_UP_FACTOR macro
     */
    #ifdef   AVERAGE_TEMP_FROM_MATRIX
        interrupt_no < 2 ? interrupt_no++ : interrupt_no = 1; 
    #endif
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
    Serial.println("°C");
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
