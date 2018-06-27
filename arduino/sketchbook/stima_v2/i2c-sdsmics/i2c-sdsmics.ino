/**********************************************************************
Copyright (C) 2017  Paolo Paruno <p.patruno@iperbole.bologna.it>
authors:
Paolo Paruno <p.patruno@iperbole.bologna.it>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of 
the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
**********************************************************************/

/*********************************************************************
 *
 * This program implements elaboration of sds011 inovafitness sensor
 * for pm2.5 and pm 10 exported to i2c interface.
 * 

elaborazioni:

a) eseguo 3 misure di fila (1 al secondo per 3 secondi) e butto via il
minimo e il massimo e tengo il valore "centrale" (internal to Sds011 library)

b) ogni 6 secondi memorizzo questo valore

c) ogni 60 secondi faccio media minimo massimo e deviazione standard

d) ogni 60 secondi si elaborano i valori orari della media, minimo,
massimo e deviazione standard dai valori sul periodo di 60 sec, ma che
equivale a farlo sui dati a) 

il sensore ha due modalità di funzionamento:

1) oneshot; dico fai una miura e mi vengono dati i dati a) (poi farò
conti come e quando voglio) posso fare una misura ogni 3 secondi circa

2)continuo: mi vengono forniti a richiesta i valori b) e d)

**********************************************************************/
/*
buffer scrivibili da i2c
viene scritto buffer1 e buffer2
viene letto buffer2
i puntatori a buffer1 e buffer2 vengono scambiati in una operazione atomica all'inizio del main loop
*/

/*
buffer leggibili da i2c
le elaborazioni scrivono sempre su buffer1
viene sempre letto buffer2
i puntatori a buffer1 e buffer2 vengono scambiati in una operazione atomica al comando stop
*/

#define VERSION 04             //Software version for cross checking

#include <limits.h>
#include <avr/wdt.h>
#include "Wire.h"
#include "registers-sdsmics.h"         //Register definitions
#include "config.h"
#include "IntBuffer.h"
#include "FloatBuffer.h"

#ifdef SDS011PRESENT
#include "Sds011.h"
#endif
#ifdef MICS4514PRESENT
#include "Mics4514.h"
#endif

#include "EEPROMAnything.h"
#include "Calibration.h"
#include <ArduinoLog.h>

// logging level at compile time
// Available levels are:
// LOG_LEVEL_SILENT, LOG_LEVEL_FATAL, LOG_LEVEL_ERROR, LOG_LEVEL_WARNING, LOG_LEVEL_NOTICE, LOG_LEVEL_VERBOSE
#define LOG_LEVEL   LOG_LEVEL_NOTICE

#define REG_MAP_SIZE            sizeof(I2C_REGISTERS)       //size of register map
#define REG_PM_SIZE           sizeof(pm_t)                  //size of register map for pm
#define REG_CONO2_SIZE           sizeof(cono2_t)                  //size of register map for cono2
#define REG_WRITABLE_MAP_SIZE   sizeof(I2C_WRITABLE_REGISTERS)       //size of register map

#define MAX_SENT_BYTES     0x0F                      //maximum amount of data that I could receive from a master device (register, plus 15 byte)

char confver[9] = CONFVER; // version of configuration saved on eeprom

#ifdef SDS011PRESENT
sds011::Sds011 sensor(SERIALSDS011);
#endif
#ifdef MICS4514PRESENT
mics4514::Mics4514 sensormics(COPIN,NO2PIN,HEATERPIN,SCALE1PIN,SCALE2PIN);

// NO2 Sensor calibration
calibration::Calibration NO2Cal;

// CO Sensor calibration
calibration::Calibration COCal;

// PM2 Sensor calibration
calibration::Calibration PM25Cal;

// PM10 Sensor calibration
calibration::Calibration PM10Cal;

#endif

#ifdef SDS011PRESENT
IntBuffer cbpm2560n;
IntBuffer cbpm1060n;
IntBuffer cbpm2560m;
IntBuffer cbpm1060m;
IntBuffer cbpm2560x;
IntBuffer cbpm1060x;
FloatBuffer cbsum2pm25;
FloatBuffer cbsum2pm10;
FloatBuffer cbsumpm25;
FloatBuffer cbsumpm10;
#endif

#ifdef MICS4514PRESENT
IntBuffer cbco60n;
IntBuffer cbno260n;
IntBuffer cbco60m;
IntBuffer cbno260m;
IntBuffer cbco60x;
IntBuffer cbno260x;
FloatBuffer cbsum2co;
FloatBuffer cbsumco;
FloatBuffer cbsum2no2;
FloatBuffer cbsumno2;
#endif

typedef struct {
  uint8_t    sw_version;                          // Version of the I2C_SDS011 sw
} status_t;

typedef struct {
  uint16_t     pm25;
  uint16_t     pm10;
  uint16_t     minpm25;
  uint16_t     minpm10;
  uint16_t     meanpm25;
  uint16_t     meanpm10;
  uint16_t     maxpm25;
  uint16_t     maxpm10;
  uint16_t     sigmapm25;
  uint16_t     sigmapm10;
  uint16_t     pm25sample;
  uint16_t     pm10sample;
} pm_t;

typedef struct {
  uint16_t     co;
  uint16_t     no2;
  uint16_t     minco;
  uint16_t     minno2;
  uint16_t     meanco;
  uint16_t     meanno2;
  uint16_t     maxco;
  uint16_t     maxno2;
  uint16_t     sigmaco;
  uint16_t     sigmano2;
  uint16_t     coresistance;
  uint16_t     no2resistance;
} cono2_t;

typedef struct {

//Status registers
  status_t     status;                   // 0x00  status register

//data
  pm_t                pm;                     // 0x01 pm
  cono2_t             cono2;                  // 0x15 pm
} I2C_REGISTERS;


typedef struct {

  //sample mode
  bool                  oneshot;                  // one shot active
  uint8_t               i2c_address;              // i2c bus address (short unsigned int)

#ifdef MICS4514PRESENT
  // calibration data
  // Define the number of calibration points
  uint8_t no2numPoints;
  // Concentratios used in calibration process
  float no2concentrations[MAX_POINTS];
  // Calibration resistances obtained during calibration process
  float no2resistences[MAX_POINTS];
  
  // Define the number of calibration points
  uint8_t conumPoints;
  // Concentrations used in calibration process
  float coconcentrations[MAX_POINTS];
  // Calibration resistences obtained during calibration process (in KOHMs)
  float coresistences[MAX_POINTS];
#endif

#ifdef SDS011PRESENT
  // calibration data
  // Define the number of calibration points
  uint8_t pm25numPoints;
  // Concentratios used in calibration process
  float pm25concentrations[MAX_POINTS];
  // Calibration samples obtained during calibration process
  float pm25samples[MAX_POINTS];
  
  // Define the number of calibration points
  uint8_t pm10numPoints;
  // Concentrations used in calibration process
  float pm10concentrations[MAX_POINTS];
  // Calibration samples obtained during calibration process
  float pm10samples[MAX_POINTS];
#endif



  
  void save (int* p) volatile {                            // save to eeprom

    IF_SDEBUG(Serial.print(F("oneshot: "))); IF_SDEBUG(Serial.println(oneshot));
    IF_SDEBUG(Serial.print(F("i2c address: "))); IF_SDEBUG(Serial.println(i2c_address));

    *p+=EEPROM_writeAnything(*p, oneshot);
    *p+=EEPROM_writeAnything(*p, i2c_address);
#ifdef MICS4514PRESENT
    *p+=EEPROM_writeAnything(*p, no2numPoints);    
    *p+=EEPROM_writeAnything(*p, no2concentrations);
    *p+=EEPROM_writeAnything(*p, no2resistences);
    *p+=EEPROM_writeAnything(*p, conumPoints);
    *p+=EEPROM_writeAnything(*p, coconcentrations);
    *p+=EEPROM_writeAnything(*p, coresistences);
#endif
#ifdef SDS011PRESENT
    *p+=EEPROM_writeAnything(*p, pm25numPoints);    
    *p+=EEPROM_writeAnything(*p, pm25concentrations);
    *p+=EEPROM_writeAnything(*p, pm25samples);
    *p+=EEPROM_writeAnything(*p, pm10numPoints);
    *p+=EEPROM_writeAnything(*p, pm10concentrations);
    *p+=EEPROM_writeAnything(*p, pm10samples);
#endif

  }
  
  void load (int* p) volatile {                            // load from eeprom
    *p+=EEPROM_readAnything(*p, oneshot);
    *p+=EEPROM_readAnything(*p, i2c_address);
#ifdef MICS4514PRESENT
    *p+=EEPROM_readAnything(*p, no2numPoints);
    *p+=EEPROM_readAnything(*p, no2concentrations);
    *p+=EEPROM_readAnything(*p, no2resistences);
    *p+=EEPROM_readAnything(*p, conumPoints);
    *p+=EEPROM_readAnything(*p, coconcentrations);
    *p+=EEPROM_readAnything(*p, coresistences);
#endif
#ifdef SDS011PRESENT
    *p+=EEPROM_readAnything(*p, pm25numPoints);    
    *p+=EEPROM_readAnything(*p, pm25concentrations);
    *p+=EEPROM_readAnything(*p, pm25samples);
    *p+=EEPROM_readAnything(*p, pm10numPoints);
    *p+=EEPROM_readAnything(*p, pm10concentrations);
    *p+=EEPROM_readAnything(*p, pm10samples);
#endif
  }
} I2C_WRITABLE_REGISTERS;


volatile static I2C_REGISTERS    i2c_buffer1;
volatile static I2C_REGISTERS    i2c_buffer2;

volatile static I2C_REGISTERS* i2c_dataset1;
volatile static I2C_REGISTERS* i2c_dataset2;
volatile static I2C_REGISTERS* i2c_datasettmp;

volatile static I2C_WRITABLE_REGISTERS  i2c_writablebuffer1;
volatile static I2C_WRITABLE_REGISTERS  i2c_writablebuffer2;

volatile static I2C_WRITABLE_REGISTERS* i2c_writabledataset1;
volatile static I2C_WRITABLE_REGISTERS* i2c_writabledataset2;
volatile static I2C_WRITABLE_REGISTERS* i2c_writabledatasettmp;

volatile static uint8_t         receivedCommands[MAX_SENT_BYTES];
volatile static uint8_t         new_command;                        //new command received (!=0)

#ifdef SDS011PRESENT
float meanpm25;
float meanpm10;
long int minpm25;
long int minpm10;
long int maxpm25;
long int maxpm10;
float sum2pm25;
float sumpm25;
float sum2pm10;
float sumpm10;
#endif

#ifdef MICS4514PRESENT
float meanco;
float meanno2;
long int minco;
long int minno2;
long int maxco;
long int maxno2;
float sum2co;
float sumco;
float sum2no2;
float sumno2;
#endif

uint8_t nsample1;

// one shot management
static bool oneshot;
static bool start=false;
static bool stop=false;

volatile unsigned int count;

unsigned long starttime;
boolean forcedefault=false;

//////////////////////////////////////////////////////////////////////////////////////
// I2C handlers
// Handler for requesting data
//
void requestEvent()
{
  Wire.write(((uint8_t *)i2c_dataset2)+receivedCommands[0],32);
  //Write up to 32 byte, since master is responsible for reading and sending NACK
  //32 byte limit is in the Wire library, we have to live with it unless writing our own wire library

  //Serial.print("receivedCommands: ");
  //Serial.println(receivedCommands[0]);
  //Serial.println(*((uint8_t *)(i2c_dataset2)+receivedCommands[0]));
  //Serial.println(*((uint8_t *)(i2c_dataset2)+receivedCommands[0]+1));
  //Serial.println(*((uint8_t *)(i2c_dataset2)+receivedCommands[0]+2));
  //Serial.println(*((uint8_t *)(i2c_dataset2)+receivedCommands[0]+3));
}

//Handler for receiving data
void receiveEvent( int bytesReceived)
{
  uint8_t  *ptr1, *ptr2;
     //Serial.print("received:");
     for (int a = 0; a < bytesReceived; a++) {
          if (a < MAX_SENT_BYTES) {
               receivedCommands[a] = Wire.read();
	       //Serial.println(receivedCommands[a]);
          } else {
               Wire.read();  // if we receive more data then allowed just throw it away
          }
     }

 
     if (bytesReceived == 1){
       //read address for a given register
       //Addressing over the reg_map fallback to first byte
       if(bytesReceived == 1 && ( (receivedCommands[0] < 0) || (receivedCommands[0] >= REG_MAP_SIZE))) {
	 receivedCommands[0]=0;
       }
       //IF_SDEBUG(Serial.print("set register:"));IF_SDEBUG(Serial.println(receivedCommands[0]));
       return;
     }


    if (bytesReceived == 2){
       // check for a command
       if (receivedCommands[0] == I2C_SDSMICS_COMMAND) {
	 //IF_SDEBUG(Serial.print("received command:"));IF_SDEBUG(Serial.println(receivedCommands[1]));
	 //if (new_command != 0) IF_SDEBUG(Serial.print("command overflow !"));
	 new_command = receivedCommands[1]; return; }
     }


     //More than 1 byte was received, so there is definitely some data to write into a register
     //Check for writeable registers and discard data is it's not writeable

     //IF_SDEBUG(Serial.println("data for write: "));
     //IF_SDEBUG(Serial.println(receivedCommands[0]));
     //IF_SDEBUG(Serial.println(receivedCommands[1]));
     
     if ((receivedCommands[0]>=I2C_SDSMICS_MAP_WRITABLE) && (receivedCommands[0] < (I2C_SDSMICS_MAP_WRITABLE+REG_WRITABLE_MAP_SIZE))) {    
       if ((receivedCommands[0]+(unsigned int)(bytesReceived-1)) <= (I2C_SDSMICS_MAP_WRITABLE+REG_WRITABLE_MAP_SIZE)) {
	 //Writeable registers
	 // the two buffer should be in sync
	 //ptr1 = (uint8_t *)i2c_writabledataset1+receivedCommands[0]-I2C_SDSMICS_MAP_WRITABLE;
	 ptr2 = (uint8_t *)i2c_writabledataset2+receivedCommands[0]-I2C_SDSMICS_MAP_WRITABLE;
	 for (int a = 1; a < bytesReceived; a++) { 
	   //IF_SDEBUG(Serial.print("write in writable buffer:"));IF_SDEBUG(Serial.println(a));IF_SDEBUG(Serial.println(receivedCommands[a]));
	   //*ptr1++ = receivedCommands[a];
	   *ptr2++ = receivedCommands[a];
	 }
	 // new data written
       }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {

  uint8_t i;

  /*
  Nel caso di un chip in standalone senza bootloader, la prima
  istruzione che è bene mettere nel setup() è sempre la disattivazione
  del Watchdog stesso: il Watchdog, infatti, resta attivo dopo il
  reset e, se non disabilitato, esso può provare il reset perpetuo del
  microcontrollore
  */
  wdt_disable();

  // enable watchdog with timeout to 8s
  wdt_enable(WDTO_8S);

  IF_SDEBUG(Serial.begin(115200));        // connect to the serial port
  // Pass log level, whether to show log level, and print interface.
  // Available levels are:
  // LOG_LEVEL_SILENT, LOG_LEVEL_FATAL, LOG_LEVEL_ERROR, LOG_LEVEL_WARNING, LOG_LEVEL_NOTICE, LOG_LEVEL_VERBOSE
  // Note: if you want to fully remove all logging code, change #define LOG_LEVEL ....
  //       this will significantly reduce your project size

  // set runtime log level to the same of compile time
  Log.begin(LOG_LEVEL, &Serial);

  LOGN(F("Start firmware version: %d" CR),VERSION);

  // inizialize double buffer
  i2c_dataset1=&i2c_buffer1;
  i2c_dataset2=&i2c_buffer2;

  // inizialize writable double buffer
  i2c_writabledataset1=&i2c_writablebuffer1;
  i2c_writabledataset2=&i2c_writablebuffer2;

#define SAMPLE1 60000/SAMPLERATE
#define SAMPLE2 60

#ifdef SDS011PRESENT
  meanpm25=0.;
  meanpm10=0.;

  maxpm25=-1;
  maxpm10=-1;

  minpm25=LONG_MAX;
  minpm10=LONG_MAX;

  sum2pm25=0;
  sumpm25=0;

  sum2pm10=0;
  sumpm10=0;
#endif
  
#ifdef MICS4514PRESENT
  meanco=0.;
  meanno2=0.;

  maxco=-1;
  maxno2=-1;

  sum2co=0;
  sumco=0;

  sum2no2=0;
  sumno2=0;

  minco=LONG_MAX;
  minno2=LONG_MAX;
#endif

  nsample1=1;

#ifdef SDS011PRESENT
  cbpm2560n.init(SAMPLE2);
  cbpm1060n.init(SAMPLE2);

  cbpm2560m.init(SAMPLE2);
  cbpm1060m.init(SAMPLE2);

  cbpm2560x.init(SAMPLE2);
  cbpm1060x.init(SAMPLE2);

  cbsum2pm25.init(SAMPLE2);
  cbsum2pm10.init(SAMPLE2);

  cbsumpm25.init(SAMPLE2);
  cbsumpm10.init(SAMPLE2);
#endif

#ifdef MICS4514PRESENT
  cbco60n.init(SAMPLE2);
  cbno260n.init(SAMPLE2);

  cbco60m.init(SAMPLE2);
  cbno260m.init(SAMPLE2);

  cbco60x.init(SAMPLE2);
  cbno260x.init(SAMPLE2);

  cbsum2co.init(SAMPLE2);
  cbsumco.init(SAMPLE2);

  cbsum2no2.init(SAMPLE2);
  cbsumno2.init(SAMPLE2);
#endif

  IF_SDEBUG(Serial.println(F("i2c_dataset 1&2 set to 1")));

  uint8_t *ptr;
  //Init to FF i2c_dataset1;
  ptr = (uint8_t *)i2c_dataset1;
  for (i=0;i<REG_MAP_SIZE;i++) { *ptr |= 0xFF; ptr++;}

  //Init to FF i2c_dataset1;
  ptr = (uint8_t *)i2c_dataset2;
  for (i=0;i<REG_MAP_SIZE;i++) { *ptr |= 0xFF; ptr++;}



  IF_SDEBUG(Serial.println(F("i2c_writabledataset 1&2 set to 1")));
  //Init to FF i2c_writabledataset1;
  ptr = (uint8_t *)i2c_writabledataset1;
  for (i=0;i<REG_WRITABLE_MAP_SIZE;i++) { *ptr |= 0xFF; ptr++;}

  //Init to FF i2c_writabledataset2;
  ptr = (uint8_t *)i2c_writabledataset2;
  for (i=0;i<REG_WRITABLE_MAP_SIZE;i++) { *ptr |= 0xFF; ptr++;}


  //Set up default parameters
  i2c_dataset1->status.sw_version          = VERSION;
  i2c_dataset2->status.sw_version          = VERSION;


  pinMode(FORCEDEFAULTPIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT); 

  if (digitalRead(FORCEDEFAULTPIN) == LOW) {
    digitalWrite(LEDPIN, HIGH);
    forcedefault=true;
  }


  // load configuration saved on eeprom
  IF_SDEBUG(Serial.println(F("try to load configuration from eeprom")));
  int p=0;
  // check for configuration version on eeprom
  char EE_confver[9];
  p+=EEPROM_readAnything(p, EE_confver);

  if((strcmp(EE_confver,confver ) == 0) && !forcedefault)
    {
      //load writable registers
      IF_SDEBUG(Serial.println(F("load writable registers from eeprom")));
      i2c_writabledataset1->load(&p);
      i2c_writabledataset2->oneshot=i2c_writabledataset1->oneshot;
      i2c_writabledataset2->i2c_address=i2c_writabledataset1->i2c_address;
#ifdef MICS4514PRESENT
      //calibration
      i2c_writabledataset2->no2numPoints=i2c_writabledataset1->no2numPoints;
      memcpy(const_cast<float*>(i2c_writabledataset2->no2concentrations), const_cast<float*>(i2c_writabledataset1->no2concentrations), sizeof i2c_writabledataset1->no2concentrations);
      memcpy(const_cast<float*>(i2c_writabledataset2->no2resistences), const_cast<float*>(i2c_writabledataset1->no2resistences), sizeof i2c_writabledataset1->no2resistences); 
      i2c_writabledataset2->conumPoints=i2c_writabledataset1->conumPoints;
      memcpy(const_cast<float*>(i2c_writabledataset2->coconcentrations), const_cast<float*>(i2c_writabledataset1->coconcentrations), sizeof i2c_writabledataset1->coconcentrations);
      memcpy(const_cast<float*>(i2c_writabledataset2->coresistences), const_cast<float*>(i2c_writabledataset1->coresistences), sizeof i2c_writabledataset1->coresistences);
#endif
#ifdef SDS011PRESENT
      //calibration
      i2c_writabledataset2->pm25numPoints=i2c_writabledataset1->pm25numPoints;
      memcpy(const_cast<float*>(i2c_writabledataset2->pm25concentrations), const_cast<float*>(i2c_writabledataset1->pm25concentrations), sizeof i2c_writabledataset1->pm25concentrations);
      memcpy(const_cast<float*>(i2c_writabledataset2->pm25samples), const_cast<float*>(i2c_writabledataset1->pm25samples), sizeof i2c_writabledataset1->pm25samples); 
      i2c_writabledataset2->pm10numPoints=i2c_writabledataset1->pm10numPoints;
      memcpy(const_cast<float*>(i2c_writabledataset2->pm10concentrations), const_cast<float*>(i2c_writabledataset1->pm10concentrations), sizeof i2c_writabledataset1->pm10concentrations);
      memcpy(const_cast<float*>(i2c_writabledataset2->pm10samples), const_cast<float*>(i2c_writabledataset1->pm10samples), sizeof i2c_writabledataset1->pm10samples); 
#endif

    }
  else
    {
      IF_SDEBUG(Serial.println(F("EEPROM data not useful or set pin activated")));
      IF_SDEBUG(Serial.println(F("set default values for writable registers")));
      // set default to oneshot
      i2c_writabledataset1->oneshot=true;
      i2c_writabledataset2->oneshot=true;
      i2c_writabledataset1->i2c_address = I2C_SDSMICS_DEFAULTADDRESS;
      i2c_writabledataset2->i2c_address = I2C_SDSMICS_DEFAULTADDRESS;

#ifdef MICS4514PRESENT
      //calibration
      i2c_writabledataset1->no2numPoints=NO2NUMPOINTS;
      i2c_writabledataset2->no2numPoints=NO2NUMPOINTS;
      
      i2c_writabledataset1->no2concentrations[0]= POINT1_PPM_NO2;
      i2c_writabledataset1->no2concentrations[1]= POINT2_PPM_NO2;
      i2c_writabledataset1->no2concentrations[2]= POINT3_PPM_NO2;
      i2c_writabledataset2->no2concentrations[0]= POINT1_PPM_NO2;
      i2c_writabledataset2->no2concentrations[1]= POINT2_PPM_NO2;
      i2c_writabledataset2->no2concentrations[2]= POINT3_PPM_NO2;
      
      i2c_writabledataset1->no2resistences[0]= POINT1_RES_NO2;
      i2c_writabledataset1->no2resistences[1]= POINT2_RES_NO2;
      i2c_writabledataset1->no2resistences[2]= POINT3_RES_NO2;
      i2c_writabledataset2->no2resistences[0]= POINT1_RES_NO2;
      i2c_writabledataset2->no2resistences[1]= POINT2_RES_NO2;
      i2c_writabledataset2->no2resistences[2]= POINT3_RES_NO2;
      
      i2c_writabledataset1->conumPoints=CONUMPOINTS;
      i2c_writabledataset2->conumPoints=CONUMPOINTS;
      
      i2c_writabledataset1->coconcentrations[0]= POINT1_PPM_CO;
      i2c_writabledataset1->coconcentrations[1]= POINT2_PPM_CO;
      i2c_writabledataset1->coconcentrations[2]= POINT3_PPM_CO;
      i2c_writabledataset2->coconcentrations[0]= POINT1_PPM_CO;
      i2c_writabledataset2->coconcentrations[1]= POINT2_PPM_CO;
      i2c_writabledataset2->coconcentrations[2]= POINT3_PPM_CO;
      
      i2c_writabledataset1->coresistences[0]= POINT1_RES_CO;
      i2c_writabledataset1->coresistences[1]= POINT2_RES_CO;
      i2c_writabledataset1->coresistences[2]= POINT3_RES_CO;
	
      i2c_writabledataset2->coresistences[0]= POINT1_RES_CO;
      i2c_writabledataset2->coresistences[1]= POINT2_RES_CO;
      i2c_writabledataset2->coresistences[2]= POINT3_RES_CO;
#endif

#ifdef SDS011PRESENT
      //calibration
      i2c_writabledataset1->pm25numPoints=0;
      i2c_writabledataset2->pm25numPoints=0;
      
      i2c_writabledataset1->pm10numPoints=0;
      i2c_writabledataset2->pm10numPoints=0;
      
#endif      
    }

  oneshot=i2c_writabledataset2->oneshot;

  IF_SDEBUG(Serial.print(F("i2c_address: ")));
  IF_SDEBUG(Serial.println(i2c_writabledataset1->i2c_address));
  IF_SDEBUG(Serial.print(F("oneshot: ")));
  IF_SDEBUG(Serial.println(i2c_writabledataset1->oneshot));


#ifdef MICS4514PRESENT
  NO2Cal.setCalibrationPoints(const_cast<float*>(i2c_writabledataset1->no2resistences), const_cast<float*>(i2c_writabledataset1->no2concentrations), i2c_writabledataset1->no2numPoints);
  COCal.setCalibrationPoints(const_cast<float*>(i2c_writabledataset1->coresistences), const_cast<float*>(i2c_writabledataset1->coconcentrations), i2c_writabledataset1->conumPoints);
#endif

#ifdef SDS011PRESENT
  PM25Cal.setCalibrationPoints(const_cast<float*>(i2c_writabledataset1->pm25samples), const_cast<float*>(i2c_writabledataset1->pm25concentrations), i2c_writabledataset1->pm25numPoints);
  PM10Cal.setCalibrationPoints(const_cast<float*>(i2c_writabledataset1->pm10samples), const_cast<float*>(i2c_writabledataset1->pm10concentrations), i2c_writabledataset1->pm10numPoints);
#endif
  
  //Start I2C communication routines
  Wire.begin(i2c_writabledataset1->i2c_address);

  //The Wire library enables the internal pullup resistors for SDA and SCL.
  //You can turn them off after Wire.begin()
  // do not need this with patched Wire library
  //digitalWrite( SDA, LOW);
  //digitalWrite( SCL, LOW);
  //digitalWrite( SDA, HIGH);
  //digitalWrite( SCL, HIGH);

  Wire.onRequest(requestEvent);          // Set up event handlers
  Wire.onReceive(receiveEvent);

#ifdef SDS011PRESENT
  SERIALSDS011.begin(9600);
  sensor.set_mode(sds011::QUERY);
#endif
  
  if (oneshot){
#ifdef SDS011PRESENT
    sensor.set_sleep(true);
#endif
#ifdef MICS4514PRESENT
    sensormics.sleep();
#endif    
  }else{
#ifdef MICS4514PRESENT
    sensormics.fast_heat();
#endif    
  }
  starttime = millis()+SAMPLERATE;

  IF_SDEBUG(Serial.println(F("end setup")));

}


void mgr_command(){

  static uint8_t _command;
  
  //Check for new incoming command on I2C
  if (new_command != 0) {
    _command = new_command;                                                   //save command byte for processing
    new_command = 0;                                                          //clear it
    //_command = _command & 0x0F;                                               //empty 4MSB bits   
    switch (_command) {

      /*
    case I2C_PWM_COMMAND_TAKE:
      {
	LOGN(F("COMMAND: take" CR));
      
	//LOGN("writable buffer exchange"));
	// disable interrupts for atomic operation
	noInterrupts();

	// copy writable registers
	memcpy ( (void *)i2c_writabledataset1, (void *)i2c_writabledataset2, REG_WRITABLE_MAP_SIZE );
      */
	/*
	//exchange double buffer
	i2c_writabledatasettmp=i2c_writabledataset1;
	i2c_writabledataset1=i2c_writabledataset2;
	i2c_writabledataset2=i2c_writabledatasettmp;
	*/
      /*
	interrupts();
	
	take=true;
	
	break;
      }
    */
    case I2C_SDSMICS_COMMAND_ONESHOT_START:
      {
	LOGN(F("COMMAND: oneshot start" CR));
      
	if (!i2c_writabledataset1->oneshot) break;
	
	start=true;
	
	//LOGN(F("reset registers to missing" CR));
	uint8_t *ptr;
	//Init to FF i2c_dataset1;
  	ptr = (uint8_t *)&i2c_dataset1->pm;
	for (int i=0;i<REG_PM_SIZE+REG_CONO2_SIZE;i++) { *ptr |= 0xFF; ptr++;}

	// disable interrupts for atomic operation
	noInterrupts();
	//exchange double buffer
	LOGN(F("exchange double buffer" CR));
	i2c_datasettmp=i2c_dataset1;
	i2c_dataset1=i2c_dataset2;
	i2c_dataset2=i2c_datasettmp;
	interrupts();
	// new data published
	//Init to FF i2c_dataset1;
  	ptr = (uint8_t *)&i2c_dataset1->pm;
	for (int i=0;i<REG_PM_SIZE+REG_CONO2_SIZE;i++) { *ptr |= 0xFF; ptr++;}
	break;
      }
    case I2C_SDSMICS_COMMAND_STOP:
    case I2C_SDSMICS_COMMAND_ONESHOT_STOP:
      {
	LOGN(F("COMMAND: oneshot stop" CR));
	
	if (!i2c_writabledataset1->oneshot) break;
	
	// disable interrupts for atomic operation
	noInterrupts();
	//exchange double buffer
	LOGN(F("exchange double buffer" CR));
	i2c_datasettmp=i2c_dataset1;
	i2c_dataset1=i2c_dataset2;
	i2c_dataset2=i2c_datasettmp;
	interrupts();
	// new data published
	
	LOGN(F("clean buffer" CR));
	uint8_t *ptr;
	//Init to FF i2c_dataset1;
  	ptr = (uint8_t *)&i2c_dataset1->pm;
	for (int i=0;i<REG_PM_SIZE+REG_CONO2_SIZE;i++) { *ptr |= 0xFF; ptr++;}
	
	stop=true;
	break;
      }
    case I2C_SDSMICS_COMMAND_SAVE:
      {
	LOGN(F("COMMAND: save" CR));
      
	// save configuration to eeprom
	LOGN(F("save configuration to eeprom" CR));

	int p=0;

	// save configuration version on eeprom
	p+=EEPROM_writeAnything(p, confver);
	//save writable registers
	i2c_writabledataset2->save(&p);

	break;
      }
    default:
      {
	LOGN(F("WRONG command" CR));
	break;
      }	
    } //switch  
  }

  if (!i2c_writabledataset1->oneshot){
    // continuos mode
    LOGN(F("expose new measure in continuos mode" CR));

      // disable interrupts for atomic operation
      noInterrupts();
      //exchange double buffer
      LOGN(F("exchange double buffer" CR));
      i2c_datasettmp=i2c_dataset1;
      i2c_dataset1=i2c_dataset2;
      i2c_dataset2=i2c_datasettmp;
      interrupts();
      // new data published
      
      LOGN(F("clean buffer" CR));
      uint8_t *ptr;
      //Init to FF i2c_dataset1;
      ptr = (uint8_t *)&i2c_dataset1->pm;
      for (int i=0;i<REG_PM_SIZE+REG_CONO2_SIZE;i++) { *ptr |= 0xFF; ptr++;}      
  }
  
  //LOGN(F("oneshot : %T" CR),i2c_writabledataset2->oneshot);
  //LOGN(F("oneshot start : %T" CR), start);
  //LOGN(F("oneshot stop  : %T" CR)), stop);

}

void loop() {

  static uint8_t _command;
  //unsigned int pm25;
  //unsigned int pm10;
  int pm25;
  int pm10;
  unsigned int co;
  unsigned int no2;
  
  float mean;
  
  uint8_t i;
  bool ok;

  wdt_reset();

  mgr_command();
  
  if (oneshot) {
    if (!start) return;
    IF_SDEBUG(Serial.println(F("reset everythink to missing")));
    uint8_t *ptr;
    //Init to FF i2c_dataset2;
    ptr = (uint8_t *)&i2c_dataset2->pm;
    for (i=0;i<REG_PM_SIZE+REG_CONO2_SIZE;i++) { *ptr |= 0xFF; ptr++;}
  } else  {
    // comment this if you manage continous mode
    // in this case timing is getted from sensor that send valuer every SAMPLERATE us
    long int timetowait= SAMPLERATE - (millis() - starttime) ;
    //IF_SDEBUG(Serial.print("elapsed time: "));
    //IF_SDEBUG(Serial.println(millis() - starttime));
    if (timetowait > 0) {
      return;
    }

    if (timetowait < -10) IF_SDEBUG(Serial.println("WARNIG: timing error , I am late"));    
    starttime = millis()+timetowait;

  }


  if (oneshot){
#ifdef SDS011PRESENT
    sensor.set_sleep(false);
#endif
#ifdef MICS4514PRESENT
    IF_SDEBUG(Serial.print("start fast heat: "));
    IF_SDEBUG(Serial.println(millis() - starttime));
    sensormics.blocking_fast_heat();
    IF_SDEBUG(Serial.print("end fast heat: "));
    IF_SDEBUG(Serial.println(millis() - starttime));
#endif
  }
  delay(10);

#ifdef SDS011PRESENT
  IF_SDEBUG(Serial.print("start query sds: "));
  IF_SDEBUG(Serial.println(millis() - starttime));

  ok = sensor.query_data_auto(&pm25, &pm10, 3);
  if (oneshot) sensor.set_sleep(true);

  IF_SDEBUG(Serial.print("end query sds: "));
  IF_SDEBUG(Serial.println(millis() - starttime));
  
  wdt_reset();

  if (ok){

    float ppm;

    IF_SDEBUG(Serial.print(F("pm25 uncalibrated: ")));
    IF_SDEBUG(Serial.println(pm25));

    i2c_dataset1->pm.pm25sample=pm25;
  
    if (PM25Cal.getConcentration(float(pm25),&ppm)){
      IF_SDEBUG(Serial.print("pm25 calibrated"));
    } else {
      IF_SDEBUG(Serial.print("pm25 default calibrated: "));
    }
    IF_SDEBUG(Serial.println(ppm));
    i2c_dataset1->pm.pm25=round(ppm);

    
    IF_SDEBUG(Serial.print(F("pm10 uncalibrated: ")));
    IF_SDEBUG(Serial.println(pm10));

    i2c_dataset1->pm.pm10sample=pm10;
  
    if (PM10Cal.getConcentration(float(pm10),&ppm))  {
      IF_SDEBUG(Serial.print("pm10 calibrated"));
    }else{
      IF_SDEBUG(Serial.print("pm10 default calibrated: "));
    }
    IF_SDEBUG(Serial.println(ppm));
    i2c_dataset1->pm.pm10=round(ppm);

    
  }  else {
    IF_SDEBUG(Serial.println(F("ERROR getting sds011 values !")));
  }
#endif

#ifdef MICS4514PRESENT
  IF_SDEBUG(Serial.print(F("start query mics: ")));
  IF_SDEBUG(Serial.println(millis() - starttime));

  ok = sensormics.query_data_auto(&co, &no2, 3);
  if (oneshot) sensormics.sleep();

  IF_SDEBUG(Serial.print(F("end query mics: ")));
  IF_SDEBUG(Serial.println(millis() - starttime));

  wdt_reset();

  if (ok){
    
    float ppm;

    IF_SDEBUG(Serial.print(F("co uncalibrated: ")));
    IF_SDEBUG(Serial.println(co));

    i2c_dataset1->cono2.coresistance=co;
    
    if (COCal.getConcentration(float(co)/1000.,&ppm))
      {
	IF_SDEBUG(Serial.print("co ppm: "));
	IF_SDEBUG(Serial.println(ppm));
	i2c_dataset1->cono2.co=round(ppm*COPPM2UGM3);
      }

    IF_SDEBUG(Serial.print(F("NO2 uncalibrated: ")));
    IF_SDEBUG(Serial.println(no2));

    i2c_dataset1->cono2.no2resistance=no2;
    
    if (NO2Cal.getConcentration(float(no2)/1000.,&ppm))
      {
	IF_SDEBUG(Serial.print("no2 ppm: "));
	IF_SDEBUG(Serial.println(ppm));
	i2c_dataset1->cono2.no2=round(ppm*NO2PPM2UGM3);
      }
    
    IF_SDEBUG(Serial.print("co: "));
    IF_SDEBUG(Serial.println(i2c_dataset1->cono2.co));
    IF_SDEBUG(Serial.print("no2: "));
    IF_SDEBUG(Serial.println(i2c_dataset1->cono2.no2));
  }  else {
    IF_SDEBUG(Serial.println(F("ERROR getting mics4514 values !")));
  }
#endif

  if (oneshot) {
    //if one shot we have finish
    IF_SDEBUG(Serial.println(F("oneshot end")));
    start=false;    
    return;
  }

  // statistical processing

  float sum260, sum60;

  // first level mean

  IF_SDEBUG(Serial.print("data in store first: "));
  IF_SDEBUG(Serial.println(nsample1));


#ifdef SDS011PRESENT
  
  // first level min
  if (minpm25 > pm25) minpm25=pm25;
  if (minpm10 > pm10) minpm10=pm10;

  // first level mean
  meanpm25 += (float(pm25) - meanpm25) / nsample1;
  meanpm10 += (float(pm10) - meanpm10) / nsample1;

  // first level max
  if (maxpm25 < pm25) maxpm25=pm25;
  if (maxpm10 < pm10) maxpm10=pm10;

  // sigma
  sum2pm25+=pm25*pm25;
  sumpm25+=pm25;

  sum2pm10+=pm10*pm10;
  sumpm10+=pm10;

  if (nsample1 == SAMPLE1) {
    IF_SDEBUG(Serial.print("minpm25: "));
    IF_SDEBUG(Serial.println(minpm25));
    IF_SDEBUG(Serial.print("minpm10: "));
    IF_SDEBUG(Serial.println(minpm10));

    IF_SDEBUG(Serial.print("meanpm25: "));
    IF_SDEBUG(Serial.println(meanpm25));
    IF_SDEBUG(Serial.print("meanpm10: "));
    IF_SDEBUG(Serial.println(meanpm10));

    IF_SDEBUG(Serial.print("maxpm25: "));
    IF_SDEBUG(Serial.println(maxpm25));
    IF_SDEBUG(Serial.print("maxpm10: "));
    IF_SDEBUG(Serial.println(maxpm10));

    cbpm2560n.autoput(minpm25);
    cbpm1060n.autoput(minpm10);

    cbpm2560m.autoput(meanpm25);
    cbpm1060m.autoput(meanpm10);

    cbpm2560x.autoput(maxpm25);
    cbpm1060x.autoput(maxpm10);

    cbsum2pm25.autoput(sum2pm25);
    cbsumpm25.autoput(sumpm25);

    cbsum2pm10.autoput(sum2pm10);
    cbsumpm10.autoput(sumpm10);


    minpm25=LONG_MAX;
    meanpm25=0.;
    maxpm25=0.;

    minpm10=LONG_MAX;
    meanpm10=0.;
    maxpm10=0.;

    sum2pm25=0;
    sumpm25=0;

    sum2pm10=0;
    sumpm10=0;

  }

  // sigma

  if (cbsum2pm25.getSize() == cbsum2pm25.getCapacity() && cbsumpm25.getSize() == cbsumpm25.getCapacity()){
    sum260=0;
    for (i=0 ; i < cbsum2pm25.getCapacity() ; i++){
      sum260 += cbsum2pm25.peek(i);
    }

    sum60=0;
    for (i=0 ; i < cbsumpm25.getCapacity() ; i++){
      sum60 += cbsumpm25.peek(i);
    }
	
    i2c_dataset1->pm.sigmapm25=round(sqrt((sum260-(sum60*sum60)/(SAMPLE1*SAMPLE2))/(SAMPLE1*SAMPLE2)));
      
  }else{
    i2c_dataset1->pm.sigmapm25=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("sigma pm25: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.sigmapm25));


  if (cbsum2pm10.getSize() == cbsum2pm10.getCapacity() && cbsumpm10.getSize() == cbsumpm10.getCapacity()){
    sum260=0;
    for (i=0 ; i < cbsum2pm10.getCapacity() ; i++){
      sum260 += cbsum2pm10.peek(i);
    }

    sum60=0;
    for (i=0 ; i < cbsumpm10.getCapacity() ; i++){
      sum60 += cbsumpm10.peek(i);
    }
	
    i2c_dataset1->pm.sigmapm10=round(sqrt((sum260-(sum60*sum60)/(SAMPLE1*SAMPLE2))/(SAMPLE1*SAMPLE2)));
      
  }else{
    i2c_dataset1->pm.sigmapm10=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("sigma pm10: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.sigmapm10));

#endif


#ifdef MICS4514PRESENT
  
  // first level min
  if (minco > co) minco=co;
  if (minno2 > no2) minno2=no2;

  // first level mean
  meanco += (float(co) - meanco) / nsample1;
  meanno2 += (float(no2) - meanno2) / nsample1;

  // first level max
  if (maxco < co) maxco=co;
  if (maxno2 < no2) maxno2=no2;

  // sigma
  sum2co+=co*co;
  sumco+=co;

  sum2no2+=no2*no2;
  sumno2+=no2;

  if (nsample1 == SAMPLE1) {
    IF_SDEBUG(Serial.print("minco: "));
    IF_SDEBUG(Serial.println(minco));
    IF_SDEBUG(Serial.print("minno2: "));
    IF_SDEBUG(Serial.println(minno2));

    IF_SDEBUG(Serial.print("meanco: "));
    IF_SDEBUG(Serial.println(meanco));
    IF_SDEBUG(Serial.print("meanno2: "));
    IF_SDEBUG(Serial.println(meanno2));

    IF_SDEBUG(Serial.print("maxco: "));
    IF_SDEBUG(Serial.println(maxco));
    IF_SDEBUG(Serial.print("maxno2: "));
    IF_SDEBUG(Serial.println(maxno2));

    cbco60n.autoput(minco);
    cbno260n.autoput(minno2);

    cbco60m.autoput(meanco);
    cbno260m.autoput(meanno2);

    cbco60x.autoput(maxco);
    cbno260x.autoput(maxno2);

    cbsum2co.autoput(sum2co);
    cbsumco.autoput(sumco);

    cbsum2no2.autoput(sum2no2);
    cbsumno2.autoput(sumno2);


    minco=LONG_MAX;
    meanco=0.;
    maxco=0.;

    minno2=LONG_MAX;
    meanno2=0.;
    maxno2=0.;

    sum2co=0;
    sumco=0;

    sum2no2=0;
    sumno2=0;

  }

  // sigma

  if (cbsum2co.getSize() == cbsum2co.getCapacity() && cbsumco.getSize() == cbsumco.getCapacity()){
    sum260=0;
    for (i=0 ; i < cbsum2co.getCapacity() ; i++){
      sum260 += cbsum2co.peek(i);
    }

    sum60=0;
    for (i=0 ; i < cbsumco.getCapacity() ; i++){
      sum60 += cbsumco.peek(i);
    }
	
    i2c_dataset1->cono2.sigmaco=round(sqrt((sum260-(sum60*sum60)/(SAMPLE1*SAMPLE2))/(SAMPLE1*SAMPLE2)));
      
  }else{
    i2c_dataset1->cono2.sigmaco=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("sigma co: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.sigmaco));


  if (cbsum2no2.getSize() == cbsum2no2.getCapacity() && cbsumno2.getSize() == cbsumno2.getCapacity()){
    sum260=0;
    for (i=0 ; i < cbsum2no2.getCapacity() ; i++){
      sum260 += cbsum2no2.peek(i);
    }

    sum60=0;
    for (i=0 ; i < cbsumno2.getCapacity() ; i++){
      sum60 += cbsumno2.peek(i);
    }
	
    i2c_dataset1->cono2.sigmano2=round(sqrt((sum260-(sum60*sum60)/(SAMPLE1*SAMPLE2))/(SAMPLE1*SAMPLE2)));
      
  }else{
    i2c_dataset1->cono2.sigmano2=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("sigma no2: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.sigmano2));

#endif

  
  if (nsample1 == SAMPLE1) {
    nsample1=0;
  }

  nsample1++;



#ifdef SDS011PRESENT

  // second level pm25

  IF_SDEBUG(Serial.print("data in store second pm25 min: "));
  IF_SDEBUG(Serial.println(cbpm2560n.getSize()));

  if (cbpm2560n.getSize() == cbpm2560n.getCapacity()){
    i2c_dataset1->pm.minpm25=LONG_MAX;

    for (i=0 ; i < cbpm2560n.getCapacity() ; i++){
      i2c_dataset1->pm.minpm25 = min(cbpm2560n.peek(i), i2c_dataset1->pm.minpm25);
    }

  }else{
    i2c_dataset1->pm.minpm25=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("pm25 second min: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.minpm25));


  IF_SDEBUG(Serial.print("data in store second pm25 mean: "));
  IF_SDEBUG(Serial.println(cbpm2560m.getSize()));

  if (cbpm2560m.getSize() == cbpm2560m.getCapacity()){
    mean=0;
    for (i=0 ; i < cbpm2560m.getCapacity() ; i++){
      mean += (cbpm2560m.peek(i) - mean) / (i+1);
    }

    i2c_dataset1->pm.meanpm25=round(mean);

  }else{
    i2c_dataset1->pm.meanpm25=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("pm25 second mean: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.meanpm25));


  IF_SDEBUG(Serial.print("data in store second pm25 max: "));
  IF_SDEBUG(Serial.println(cbpm2560x.getSize()));

  if (cbpm2560x.getSize() == cbpm2560x.getCapacity()){
    i2c_dataset1->pm.maxpm25=0;
    for (i=0 ; i < cbpm2560x.getCapacity() ; i++){
      i2c_dataset1->pm.maxpm25 = max(cbpm2560x.peek(i), i2c_dataset1->pm.maxpm25);
    }
  }else{
    i2c_dataset1->pm.maxpm25=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("pm25 second max: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.maxpm25));


  // second level pm10

  IF_SDEBUG(Serial.print("data in store second pm10 min: "));
  IF_SDEBUG(Serial.println(cbpm1060n.getSize()));

  if (cbpm1060n.getSize() == cbpm1060n.getCapacity()){
    i2c_dataset1->pm.minpm10=LONG_MAX;
    for (i=0 ; i < cbpm1060n.getCapacity() ; i++){
      i2c_dataset1->pm.minpm10 = min(cbpm1060n.peek(i), i2c_dataset1->pm.minpm10);
    }
  }else{
    i2c_dataset1->pm.minpm10=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("pm10 second min: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.minpm10));


  IF_SDEBUG(Serial.print("data in store second pm10 mean: "));
  IF_SDEBUG(Serial.println(cbpm1060m.getSize()));

  if (cbpm1060m.getSize() == cbpm1060m.getCapacity()){
    mean=0;
    for (i=0 ; i < cbpm1060m.getCapacity() ; i++){
      mean += (cbpm1060m.peek(i) - mean) / (i+1);
    }

    i2c_dataset1->pm.meanpm10=round(mean);

  }else{
    i2c_dataset1->pm.meanpm10=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("pm10 second mean: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.meanpm10));


  IF_SDEBUG(Serial.print("data in store second pm10 max: "));
  IF_SDEBUG(Serial.println(cbpm1060x.getSize()));

  if (cbpm1060x.getSize() == cbpm1060x.getCapacity()){
    i2c_dataset1->pm.maxpm10=0;
    for (i=0 ; i < cbpm1060x.getCapacity() ; i++){
      i2c_dataset1->pm.maxpm10 = max(cbpm1060x.peek(i), i2c_dataset1->pm.maxpm10);
    }
  }else{
    i2c_dataset1->pm.maxpm10=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("pm10 second max: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->pm.maxpm10));

#endif


#ifdef MICS4514PRESENT

  // second level co

  IF_SDEBUG(Serial.print("data in store second co min: "));
  IF_SDEBUG(Serial.println(cbco60n.getSize()));

  if (cbco60n.getSize() == cbco60n.getCapacity()){
    i2c_dataset1->cono2.minco=LONG_MAX;

    for (i=0 ; i < cbco60n.getCapacity() ; i++){
      i2c_dataset1->cono2.minco = min(cbco60n.peek(i), i2c_dataset1->cono2.minco);
    }

  }else{
    i2c_dataset1->cono2.minco=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("co second min: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.minco));


  IF_SDEBUG(Serial.print("data in store second co mean: "));
  IF_SDEBUG(Serial.println(cbco60m.getSize()));

  if (cbco60m.getSize() == cbco60m.getCapacity()){
    mean=0;
    for (i=0 ; i < cbco60m.getCapacity() ; i++){
      mean += (cbco60m.peek(i) - mean) / (i+1);
    }

    i2c_dataset1->cono2.meanco=round(mean);

  }else{
    i2c_dataset1->cono2.meanco=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("co second mean: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.meanco));


  IF_SDEBUG(Serial.print("data in store second co max: "));
  IF_SDEBUG(Serial.println(cbco60x.getSize()));

  if (cbco60x.getSize() == cbco60x.getCapacity()){
    i2c_dataset1->cono2.maxco=0;
    for (i=0 ; i < cbco60x.getCapacity() ; i++){
      i2c_dataset1->cono2.maxco = max(cbco60x.peek(i), i2c_dataset1->cono2.maxco);
    }
  }else{
    i2c_dataset1->cono2.maxco=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("co second max: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.maxco));


  // second level no2

  IF_SDEBUG(Serial.print("data in store second no2 min: "));
  IF_SDEBUG(Serial.println(cbno260n.getSize()));

  if (cbno260n.getSize() == cbno260n.getCapacity()){
    i2c_dataset1->cono2.minno2=LONG_MAX;
    for (i=0 ; i < cbno260n.getCapacity() ; i++){
      i2c_dataset1->cono2.minno2 = min(cbno260n.peek(i), i2c_dataset1->cono2.minno2);
    }
  }else{
    i2c_dataset1->cono2.minno2=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("no2 second min: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.minno2));


  IF_SDEBUG(Serial.print("data in store second no2 mean: "));
  IF_SDEBUG(Serial.println(cbno260m.getSize()));

  if (cbno260m.getSize() == cbno260m.getCapacity()){
    mean=0;
    for (i=0 ; i < cbno260m.getCapacity() ; i++){
      mean += (cbno260m.peek(i) - mean) / (i+1);
    }

    i2c_dataset1->cono2.meanno2=round(mean);

  }else{
    i2c_dataset1->cono2.meanno2=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("no2 second mean: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.meanno2));


  IF_SDEBUG(Serial.print("data in store second no2 max: "));
  IF_SDEBUG(Serial.println(cbno260x.getSize()));

  if (cbno260x.getSize() == cbno260x.getCapacity()){
    i2c_dataset1->cono2.maxno2=0;
    for (i=0 ; i < cbno260x.getCapacity() ; i++){
      i2c_dataset1->cono2.maxno2 = max(cbno260x.peek(i), i2c_dataset1->cono2.maxno2);
    }
  }else{
    i2c_dataset1->cono2.maxno2=MISSINTVALUE;
  }

  IF_SDEBUG(Serial.print("no2 second max: "));
  IF_SDEBUG(Serial.println(i2c_dataset1->cono2.maxno2));

#endif

  
  digitalWrite(LEDPIN,!digitalRead(LEDPIN));  // blink Led

}  
