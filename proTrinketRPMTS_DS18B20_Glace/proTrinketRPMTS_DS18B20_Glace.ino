#include <SFE_CC3000.h>
#include <SFE_CC3000_Client.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

//DS18B20 Define
#define ONE_WIRE_BUS 3

//CC3000 Define
#define CC3000_INT 3
#define CC3000_EN 5
#define CC3000_CS 10
#define IP_ADDR_LEN     4   // Length of IP address in bytes

//SHT1x define
#define dataPin 6
#define clockPin 8

// Watchdog define
#define LOGGING_FREQ_SECONDS   900       // Seconds to wait before a new sensor reading is logged.
#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8  // Number of times to sleep (for 8 seconds) before
                                                         // a sensor reading is taken and sent to the server.
                                                         // Don't change this unless you also change the 
                                                         // watchdog timer configuration.
                                                         
//Constantes Wifi
char ap_ssid[] = "RPMTS";                // SSID of network
char ap_password[] = "rpmtspassword";        // Password of network
unsigned int ap_security = WLAN_SEC_WPA2; // Security of network
unsigned int timeout = 30000;             // Milliseconds
char server[] = "rpmts.com";      // Remote host site

//Instance des objets

SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);
SFE_CC3000_Client client = SFE_CC3000_Client(wifi);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

int sleepIterations = 0;
uint32_t ip;
volatile bool watchdogActivated = false;

float tempCGlace;
void logSensorReading();
void setupWiFiCC3000();
void printTemperature(DeviceAddress deviceAddress);

boolean setupWifiCC3000(){
  ConnectionInfo connection_info;
  int i;

  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() )
  {
    Serial.println(F("CC3000 Ready!"));
  }
  else
  {
    // Error: 0 - Something went wrong during CC3000 init!
    Serial.println(F("Error: 0"));
    return false;
  }

  // Connect using DHCP
  Serial.print(F("Connecting to: "));
  Serial.println(ap_ssid);
  if(!wifi.connect(ap_ssid, ap_security, ap_password, timeout))
  {
    // Error: 1 - Could not connect to AP
    Serial.println(F("Error: 1"));
  }

  // Gather connection details and print IP address
  if ( !wifi.getConnectionInfo(connection_info) ) 
  {
    // Error: 2 - Could not obtain connection details
    Serial.println(F("Error: 2"));
    return false;
  }
  else
  {
    Serial.print(F("My IP: "));
    for (i = 0; i < IP_ADDR_LEN; i++)
    {
      Serial.print(connection_info.ip_address[i]);
      if ( i < IP_ADDR_LEN - 1 )
      {
        Serial.print(".");
      }
    }
    Serial.println();
  }
  return true;
}

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

void printTemperature(DeviceAddress deviceAddress)
{
   // method 2 - faster
  tempCGlace = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempCGlace);
}


// Query des sensor 
void logSensorReading() {

  // Take a sensor reading

  sensors.requestTemperatures();
  printTemperature(insideThermometer);

if ( !client.connect(server, 80) )
  {
    // Error: 4 - Could not make a TCP connection
    Serial.println(F("Error: 4"));
  }

  // Post the data! Request should look a little something like:
  // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&time=5201 HTTP/1.1\n
  // Host: data.sparkfun.com\n
  // Connection: close\n
  // \n
  client.print("GET /input/");
  client.print(/*publicKey*/"");
  client.print("?private_key=");
  client.print(/*privateKey*/"");
  for (int i=0; i</*NUM_FIELDS*/10; i++)
  {
    client.print("&");
    client.print(/*fieldNames[i]*/ "");
    client.print("=");
    client.print(/*fieldData[i]*/ "");
  }
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(server);
  client.println("Connection: close");
  client.println();

  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      Serial.print(c);
    }      
  }
  Serial.println();
  
  
  // Note that if you're sending a lot of data you
  // might need to tweak the delay here so the CC3000 has
  // time to finish sending all the data before shutdown.
  delay(100);
  
}


void setup(void)
{  
  Serial.begin(115200);
  
  // Initialize the CC3000.
  Serial.println(F("\nInitializing CC3000..."));
  
  setupWifiCC3000();

  //start DS18B20
  sensors.begin();
  delay(1000);
   
 
  // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
  
  Serial.println(F("Setup complete."));
}

void loop(void)
{
  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated)
  {
    watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      // Reset the number of sleep iterations.
      sleepIterations = 0;
      // Log the sensor data (waking the CC3000, etc. as needed)
      if (setupWifiCC3000()) {
        logSensorReading();
      }
    }
  }
  
  // Go to sleep!
  sleep();
}

// Put the Arduino to sleep
void sleep()
{
  // fermer tous les sensor via le shutdown du vreg
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);

  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
  delay(1000);
  digitalWrite(A0, HIGH);
}


