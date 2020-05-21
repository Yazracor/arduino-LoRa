#include <SPI.h>
#include <LoRa.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

ISR(WDT_vect) {
}

volatile bool dio0_rise;
volatile bool dio1_rise;

byte dio0_pin = 2;
byte dio1_pin = 3;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  pinMode(dio0_pin, INPUT);
  pinMode(dio1_pin, INPUT);

  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // wdt sleep settings:
  // 10.8 Watchdog Timer
  WDTCSR |= 1<<WDCE | 1<<WDE;

  // Table 10-3. Watchdog Timer Prescale Select
  //WDTCSR = WDTO_8S; // 1<<WDP3 | 1<<WDP0;           // 8 seconds
  //WDTCSR = WDTO_4S; // 1<<WDP3;                     // 4 seconds
  //WDTCSR = WDTO_2S; // 1<<WDP2 | 1<<WDP1 | 1<<WDP0; // 2 seconds
  //WDTCSR = WDTO_1S; // 1<<WDP2 | 1<<WDP1;           // 1 seconds
  //WDTCSR = WDTO_30MS;
  WDTCSR = WDTO_15MS;

  WDTCSR |= 1<<WDIE;

  // Setup external interrupt INT0 and INT1
  //
  //12.2 Register Description
  //12.2.1 EICRA – External Interrupt Control Register A
  EICRA = 0;
  //EICRA |= (1<<ISC10); // Any logical change on INT1 generates an interrupt request.
  //EICRA |= (1<<ISC00); // Any logical change on INT0 generates an interrupt request.
  EICRA |= 1<<ISC11 | 1<<ISC10; // The rising edge of INT1 generates an interrupt request.
  EICRA |= 1<<ISC01 | 1<<ISC00; // The rising edge of INT0 generates an interrupt request.

  //12.2.2 EIMSK – External Interrupt Mask Register
  EIMSK = 0;
  EIMSK |= (1<<INT1); // Bit 1 – INT1: External Interrupt Request 1 Enable
  EIMSK |= (1<<INT0); // Bit 0 – INT0: External Interrupt Request 0 Enable

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  LoRa.setPreambleLength(30);

  // initiates the first cadMode
  LoRa.cadMode();
}

void go_to_sleep() {
   // reset wdt counter
   wdt_reset();

   //sleep_mode: Put the device into sleep mode, taking care of setting the SE bit before, and clearing it afterwards.
   sleep_mode();
}

ISR (INT0_vect){
  dio0_rise = true;
}

ISR (INT1_vect){
  dio1_rise = true;
}

void parse_packet(){
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }  
}

void loop() {
   if (LoRa.cadModeActive && dio0_rise){
      // dio0: CadDone
      // dio1: CadDetected
      
      if (dio1_rise){
         dio0_rise = false;
         dio1_rise = false;
         // prepare radio to receive a single packet
         LoRa.setRxSingle();
      } else {

         // nothing detected: wait before initiating the next cadMode
         // put both radio and microcontroller to sleep
         LoRa.sleep();
         go_to_sleep();
         
         dio0_rise = false;
         dio1_rise = false;
         LoRa.cadMode();
      }
   } else if (LoRa.rxSingleMode){
      // dio0: RxDone
      // dio1: RxTimeout
      
      // only take action if either of the pins went high
      if (dio0_rise || dio1_rise){
         if (dio0_rise){
            parse_packet();
            Serial.flush();
         }
    
         dio0_rise = false;
         dio1_rise = false;
         LoRa.cadMode();
      }
   }

   go_to_sleep();
}
