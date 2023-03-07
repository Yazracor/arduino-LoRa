#include <SPI.h>
#include <LoRa.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

ISR(WDT_vect) {
}

volatile bool dio0_rise;
//volatile bool dio1_rise;

byte dio0_pin = 2;
//byte dio1_pin = 3;

void setup() {
  // disable ADC
  ADCSRA = 0;

  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  pinMode(dio0_pin, INPUT);
  //pinMode(dio1_pin, INPUT);

  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // wdt sleep settings:
  // 10.8 Watchdog Timer
  WDTCSR |= 1<<WDCE | 1<<WDE;

  // Table 10-3. Watchdog Timer Prescale Select
  //WDTCSR = 1<<WDP3 | 1<<WDP0;           // 8 seconds
  //WDTCSR = 1<<WDP3;                     // 4 seconds
  //WDTCSR = 1<<WDP2 | 1<<WDP1 | 1<<WDP0; // 2 seconds
  //WDTCSR = 1<<WDP2 | 1<<WDP1;           // 1 seconds
  //WDTCSR = 1<<WDP0;                     // 32 ms
  WDTCSR = 0;                             // 16 ms

  WDTCSR |= 1<<WDIE;

  // Setup external interrupt INT0 and INT1
  //
  //12.2 Register Description
  //12.2.1 EICRA – External Interrupt Control Register A
  EICRA = 0;
  //EICRA |= (1<<ISC10); // Any logical change on INT1 generates an interrupt request.
  //EICRA |= (1<<ISC00); // Any logical change on INT0 generates an interrupt request.
  //EICRA |= 1<<ISC11 | 1<<ISC10; // The rising edge of INT1 generates an interrupt request.
  EICRA |= 1<<ISC01 | 1<<ISC00; // The rising edge of INT0 generates an interrupt request.

  //12.2.2 EIMSK – External Interrupt Mask Register
  EIMSK = 0;
  //EIMSK |= (1<<INT1); // Bit 1 – INT1: External Interrupt Request 1 Enable
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

//ISR (INT1_vect){
//  dio1_rise = true;
//}

bool parse_packet(){
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
    return true;
  }

  return false;
}

void send_message(){
  LoRa.beginPacket();
  LoRa.print("hello");
  LoRa.endPacket();
}

void loop() {
   if (LoRa.cadModeActive && dio0_rise){
      // dio0: CadDone
      // dio1: CadDetected
      dio0_rise = false;
      // dio1 is not used in this sketch: CadDetected is looked up in RegIrqFlags
      
      if (LoRa.irqCadDetected()){
         LoRa.receive();

         // these attributes are not really needed in this sketch anymore
         LoRa.cadModeActive = false;
         //LoRa.rxSingleMode = true;

         // sleep a while until RxDone is triggered
         for (uint8_t i = 0; i < 20; i++){
            go_to_sleep();
            if (dio0_rise){
               dio0_rise = false;
               parse_packet();
               Serial.flush();
               break;
            }
         }

         // this idle() solves the problem of the power consumption sometimes staying at 12 mA when
         // a packet couldn't be received
         LoRa.idle();

         LoRa.cadMode();
      } else {
         // nothing detected: wait before initiating the next cadMode
         // put both radio and microcontroller to sleep
         LoRa.sleep();
         go_to_sleep();

         /*
         // send a message periodically
         static uint16_t send_message_counter{0};
         send_message_counter++;
         if (send_message_counter > 250){
            send_message_counter = 0;
            send_message();
         }
         */

         LoRa.cadMode();
      }
   }
   
   go_to_sleep();
}
