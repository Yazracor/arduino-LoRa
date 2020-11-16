#include <SPI.h>
#include <LoRa.h>

uint32_t counter          = 0;
uint32_t init_radio_count = 0;

uint8_t pinchange_pin = 0;
volatile bool dio0_rise{false};
volatile uint8_t rtc_ovf{0};

uint8_t probe_pin = 1; // pin 1 / PA23

void setup() {
  turn_off_usb();
  setup_pinchange_pin();
  setup_rtc();

  //pinMode(probe_pin, OUTPUT);

  Serial1.begin(9600);
  Serial1.println("LoRa receiver");
  Serial1.flush();
  
  while (!init_radio()){
      delay(1000);
  }

  power_save_test(); // try to lower power consumption
}

void power_save_test(){
  //PM->AHBMASK.bit.USB_  = 0;
  //PM->AHBMASK.bit.USB_  = 0;

  //PM->APBAMASK.reg = 0;
  PM->APBAMASK.bit.PAC0_    = 0;
  PM->APBAMASK.bit.SYSCTRL_ = 0;
  PM->APBAMASK.bit.GCLK_    = 0;
  PM->APBAMASK.bit.WDT_     = 0;
  //PM->APBAMASK.bit.GCLK_ = 0;
  //PM->APBAMASK.bit.EIC_ = 1;
  
  PM->APBBMASK.reg = 0;
  PM->APBBMASK.bit.PORT_  = 1; // port for pins (led)

  //PM->APBCMASK.reg = 0;

  // Atmel-42248-SAM-D20-Power-Measurements_ApplicationNote_AT04188.pdf
  // 4.2 Configuring the Power Manager
  // In addition to disabling the synchronous clocks on the APBx buses some of the clocks on the high speed bus need
  // to be disabled. The clocks to be disabled are the CLK_HPB1_AHB, CLK_HBP2_AHB and CLK_DSU_AHB clocks.
  //PM->AHBMASK.bit.HPB1_ = 0;
  //PM->AHBMASK.bit.HPB2_ = 0;
  //PM->AHBMASK.bit.DSU_  = 0;

  PM->APBASEL.reg = 4;
  PM->APBBSEL.reg = 4;
  PM->APBCSEL.reg = 4;
}

void setup_rtc(){
  // Configure the external crystal----------------------------------------------
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_ONDEMAND |       // Enble one demand mode
                         SYSCTRL_XOSC32K_RUNSTDBY |       // Enable run-on-standby mode
                         SYSCTRL_XOSC32K_EN32K |          // Enable the crystal oscillator IO pads
                         SYSCTRL_XOSC32K_XTALEN |         // Enable the crystal oscillator
                         SYSCTRL_XOSC32K_STARTUP(6) |     // Set the crystal start-up time
                         SYSCTRL_XOSC32K_ENABLE;          // Enable the oscillator

  // Configure clock source and clock generators (gclk.h)------------------------   
  GCLK->GENDIV.reg =  GCLK_GENDIV_ID(4) |     // Select GLCK4
                      //GCLK_GENDIV_DIV(4);   // Select clock divisor to divide by 32 (2 ^ (4 + 1)) to generate 1.024kHz
                      GCLK_GENDIV_DIV(32);    // Select clock divisor to divide by 32
                      //GCLK_GENDIV_DIV(128); // Select clock divisor to divide by 128                      
  while (GCLK->STATUS.bit.SYNCBUSY);          // Wait for synchronization
 
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(4) |        // Select GCLK4
                      GCLK_GENCTRL_SRC_XOSC32K |  // Select generic clock source as the external 32.768kHz crystal                     
                      GCLK_GENCTRL_IDC |          // Improve duty cycle for odd div factors
                      //GCLK_GENCTRL_RUNSTDBY |   // Enable run standby mode                 
                      //GCLK_GENCTRL_DIVSEL |     // Set GLCK divisor as 2 to the power of (divisor) value // zelf uitgecomment!!
                      GCLK_GENCTRL_GENEN;         // Enable GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
     
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK4 |  // Select GCLK4
                      GCLK_CLKCTRL_ID_RTC |     // Connect to the RTC
                      GCLK_CLKCTRL_CLKEN;       // Enable GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);            // Wait for synchronization
  
  // RTC configuration (rtc.h)--------------------------------------------------                                             
  RTC->MODE1.CTRL.bit.ENABLE = 0;                    // Disable the RTC
  while (RTC->MODE1.STATUS.bit.SYNCBUSY);            // Wait for synchronization

  RTC->MODE1.CTRL.bit.SWRST = 1;                     // Software reset the RTC
  while (RTC->MODE1.STATUS.bit.SYNCBUSY);            // Wait for synchronization
 
  RTC->MODE1.CTRL.reg |=  //RTC_MODE1_CTRL_PRESCALER_DIV1024 |  // Set prescaler to 1024 (commented out!)
                          RTC_MODE1_CTRL_MODE_COUNT16;          // Set RTC to mode 1, 16-bit timer                         
 
 
  RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(30);   // interrupt every 30 ms
  while (RTC->MODE1.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  // Configure RTC interrupts ------------------------------------------
  RTC->MODE1.INTENSET.reg = RTC_MODE1_INTENSET_OVF;  // Enable RTC overflow interrupts


  NVIC_SetPriority(RTC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for RTC
  NVIC_EnableIRQ(RTC_IRQn);         // Connect RTC to Nested Vector Interrupt Controller (NVIC)

  // Enable Deep Sleep Mode--------------------------------------------------------------
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // | SCB_SCR_SLEEPONEXIT_Msk; // Put the SAMD21 in deep sleep upon executing the __WFI() function
  //NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_SLEEPPRM_DISABLED;           // Disable auto power reduction during sleep - SAMD21 Errata 1.14.2

  // Enable RTC--------------------------------------------------------------
  RTC->MODE1.CTRL.bit.ENABLE = 1;                       // Enable the RTC
  while (RTC->MODE1.STATUS.bit.SYNCBUSY);               // Wait for synchronization
}

void setup_pinchange_pin(){ 
  pinMode(pinchange_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinchange_pin), pin_change, RISING);

  // select clock for EIC: External Interrupt Controller
  // OSCULP32K: GCLK2
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK2 |  // Select GCLK2
                      GCLK_CLKCTRL_ID_EIC |     // External Interrupt Controller
                      GCLK_CLKCTRL_CLKEN;       // Enable GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);            // Wait for synchronization
}

void pin_change(){
  dio0_rise = true;
}

bool init_radio(){
  init_radio_count++;

  LoRa.setPins(7, -1, 1);    // set CS, reset, IRQ pin: mkrzero + rfm95
  //LoRa.setPins(12, -1, 6); // set CS, reset, IRQ pin: Sparkfun Pro RF
  //LoRa.setPins(8, -1, 4);  // set CS, reset, IRQ pin: Adafruit Feather M0 LoRa

  // setting lower SPI clock solved problems one time, not needed anymore
  // default frequency: 8 MHz at 48 MHz systemclock
  //LoRa.setSPIFrequency(200000); 200 kHz

  if (init_radio_count > 10){
    Serial1.print("check radio");
    while(1){}
  }
  
  if (!LoRa.begin(915E6)) {
    Serial1.print("Starting LoRa failed! init_radio_count: ");
    Serial1.println(init_radio_count);
    return false;
  }

  LoRa.setPreambleLength(30);

  LoRa.sleep();
  return true;
}

void wfi_sleep(){
  //digitalWrite(probe_pin, LOW);
  //REG_PORT_OUTCLR0 = 1<<23; // PA23
  
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  __WFI();
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

  //digitalWrite(probe_pin, HIGH);
  //REG_PORT_OUTSET0 = 1<<23; // PA23 
}

void RTC_Handler(void){
  rtc_ovf++;
  RTC->MODE1.INTFLAG.bit.OVF = 1;     // Reset the overflow interrupt flag
}

void turn_off_usb(){
  // effort to lower power consumption
  USBDevice.detach();

  // doesn't seem to help anything
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_USB; 
                      //GCLK_CLKCTRL_GEN_GCLK0 | // Select GCLK0
                      //GCLK_CLKCTRL_ID_USB |     
                      //GCLK_CLKCTRL_CLKEN;      // enable clock
  while (GCLK->STATUS.bit.SYNCBUSY);             // Wait for synchronization

  PM->AHBMASK.bit.USB_  = 0;
  PM->APBBMASK.bit.USB_ = 0;

  // turn off USB interrupt
  NVIC->ICER[0] = 1<<USB_IRQn;  
}

bool parse_packet(){
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial1.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial1.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial1.print("' with RSSI ");
    Serial1.println(LoRa.packetRssi());
    return true;
  }

  return false;
}

void loop() {
  static uint16_t display_on_counter = 0;
  static uint16_t count{0};
  count++;

  if (LoRa.cadModeActive && dio0_rise){
      // dio0: CadDone
      // dio1: CadDetected
      dio0_rise = false;
      // dio1 is not used in this sketch: CadDetected is looked up in RegIrqFlags
      
      if (LoRa.irqCadDetected()){
         LoRa.receive();

         LoRa.cadModeActive = false;

         // sleep a while until RxDone is triggered
         for (uint8_t i = 0; i < 15; i++){
            wfi_sleep();
            if (dio0_rise){
               while (dio0_rise){
                  // multiple packets could arrive while the first is being processed
                  // keep checking for RxDone 
                  dio0_rise = false;
                  parse_packet();
               }

               Serial1.flush();
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
         wfi_sleep();

         LoRa.cadMode();
      }
   } else {
      // setup cadmode: normally this section is run once in the first itereration of the loop
      LoRa.cadMode();
   }

   wfi_sleep();
}
