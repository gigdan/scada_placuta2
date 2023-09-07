#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define UDP_TX_PACKET_MAX_SIZE 54  //marim la 40 dimensiunea bufferului UDP

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) != _BV(bit))

#define ACK_OK 0x01
#define ACK_NOT_OK 0xff
#define NR_RETRY 0x03

#define MIN_INTERVAL 2000
#define TIMEOUT -1

#define TIMEOUT_DHT 100
#define DHT11_PIN 7

#define DEVICE_DEFINED 0
#define DEVICE_SERIAL_ID 1
#define MAC_START 5         //address in EEPROM
#define IP_SERVER 11        //address in EEPROM
#define IP_DEVICE 15        //address in EEPROM
#define CONNECT_PORT 19     //address in EEPROM
#define BASE_IND_EVENTS 35  //address in EEPROM
#define BASE_EVENTS 40      //address in EEPROM

int board_Type = 2;  //1-white ; 2-yellow
bool debugOn = true;
unsigned long serialIdDevice = 111401;
byte eepromDefault = 0xa3;

//111111-111210 white board
//111211-111396 yellow board


byte mac[] = { 0x44, 0xA2, 0xDA, 0x0D, 0x5C, 0x81 };  //se seteaza mai jos ultimii 2 octeti in functie de serialIdDevice
byte ip_device[] = { 10, 150, 0, 254 };
byte ip_server[] = { 10, 150, 0, 55 };
byte eth_Gateway[] = { 0, 0, 0, 0 };
byte eth_Subnet[] = { 255, 255, 0, 0 };



typedef union u_long {
  unsigned long valLong;
  byte valByte[4];
} uLong;

typedef union u_int {
  unsigned int valInt;
  byte valByte[2];
} uInt;

typedef struct ev_Struct {
  uint8_t eventType_Ev;
  uint8_t eventData_Ev;
  uint8_t eventData_Ev1;
  uint8_t eventData_Ev2;
  uint8_t eventData_Ev3;
  uint8_t eventData_EvOut1;
  uint8_t eventData_EvOut2;
  uint8_t eventData_EvOut3;
  uint8_t eventData_EvOut4;
  unsigned long packetId_Ev;
  unsigned long unixTime_Ev;
  unsigned long temperature_Ev;
  uint8_t humidity_Ev;
} uEvent;



uInt port;

uEvent currentEvent;                   //capul stivei
volatile static uint8_t indEvent = 0;  //indexul in stiva al evenimentului

uEvent event;

uint8_t c = 0, I_RH, D_RH, I_Temp, D_Temp, CheckSum;

//uint8_t nrRetrySendUDP = 0;

uint8_t packetBuffer[UDP_TX_PACKET_MAX_SIZE];
uint8_t packetBufferRec[UDP_TX_PACKET_MAX_SIZE];

uLong snDeviceUN;
uLong unixTimeUN;
uLong lengthDataPckUN;
uLong packetIdUN;
uLong temperatureUN;

unsigned long unixTime = 0;        // from 2019
unsigned long lengthDataPck = 33;  //valoare fixa a pachetului
unsigned long packetId = 1;

unsigned long temperature = 21;
uint8_t humidity = 55;

uint8_t timerOutSendData = 0;

uLong unixTimeUN_Rec;
uLong lengthDataPckUN_Rec;
uLong packetIdUN_Rec;

uint8_t sendPeriod = 10;
// Set 1 - testat
unsigned char contactStatus_PD4 = 2;  //1 deschis; 2 - inchis D4
unsigned char contactStatus_PD7 = 2;  //1 deschis; 2 - inchis D7
// Set 2 - de testat
unsigned char contactStatus_PB0 = 2;  //1 deschis; 2 - inchis D8
unsigned char contactStatus_PB1 = 2;  //1 deschis; 2 - inchis D9
// Set 1 - testat
unsigned char iesireStatus_PC4 = 2;  //1 deschis; 2 - inchis A4
unsigned char iesireStatus_PC5 = 2;  //1 deschis; 2 - inchis A5
// Set 2 - de testat
unsigned char iesireStatus_PC3 = 2;  //1 deschis; 2 - inchis A3
unsigned char iesireStatus_PC2 = 2;  //1 deschis; 2 - inchis A2

unsigned char eventType;  // 2-de la timer de 1 minut; 1- de la deschidere cutie

uint8_t sendingPeriod = 0;
uint8_t ackPckRec = 0;


EthernetUDP Udp;

uint8_t timeoutInput = 0;
bool resetSystem = false;
uint8_t eepromDefined = 0;
uint8_t second = 0;

bool isOnline = false;
uint8_t isOfflineCounter = 1;
uint8_t noPacketReset = 5;  //daca nu am primit un ack timp de  5 packete...atunci reset ethernet
bool dataFromOffline = false;

bool isSendSettings = false;

bool errorDHT = false;
bool blinklight = false;
/////////////////////////////print HEX/////////////////////////
void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

/////////////////////////////blinLed/////////////////////////
void blinkLed() {
  if (blinklight)
    PORTD |= (1 << PIND4);
  else
    PORTD &= ~(1 << PIND4);

  blinklight = 1 - blinklight;
}


/////////////////////////////initEthernet/////////////////////////
void initEthernet() {
  isOfflineCounter = 1;


  if (debugOn) {
    Serial.println(F("Init ethernet..."));
  }

  if (board_Type == 1) {
    if (debugOn) {
      Serial.println(F("White board..."));
    }
    //white board
    PORTD &= ~(1 << PIND5);  //set 0
    delay(200);
    PORTD |= (1 << PIND5);  //set 1
    delay(200);
    PORTD &= ~(1 << PIND5);  //set 0
  } else {
    if (debugOn) {
      Serial.println(F("Yellow board..."));
    }
    //yellow board
    PORTD |= (1 << PIND5);  //set 1
    delay(200);
    PORTD &= ~(1 << PIND5);  //set 0
    delay(200);
    PORTD |= (1 << PIND5);  //set
  }







  if (debugOn) {
    Serial.print(F("Starting ethernet..."));
  }


  //dns server ip
  //mac, ipDevice, ipServer, ethGateway, ethSubnet
  // the dns server ip
  IPAddress dnServer(ip_server[0], ip_server[1], ip_server[2], ip_server[3]);
  // the router's gateway address:
  IPAddress ethGateway(eth_Gateway[0], eth_Gateway[1], eth_Gateway[2], eth_Gateway[3]);
  // the subnet:
  IPAddress ethSubnet(eth_Subnet[0], eth_Subnet[1], eth_Subnet[2], eth_Subnet[3]);
  //the IP address is dependent on your network
  IPAddress localIp(ip_device[0], ip_device[1], ip_device[2], ip_device[3]);

  port.valInt = 17173;



  Ethernet.begin(mac, localIp, dnServer, ethGateway, ethSubnet);

  delay(20);
  Udp.begin(port.valInt);

  delay(20);

  if (debugOn) {
    Serial.print("IP : ");
    Serial.println(Ethernet.localIP());
    Serial.print("Port UDP : ");
    Serial.println(port.valInt);
  }
}


void setup() {
  delay(100);
  noInterrupts();
  ////////////////////////////////watchdog/////////////////////////////////
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | (1 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);

  ////////////////////////////////INITIALIZARE HARDWARE/////////////////////////////////
  DDRD |= (1 << PIND5);  //PD5 reset retea
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 7813;             ////62500;//31250;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  DDRD |= (1 << PIND4);  //PD4 iesire pt led
  PORTD &= ~(1 << PIND4);
  // Intrari:
  DDRD &= ~(1 << PIND3);  //PD6 o facem intrare pt switch OBS: este port RESET -> Nu are legatura cu butoanele

  DDRD &= ~(1 << PIND2);  //PD5 o facem intrare pt switch
  DDRD &= ~(1 << PIND6);  //PD7 o facem intrare pt switch

  DDRB &= ~(1 << PINB0);
  DDRB &= ~(1 << PINB0);

  PORTD |= (1 << PIND3);  //activam pull up resistor OBS: este port RESET -> Nu are legatura cu butoanele

  PORTD |= (1 << PIND2);  //activam pull up resistor
  PORTD |= (1 << PIND6);  //activam pull up resistor

  PORTB |= (1 << PINB0);
  PORTB |= (1 << PINB1);

  // Iesiri:
  DDRC |= (1 << PINC4);  // P4(A4) iesire pompa hidraulica
  DDRC |= (1 << PINC5);  // P5(A5) iesire pompa hidraulica

  DDRC |= (1 << PINC3);
  DDRC |= (1 << PINC2);

  PORTC &= ~(1 << PINC4);
  PORTC &= ~(1 << PINC5);

  PORTC &= ~(1 << PINC3);
  PORTC &= ~(1 << PINC2);

  PCICR |= (1 << PCIE2) | (1 << PCIE0);  //| (1 << PCIE1);  //PCINT17-PCINT23 in sus sau PCINT8-14 portc pini analog

  PCMSK2 |= (1 << PCINT19) | (1 << PCINT18) | (1 << PCINT22);  // Pentru PORTD
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);                     // Pentru PORTB

  // PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);

  ////////////////////////////////INIT SERIAL/////////////////////////////////
  if (debugOn) {
    Serial.begin(9600);
  }

  if (debugOn) {
    Serial.println("Sistem start");
  }



  snDeviceUN.valLong = serialIdDevice;

  mac[5] = snDeviceUN.valByte[0];
  mac[4] = snDeviceUN.valByte[1];

  if (debugOn) {
    Serial.println("MAC: ");
    for (int i = 0; i < 6; i++) {
      printHex(mac[i]);
    }
    Serial.println(" ");
  }


  eepromDefined = EEPROM.read(0);

  if (eepromDefined != eepromDefault) {
    //scriem eeprom default
    EEPROM.write(0, eepromDefault);
    writeIPMAC(ip_server, ip_device, mac, snDeviceUN.valByte, port.valByte, eth_Gateway, eth_Subnet);

    if (debugOn)
      Serial.println("Scrie eeprom default");

    EEPROM.write(BASE_IND_EVENTS, 0);
    indEvent = 0;
  } else {
    //citim setarile din eeprom
    readIPMAC();
    if (debugOn)
      Serial.println("Citeste eeprom");
    indEvent = EEPROM.read(BASE_IND_EVENTS);
  }

  ////////////////////////////////UDP//////////////////////////////////////////
  initEthernet();
  ////////////////////////////////SET SERIAL DEVICE///////////////////////////
  interrupts();
  // Set testat
  if (PIND & 0x08)  //contact deschis
  {
    eventType = 1;
    contactStatus_PD4 = 1;
    PORTC |= (1 << PINC4);  // Deschid P4
    iesireStatus_PC4 = 1;
  } else {  //contact inchis
    eventType = 1;
    contactStatus_PD4 = 2;
    PORTC &= ~(1 << PINC4);  // Inchid P4
    iesireStatus_PC4 = 2;
  }

  if (PIND & 0x40)  //contact deschis
  {
    eventType = 1;
    contactStatus_PD7 = 1;
    PORTC |= (1 << PINC5);  // Deschid P5
    iesireStatus_PC5 = 1;   //if(iesireStatus_PC5 == 1){Serial.println("iesire Deschisa P5");} // MESAJ DEBUG PORT DESCHIS
  } else {                  //contact inchis
    eventType = 1;
    contactStatus_PD7 = 2;
    PORTC &= ~(1 << PINC5);  // Inchid P5
    iesireStatus_PC5 = 2;
  }
  // Set de testat
  if (PINB & 0x00)  //contact deschis pin 8
  {
    eventType = 1;
    contactStatus_PB0 = 1;
    PORTC |= (1 << PINC3);  // Deschid P3
    iesireStatus_PC3 = 1;
  } else {  //contact inchis
    eventType = 1;
    contactStatus_PB0 = 2;
    PORTC &= ~(1 << PINC3);  // Inchid P3
    iesireStatus_PC3 = 2;
  }

  if (PINB & 0x01)  //contact deschis pin 9
  {
    eventType = 1;
    contactStatus_PB1 = 1;
    PORTC |= (1 << PINC2);  // Deschid P2
    iesireStatus_PC2 = 1;   //if(iesireStatus_PC5 == 1){Serial.println("iesire Deschisa P5");} // MESAJ DEBUG PORT DESCHIS
  } else {                  //contact inchis
    eventType = 1;
    contactStatus_PB1 = 2;
    PORTC &= ~(1 << PINC2);  // Inchid P2
    iesireStatus_PC2 = 2;
  }
}
ISR(WDT_vect)  // Watchdog timer interrupt.
{
  if (debugOn)
    Serial.println("Sistemul se va reseta");
}

ISR(PCINT2_vect) {

  if (!(PIND & 0x04))  //butonul de  resetare default
  {
    EEPROM.write(0, 0xff);
    resetSystem = true;
    if (debugOn)
      Serial.println("RESET");
  }


  if (!resetSystem) {
    if (PIND & 0x08)  //contact deschis
    {
      eventType = 1;
      contactStatus_PD4 = 1;
      PORTC |= (1 << PINC4);  // Deschid P4
      iesireStatus_PC4 = 1;
      if (debugOn)
        Serial.println("Mesaj deshis P4");
      timeoutInput = 3;
      sendingPeriod = 0;
    } else {  //contact inchis
      eventType = 1;
      contactStatus_PD4 = 2;
      PORTC &= ~(1 << PINC4);  // Inchid P4
      iesireStatus_PC4 = 2;
      if (debugOn)
        Serial.println("Mesaj inchis P4");
      timeoutInput = 3;
      sendingPeriod = 0;
    }
    if (PIND & 0x40)  //contact deschis
    {
      eventType = 1;
      contactStatus_PD7 = 1;
      PORTC |= (1 << PINC5);  // Deschid P5
      iesireStatus_PC5 = 1;
      if (debugOn)
        Serial.println("Mesaj deshis P7");
      timeoutInput = 3;
      sendingPeriod = 0;
    } else {  //contact inchis
      eventType = 1;
      contactStatus_PD7 = 2;
      PORTC &= ~(1 << PINC5);  // Inchid P5
      iesireStatus_PC5 = 2;
      if (debugOn)
        Serial.println("Mesaj inchis P7");
      timeoutInput = 3;
      sendingPeriod = 0;
    }
    // Set de testat
    if (PINB & 0x00)  //contact deschis pin 8
    {
      eventType = 1;
      contactStatus_PB0 = 1;
      PORTC |= (1 << PINC3);  // Deschid P3
      iesireStatus_PC3 = 1;
      if (debugOn)
        Serial.println("Mesaj deshis P3");
      timeoutInput = 3;
      sendingPeriod = 0;
    } else {  //contact inchis
      eventType = 1;
      contactStatus_PB0 = 2;
      PORTC &= ~(1 << PINC3);  // Inchid P3
      iesireStatus_PC3 = 2;
      if (debugOn)
        Serial.println("Mesaj inchis P3");
      timeoutInput = 3;
      sendingPeriod = 0;
    }

    if (PINB & 0x01)  //contact deschis pin 9
    {
      eventType = 1;
      contactStatus_PB1 = 1;
      PORTC |= (1 << PINC2);  // Deschid P2
      iesireStatus_PC2 = 1;   //if(iesireStatus_PC5 == 1){Serial.println("iesire Deschisa P5");} // MESAJ DEBUG PORT DESCHIS
      if (debugOn)
        Serial.println("Mesaj deshis P2");
      timeoutInput = 3;
      sendingPeriod = 0;
    } else {  //contact inchis
      eventType = 1;
      contactStatus_PB1 = 2;
      PORTC &= ~(1 << PINC2);  // Inchid P2
      iesireStatus_PC2 = 2;
      if (debugOn)
        Serial.println("Mesaj inchis P2");
      timeoutInput = 3;
      sendingPeriod = 0;
    }
  }
}



ISR(TIMER1_COMPA_vect)  //intrerupere timer1
{
  if (!resetSystem) {
    wdt_reset();
  }

  if (timerOutSendData > 0) {
    timerOutSendData--;
    if (timerOutSendData == 0) {
      if (debugOn)
        Serial.println("Timerout ZERO");

      isOnline = false;
      writeEvent(event);
    }
  }

  //aici trimite asincron
  if (timeoutInput > 0) {
    timeoutInput--;
    if (timeoutInput == 0) {
      if (debugOn)
        Serial.println("Trimite UDP");
      eventType = 1;

      //   PORTD |= (1 << PIND7);
      buildPckUDP();
      Udp.beginPacket(ip_server, port.valInt);
      Udp.write(packetBuffer, 53);

      Udp.endPacket();

      buildPckUDP();
      Udp.beginPacket({ 10, 150, 0, 253 }, port.valInt);
      Udp.write(packetBuffer, 53);

      Udp.endPacket();
      timerOutSendData = 9;
      //     PORTD &= ~(1 << PIND7);

      //verific cate pachete nu s-au trimis...si daca am atins noPacketReset atunci reset ethernet
      //daca primesc ACK atunci isOfflineCounter devine 1;
      if (isOfflineCounter % noPacketReset == 0) {
        if (debugOn) {
          Serial.print("RESET ETHERNET...isOfflineCounter=");
          Serial.println(isOfflineCounter);
        }
        initEthernet();
      } else
        isOfflineCounter++;
    }
  }

  if (second > 6) {

    //afisare led
    if (!errorDHT) {
      if (isOnline)
        blinkLed();  //ledul blincane
      else {
        PORTD &= ~(1 << PIND4);  //led stins P4
      }
    }

    if (isSendSettings) {
      isSendSettings = false;

      buildSettingsPckUDP();
      Udp.beginPacket(ip_server, port.valInt);
      Udp.write(packetBuffer, 30);

      Udp.endPacket();

      buildPckUDP();
      Udp.beginPacket({ 10, 150, 0, 253 }, port.valInt);
      Udp.write(packetBuffer, 30);

      Udp.endPacket();
    }

    second = 0;

    unixTime++;
    if (sendingPeriod >= sendPeriod - 1) {
      sendingPeriod = 0;

      if (!resetSystem) {
        Request();
        Response();
        I_RH = Receive_data();
        D_RH = Receive_data();
        I_Temp = Receive_data();
        D_Temp = Receive_data();
        CheckSum = Receive_data();

        if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum) {
          if (debugOn)
            Serial.println("Eroare DHT11");
        } else {
          humidity = I_RH;
          temperature = I_Temp * 100 + D_Temp;
        }

        //aici trimite pachet UDP
        //  if (debugOn)
        //   Serial.println("Trimite UDP SINCRON");
        eventType = 2;
        buildPckUDP();
        Udp.beginPacket(ip_server, port.valInt);
        Udp.write(packetBuffer, 53);
        // Udp.println("Mesaje UDP");

        Udp.endPacket();

        buildPckUDP();
        Udp.beginPacket({ 10, 150, 0, 253 }, port.valInt);
        Udp.write(packetBuffer, 53);

        Udp.endPacket();
        if (debugOn) {
          for (int i = 0; i < 4; i++)
            printHex(ip_server[i]);
          Serial.println(" A trimis UDP SINCRON");
        }

        //verific cate pachete nu s-au trimis...si daca am atins noPacketReset atunci reset ethernet
        //daca primesc ACK atunci isOfflineCounter devine 1;
        if (isOfflineCounter % noPacketReset == 0) {
          if (debugOn) {
            Serial.print("RESET ETHERNET...isOfflineCounter=");
            Serial.println(isOfflineCounter);
          }
          initEthernet();
        } else
          isOfflineCounter++;
      }
    } else {
      sendingPeriod++;



      if ((indEvent > 0) && isOnline) {
        if (debugOn)
          Serial.println("TRIMITE EVENIMENT DIN OFFLINE");
        buildPckUDP_FromEvents();
        Udp.beginPacket(ip_server, port.valInt);
        Udp.write(packetBuffer, 53);
        Udp.endPacket();
        buildPckUDP_FromEvents();
        Udp.beginPacket({ 10, 150, 0, 253 }, port.valInt);
        Udp.write(packetBuffer, 53);
        Udp.endPacket();
      }
    }  //minute

  } else {
    second++;
    if (errorDHT) {
      if ((second & 0x01) == 0x01)
        blinkLed();  //ledul blincane
    }
  }
}

void buildSettingsPckUDP() {
  unsigned char checksum = 0;
  unsigned char sum = 0;
  int i = 0;
  uLong serialIdUN;

  packetBuffer[0] = 0;
  packetBuffer[1] = 0;
  packetBuffer[2] = 5;
  packetBuffer[3] = 5;

  //server IP
  packetBuffer[4] = ip_server[0];
  packetBuffer[5] = ip_server[1];
  packetBuffer[6] = ip_server[2];
  packetBuffer[7] = ip_server[3];
  //device IP
  packetBuffer[8] = ip_device[0];
  packetBuffer[9] = ip_device[1];
  packetBuffer[10] = ip_device[2];
  packetBuffer[11] = ip_device[3];
  //serialID
  serialIdUN.valLong = serialIdDevice;
  packetBuffer[12] = serialIdUN.valByte[0];
  packetBuffer[13] = serialIdUN.valByte[1];
  packetBuffer[14] = serialIdUN.valByte[2];
  packetBuffer[15] = serialIdUN.valByte[3];

  packetBuffer[16] = port.valByte[0];
  packetBuffer[17] = port.valByte[1];
  packetBuffer[18] = 0;
  packetBuffer[19] = 0;
  //gateway
  packetBuffer[20] = eth_Gateway[0];
  packetBuffer[21] = eth_Gateway[1];
  packetBuffer[22] = eth_Gateway[2];
  packetBuffer[23] = eth_Gateway[3];

  //gateway
  packetBuffer[24] = eth_Subnet[0];
  packetBuffer[25] = eth_Subnet[1];
  packetBuffer[26] = eth_Subnet[2];
  packetBuffer[27] = eth_Subnet[3];


  for (i = 4; i <= 27; i++) {
    sum = sum + packetBuffer[i];
    checksum = checksum ^ packetBuffer[i];
  }

  packetBuffer[28] = checksum;
  packetBuffer[29] = sum;
}



void buildPckUDP_FromEvents() {
  unsigned char checksum = 0;
  unsigned char sum = 0;
  int i = 0;


  //scoatem din stiva elementul curent
  //daca primeste ACK il scoatem din stiva
  if (debugOn)
    Serial.print("Trimite EVENIMENT DIN OFFLINE");

  currentEvent = readEvent();

  unixTimeUN.valLong = currentEvent.unixTime_Ev;
  lengthDataPckUN.valLong = lengthDataPck;
  packetIdUN.valLong = currentEvent.packetId_Ev;
  temperatureUN.valLong = currentEvent.temperature_Ev;


  packetBuffer[0] = 0;
  packetBuffer[1] = 0;
  packetBuffer[2] = 0;
  packetBuffer[3] = 0x02;  //tip pachet

  packetBuffer[4] = lengthDataPckUN.valByte[0];  //lungime pachet
  packetBuffer[5] = lengthDataPckUN.valByte[1];
  packetBuffer[6] = lengthDataPckUN.valByte[2];
  packetBuffer[7] = lengthDataPckUN.valByte[3];  //lungime pachet

  packetBuffer[8] = snDeviceUN.valByte[0];  //device serial number
  packetBuffer[9] = snDeviceUN.valByte[1];
  packetBuffer[10] = snDeviceUN.valByte[2];
  packetBuffer[11] = snDeviceUN.valByte[3];  //device serial number

  packetBuffer[12] = 0x01;  //number of fix data

  packetBuffer[13] = unixTimeUN.valByte[0];  //unix time
  packetBuffer[14] = unixTimeUN.valByte[1];
  packetBuffer[15] = unixTimeUN.valByte[2];
  packetBuffer[16] = unixTimeUN.valByte[3];  //unix time

  packetBuffer[17] = packetIdUN.valByte[0];  //packet id
  packetBuffer[18] = packetIdUN.valByte[1];
  packetBuffer[19] = packetIdUN.valByte[2];
  packetBuffer[20] = packetIdUN.valByte[3];  //pachet id

  packetBuffer[21] = 2;  //intotdeauna asincron  cand e din offline(din eeprom)

  packetBuffer[22] = 0x05;

  packetBuffer[23] = 0x04;
  packetBuffer[24] = 128;
  packetBuffer[25] = currentEvent.eventData_Ev;  //1 deschis; 2 - inchis
  packetBuffer[26] = 0x81;
  packetBuffer[27] = currentEvent.humidity_Ev;
  packetBuffer[28] = 0x82;
  packetBuffer[29] = currentEvent.eventData_Ev1;  //1 deschis; 2 - inchis
  packetBuffer[30] = 0x83;
  packetBuffer[31] = currentEvent.eventData_EvOut1;
  packetBuffer[32] = 0x84;
  packetBuffer[33] = currentEvent.eventData_EvOut2;
  packetBuffer[34] = 0x85;
  packetBuffer[35] = currentEvent.eventData_EvOut3;
  packetBuffer[36] = 0x86;
  packetBuffer[37] = currentEvent.eventData_EvOut4;
  packetBuffer[38] = 0x87;
  packetBuffer[39] = currentEvent.eventData_Ev2;
  packetBuffer[40] = 0x88;
  packetBuffer[41] = currentEvent.eventData_Ev2;
  packetBuffer[42] = 0x00;
  packetBuffer[43] = 0x01;

  packetBuffer[44] = 0x88;

  packetBuffer[45] = temperatureUN.valByte[0];  //temperature
  packetBuffer[46] = temperatureUN.valByte[1];
  packetBuffer[47] = temperatureUN.valByte[2];
  packetBuffer[48] = temperatureUN.valByte[3];

  packetBuffer[49] = 0x00;  //NoOfEightByteIO
  packetBuffer[50] = 0x01;  //numberOfData

  for (i = 8; i <= 50; i++) {
    sum = sum + packetBuffer[i];
    checksum = checksum ^ packetBuffer[i];
  }

  packetBuffer[51] = checksum;
  packetBuffer[52] = sum;

  dataFromOffline = true;
}



void buildPckUDP() {
  unsigned char checksum = 0;
  unsigned char sum = 0;
  int i = 0;

  unixTimeUN.valLong = unixTime;
  lengthDataPckUN.valLong = lengthDataPck;
  packetIdUN.valLong = packetId;
  temperatureUN.valLong = temperature;


  packetBuffer[0] = 0;
  packetBuffer[1] = 0;
  packetBuffer[2] = 0;
  packetBuffer[3] = 0x01;  //tip pachet

  packetBuffer[4] = lengthDataPckUN.valByte[0];  //lungime pachet
  packetBuffer[5] = lengthDataPckUN.valByte[1];
  packetBuffer[6] = lengthDataPckUN.valByte[2];
  packetBuffer[7] = lengthDataPckUN.valByte[3];  //lungime pachet

  packetBuffer[8] = snDeviceUN.valByte[0];  //device serial number
  packetBuffer[9] = snDeviceUN.valByte[1];
  packetBuffer[10] = snDeviceUN.valByte[2];
  packetBuffer[11] = snDeviceUN.valByte[3];  //device serial number

  packetBuffer[12] = 0x01;  //number of fix data

  packetBuffer[13] = unixTimeUN.valByte[0];  //unix time
  packetBuffer[14] = unixTimeUN.valByte[1];
  packetBuffer[15] = unixTimeUN.valByte[2];
  packetBuffer[16] = unixTimeUN.valByte[3];  //unix time

  packetBuffer[17] = packetIdUN.valByte[0];  //packet id
  packetBuffer[18] = packetIdUN.valByte[1];
  packetBuffer[19] = packetIdUN.valByte[2];
  packetBuffer[20] = packetIdUN.valByte[3];  //pachet id

  packetBuffer[21] = eventType;  // 1-de la timer de 1 minut; 2- de la deschidere cutie

  packetBuffer[22] = 0x05;

  packetBuffer[23] = 0x04;
  packetBuffer[24] = 128;
  packetBuffer[25] = contactStatus_PD4;  //1 deschis; 2 - inchis
  //----------------------------------
  packetBuffer[26] = 0x81;
  packetBuffer[27] = humidity;
  packetBuffer[28] = 0x82;
  packetBuffer[29] = contactStatus_PD7;  //1 deschis; 2 - inchis
  packetBuffer[30] = 0x83;
  packetBuffer[31] = iesireStatus_PC4;
  packetBuffer[32] = 0x84;
  packetBuffer[33] = iesireStatus_PC5;

  packetBuffer[34] = 0x85;
  packetBuffer[35] = contactStatus_PB0;  //1 deschis; 2 - inchis
  packetBuffer[36] = 0x86;
  packetBuffer[37] = contactStatus_PB1;  //1 deschis; 2 - inchis

  packetBuffer[38] = 0x87;
  packetBuffer[39] = iesireStatus_PC2;
  packetBuffer[40] = 0x88;
  packetBuffer[41] = iesireStatus_PC3;

  packetBuffer[42] = 0x00;

  packetBuffer[43] = 0x01;
  packetBuffer[44] = 0x88;

  packetBuffer[45] = temperatureUN.valByte[0];  //temperature
  packetBuffer[46] = temperatureUN.valByte[1];
  packetBuffer[47] = temperatureUN.valByte[2];
  packetBuffer[48] = temperatureUN.valByte[3];

  packetBuffer[49] = 0x00;  //NoOfEightByteIO
  packetBuffer[50] = 0x01;  //numberOfData



  for (i = 8; i <= 50; i++) {
    sum = sum + packetBuffer[i];
    checksum = checksum ^ packetBuffer[i];
  }

  packetBuffer[51] = checksum;
  packetBuffer[52] = sum;

  if (eventType == 1) {
    event.eventType_Ev = 1;
    event.eventData_Ev = contactStatus_PD4;
    event.eventData_Ev1 = contactStatus_PD7;
    event.eventData_Ev2 = contactStatus_PB0;
    event.eventData_Ev3 = contactStatus_PB1;
    event.eventData_EvOut1 = iesireStatus_PC4;
    event.eventData_EvOut2 = iesireStatus_PC5;
    event.eventData_EvOut3 = iesireStatus_PC2;
    event.eventData_EvOut4 = iesireStatus_PC3;
    event.packetId_Ev = packetIdUN.valLong;
    event.unixTime_Ev = unixTimeUN.valLong;
    event.temperature_Ev = temperatureUN.valLong;
    event.humidity_Ev = humidity;

    dataFromOffline = false;
  }
}




void showRec() {
  int i;

  for (i = 0; i < 20; i++) {
    printHex(packetBufferRec[i]);
    if (i == 3)
      Serial.print("   Lungime: ");
    else if (i == 7)
      Serial.print("   UnixTime: ");
    else if (i == 11)
      Serial.print("   PckID: ");
    else if (i == 15)
      Serial.print("   Period: ");
    else if (i == 16)
      Serial.print("   Ack: ");
    else if (i == 17)
      Serial.print("  Checksum: ");
    else if (i == 18)
      Serial.print("   Sum: ");
    else
      Serial.print(' ');
  }
  Serial.println(" ");
}

void decodeNetPck() {
  //datele primite sunt in packetBufferRec
  uint8_t checksum = 0;
  uint8_t sum = 0;
  int i = 0;
  //am primit ceva pe UDP, placa ethernet functioneaza
  isOfflineCounter = 1;

  if ((packetBufferRec[2] == 6) && (packetBufferRec[4] == 6))  //pachet GET SETTINGS
  {
    if (debugOn)
      Serial.print("GET SETTINGS");
    isSendSettings = true;
  } else if (packetBufferRec[2] == 7)  //pachet de setari
  {
    //   if (debugOn)
    Serial.print("Pachet de setare");

    ip_server[0] = packetBufferRec[5];
    ip_server[1] = packetBufferRec[6];
    ip_server[2] = packetBufferRec[7];
    ip_server[3] = packetBufferRec[8];

    ip_device[0] = packetBufferRec[9];
    ip_device[1] = packetBufferRec[10];
    ip_device[2] = packetBufferRec[11];
    ip_device[3] = packetBufferRec[12];

    mac[0] = packetBufferRec[13];
    mac[1] = packetBufferRec[14];
    mac[2] = packetBufferRec[15];
    mac[3] = packetBufferRec[16];
    mac[4] = packetBufferRec[17];
    mac[5] = packetBufferRec[18];

    snDeviceUN.valByte[0] = packetBufferRec[19];
    snDeviceUN.valByte[1] = packetBufferRec[20];
    snDeviceUN.valByte[2] = packetBufferRec[21];
    snDeviceUN.valByte[3] = packetBufferRec[22];

    port.valByte[0] = packetBufferRec[23];
    port.valByte[1] = packetBufferRec[24];
    //byte-ul 25 si byteul26 sunt partea superioara a portului care , nefolositi, integer peste long

    eth_Gateway[0] = packetBufferRec[27];
    eth_Gateway[1] = packetBufferRec[28];
    eth_Gateway[2] = packetBufferRec[29];
    eth_Gateway[3] = packetBufferRec[30];

    eth_Subnet[0] = packetBufferRec[31];
    eth_Subnet[1] = packetBufferRec[32];
    eth_Subnet[2] = packetBufferRec[33];
    eth_Subnet[3] = packetBufferRec[34];

    writeIPMAC(ip_server, ip_device, mac, snDeviceUN.valByte, port.valByte, eth_Gateway, eth_Subnet);

    resetSystem = true;

  } else {
    if (debugOn)
      Serial.print("Decode ACK..");

    lengthDataPckUN_Rec.valByte[0] = packetBufferRec[4];
    lengthDataPckUN_Rec.valByte[1] = packetBufferRec[5];
    lengthDataPckUN_Rec.valByte[2] = packetBufferRec[6];
    lengthDataPckUN_Rec.valByte[3] = packetBufferRec[7];

    checksum = 0;  //de la trimitere
    sum = 0;

    for (i = 8; i <= 17; i++) {
      sum = sum + packetBufferRec[i];
      checksum = checksum ^ packetBufferRec[i];
    }

    // showRec();

    // Serial.print("  Checksum: ");
    // printHex(checksum);
    // Serial.print("  Sum: ");
    // printHex(sum);

    // Serial.println(" ");

    if ((checksum == packetBufferRec[18]) && (sum == packetBufferRec[19]))  //se valideaza checksumul
    {
      //  Serial.println("ACK PAS 2");
      if (lengthDataPckUN_Rec.valLong == 10)  //lungime pachet date trebuie sa fie 10
      {
        //   Serial.println("ACK PAS 3");
        unixTimeUN_Rec.valByte[0] = packetBufferRec[8];
        unixTimeUN_Rec.valByte[1] = packetBufferRec[9];
        unixTimeUN_Rec.valByte[2] = packetBufferRec[10];
        unixTimeUN_Rec.valByte[3] = packetBufferRec[11];

        packetIdUN_Rec.valByte[0] = packetBufferRec[12];
        packetIdUN_Rec.valByte[1] = packetBufferRec[13];
        packetIdUN_Rec.valByte[2] = packetBufferRec[14];
        packetIdUN_Rec.valByte[3] = packetBufferRec[15];

        sendPeriod = packetBufferRec[16];
        if (sendPeriod < 3)
          sendPeriod = 10;
        if (sendPeriod > 250)
          sendPeriod = 10;

        //  Serial.print("  Perioada: ");
        //  printHex(sendPeriod);

        ackPckRec = packetBufferRec[17];

        if (ackPckRec == ACK_OK) {
          timerOutSendData = 0;
          isOnline = true;

          if (packetId == packetIdUN.valLong)
            packetId++;  //incrementam Id-ul de pachet
          if (packetId > 19761107)
            packetId = 1;

          unixTime = unixTimeUN_Rec.valLong;

          if (dataFromOffline) {
            deleteEvent();
          }
          if (debugOn)
            Serial.println("ACK OK");
        }
      }
    }
  }
}


void loop() {
  int packetSize = Udp.parsePacket();

  if (packetSize) {  //if data available, prints following;
    if (debugOn) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
    }
    Udp.read(packetBufferRec, UDP_TX_PACKET_MAX_SIZE);

    delay(10);
    decodeNetPck();
  }
  delay(100);
}



/*******DHT11*********/
void Request()  //trimitem un puls de start
{
  DDRD |= (1 << DHT11_PIN);
  PORTD &= ~(1 << DHT11_PIN);  // LOW
  _delay_ms(20);
  PORTD |= (1 << DHT11_PIN);  //HIGH
}

void Response() {  //primim raspuns de la DHT11
  unsigned int timeOut = 0;

  errorDHT = false;

  DDRD &= ~(1 << DHT11_PIN);
  while ((PIND & (1 << DHT11_PIN)) && (timeOut < TIMEOUT_DHT)) {
    _delay_us(10);
    timeOut++;
  }

  if (timeOut == TIMEOUT_DHT) {
    errorDHT = true;
    return;
  }

  timeOut = 0;
  while (((PIND & (1 << DHT11_PIN)) == 0) && (timeOut < TIMEOUT_DHT)) {
    _delay_us(10);
    timeOut++;
  }
  if (timeOut == TIMEOUT_DHT) {
    errorDHT = true;
    return;
  }

  timeOut = 0;
  while ((PIND & (1 << DHT11_PIN)) && (timeOut < TIMEOUT_DHT)) {
    _delay_us(10);
    timeOut++;
  }
  if (timeOut == TIMEOUT_DHT) {
    errorDHT = true;
    return;
  }
}

uint8_t Receive_data() {
  unsigned int timeOutRec = 0;

  errorDHT = false;
  for (int q = 0; q < 8; q++) {
    while (((PIND & (1 << DHT11_PIN)) == 0) && (timeOutRec < TIMEOUT_DHT))  //verifica daca e bit 0 sau 1
    {
      _delay_us(10);
      timeOutRec++;
    }
    if (timeOutRec == TIMEOUT_DHT) {
      errorDHT = true;
      return;
    }
    _delay_us(30);
    if (PIND & (1 << DHT11_PIN))  //daca pulsul HIGH mai mare de 30msec /* if high pulse is greater than 30ms */
      c = (c << 1) | (0x01);      //este HIGH
    else
      c = (c << 1);  //este LOW

    timeOutRec = 0;
    while ((PIND & (1 << DHT11_PIN)) && (timeOutRec < TIMEOUT_DHT)) {
      _delay_us(10);
      timeOutRec++;
    }
  }
  return c;
}
/*******DHT11*********/

/*******EVENIMENTE SALVATE IN EEPROM************/
void writeEvent(uEvent event) {
  int address, i;
  uint8_t buffer[24];
  uLong val;

  buffer[0] = event.eventType_Ev;
  buffer[1] = event.eventData_Ev;
  val.valLong = event.packetId_Ev;
  buffer[2] = val.valByte[0];
  buffer[3] = val.valByte[1];
  buffer[4] = val.valByte[2];
  buffer[5] = val.valByte[3];
  val.valLong = event.unixTime_Ev;
  buffer[6] = val.valByte[0];
  buffer[7] = val.valByte[1];
  buffer[8] = val.valByte[2];
  buffer[9] = val.valByte[3];
  val.valLong = event.temperature_Ev;
  buffer[10] = val.valByte[0];
  buffer[11] = val.valByte[1];
  buffer[12] = val.valByte[2];
  buffer[13] = val.valByte[3];
  buffer[14] = event.humidity_Ev;
  buffer[15] = event.eventData_Ev1;
  buffer[16] = event.eventData_EvOut1;
  buffer[17] = event.eventData_EvOut2;
  buffer[18] = event.eventData_EvOut3;
  buffer[19] = event.eventData_EvOut4;
  buffer[20] = event.eventData_Ev2;
  buffer[21] = event.eventData_Ev3;
  address = indEvent * 22;
  indEvent++;
  EEPROM.write(BASE_IND_EVENTS, indEvent);
  for (i = 0; i < 22; i++) {
    EEPROM.write(BASE_EVENTS + address + i, buffer[i]);
  }
  if (debugOn)
    Serial.println("WRITE  EVENT");
}

uEvent readEvent() {
  int address, i;
  uint8_t buffer[24];
  uint8_t d;
  uLong val;
  uEvent event;

  if (indEvent > 0)  //exista cel putin un eveniment in eeprom
  {
    address = (indEvent - 1) * 22;
    for (i = 0; i < 22; i++) {
      buffer[i] = EEPROM.read(BASE_EVENTS + address + i);
    }

    event.eventType_Ev = buffer[0];
    event.eventData_Ev = buffer[1];
    val.valByte[0] = buffer[2];
    val.valByte[1] = buffer[3];
    val.valByte[2] = buffer[4];
    val.valByte[3] = buffer[5];
    event.packetId_Ev = val.valLong;
    val.valByte[0] = buffer[6];
    val.valByte[1] = buffer[7];
    val.valByte[2] = buffer[8];
    val.valByte[3] = buffer[9];
    event.unixTime_Ev = val.valLong;
    val.valByte[0] = buffer[10];
    val.valByte[1] = buffer[11];
    val.valByte[2] = buffer[12];
    val.valByte[3] = buffer[13];
    event.temperature_Ev = val.valLong;
    event.humidity_Ev = buffer[14];
    event.eventData_Ev1 = buffer[15];
    event.eventData_EvOut1 = buffer[16];
    event.eventData_EvOut2 = buffer[17];
    event.eventData_EvOut3 = buffer[18];
    event.eventData_EvOut4 = buffer[19];
    event.eventData_Ev2 = buffer[20];
    event.eventData_Ev3 = buffer[21];
    Serial.println("READ  EVENT");
    return event;
  }
  event.eventData_EvOut1 = 0;
  event.eventData_EvOut2 = 0;
  event.eventData_EvOut3 = 0;
  event.eventData_EvOut4 = 0;
  event.eventType_Ev = 0;
  event.eventData_Ev1 = 0;
  event.eventData_Ev = 0;
  event.eventData_Ev2 = 0;
  event.eventData_Ev3 = 0;
  event.packetId_Ev = 0;
  event.unixTime_Ev = 0;
  event.temperature_Ev = 0;
  event.humidity_Ev = 0;
  return event;
}

bool deleteEvent()  //umbla doar la indexul de adresa indEvent;
{
  if (indEvent > 0) {
    indEvent--;
    if (debugOn)
      Serial.println("DELETE  EVENT");

    EEPROM.write(BASE_IND_EVENTS, indEvent);
    return true;
  }
  return false;
}

void readIPMAC() {

  for (int i = 1; i < 4; i++)
    snDeviceUN.valByte[i - 1] = EEPROM.read(i);

  for (int i = 5; i < 11; i++)
    mac[i - 5] = EEPROM.read(i);

  for (int i = 11; i < 15; i++)
    ip_server[i - 11] = EEPROM.read(i);

  for (int i = 15; i < 19; i++)
    ip_device[i - 15] = EEPROM.read(i);

  for (int i = 19; i < 21; i++)
    port.valByte[i - 19] = EEPROM.read(i);

  for (int i = 21; i < 25; i++)
    eth_Gateway[i - 21] = EEPROM.read(i);

  for (int i = 25; i < 29; i++)
    eth_Subnet[i - 25] = EEPROM.read(i);
}

void writeIPMAC(byte ip_server[], byte ip_device[], byte mac[], byte serial[], byte port[], byte gateway[], byte subnet[]) {

  for (int i = 1; i < 4; i++)
    EEPROM.write(i, serial[i - 1]);

  for (int i = 5; i < 11; i++)
    EEPROM.write(i, mac[i - 5]);

  for (int i = 11; i < 15; i++)
    EEPROM.write(i, ip_server[i - 11]);

  for (int i = 15; i < 19; i++)
    EEPROM.write(i, ip_device[i - 15]);

  for (int i = 19; i < 21; i++)
    EEPROM.write(i, port[i - 19]);

  for (int i = 21; i < 25; i++)  //gateway
    EEPROM.write(i, gateway[i - 21]);

  for (int i = 25; i < 29; i++)  //subnet
    EEPROM.write(i, subnet[i - 25]);
}

/*******EVENIMENTE SALVATE IN EEPROM************/
