#include "xxhash.h"
#include <due_can.h>

#define PIN_HIGH_BEAM 22
#define PIN_FUEL 2 //23
#define PIN_ILL 24
//#define PIN_SPEED 25 //Speed is on DAC1
#define PIN_CHECK_ENGINE 26
#define PIN_ABS 27
#define PIN_EBD 28 //aka Brake
#define PIN_TREAD 29 //PIN_HIGH_SPEED
#define PIN_TURN_L 30
#define PIN_TURN_R 31
#define PIN_ESC 32
#define PIN_ESC_OFF 33
#define PIN_SET 34
#define PIN_CRUISE 35
#define PIN_FOG 36
#define PIN_OIL 37
#define PIN_DOOR_OPEN 38
#define PIN_TRUNK_OPEN 39
#define PIN_CHARGE 40

const int PINS[] = {PIN_HIGH_BEAM, PIN_FUEL, PIN_ILL, PIN_CHECK_ENGINE, PIN_ABS, PIN_EBD, /*PIN_HIGH_SPEED,*/ PIN_TURN_L, PIN_TURN_R, PIN_ESC, PIN_ESC_OFF, PIN_SET, PIN_CRUISE, PIN_FOG, PIN_OIL, PIN_DOOR_OPEN, PIN_TRUNK_OPEN, PIN_CHARGE};
#define NUM_PINS 17

//const long microToSec = 1000;
byte byteBuffer[96];

//float waitDelay = 0;

CAN_FRAME canRpm;
CAN_FRAME canTemp;
CAN_FRAME canGear;
CAN_FRAME canEps;

struct dataPacket {
  unsigned int* timeOrder;    // time in milliseconds (to check order)
  char* car;                  // Car name (4 bytes)
  unsigned short* flags;      // Info (see OG_x below)
  unsigned char* gear;        // Reverse:0, Neutral:1, First:2...
  unsigned char* plid;        // Unique ID of viewed player (0 = none)
  float* speedMS;             // M/S
  float* rpm;                 // RPM
  float* turbo;               // BAR
  float* engtemp;             // C
  float* fuel;                // 0 to 1
  float* oilpressure;         // BAR
  float* oiltemp;             // C
  unsigned int* dashlights;   // Dash lights available (see DL_x below)
  unsigned int* showlights;   // Dash lights currently switched on
    //DL_SHIFT,            bit 0    - shift light
    //DL_FULLBEAM,         bit 1    - full beam
    //DL_HANDBRAKE,        bit 2    - handbrake
    //DL_PITSPEED,         bit 3    - pit speed limiter
    //DL_TC,               bit 4    - TC active or switched off
    //DL_SIGNAL_L,         bit 5    - left turn signal
    //DL_SIGNAL_R,         bit 6    - right turn signal
    //DL_SIGNAL_ANY,       bit 7    - shared turn signal
    //DL_OILWARN,          bit 8    - oil pressure warning
    //DL_BATTERY,          bit 9    - battery warning
    //DL_ABS,              bit 10   - ABS active or switched off
    //DL_ESC,              bit 11   - ESC
  float* throttle;            // 0 to 1
  float* brake;               // 0 to 1
  float* clutch;              // 0 to 1
  char* display1;             // Usually Fuel (16)
  char* display2;             // Usually Settings (16)

  XXH32_hash_t hash;
  XXH32_hash_t* hashRecv;
} clusterData;

void setup() {
  setupDac();

  pinMode(PIN_HIGH_BEAM, OUTPUT);
  pinMode(PIN_FUEL, OUTPUT);
  pinMode(PIN_ILL, OUTPUT);
  pinMode(PIN_CHECK_ENGINE, OUTPUT);
  pinMode(PIN_ABS, OUTPUT);
  pinMode(PIN_EBD, OUTPUT);
  //pinMode(PIN_HIGH_SPEED, OUTPUT);
  pinMode(PIN_TURN_L, OUTPUT);
  pinMode(PIN_TURN_R, OUTPUT);
  pinMode(PIN_ESC, OUTPUT);
  pinMode(PIN_ESC_OFF, OUTPUT);
  pinMode(PIN_SET, OUTPUT);
  pinMode(PIN_CRUISE, OUTPUT);
  pinMode(PIN_FOG, OUTPUT);
  pinMode(PIN_OIL, OUTPUT);
  pinMode(PIN_DOOR_OPEN, OUTPUT);
  pinMode(PIN_TRUNK_OPEN, OUTPUT);
  pinMode(PIN_CHARGE, OUTPUT);

  populateDataPacket(reinterpret_cast<unsigned char*>(byteBuffer), clusterData);
  Can0.begin(CAN_BPS_500K);

  //RPM CAN frame
  canRpm.id = 0x316;
  canRpm.extended = false;
  canRpm.priority = 4;
  canRpm.length = 8;
  canRpm.data.value = 0;

  //Temperature CAN frame
  canTemp.id = 0x329;
  canTemp.extended = false;
  canTemp.priority = 4;
  canTemp.length = 8;
  canTemp.data.value = 0;

  //Gear CAN frame
  canGear.id = 0x43F;
  canGear.extended = false;
  canGear.priority = 4;
  canGear.length = 8;
  canGear.data.value = 0;

  //EPS CAN frame
  canEps.id = 0x5E4;
  canEps.extended = false;
  canEps.priority = 4;
  canEps.length = 3;
  canEps.data.value = 0;
  
  SerialUSB.begin(115200);
  delay(1000);

  startupAnimation();
}

void setupDac() {
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                     // Enable peripheral clock
  REG_PIOB_ABSR |= PIO_ABSR_P16;                        // Select peripheral B
  REG_PIOB_PDR  |= PIO_PDR_P16;                         // Allow DAC to control output line
  REG_PWM_CLK    = PWM_CLK_PREA(0) | PWM_CLK_DIVA(255); // Set overall clock divider to 255 (~329 kHz)
  REG_PWM_CMR0   = PWM_CMR_CPRE_CLKA;                   // Set clock source to CLKA
  REG_PWM_CPRD0  = 65535;                               // Set period to 100%
  REG_PWM_CDTY0  = 16;                                  // Set duty cycle to nearly 0% (pulse)
  REG_PWM_ENA    = PWM_ENA_CHID0;                       // Enable PWM on channel 0
}

void digitalWriteArray(const int* pins, int numPins, bool state) {
  for (int i = 0; i < numPins; ++i) {
    digitalWrite(pins[i], state);
  }
}

void digitalWriteArrayOneByOne(const int* pins, int numPins, bool state, int delayTime) {
  for (int i = 0; i < numPins; ++i) {
    digitalWrite(pins[i], state);
    delay(delayTime);
    digitalWrite(pins[i], !state);
  }
}

void startupAnimation() {
  digitalWriteArray(PINS, NUM_PINS, LOW);
  //digitalWriteArrayOneByOne(PINS, NUM_PINS, HIGH, 500);
  //delay(400);
  
  digitalWriteArray(PINS, NUM_PINS, HIGH);
  delay(1000);
  digitalWriteArray(PINS, NUM_PINS, LOW);
  digitalWrite(PIN_ABS, HIGH);
  digitalWrite(PIN_ILL, HIGH);
  digitalWrite(PIN_TURN_L, HIGH);
  digitalWrite(PIN_TURN_R, HIGH);
  digitalWrite(PIN_HIGH_BEAM, HIGH);
  digitalWrite(PIN_EBD, HIGH);
  digitalWrite(PIN_FOG, HIGH);
  //analogWrite(PIN_FUEL, 128);
}

void serialReader() {
  if (SerialUSB.available() > 0) {
    byte bytesRead = SerialUSB.readBytesUntil('\n', byteBuffer, 96);
    calcPacketHash(reinterpret_cast<unsigned char*>(byteBuffer), clusterData);
    updateData();
  }
}

void updateData() {
  if (clusterData.hash == *clusterData.hashRecv) {
    //Speedometer
    //REG_PWM_CPRD0 = calcDelay(*clusterData.speedMS);
    REG_PWM_CPRDUPD0 = calcDelay(*clusterData.speedMS);

    //Fuel
    //analogWrite(PIN_FUEL, 512.1 * (*clusterData.fuel) + 2683.0611);
    analogWrite(PIN_FUEL, 128);

    //Lights directly controllable
    digitalWrite(PIN_ABS, !(*clusterData.showlights & 1<<10));
    digitalWrite(PIN_ESC, !!(*clusterData.showlights & 1<<11) & !(*clusterData.showlights & 1<<14));
    digitalWrite(PIN_HIGH_BEAM, !(*clusterData.showlights & 1<<1));
    digitalWrite(PIN_TURN_L, !((*clusterData.showlights & 1<<5) | (*clusterData.showlights & 1<<7)));
    digitalWrite(PIN_TURN_R, !((*clusterData.showlights & 1<<6) | (*clusterData.showlights & 1<<7)));
    digitalWrite(PIN_OIL, !!(*clusterData.showlights & 1<<8));
    digitalWrite(PIN_ILL, !(*clusterData.showlights & 1<<12));
    digitalWrite(PIN_CHECK_ENGINE, !!(*clusterData.showlights & 1<<13));
    digitalWrite(PIN_FOG, !(*clusterData.showlights & 1<<15));
    digitalWrite(PIN_EBD, !(*clusterData.showlights & 1<<2));
    //digitalWrite(PIN_HIGH_SPEED, *clusterData.speedMS > 55);
    digitalWrite(PIN_DOOR_OPEN, !!(*clusterData.showlights & 1<<18));
    digitalWrite(PIN_TRUNK_OPEN, !!(*clusterData.showlights & 1<<19));
    digitalWrite(PIN_CHARGE, !!(*clusterData.showlights & 1<<9));
    digitalWrite(PIN_ESC_OFF, !!(*clusterData.showlights & 1<<14));
    digitalWrite(PIN_SET, !!(*clusterData.showlights & 1<<17));
    digitalWrite(PIN_CRUISE, !!(*clusterData.showlights & 1<<16));
    digitalWrite(PIN_TREAD, !!(*clusterData.showlights & 1<<21));

    int rpm = static_cast<int>(*clusterData.rpm) * 4;
    canRpm.data.byte[2]= (byte) (rpm & 0xFF);
    canRpm.data.byte[3]= (byte) ((rpm & ~0xFF) >> 8);
    Can0.sendFrame(canRpm);
  
    canTemp.data.byte[1]= (byte) ((static_cast<int>(*clusterData.engtemp) + 48) * (1 + (1/3)));
    Can0.sendFrame(canTemp);

    canGear.data.byte[1] = (byte) *clusterData.gear;// & 0x0F;
    Can0.sendFrame(canGear);

    Can0.sendFrame(canEps);
  }
}

void loop() {
  serialReader();
}

int calcDelay(float mph) {
  /*if (mph < 1) {
    return 0;
  }
  //return 291059.0 / mph;
  return 130115.015 / mph;*/
  int piece = mph * 0.894774;//517; //Scale mph from m/s and figure out what part of the piecewise function it falls in
  switch (piece) {
    case 0:
      //Linear approximation between actual measured values (every 2.5 mph)
      return -29111.488893*mph+65535;
    case 1:
      return -10737.294197*mph+45000;
    case 2:
      return -4026.485324*mph+30000;
    case 3:
      return -3131.710807*mph+27000;
    case 4:
      return -2013.242662*mph+22000;
    case 5:
      return -1118.468145*mph+17000;
    case 6:
      return -1118.468145*mph+17000;
    case 7:
      return -492.125984*mph+12100;
    case 8:
      return -1073.729419*mph+17300;
    case 9:
      return -447.387258*mph+11000;
    case 10:
      return -536.864709*mph+12000;
    case 11:
      return -357.909806*mph+9800;
    case 12:
      return -402.648532*mph+10400;
    case 13:
      return -268.432354*mph+8450;
    case 14:
      return -268.432354*mph+8450;
    case 15:
      return -223.693629*mph+7700;
    case 16:
      return -223.693629*mph+7700;
    case 17:
      return -134.216177*mph+6000;
    case 18:
      return -178.954903*mph+6900;
    case 19:
      return -134.216177*mph+5950;
    case 20:
      return -134.216177*mph+5950;
    case 21:
      return -98.425196*mph+5110;
    case 22:
      return -111.846814*mph+5440;
    case 23:
      return -93.951324*mph+4980;
    case 24:
      return -71.581961*mph+4380;
    case 25:
      return -93.951324*mph+5005;
    case 26:
      return -89.477451*mph+4875;
    case 27:
      return -67.108088*mph+4200;
    case 28:
      return -67.108088*mph+4200;
    case 29:
      return -58.160343*mph+3910;
    case 30:
      return -53.686471*mph+3760;
    case 31:
      return -58.160343*mph+3915;
    case 32:
      return -53.686471*mph+3755;
    case 33:
      return -44.738725*mph+3425;
    case 34:
      return -40.264853*mph+3255;
    case 35:
      return -40.264853*mph+3255;
    case 36:
      return -40.264853*mph+3255;
    case 37:
      return -35.790980*mph+3070;
    case 38:
      return -26.843235*mph+2690;
    case 39:
      return -44.738725*mph+3470;
    case 40:
      return -31.317108*mph+2870;
    case 41:
      return -31.317108*mph+2870;
    case 42:
      return -26.843235*mph+2660;
    case 43:
      return -31.317108*mph+2875;
    case 44:
      return -26.843235*mph+2655;
    case 45:
      return -26.843235*mph+2655;
    case 46:
      return -22.369362*mph+2425;
    case 47:
      return -26.843235*mph+2660;
    case 48:
      return -22.369362*mph+2420;
    case 49:
      return -20.579813*mph+2322;
    case 50:
      return -19.685039*mph+2272;
    case 51:
      return -19.685039*mph+2271;
    case 52:
      return -17.895490*mph+2168;
    case 53:
      return -20.579813*mph+2327;
    case 54:
      return -8.947745*mph+1625;
    case 55:
      return -25.053686*mph+2615;
    default: //140mph
      return 16.729087*mph;
  }
}

void populateDataPacket(unsigned char* data, dataPacket& packet) {
  packet.timeOrder = reinterpret_cast<unsigned int*>(data);
  packet.car = reinterpret_cast<char*>(data + 4);
  packet.flags = reinterpret_cast<unsigned short*>(data + 8);
  packet.gear = reinterpret_cast<unsigned char*>(data + 10);
  packet.speedMS = reinterpret_cast<float*>(data + 12);
  packet.rpm = reinterpret_cast<float*>(data + 16);
  packet.turbo = reinterpret_cast<float*>(data + 20);
  packet.engtemp = reinterpret_cast<float*>(data + 24);
  packet.fuel = reinterpret_cast<float*>(data + 28);
  packet.oilpressure = reinterpret_cast<float*>(data + 32);
  packet.oiltemp = reinterpret_cast<float*>(data + 36);
  packet.dashlights = reinterpret_cast<unsigned int*>(data + 40);
  packet.showlights = reinterpret_cast<unsigned int*>(data + 44);
  packet.throttle = reinterpret_cast<float*>(data + 48);
  packet.brake = reinterpret_cast<float*>(data + 52);
  packet.clutch = reinterpret_cast<float*>(data + 56);
  packet.display1 = reinterpret_cast<char*>(data + 60);
  packet.display2 = reinterpret_cast<char*>(data + 76);

  packet.hashRecv = reinterpret_cast<XXH32_hash_t*>(data + 92);
}

void calcPacketHash(unsigned char* data, dataPacket& packet) {
  packet.hash = XXH32(data, 92, 0xebac6cdb); //hash is a uint32. Use a random seed (same for client)
}
