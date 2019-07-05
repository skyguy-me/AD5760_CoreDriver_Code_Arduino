//Authored by Gokul
//Adapted from inital provided code from Yang Zhengmin


//#include <SPI.h>
// subtract 4 from each pin number of respective IO
#define MISO 46 //50
#define MOSI 47 //51
#define SCK 48 //52
#define SS 49 //53
#define LDAC 37 // define LDAC digital pin as pin 37
#define RESET 36 // define reset digital pin as pin 36

//SPISettings settings(100000, MSBFIRST, SPI_MODE0);


float voltage = 0;
float _step = 0.0003;


double testArray[21] = {0.466, 6.449, 6.690, 3.552, 8.886, 9.672, 5.541, 5.717, 7.909, 5.536, 4.989, 5.667, 9.440, 6.701, 7.108, 5.569, 2.791, 5.553, 8.111, 5.187, 4.110};
int j  = 0;

int i = 0;


void setup() {
  Serial.begin(9600);

  // set up pins
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(LDAC, OUTPUT);
  pinMode(RESET, OUTPUT);


  digitalWrite(SS, HIGH);
  digitalWrite(MOSI, LOW);
  digitalWrite(SCK, LOW);

  initialize();


  Serial.println("Started buffer output stability analysis:");
  Serial.println();
  Serial.println();


  for (i; i < 21; i++) {
    setVoltage(testArray[i]);
    delay(1200000);
  }

  i = 0;
}

void loop() {
  //each step 0.0001525 volts (16 bit)
  //thus minimum differnce must be 0.0003 V
  /*
    while (voltage <= 10.000) {
      voltage += _step;
      setVoltage((double)(voltage));
      delay(350);
    }
  */

  Serial.println("Started loop stability analysis");
  Serial.println();
  Serial.println();

  for (j; j < 1200000; j++) {
    if (i < 21) {
      setVoltage(testArray[i]);
      delay(1);
    }
  }
  j = 0;
  i++;
}



void setVoltage(double voltage) {
  uint16_t data = (uint16_t) (voltage * 65535 / 10);
  data = data & 0xffff;
  Serial.print("Voltage Set: ");
  Serial.print(voltage, 4);
  Serial.print("   Binary Voltage: ");
  Serial.print(data, BIN);

  uint8_t first = 0b00010000 | (data >> 12);
  uint16_t second = (data << 4) & (0xffff);

  Serial.print("   Binary Control Signal: ");
  Serial.print(first, BIN);
  Serial.println(second, BIN);

  digitalWrite(SS, LOW);
  sendByte(first);
  sendByte((uint8_t)((second >> 8) & 0xff));
  sendByte((uint8_t)((second) & 0xf0));
  digitalWrite(SS, HIGH);
}

void setSoftReg() {
  digitalWrite(SS, LOW);
  uint8_t high = 0b01000000;
  uint8_t mid = 0b00000000;
  uint8_t low = 0b00000000;
  sendByte(high);
  sendByte(mid);
  sendByte(low);
  digitalWrite(SS, HIGH);
}

void setControlReg() {
  digitalWrite(SS, LOW);
  uint8_t high = 0b00100000;
  uint8_t mid = 0b00000000;
  uint8_t low = 0b00010010;
  sendByte(high);
  sendByte(mid);
  sendByte(low);
  digitalWrite(SS, HIGH);
}

void initialize() {
  digitalWrite(SS, LOW);
  uint8_t high = 0b01000000;
  uint8_t mid = 0b00000000;
  uint8_t low = 0b00000000;
  sendByte(high);
  //digitalWrite(MOSI, LOW);
  sendByte(mid);
  //digitalWrite(MOSI, LOW);
  sendByte(low);
  //digitalWrite(MOSI, LOW);
  digitalWrite(SS, HIGH);

  digitalWrite(SS, LOW);
  uint8_t c_high = 0b00100000;
  uint8_t c_mid = 0b00000000;
  uint8_t c_low = 0b00010010;
  sendByte(c_high);
  //digitalWrite(MOSI, LOW);
  sendByte(c_mid);
  //digitalWrite(MOSI, LOW);
  sendByte(c_low);
  //digitalWrite(MOSI, LOW);
  digitalWrite(SS, HIGH);
}

/*
  void updateBuffer(){
  digitalWrite(LDAC, HIGH);
  delayMicroseconds(5);
  digitalWrite(LDAC, LOW);
  }
*/
void sendByte(uint8_t value) {
  //ok
  int i = 0;
  for (i; i < 8; i++) {
    digitalWrite(SCK, HIGH);
    if ((value & 0x80) == 0x80) {
      digitalWrite(MOSI, HIGH);
      //delay(16);
      //Serial.println("y");
    } else {
      digitalWrite(MOSI, LOW);
      //delay(16);
      //Serial.println("n");
    }
    //delayMicroseconds(16);
    value = value << 1;
    digitalWrite(SCK, LOW);
    //delayMicroseconds(16);
  }
}
