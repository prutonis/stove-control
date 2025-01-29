#include <Arduino.h>
#include <EEPROM.h>
#include <LibPrintf.h>
#include <SoftwareSerial.h>
#include <StateMachine.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <VirtualWire.h>

#define DEVICE_ID "SC"
#define DEVICE_SID "SC:"
#define MASTER_ID "MC"
#define MASTER_SID "MC:"

// SERIAL
// pin 7 - RX
// pin 8 - TX
// pin 9 - Direction Control --> when LOW = RX, when HIGH = TX
#define SER1_RX 7
#define SER1_TX 8
#define SER1_TDC 9
#define SER1_BUF_LEN 128
#define VW_MAX_MESSAGE_LEN 32
SoftwareSerial ser1(SER1_RX, SER1_TX);  // RX, TX

#define LED LED_BUILTIN

#define resetTimer(x) x = millis()
#define timerExceed(x, y) (millis() - x) > y

#define TEMP_SENSOR A0

#define NOMINAL_RESISTANCE 10000       //Nominal resistance at 25⁰C
#define NOMINAL_TEMPERATURE 25   // temperature for nominal resistance (almost always 25⁰ C)
#define BETA 3950  // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define R_REF 10000   //Value of  resistor used for the voltage divider

#define M_GET_STATUS "status?"
#define M_REPLY_STATUS "%sstatus=%s"
#define M_GET_STOVE_TEMP "stove?"
#define M_REPLY_STOVE_TEMP "%sstove=%d"
#define M_REPLY_UNKNOWN_CMD "%s#?"
#define M_PUMP_ON "pump_on"
#define M_PUMP_OFF "pump_off"
#define M_GAS_BOILER_ON "gb_on"
#define M_GAS_BOILER_OFF "gb_off"
#define M_REPLY_OK "%sOK"
#define M_STOVE_PUMP_START_TEMP "set_spt="
#define M_GET_INFO "info"
#define M_START_STOP_TEMP_DELTA "set_delta="
#define M_REMOTE_CTRL_OFF "rc_off"

#define PUMP_START_TEMP_EEPROM_ADDR 16
#define START_STOP_TEMP_DELTA_EEPROM_ADDR 18

#define stringStartsWith(s, p) (strncmp(s, p, strlen(p)) == 0)
#define stringEquals(s1, s2) (strcmp(s1, s2) == 0)
#define remoteControlOn() stoveCtl.remoteCtrl = true
#define remoteControlOff() stoveCtl.remoteCtrl = false


#define STOVE_PUMP_PIN A2
#define GAS_BOILER_PIN A1

#define RADIO_POWER_PIN 3
/********************************************************************/
// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 4
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
/********************************************************************/

void serialReceive(void);
void serialTransmit(void);
void stovePumpSet(bool onOff);
void gasBoilerSet(bool onOff);
void readThermistor(void);
void readDS18B20();
bool transitionS0S1();
bool transitionS1S0();

void st_idle(void);
void st_pump_on(void);
void st_gas_stove_on(void);
uint8_t checkSum(char *msg);

float vr, tv, temperature;  // variables for thermistor
uint32_t t0, t1, t2, t3, t4, t5;  // timers
uint16_t pulseCt;
bool last = HIGH, vwMr = false;
uint8_t ct = 0;
char serialBuffer[SER1_BUF_LEN] = "";
char vwbuffer[VW_MAX_MESSAGE_LEN] = "";
struct SerCtl {
    char *rmsg;
    char *rcmd;
    const char *tmsg;
    uint8_t ridx;
    uint8_t tidx;
} serCtl = {
    serialBuffer,
    serialBuffer + strlen(DEVICE_SID),
    serialBuffer + (SER1_BUF_LEN / 2),
    0,
    0};

struct StoveCtl {
    bool remoteCtrl;
    int lastTemp;
    int currentTemp;
    int startPumpTemp;
    uint8_t startStopTempDelta;
    bool gasBoilerOn;
    bool stovePumpOn;
} stoveCtl = {false, 0, 0, 40, 5, false, false};

StateMachine srSm = StateMachine();
State *S0 = srSm.addState(&serialReceive);
State *S1 = srSm.addState(&serialTransmit);

StateMachine sm = StateMachine();
State *M0 = sm.addState(&st_idle);
State *M1 = sm.addState(&st_pump_on);
State *M2 = sm.addState(&st_gas_stove_on);

void printInfo(bool printToTxBuffer);

void setup() {
    // turn on 
    pinMode(RADIO_POWER_PIN, OUTPUT);
    digitalWrite(RADIO_POWER_PIN, HIGH);

    S0->addTransition(&transitionS0S1, S1);
    S1->addTransition(&transitionS1S0, S0);
    // SERIAL to PC configuration
    Serial.begin(9600);
    while (!Serial) {
        // Wait for serial port to connect.
        // Needed for native USB port only.
        ;
    }
    Serial.println("SOBA-CAZAN Pornit...");


    // Serial to RPI Configuration
    pinMode(SER1_TDC, OUTPUT);
    digitalWrite(SER1_TDC, LOW);
    ser1.begin(9600);

    pinMode(TEMP_SENSOR, INPUT);
    pinMode(GAS_BOILER_PIN, OUTPUT);
    pinMode(STOVE_PUMP_PIN, OUTPUT);
    analogReference(DEFAULT); // 5V?
    analogReadResolution(12); // 12 bits
    stovePumpSet(false);
    gasBoilerSet(false);

    // read current value of STOVE TEMP
    uint8_t storedTemp = EEPROM.read(PUMP_START_TEMP_EEPROM_ADDR);
    if (storedTemp > 0 && storedTemp < 70) {
        stoveCtl.startPumpTemp = storedTemp;
    } else {
        EEPROM.write(PUMP_START_TEMP_EEPROM_ADDR, 27);
    }

    uint8_t tempDelta = EEPROM.read(START_STOP_TEMP_DELTA_EEPROM_ADDR);
    if (tempDelta > 0 && tempDelta < 20) {
        stoveCtl.startStopTempDelta = tempDelta;
    } else {
        EEPROM.write(START_STOP_TEMP_DELTA_EEPROM_ADDR, 3);
    }

    printf("Stored Start Pump temp = %d, delta = %d\n", storedTemp, tempDelta);
    sensors.begin();

    // Virtual wire setup
    vw_set_rx_pin(2);
    vw_setup(2000);  // Bits per sec
    vw_rx_start();  // Start the receiver PLL running
}

void loop() {
    if (timerExceed(t1, 5000)) {
        resetTimer(t1);
        readDS18B20();
        readThermistor();
        printf("State: %d\n", sm.currentState);
        printInfo(false);
    }
    if (timerExceed(t2, 10000)) {
        resetTimer(t2);
        if (stoveCtl.startPumpTemp > 0 && !stoveCtl.remoteCtrl) {
            if (stoveCtl.currentTemp > stoveCtl.startPumpTemp && !stoveCtl.stovePumpOn) {
                stovePumpSet(true);
                Serial.println("STOVE PUMP ON");
                sm.transitionTo(M1);
            } else if (stoveCtl.currentTemp < (stoveCtl.startPumpTemp - stoveCtl.startStopTempDelta) && stoveCtl.stovePumpOn) {
                stovePumpSet(false);
                Serial.println("STOVE PUMP OFF");
                sm.transitionTo(M0);
            }
        }
    }
    sm.run();
    srSm.run();

    uint8_t vwbuf[VW_MAX_MESSAGE_LEN];
    uint8_t vwbuflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(&vwbuf[0], &vwbuflen)) {  // Non-blocking
        for (uint8_t i = 0; i < vwbuflen; i++) {
            vwbuffer[i] = vwbuf[i];
        }
        vwbuffer[vwbuflen] = '\0';
        printf("Radio msg[size=%d]: %s\n", vwbuflen, vwbuffer);
        
    }
}

void serialReceive() {
    if (srSm.executeOnce) {
        digitalWrite(SER1_TDC, LOW);
        delay(100);
    }
    if (ser1.available()) {
        char ch = ser1.read();
        Serial.write(ch);
        if (ch == '\n' || ch == '\r') {
            serCtl.rmsg[serCtl.ridx] = '\0';
            serCtl.ridx = 0;
            if (stringStartsWith(serCtl.rmsg, DEVICE_SID)) {
                serCtl.rcmd = serCtl.rmsg + strlen(DEVICE_SID);
                printf("\nReceived: %s\n", serCtl.rcmd);
            }
            return;
        }
        serCtl.rmsg[serCtl.ridx++] = ch;
        serCtl.ridx &= 0b00011111;
    }
}

void serialTransmit() {
    digitalWrite(SER1_TDC, HIGH);
    delay(100);
    printf("Process cmd '%s'\n", serCtl.rcmd);
    if (stringStartsWith(serCtl.rcmd, M_GET_STOVE_TEMP)) {
        sprintf(serCtl.tmsg, M_REPLY_STOVE_TEMP, MASTER_SID, stoveCtl.currentTemp);
        printf("Reply stove temp = %d\n", stoveCtl.currentTemp);
    } else if (stringStartsWith(serCtl.rcmd, M_GET_STATUS)) {
        sprintf(serCtl.tmsg, M_REPLY_STATUS, MASTER_SID, "OK");
        printf("Reply status...\n");
    } else if (stringStartsWith(serCtl.rcmd, M_PUMP_ON)) {
        printf("Pump on\n");
        sprintf(serCtl.tmsg, "%spump_on:ok", MASTER_SID);
        stovePumpSet(true);
        remoteControlOn();
        sm.transitionTo(M1);
    } else if (stringStartsWith(serCtl.rcmd, M_PUMP_OFF)) {
        printf("Pump off\n");
        sprintf(serCtl.tmsg, "%spump_off:ok", MASTER_SID);
        stovePumpSet(false);
        remoteControlOn();
        sm.transitionTo(M0);
    } else if (stringStartsWith(serCtl.rcmd, M_GAS_BOILER_ON)) {
        printf("Gas Boiler on\n");
        sprintf(serCtl.tmsg, "%sgas_boiler_on:ok", MASTER_SID);
        gasBoilerSet(true);
        remoteControlOn();
        sm.transitionTo(M2);
    } else if (stringStartsWith(serCtl.rcmd, M_GAS_BOILER_OFF)) {
        printf("Gas Boiler off\n");
        sprintf(serCtl.tmsg, "%sgas_boiler_off:ok", MASTER_SID);
        gasBoilerSet(false);
        remoteControlOn();
        sm.transitionTo(M0);
    } else if (stringStartsWith(serCtl.rcmd, M_REMOTE_CTRL_OFF)) {
        printf("Remote control off\n");
        sprintf(serCtl.tmsg, "%sremote_ctrl:off", MASTER_SID);
        remoteControlOff();
        sm.transitionTo(M0);
    } else if (stringStartsWith(serCtl.rcmd, M_STOVE_PUMP_START_TEMP)) {
        char *p = serCtl.rcmd + strlen(M_STOVE_PUMP_START_TEMP);
        String s = String(p);
        int newTemp = s.toInt();
        printf("Set new startPumpTemp=%d\n", newTemp);
        stoveCtl.startPumpTemp = newTemp;
        EEPROM.write(PUMP_START_TEMP_EEPROM_ADDR, stoveCtl.startPumpTemp);
        sprintf(serCtl.tmsg, "%sset_start_temp=%d:ok", MASTER_SID, stoveCtl.startPumpTemp);
    } else if (stringStartsWith(serCtl.rcmd, M_START_STOP_TEMP_DELTA)) {
        char *p = serCtl.rcmd + strlen(M_START_STOP_TEMP_DELTA);
        String s = String(p);
        uint8_t newDelta = s.toInt();
        printf("Set new delta=%d\n", newDelta);
        stoveCtl.startStopTempDelta = newDelta;
        EEPROM.write(START_STOP_TEMP_DELTA_EEPROM_ADDR, stoveCtl.startStopTempDelta);
        sprintf(serCtl.tmsg, "%sset_delta_temp=%d:ok", MASTER_SID, stoveCtl.startStopTempDelta);
    } else if (stringStartsWith(serCtl.rcmd, M_GET_INFO)) {
        printInfo(true);
    } else {
        sprintf(serCtl.tmsg, M_REPLY_UNKNOWN_CMD, MASTER_SID);
        printf("Unknown cmd.\n");
    }
    uint8_t crc = checkSum(serCtl.tmsg);
    // printf("Message to send: %s, checksum=%X\n", serCtl.tmsg, crc);
    sprintf(serCtl.tmsg, "%s|0X%X|", serCtl.tmsg, crc);
    ser1.println(serCtl.tmsg);
    serCtl.rcmd = NULL;
}

bool transitionS0S1() {
    if (serCtl.rcmd != NULL) {
        return true;
    }
    return false;
}

bool transitionS1S0() {
    return !transitionS0S1();
}

void stovePumpSet(bool onOff) {
    if (onOff) {
        stoveCtl.stovePumpOn = true;
        stoveCtl.gasBoilerOn = false;
        digitalWrite(GAS_BOILER_PIN, LOW);
        digitalWrite(STOVE_PUMP_PIN, HIGH);
    } else {
        stoveCtl.stovePumpOn = false;
        digitalWrite(STOVE_PUMP_PIN, LOW);
    }
}

void gasBoilerSet(bool onOff) {
    if (onOff) {
        stoveCtl.stovePumpOn = false;
        stoveCtl.gasBoilerOn = true;
        digitalWrite(STOVE_PUMP_PIN, LOW);
        digitalWrite(GAS_BOILER_PIN, HIGH);
    } else {
        stoveCtl.gasBoilerOn = false;
        digitalWrite(GAS_BOILER_PIN, LOW);
    }
}

void readThermistor() {
    tv = analogRead(TEMP_SENSOR);  // Acquisition analog value of VRT
    Serial.print("ADC readings ");
    Serial.println(tv);
    
    vr = 4095.0 / tv - 1.0;      // Acquisition of resistance value of thermistor
    
    vr = R_REF / vr;  // Calculate the resistance value of the thermistor
    
    //Serial.print("Thermistor resistance ");
    Serial.println(vr);

    temperature = vr / NOMINAL_RESISTANCE;     // (R/Ro)
    temperature = log(temperature);                  // ln(R/Ro)
    temperature /= BETA;                   // 1/B * ln(R/Ro)
    temperature += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
    temperature = 1.0 / temperature;                 // Invert
    temperature -= 273.15;                         // convert absolute temp to C
    //Serial.print("Temperature ");
    Serial.println(temperature);
    // stoveCtl
    stoveCtl.lastTemp = stoveCtl.currentTemp;
    stoveCtl.currentTemp = temperature;
}

void printInfo(bool printToTxBuffer) {
    if (printToTxBuffer) {
        sprintf(serCtl.tmsg, "%src=%d,ct=%d,lt=%d,spt=%d,delta=%d,gb=%d,sp=%d,vw=%s", MASTER_SID, stoveCtl.remoteCtrl, stoveCtl.currentTemp, stoveCtl.lastTemp, stoveCtl.startPumpTemp, stoveCtl.startStopTempDelta, stoveCtl.gasBoilerOn, stoveCtl.stovePumpOn, vwbuffer);
    }
    printf("Status: rc=%d,ct=%d,lt=%d,spt=%d,delta=%d,gb=%d,sp=%d,vw=%s\n", stoveCtl.remoteCtrl, stoveCtl.currentTemp, stoveCtl.lastTemp, stoveCtl.startPumpTemp, stoveCtl.startStopTempDelta, stoveCtl.gasBoilerOn, stoveCtl.stovePumpOn, vwbuffer);
    vwbuffer[0] = '\0';
}

void readDS18B20() {
    sensors.requestTemperatures();
      // Get the temperature in Celsius
    float temperatureC = sensors.getTempCByIndex(0);

  // Check if the reading is valid
  if (temperatureC != DEVICE_DISCONNECTED_C) {
    printf("DS18B20=%f°C\r\n", temperatureC);
  } else {
    printf("Error: Could not read temperature data\n");
  }
    
    //stoveCtl.lastTemp = stoveCtl.currentTemp;
    //stoveCtl.currentTemp = sensors.getTempCByIndex(0);
}

uint8_t lct = 0;

void st_idle(void) {
    if (lct == 0 && timerExceed(t0, 3000)) {
        resetTimer(t0);
        digitalWrite(LED, HIGH);
        lct++;
    }
    if (lct == 1 && timerExceed(t0, 40)) {
        resetTimer(t0);
        digitalWrite(LED, LOW);
        lct = 0;
    }
}

void st_pump_on(void) {
    if (lct == 0 && timerExceed(t0, 1000)) {
        resetTimer(t0);
        digitalWrite(LED, HIGH);
        lct++;
    }
    if (lct > 0 && lct % 2 == 1 && timerExceed(t0, 50)) {
        resetTimer(t0);
        digitalWrite(LED, LOW);
        lct++;
        lct &= 0b00001111;
    }
    if (lct > 0 && lct % 2 == 0 && timerExceed(t0, 100)) {
        resetTimer(t0);
        digitalWrite(LED, HIGH);
        lct++;
    }
}

void st_gas_stove_on(void) {
    if (timerExceed(t0, 700)) {
        resetTimer(t0);
        digitalWrite(LED, !digitalRead(LED));
    }
}

uint8_t checkSum(char *msg) {
    uint8_t x = 0;
    for (uint8_t i = 0; i < strlen(msg); i++) {
        x = (x + msg[i]) & 0xff;
    }
    return 0xff - x;
}