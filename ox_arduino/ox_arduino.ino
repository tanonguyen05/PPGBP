#include "Wire.h"
#include "MAX30105.h"
MAX30105 Sensor_0;
MAX30105 Sensor_1;

int val;
int time_now;

#define TCA_Address (0x70) // TCA9548A Encoders address
#define MOVE_AVG_WINDOW (5)

// Initialize I2C buses using TCA9548A I2C Multiplexer
void tcaselect(uint8_t i2c_bus) {
    if (i2c_bus > 7) return;
    Wire.beginTransmission(TCA_Address);
    Wire.write(1 << i2c_bus);
    Wire.endTransmission(); 
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    //Setup to sense a nice looking saw tooth on the plotter
    byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
    byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    int sampleRate = 1000; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

    //---------------------------------------------------------------------------------------------------------------------------- BUS 0
    tcaselect(0);
    if (!Sensor_0.begin(Wire, I2C_SPEED_FAST, MAX30105_ADDRESS)) abort();
    Sensor_0.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings 
    
    //---------------------------------------------------------------------------------------------------------------------------- BUS 1
    tcaselect(1);
    if (!Sensor_1.begin(Wire, I2C_SPEED_FAST, MAX30105_ADDRESS)) abort();
    Sensor_1.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop()
{
    tcaselect(0);
    val = Sensor_0.getIR();
    Serial.println(val); //Send raw data to plotter

    tcaselect(1);
    val = Sensor_1.getIR();
    Serial.println(val); //Send raw data to plotter

    Serial.println("/");
    Serial.flush();
}
