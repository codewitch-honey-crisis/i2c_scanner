#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO 18     
#define I2C_MASTER_SDA_IO 17
#define I2C Wire
#define I2C_SDA 17
#define I2C_SCL 18
#include "driver/i2c.h"
// ESP32 I2C Scanner
// Based on code of Nick Gammon  http://www.gammon.com.au/forum/?id=10896
// ESP32 DevKit - Arduino IDE 1.8.5
// Device tested PCF8574 - Use pullup resistors 3K3 ohms !
// PCF8574 Default Freq 100 KHz
#include <Arduino.h>
#include <Wire.h>
#include <thread.hpp>
#include <atomic>
// shut the compiler up
namespace arduino {}
using namespace arduino;
using namespace freertos;

static thread updater;
static SemaphoreHandle_t update_sync;
static volatile std::atomic_bool updater_ran;
struct i2c_data {
    uint32_t banks[4];
};
i2c_data i2c_addresses;
i2c_data i2c_addresses_old;
void update_task(void* state) {
    while (true) {
      #ifdef I2C_SDA
        I2C.begin(I2C_SDA, I2C_SCL);
      #else
        I2C.begin();
      #endif  
        //i2c_set_pin(0, I2C_SDA, I2C_SCL, true, true, I2C_MODE_MASTER);
        I2C.setTimeOut(uint16_t(-1));
        uint32_t banks[4];
        memset(banks,0,sizeof(banks));
        for (byte i = 0; i < 127; i++) {
            I2C.beginTransmission(i);        // Begin I2C transmission Address (i)
            if (I2C.endTransmission() == 0)  // Receive 0 = success (ACK response)
            {
                banks[i/32]|=(1<<(i%32));
            }
        }
        I2C.end();
        xSemaphoreTake(update_sync,portMAX_DELAY);
        memcpy(i2c_addresses.banks,banks,sizeof(banks));
        xSemaphoreGive(update_sync);
        updater_ran = true;
        delay(1000);
    }
}
void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
    pinMode(21,OUTPUT);
    digitalWrite(21,HIGH);
    memset(&i2c_addresses_old,0,sizeof(i2c_addresses_old));
    memset(&i2c_addresses,0,sizeof(i2c_addresses));
    updater_ran = false;
    update_sync = xSemaphoreCreateMutex();
    updater = thread::create_affinity(1-thread::current().affinity(),update_task,nullptr,10,2000);
    updater.start();
}
void loop() {
  uint32_t banks[4];
  if(updater_ran) {
    xSemaphoreTake(update_sync,portMAX_DELAY);
    memcpy(banks,i2c_addresses.banks,sizeof(banks));
    xSemaphoreGive(update_sync);
    if(updater_ran && memcmp(banks,i2c_addresses_old.banks,sizeof(banks))) {
      bool found = false;
      for(int i = 0; i< 128;++i) {
        int mask = 1<<(i%32);
        int bank = i/32;
        if(banks[bank]&mask) {
          found = true;
          Serial.printf("0x%02X (%d)\n",i,i);
        }
      }
      if(!found) {
        Serial.println("<none>");
      }
      memcpy(i2c_addresses_old.banks,banks,sizeof(banks));
    }
  }
  
}

