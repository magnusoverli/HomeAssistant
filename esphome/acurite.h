#include "esphome.h"

class acu {
public:
   acu(uint16_t id, const char *str) {
      char name[40];

      sprintf(name, "%s_temp", str);
      this->temp.set_name(name);
      this->temp.set_unit_of_measurement("\302\260C");
      this->temp.set_icon("mdi:thermometer");
      this->temp.set_accuracy_decimals(1);
      App.register_sensor(&this->temp);

      sprintf(name, "%s_humidity", str);
      this->humidity.set_name(name);
      this->humidity.set_unit_of_measurement("%");
      this->humidity.set_icon("mdi:water-percent");
      this->humidity.set_accuracy_decimals(0);
      App.register_sensor(&this->humidity);

      sprintf(name, "%s_battery", str);
      this->battery.set_name(name);
      this->battery.set_icon("mdi:battery-alert");
      this->battery.set_device_class("battery");
      App.register_binary_sensor(&this->battery);
   }

   void publish(float temp, uint8_t humidity, bool battery) {
      this->temp.publish_state(temp);
      this->humidity.publish_state(humidity);
      this->battery.publish_state(battery);
   }

protected:
   TextSensor ch;
   Sensor temp;
   Sensor humidity;
   BinarySensor battery;
};

static uint8_t msg1[8];
static volatile bool have_msg1;
static uint8_t msg2[6];
static volatile bool have_msg2;

static void type1(uint32_t val, uint32_t duration)
{
   static unsigned sync = 0;
   static uint32_t t1 = 0;
   static unsigned count = 0;
   static unsigned len = 0;
   static unsigned bits = 0;
   static uint8_t cksum = 0;
   static uint8_t parity = 0;
   uint32_t t0 = duration;

   if (val) {
       t1 = duration;
       return;
   }

   if (t1 > 550 && t1 < 650 &&
       t0 > 550 && t0 < 650) {
       sync = ++count >= 4;
       return;
   }

   count = 0;

   if (!sync || have_msg1) {
       return;
   }

   if (t1 < 550) {
       unsigned b = t1 > 300;
       msg1[len] = (msg1[len] << 1) | b;
       if (len >= 3 && len <= 5) parity ^= b << len;
       if ((++bits % 8) == 0) {
           if (len < 6) cksum += msg1[len];
           ++len;
       }
       if (len < 7 && t1 + t0 < 700) {
           return;
       }
   }

   if (len == 7 && cksum == msg1[6]) {
      have_msg1 = true;
      msg1[7] = cksum;
   }

   len = 0;
   sync = 0;
   cksum = 0;
   parity = 0;
   bits = 0;
}

static void type2(uint32_t val, uint32_t duration)
{
    static unsigned sync = 0;
    static uint32_t t1 = 0;
    static unsigned count = 0;
    static unsigned len = 0;
    static unsigned bits = 0;
    uint32_t t0 = duration;

    if (val) {
        t1 = duration;
        return;
    }

    if (t1 > 1300 && t1 < 1700 &&
        t0 > 1300 && t0 < 1700) {
        sync = ++count >= 4;
        return;
    }

    count = 0;

    if (!sync || have_msg2) {
        return;
    }

    if (t1 < 250) {
        unsigned b = t0 > 800;
        msg2[len] = (msg2[len] >> 1) | (b << 7);
        ++bits;
        if (bits == 39) {
            msg2[len] >>= 1;
            ++bits;
        }
        if ((bits % 8) == 0) {
            ++len;
        }
        if (len < 5 && t1 + t0 < 1200) {
            return;
        }
    }

    if (len == 5) {
        // MISSING CHECKSUM!
        have_msg2 = true;
    }

    len = 0;
    sync = 0;
    bits = 0;
}

static uint32_t us;
static void ICACHE_RAM_ATTR rf_isr() {
   static uint32_t last;
   uint32_t val = digitalRead(27) != 0;
   uint32_t t = ESP.getCycleCount();
   uint32_t duration = (t - last) / us;

   type1(!val, duration);
   type2(!val, duration);
   last = t;
}

class Acurite : public Component, public CustomAPIDevice {
   protected:
      std::map<uint16_t, class acu*> sensors;
      bool verbose;

      void add_sensor(uint16_t id, const char *name) {
         sensors[id] = new acu(id, name);
      }

   public:
      void setup() override {
         add_sensor(0x3de3, "kitchen");
         add_sensor(0x1ba4, "cats");
         add_sensor(0x38ac, "garden");
         add_sensor(0x1ca0, "guest");
         add_sensor(0x26e3, "bedroom");
         add_sensor(0x0548, "oven");
         add_sensor(0x3fcb, "meuble");
         add_sensor(0x3102, "fridge");
         add_sensor(0x1845, "freezer");

         register_service(&Acurite::on_reboot, "reboot");
         register_service(&Acurite::on_verbose, "verbose", {"verbose"});

         us = ESP.getCpuFreqMHz();
         attachInterrupt(33, rf_isr, CHANGE);
      }

      void loop() override {
         if (have_msg1) {
            uint16_t id = (msg1[0] << 8 | msg1[1]) & 0x3fff;
            char ch = "C?BA"[msg1[0] >> 6];
            bool battery = (msg1[2] & 0x40) != 0;
            unsigned dtemp = ((msg1[4] & 0x1f) << 7 | (msg1[5] & 0x7f)) - 1000;
            uint8_t humidity = msg1[3] & 0x7f;

            if (verbose) {
               ESP_LOGD("type1", "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x cksum 0x%02x",
                     msg1[0], msg1[1], msg1[2], msg1[3], msg1[4], msg1[5], msg1[6], msg1[7]);
            }
            try {
               sensors.at(id)->publish(dtemp / 10.0, humidity, battery);
            } catch (std::out_of_range) {
               ESP_LOGW("acurite", "unknown %04x: %.1f C, %u %%",
                        id, dtemp / 10.0, humidity);
            }
            have_msg1 = false;
         }
         if (have_msg2) {
            if (verbose) {
               ESP_LOGD("type2", "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                     msg2[0], msg2[1], msg2[2], msg2[3], msg2[4]);
            }
            int8_t temp_f = msg2[0];
            uint16_t id = (msg2[2] << 8 | msg2[1]) & 0x3fff;
            char ch = "C?BA"[msg2[2] >> 6];
            bool battery = (msg2[2] & 0x2) == 0;
            int dtemp = ((int)temp_f - 32) * 50 / 9;
            try {
               sensors.at(id)->publish(dtemp / 10.0, 0, battery);
            } catch (std::out_of_range) {
               ESP_LOGW("acurite", "unknown %04x: %.1f C",
                        id, dtemp / 10.0);
            }
            have_msg2 = false;
         }
      }

      void on_verbose(int v) {
         verbose = v;
      }
      void on_reboot() {
         App.safe_reboot();
      }
};