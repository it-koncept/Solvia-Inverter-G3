// *****************************************************************
// *          ESPHome Custom Component Modbus sniffer for          *
// *              Delta Solvia Inverter 3.0 EU G3 TR               *
// *****************************************************************

#include "esphome.h"

class soliviag3 : public PollingComponent, public Sensor, public UARTDevice {
  public:
    soliviag3(UARTComponent *parent) : PollingComponent(600), UARTDevice(parent) {}
    Sensor *pv1_d_yield = new Sensor();
    Sensor *pv1_ac_power = new Sensor();
    Sensor *pv1_ac_v = new Sensor();
    Sensor *pv1_dc_v = new Sensor();
    Sensor *pv1_ac_a = new Sensor();
    Sensor *pv1_dc_a = new Sensor();
    Sensor *pv1_hs_1 = new Sensor();
    Sensor *pv1_hs_2 = new Sensor();
    Sensor *pv1_iso_plus = new Sensor();
    Sensor *pv1_iso_minus = new Sensor();
    Sensor *pv2_d_yield = new Sensor();
    Sensor *pv2_ac_power = new Sensor();
    Sensor *pv2_ac_v = new Sensor();
    Sensor *pv2_dc_v = new Sensor();
    Sensor *pv2_ac_a = new Sensor();
    Sensor *pv2_dc_a = new Sensor();
    Sensor *pv2_hs_1 = new Sensor();
    Sensor *pv2_hs_2 = new Sensor();
    Sensor *pv2_iso_plus = new Sensor();
    Sensor *pv2_iso_minus = new Sensor();
    Sensor *pv3_d_yield = new Sensor();
    Sensor *pv3_ac_power = new Sensor();
    Sensor *pv3_ac_v = new Sensor();
    Sensor *pv3_dc_v = new Sensor();
    Sensor *pv3_ac_a = new Sensor();
    Sensor *pv3_dc_a = new Sensor();
    Sensor *pv3_hs_1 = new Sensor();
    Sensor *pv3_hs_2 = new Sensor();
    Sensor *pv3_iso_plus = new Sensor();
    Sensor *pv3_iso_minus = new Sensor();
        
  void setup() override {

  }

  std::vector<int> bytes;

  //void loop() override {

  void update() {
    while(available() > 0) {
      bytes.push_back(read());      
      //make sure at least 8 header bytes are available for check
      if(bytes.size() < 8)       
      {
        continue;  
      }
      //ESP_LOGD("custom", "Checking for inverter package");
      // Check for Delta Solivia Gateway package response.
      if(bytes[0] != 0x02 || bytes[1] != 0x06 || bytes[3] != 0x96 || bytes[4] != 0x60 || bytes[5] != 0x01) {
        bytes.erase(bytes.begin()); //remove first byte from buffer
        //buffer will never get above 8 until the response is a match
        continue;
      }      
      
	    if (bytes.size() == 157) {

		if(bytes[2] == 0x01) {
        
        TwoByte dc_v_data;
        dc_v_data.Byte[0] = bytes[0x30 +6]; // Solar voltage lsb
        dc_v_data.Byte[1] = bytes[0x2F +6]; // Solar voltage msb
        TwoByte dc_a_data;
        dc_a_data.Byte[0] = bytes[0x32 +6]; // Solar current lsb
        dc_a_data.Byte[1] = bytes[0x31 +6]; // Solar current msb

        TwoByte ac_a_data;
        ac_a_data.Byte[0] = bytes[0x3A + 6]; // AC current lsb
        ac_a_data.Byte[1] = bytes[0x39 + 6]; // AC current msb
        TwoByte ac_v_data;
        ac_v_data.Byte[0] = bytes[0x3C + 6]; // AC voltage lsb
        ac_v_data.Byte[1] = bytes[0x3B + 6]; // AC voltage lsb
        TwoByte ac_power_data;
        ac_power_data.Byte[0] = bytes[0x3E +6]; // AC Power lsb
        ac_power_data.Byte[1] = bytes[0x3D +6]; // AC Power msb
        
        TwoByte iso_plus_data;
        iso_plus_data.Byte[0] = bytes[0x34 +6]; // Solar isolation resistance lsb
        iso_plus_data.Byte[1] = bytes[0x33 +6]; // Solar isolation resistance msb
        TwoByte iso_minus_data;
        iso_minus_data.Byte[0] = bytes[0x38 +6]; // Solar input MOV resistance lsb
        iso_minus_data.Byte[1] = bytes[0x37 +6]; // Solar input MOV resistance msb

        TwoByte hs_1_data;
        hs_1_data.Byte[0] = bytes[0x36 +6]; // Calculated temperature at ntc (DC side) lsb
        hs_1_data.Byte[1] = bytes[0x35 +6]; // Calculated temperature at ntc (DC side) msb
        TwoByte hs_2_data;
        hs_2_data.Byte[0] = bytes[0x42 +6]; // Calculated temperature at ntc (AC side) lsb
        hs_2_data.Byte[1] = bytes[0x41 +6]; // Calculated temperature at ntc (AC side) msb

        TwoByte d_yield_data;
        d_yield_data.Byte[0] = bytes[0x50 +6]; // Daily yield lsb - Scaling x10
        d_yield_data.Byte[1] = bytes[0x4F +6]; // Daily yield msb - Scaling x10

        char etx;
        etx = bytes[156]; // ETX byte PV1 (last byte)


        // Quick and dirty check for package integrity is done, in order to avoid irratic sensor value updates 
        // This effectively blocks out any erroneous sensor updates due to rx package corruption
        // Check if ETX = 3. If not (invalid package), ditch whole package, clear buffer and continue
        if (etx != 0x03) {
          ESP_LOGI("custom", "ETX check failure - NO sensor update! ETX: %i", etx);
          bytes.clear();
          continue;
        }
          
          pv1_d_yield->publish_state(d_yield_data.UInt16);
          pv1_dc_v->publish_state(dc_v_data.UInt16);
          pv1_dc_a->publish_state(dc_a_data.UInt16);
          pv1_ac_v->publish_state(ac_v_data.UInt16);
          pv1_ac_a->publish_state(ac_a_data.UInt16);
          pv1_ac_power->publish_state(ac_power_data.UInt16);
          pv1_hs_1->publish_state(hs_1_data.UInt16);
          pv1_hs_2->publish_state(hs_2_data.UInt16);
          pv1_iso_plus->publish_state(iso_plus_data.UInt16);
          pv1_iso_minus->publish_state(iso_minus_data.UInt16);

	        ESP_LOGI("custom", "ETX check OK: %i", etx);
                ESP_LOGI("custom", "Inverter %i", bytes[2]);
                ESP_LOGI("custom", "Daily yield: %i Wh", d_yield_data.UInt16);
	        ESP_LOGI("custom", "Current production: %i W", ac_power_data.UInt16);
        }
	if(bytes[2] == 0x02) {
        
        TwoByte dc_v_data;
        dc_v_data.Byte[0] = bytes[0x30 +6]; // Solar voltage lsb
        dc_v_data.Byte[1] = bytes[0x2F +6]; // Solar voltage msb
        TwoByte dc_a_data;
        dc_a_data.Byte[0] = bytes[0x32 +6]; // Solar current lsb
        dc_a_data.Byte[1] = bytes[0x31 +6]; // Solar current msb

        TwoByte ac_a_data;
        ac_a_data.Byte[0] = bytes[0x3A + 6]; // AC current lsb
        ac_a_data.Byte[1] = bytes[0x39 + 6]; // AC current msb
        TwoByte ac_v_data;
        ac_v_data.Byte[0] = bytes[0x3C + 6]; // AC voltage lsb
        ac_v_data.Byte[1] = bytes[0x3B + 6]; // AC voltage lsb
        TwoByte ac_power_data;
        ac_power_data.Byte[0] = bytes[0x3E +6]; // AC Power lsb
        ac_power_data.Byte[1] = bytes[0x3D +6]; // AC Power msb
        
        TwoByte iso_plus_data;
        iso_plus_data.Byte[0] = bytes[0x34 +6]; // Solar isolation resistance lsb
        iso_plus_data.Byte[1] = bytes[0x33 +6]; // Solar isolation resistance msb
        TwoByte iso_minus_data;
        iso_minus_data.Byte[0] = bytes[0x38 +6]; // Solar input MOV resistance lsb
        iso_minus_data.Byte[1] = bytes[0x37 +6]; // Solar input MOV resistance msb

        TwoByte hs_1_data;
        hs_1_data.Byte[0] = bytes[0x36 +6]; // Calculated temperature at ntc (DC side) lsb
        hs_1_data.Byte[1] = bytes[0x35 +6]; // Calculated temperature at ntc (DC side) msb
        TwoByte hs_2_data;
        hs_2_data.Byte[0] = bytes[0x42 +6]; // Calculated temperature at ntc (AC side) lsb
        hs_2_data.Byte[1] = bytes[0x41 +6]; // Calculated temperature at ntc (AC side) msb

        TwoByte d_yield_data;
        d_yield_data.Byte[0] = bytes[0x50 +6]; // Daily yield lsb - Scaling x10
        d_yield_data.Byte[1] = bytes[0x4F +6]; // Daily yield msb - Scaling x10

        char etx;
        etx = bytes[156]; // ETX byte PV1 (last byte)


        // Quick and dirty check for package integrity is done, in order to avoid irratic sensor value updates 
        // This effectively blocks out any erroneous sensor updates due to rx package corruption
        // Check if ETX = 3. If not (invalid package), ditch whole package, clear buffer and continue
        if (etx != 0x03) {
          ESP_LOGI("custom", "ETX check failure - NO sensor update! ETX: %i", etx);
          bytes.clear();
          continue;
        }
        
          pv2_d_yield->publish_state(d_yield_data.UInt16);
          pv2_dc_v->publish_state(dc_v_data.UInt16);
          pv2_dc_a->publish_state(dc_a_data.UInt16);
          pv2_ac_v->publish_state(ac_v_data.UInt16);
          pv2_ac_a->publish_state(ac_a_data.UInt16);
          pv2_ac_power->publish_state(ac_power_data.UInt16);
          pv2_hs_1->publish_state(hs_1_data.UInt16);
          pv2_hs_2->publish_state(hs_2_data.UInt16);
          pv2_iso_plus->publish_state(iso_plus_data.UInt16);
          pv2_iso_minus->publish_state(iso_minus_data.UInt16);

	        ESP_LOGI("custom", "ETX check OK: %i", etx);
                ESP_LOGI("custom", "Inverter %i", bytes[2]);
                ESP_LOGI("custom", "Daily yield: %i Wh", d_yield_data.UInt16);
	        ESP_LOGI("custom", "Current production: %i W", ac_power_data.UInt16);
         }
	if(bytes[2] == 0x03) {
        
        TwoByte dc_v_data;
        dc_v_data.Byte[0] = bytes[0x30 +6]; // Solar voltage lsb
        dc_v_data.Byte[1] = bytes[0x2F +6]; // Solar voltage msb
        TwoByte dc_a_data;
        dc_a_data.Byte[0] = bytes[0x32 +6]; // Solar current lsb
        dc_a_data.Byte[1] = bytes[0x31 +6]; // Solar current msb

        TwoByte ac_a_data;
        ac_a_data.Byte[0] = bytes[0x3A + 6]; // AC current lsb
        ac_a_data.Byte[1] = bytes[0x39 + 6]; // AC current msb
        TwoByte ac_v_data;
        ac_v_data.Byte[0] = bytes[0x3C + 6]; // AC voltage lsb
        ac_v_data.Byte[1] = bytes[0x3B + 6]; // AC voltage lsb
        TwoByte ac_power_data;
        ac_power_data.Byte[0] = bytes[0x3E +6]; // AC Power lsb
        ac_power_data.Byte[1] = bytes[0x3D +6]; // AC Power msb
        
        TwoByte iso_plus_data;
        iso_plus_data.Byte[0] = bytes[0x34 +6]; // Solar isolation resistance lsb
        iso_plus_data.Byte[1] = bytes[0x33 +6]; // Solar isolation resistance msb
        TwoByte iso_minus_data;
        iso_minus_data.Byte[0] = bytes[0x38 +6]; // Solar input MOV resistance lsb
        iso_minus_data.Byte[1] = bytes[0x37 +6]; // Solar input MOV resistance msb

        TwoByte hs_1_data;
        hs_1_data.Byte[0] = bytes[0x36 +6]; // Calculated temperature at ntc (DC side) lsb
        hs_1_data.Byte[1] = bytes[0x35 +6]; // Calculated temperature at ntc (DC side) msb
        TwoByte hs_2_data;
        hs_2_data.Byte[0] = bytes[0x42 +6]; // Calculated temperature at ntc (AC side) lsb
        hs_2_data.Byte[1] = bytes[0x41 +6]; // Calculated temperature at ntc (AC side) msb

        TwoByte d_yield_data;
        d_yield_data.Byte[0] = bytes[0x50 +6]; // Daily yield lsb - Scaling x10
        d_yield_data.Byte[1] = bytes[0x4F +6]; // Daily yield msb - Scaling x10

        char etx;
        etx = bytes[156]; // ETX byte PV1 (last byte)


        // Quick and dirty check for package integrity is done, in order to avoid irratic sensor value updates 
        // This effectively blocks out any erroneous sensor updates due to rx package corruption
        // Check if ETX = 3. If not (invalid package), ditch whole package, clear buffer and continue
        if (etx != 0x03) {
          ESP_LOGI("custom", "ETX check failure - NO sensor update! ETX: %i", etx);
          bytes.clear();
          continue;
        }
          
          pv3_d_yield->publish_state(d_yield_data.UInt16);
          pv3_dc_v->publish_state(dc_v_data.UInt16);
          pv3_dc_a->publish_state(dc_a_data.UInt16);
          pv3_ac_v->publish_state(ac_v_data.UInt16);
          pv3_ac_a->publish_state(ac_a_data.UInt16);
          pv3_ac_power->publish_state(ac_power_data.UInt16);
          pv3_hs_1->publish_state(hs_1_data.UInt16);
          pv3_hs_2->publish_state(hs_2_data.UInt16);
          pv3_iso_plus->publish_state(iso_plus_data.UInt16);
          pv3_iso_minus->publish_state(iso_minus_data.UInt16);

	        ESP_LOGI("custom", "ETX check OK: %i", etx);
                ESP_LOGI("custom", "Inverter %i", bytes[2]);
                ESP_LOGI("custom", "Daily yield: %i Wh", d_yield_data.UInt16);
	        ESP_LOGI("custom", "Current production: %i W", ac_power_data.UInt16);
         }
         bytes.clear();
      }
      else {
      }    
    }    
  }

  typedef union
  {
    unsigned char Byte[2];
    int16_t Int16;
    uint16_t UInt16;
    unsigned char UChar;
    char Char;
  }TwoByte;};
