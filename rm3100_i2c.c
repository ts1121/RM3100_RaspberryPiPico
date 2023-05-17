#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

const uint LED_PIN = 25;

//RM3100 Register Map
#define RM3100_ADDR 0x20
#define RM3100_POLL 0x00
#define RM3100_CMM 0x01

//Cycle Count register for each axis (MSB,LSB)
#define RM3100_CCX1_W 0x04 //write address
#define RM3100_CCX0_W 0x05 //write address
#define RM3100_CCY1_W 0x06 //write address
#define RM3100_CCY0_W 0x07 //write address
#define RM3100_CCZ1_W 0x08 //write address
#define RM3100_CCZ0_W 0x09 //write address

//Register For Each Sensor (MSB,MID,LSB) 
#define RM3100_MX_MSB 0x24
#define RM3100_MX_MID 0x25
#define RM3100_MX_LSB 0x26
#define RM3100_MY_MSB 0x27
#define RM3100_MY_MID 0x28
#define RM3100_MY_LSB 0x29
#define RM3100_MZ_MSB 0x2A
#define RM3100_MZ_MID 0x2B
#define RM3100_MZ_LSB 0x2C

//Other Registers
#define RM3100_TMRC 0x0B
#define RM3100_STATUS 0x34
#define RM3100_REVID 0x36

//Cycle count value for each axis
#define CCX_MSB 0x00
#define CCX_LSB 0xC8
#define CCY_MSB 0x00
#define CCY_LSB 0xC8
#define CCZ_MSB 0x00
#define CCZ_LSB 0xC8

//DRDY Pin
#define DRDY_PIN 6

/*
set_cycle_count: sets the cycle count (cc) registers for x/y/z.
@param cc_addr_msb_lsb_arr: pointer to an array consisting of {start_register_addr, msb/lsb_data_of_x/y/z_depending_on_start_register}.
@param arr_size: size of the array pointed by cc_addr_msb_lsb_arr (equal to the number of Bytes sent by Pico (includes register_addr Byte)).
E.g. uint8_t cc_addr_msb_lsb_arr[] = {RM3100_CCX1_W,CCX_MSB,CCX_LSB,CCY_MSB};
     set_cycle_count(cc_addr_msb_lsb_arr,4) -> will set RM3100_CCX1_W register with value CCX_MSB, RM3100_CCX2_W register with value CCX_LSB,
                                            & RM3100_CCY1_W register with value CCY_MSB.
                                            Reg addr automatically increments from start addr, depending on the number of Bytes values that will be set by Pico.
*/
void set_cycle_count(uint8_t *cc_addr_msb_lsb_arr, int arr_size) {
    i2c_write_blocking(i2c0, RM3100_ADDR, cc_addr_msb_lsb_arr, arr_size, false);
}


/*
get_cycle_count: reads the values from cycle count (cc) registers for x/y/z.
@param cc_addr: addr of register from where Pico will start reading.
@param cc_val: pointer to an array that will store msb/lsb values read from the x/y/z msb/lsb registers.
@param arr_size: size of the array pointed by cc_val (equal to the number of Bytes read by Pico (doesn't includes register_addr Byte)).
E.g. uint8_t cc_val[] = {0x00,0x00,0x00};
     read_cycle_count(RM3100_CCX1_W,cc_val,3) -> will read RM3100_CCX1_W register and store its value in val[0], 
                                            read RM3100_CCX0_W register and store its value in val[1],
                                            & read RM3100_CCY1_W register and store its value in val[2].
                                            Reg addr automatically increments from start addr, depending on the number of Bytes values that will be read by Pico.
*/
void get_cycle_count(uint8_t cc_addr, uint8_t *get_cc_val, int arr_size) {
    uint8_t msb_reg_addr[] = {cc_addr};

    i2c_write_blocking(i2c0, RM3100_ADDR, msb_reg_addr, 1, true); // Keep bus active with repeated start
    i2c_read_blocking(i2c0, RM3100_ADDR, get_cc_val, arr_size, false);
}

/*
set_cmm sets CMM (Continuous Measurement Mode) register.
@param set_cmm_val: 1 Byte (8 Bit) value to be set in the CMM register. Each Bit (from the total 8 Bits) corresponds to an operation, 
                    e.g. 'Start' Bit, 'ALARM' Bit etc (See RM3100 Testboard Datasheet).
*/
void set_cmm(uint8_t set_cmm_val) {
    uint8_t data[] = {RM3100_CMM, set_cmm_val};
    i2c_write_blocking(i2c0, RM3100_ADDR, data, 2, false);
}

/*
get_cmm reads CMM (Continuous Measurement Mode) register value.
@param read_cmm_val: pointer to an array that stores 1 Byte (8 Bit) value from the CMM register. 
                    Each Bit (from the total 8 Bits) corresponds to an operation, 
                    e.g. 'Start' Bit, 'ALARM' Bit etc (See RM3100 Testboard Datasheet).
*/
void get_cmm(uint8_t *get_cmm_val) {
    uint8_t cmm_reg_addr[] = {RM3100_CMM};
    i2c_write_blocking(i2c0, RM3100_ADDR, cmm_reg_addr, 1, true);
    i2c_read_blocking(i2c0, RM3100_ADDR, get_cmm_val, 1, false);
}

/*
get_mx_my_mz reads (MSB,Mid_Bit,LSB) -> (8 Bit, 8 Bit, 8Bit) readings from each of the sensor registers for each axes (X_MSB_Reg,X_Mid_Reg,X_LSB_Reg,Y_MSB_Reg...etc).
@param mxyz_val: pointer to an array that stores values from each sensor registers (X_MSB_Reg,X_Mid_Reg,X_LSB_Reg,Y_MSB_Reg...etc). 
                    Size should be 9 for the 9 registers (3 axes(x,y,z) multiplied by 3 segment registers(msb,mid,lsb) = 9).
*/
void get_mx_my_mz(uint8_t *mxyz_val) {
    uint8_t mx_reg_addr[] = {RM3100_MX_MSB};
    i2c_write_blocking(i2c0, RM3100_ADDR, mx_reg_addr, 1, true);
    i2c_read_blocking(i2c0, RM3100_ADDR, mxyz_val, 9, false);
}


int main() {
   
   bi_decl(bi_program_description("This is a test binary."));
   bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
   
   //initialize stdio
   stdio_init_all();

   //Set up LED Pin
   gpio_init(LED_PIN);
   gpio_set_dir(LED_PIN, GPIO_OUT);

   //Initialize i2c channel 0, GPIO Pins as SDA & SCL
   i2c_init(i2c0, 400000);
   gpio_set_function(4, GPIO_FUNC_I2C);
   gpio_set_function(5, GPIO_FUNC_I2C);

   //Set pull up resistors for SDA & SCL Pins
   gpio_pull_up(4);
   gpio_pull_up(5);

   //Initialize GPIO Pin as DRDY Pin
   gpio_init(DRDY_PIN);
   gpio_set_dir(DRDY_PIN, GPIO_IN); //Set DRDY Pin as input, high-> read data, low-> don't read data

   //set cc registers
   uint8_t cc_addr_msb_lsb_arr[] = {RM3100_CCX1_W, CCX_MSB, CCX_LSB, CCY_MSB, CCY_LSB, CCZ_MSB, CCZ_LSB};
   set_cycle_count(cc_addr_msb_lsb_arr,7);

   //get cc registers
   uint8_t cc_addr = RM3100_CCX1_W;
   uint8_t get_cc_val[] = {0x00,0x00,0x00,0x00,0x00,0x00};
   get_cycle_count(cc_addr,get_cc_val,6);

   //Calculate gain using cycle count
   uint16_t cycleCount = get_cc_val[0];
   cycleCount = (cycleCount << 8) | get_cc_val[1];
   float gain = (0.3671 * (float)cycleCount) + 1.5; //linear equation to calculate the gain from cycle count

   //set cmm registers
   uint8_t set_cmm_val = 0xF9; //11111001 = 0xF9
   set_cmm(set_cmm_val);

   //get cmm registers
   uint8_t get_cmm_val[] = {0x00};
   get_cmm(get_cmm_val);

   while(1) {
      //wait for DRDY pin to become high (data ready to read)  
      while(!gpio_get(DRDY_PIN));

      //To store x,y,& z 32 bit counts data
      long x = 0;
      long y = 0;
      long z = 0;

      //To store x,y, & z Micro-Tesla data
      float x_T = 0;
      float y_T = 0;
      float z_T = 0;

      //LED Blink
      gpio_put(LED_PIN,0);
      sleep_ms(250);
      gpio_put(LED_PIN, 1);

      //get mx,my,mz raw hex/binary sensor reading
      uint8_t mxyz_val[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      get_mx_my_mz(mxyz_val); 
      puts("Hello World\n");
      printf("X Axis Cycle Count MSB: 0x%02x\n", get_cc_val[0]);
      printf("X Axis Cycle Count LSB: 0x%02x\n", get_cc_val[1]);
      printf("Y Axis Cycle Count MSB: 0x%02x\n", get_cc_val[2]);
      printf("Y Axis Cycle Count LSB: 0x%02x\n", get_cc_val[3]);
      printf("Z Axis Cycle Count MSB: 0x%02x\n", get_cc_val[4]);
      printf("Z Axis Cycle Count LSB: 0x%02x\n", get_cc_val[5]);
      printf("CMM: 0x%02x\n", get_cmm_val[0]);
      printf("MX MSB: 0x%02x\n", mxyz_val[0]);
      printf("MX MID: 0x%02x\n", mxyz_val[1]);
      printf("MX LSB: 0x%02x\n", mxyz_val[2]);
      printf("MY MSB: 0x%02x\n", mxyz_val[3]);
      printf("MY MID: 0x%02x\n", mxyz_val[4]);
      printf("MY LSB: 0x%02x\n", mxyz_val[5]);
      printf("MZ MSB: 0x%02x\n", mxyz_val[6]);
      printf("MZ MID: 0x%02x\n", mxyz_val[7]);
      printf("MZ LSB: 0x%02x\n", mxyz_val[8]);

      //Converting 2s complement 24 bit (8bit MSB, 8bit MidB, 8bit LSB) to 32 bit signed integer (represents data in counts)
      if(mxyz_val[0] & 0x80) {
        x = 0xFF;
      }

      if(mxyz_val[3] & 0x80) {
        y = 0xFF;
      }

      if(mxyz_val[6] & 0x80) {
        z = 0xFF;
      }

      x = (x * 256 * 256 * 256) | (int32_t)(mxyz_val[0]) * 256 * 256 | (uint16_t)(mxyz_val[1]) * 256 | mxyz_val[2];
      y = (y * 256 * 256 * 256) | (int32_t)(mxyz_val[3]) * 256 * 256 | (uint16_t)(mxyz_val[4]) * 256 | mxyz_val[5];
      z = (z * 256 * 256 * 256) | (int32_t)(mxyz_val[6]) * 256 * 256 | (uint16_t)(mxyz_val[7]) * 256 | mxyz_val[8];

      //Print 32-bit data in counts
      printf("Data in counts:\n");
      printf("X: %ld\n",x);
      printf("Y: %ld\n",y);
      printf("Z: %ld\n",z);

      //Convert 32-bit count data into Micro-Tesla data
      x_T = (float)(x)/gain;
      y_T = (float)(y)/gain;
      z_T = (float)(z)/gain;

      //Calculate horizontal component
      float dbH_dt = sqrtf((x_T * x_T) + (y_T * y_T));

      //Print Data in Micro-Tesla
      printf("Data in Micro-Tesla:\n");
      printf("X: %f\n",x_T);
      printf("Y: %f\n",y_T);
      printf("Z: %f\n",z_T);
      printf("dBH/dt: %f\n",dbH_dt);

      sleep_ms(500);

   }

    return 0;
}

/*
void set_cycle_count() {
    uint8_t msb_reg_addr = 0x04;
    uint8_t lsb_reg_addr = 0x05;

    uint8_t msb_data[] = {msb_reg_addr, 0x00};
    uint8_t lsb_data[] = {lsb_reg_addr, 0xC8};

    i2c_write_blocking(i2c0, RM3100_ADDR, msb_data, 2, false);
    i2c_write_blocking(i2c0, RM3100_ADDR, lsb_data, 2, false);
}


void set_cycle_count_2(uint8_t msb, uint8_t lsb) {
    uint8_t data[] = {RM3100_CCX1_W, msb, lsb};

    i2c_write_blocking(i2c0, RM3100_ADDR, data, 3, false);
}
*/

/*
void read_cycle_count(uint8_t *msb, uint8_t *lsb) {
    uint8_t msb_reg_addr[] = {0x04};
    uint8_t lsb_reg_addr[] = {0x05};

    i2c_write_blocking(i2c0, RM3100_ADDR, msb_reg_addr, 1, true); // Keep bus active with repeated start
    i2c_read_blocking(i2c0, RM3100_ADDR, msb, 1, false);

    i2c_write_blocking(i2c0, RM3100_ADDR, lsb_reg_addr, 1, true);
    i2c_read_blocking(i2c0, RM3100_ADDR, lsb, 1, false);
}


void read_cycle_count_2(uint8_t *val) {
    uint8_t msb_reg_addr[] = {RM3100_CCX1_W};

    i2c_write_blocking(i2c0, RM3100_ADDR, msb_reg_addr, 1, true); // Keep bus active with repeated start
    i2c_read_blocking(i2c0, RM3100_ADDR, val, 2, false);
}
*/