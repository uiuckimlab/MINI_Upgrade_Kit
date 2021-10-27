/* ------------------------------------------------------------ *
 * file:        getbno055.c                                     *
 * purpose:     Sensor control and data extraction program for  *
 *              the Bosch BNO055 absolute orientation sensor    *
 *                                                              *
 * return:      0 on success, and -1 on errors.                 *
 *                                                              *
 * requires:	I2C headers, e.g. sudo apt install libi2c-dev   *
 *                                                              *
 * compile:	gcc -o getbno055 i2c_bno055.c getbno055.c       *
 *                                                              *
 * example:	./getbno055 -t eul  -o bno055.htm               *
 *                                                              *
 * author:      05/04/2018 Frank4DD                             *
 * ------------------------------------------------------------ */
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <time.h>
#include "getbno055.h"

/* ------------------------------------------------------------ *
 * Global variables and defaults                                *
 * ------------------------------------------------------------ */
int verbose = 0;
int outflag = 0;
int argflag = 0; // 1 dump, 2 reset, 3 load calib, 4 write calib
char opr_mode[9] = {0};
char pwr_mode[8] = {0};
char datatype[256];
char senaddr[256] = "0x28";
char i2c_bus[256] = I2CBUS;
char htmfile[256];
char calfile[256];

void delay(int number_of_seconds)
{
   // Converting time into milli_seconds
   int milli_seconds = 1000 * number_of_seconds;

   // Storing start time
   clock_t start_time = clock();

   // looping till required time is not achieved
   while (clock() < start_time + milli_seconds)
      ;
}

int main()
{
   get_i2cbus(i2c_bus, senaddr);
   set_mode(imu);

   while (1)
   {

      struct bnoacc bnodacc;
      get_acc(&bnodacc);
      printf("ACC %3.2f %3.2f %3.2f  ", bnodacc.adata_x, bnodacc.adata_y, bnodacc.adata_z);

      struct bnogyr bnodgyr;
      get_gyr(&bnodgyr);
      printf("GYR %3.3f %3.3f %3.3f  ", bnodgyr.gdata_x, bnodgyr.gdata_y, bnodgyr.gdata_z);

      struct bnoeul bnodeul;
      get_eul(&bnodeul);
      printf("EUL %3.2f %3.2f %3.2f\n", bnodeul.eul_head, bnodeul.eul_roll, bnodeul.eul_pitc);
      usleep(10000);
   }
}
