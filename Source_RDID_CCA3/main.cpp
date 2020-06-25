#include "SimpleGPIO_SPI.h"
#include "SPI_SS_Def.H"

#include <string.h>
#include <unistd.h>
#include <stdlib.h>
//#include <spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

//definitions for AT25512 device
#define WRITE_CYCLE_TIME (5000)                     //AT25512 write cycle time in us
#define WRSR (0x01)                              //AT25512 write status register
#define WRITE (0x02)                           //AT25512 write data to memory array
#define READ (0x03)                              //AT25512 read data from memory array
#define WRDI (0x04)                              //AT25512 reset write enable latch
#define RDSR (0x05)                              //AT25512 read status register
#define WREN (0x06)                              //AT25512 set write enable latch
static void pabort(const char *s)
{
   perror(s);
   abort();
}
typedef unsigned char tByte;
char ReadStatusRegCmd[1] = {RDSR,};
int speed=100000, mode=0,bits=8;

const char *DIRECTION_SET_GPIO48="echo high > /sys/class/gpio/gpio48/direction";
const char *DIRECTION_SET_GPIO50="echo high > /sys/class/gpio/gpio50/direction";
const char *DIRECTION_SET_GPIO51=	"echo high > /sys/class/gpio/gpio51/direction";
const char *DIRECTION_SET_GPIO60=	"echo high > /sys/class/gpio/gpio60/direction";
const char *DIRECTION_SET_GPIO115=	"echo high > /sys/class/gpio/gpio115/direction";
const char *DIRECTION_SET_GPIO49="echo high > /sys/class/gpio/gpio49/direction";//qhc


const char *Export_GPIO48="echo 48 > /sys/class/gpio/export";
const char *Export_GPIO50=" echo 50 > /sys/class/gpio/export";
const char *Export_GPIO51="echo 51 > /sys/class/gpio/export";
const char *Export_GPIO60="echo 60 > /sys/class/gpio/export";


const char *DIRECTION_CLEAR_GPIO48="echo low > /sys/class/gpio/gpio48/direction";
const char *DIRECTION_CLEAR_GPIO50="echo low > /sys/class/gpio/gpio50/direction";
const char *DIRECTION_CLEAR_GPIO51=	"echo low > /sys/class/gpio/gpio51/direction";
const char *DIRECTION_CLEAR_GPIO60=	"echo low > /sys/class/gpio/gpio60/direction";
const char *DIRECTION_CLEAR_GPIO115=	"echo low > /sys/class/gpio/gpio115/direction";
const char *DIRECTION_CLEAR_GPIO49="echo low > /sys/class/gpio/gpio49/direction";//qhc
////////////////////////////////////////////////////

const char *Clear_GPIO48="echo 0 > /sys/class/gpio/gpio48/value";
const char *Clear_GPIO50="echo 0 > /sys/class/gpio/gpio50/value";
const char *Clear_GPIO51="echo 0 > /sys/class/gpio/gpio51/value";
const char *Clear_GPIO60="echo 0 > /sys/class/gpio/gpio60/value";

const char *Set_GPIO48="echo 1 > /sys/class/gpio/gpio48/value";
const char *Set_GPIO50="echo 1 > /sys/class/gpio/gpio50/value";
const char *Set_GPIO51="echo 1 > /sys/class/gpio/gpio51/value";
const char *Set_GPIO60="echo 1 > /sys/class/gpio/gpio60/value";
	int ReadFlashID_Cs(int fd,tByte Cs)
	{
	int memorystatus=0;

		//////////////////////
		 int ret;
		   int address;
		   char addresslow, addressmid,addresshigh;
 		   char readcommand[1] = {0x9f,};// addresshigh, addressmid,addresslow ,};

		    		    uint8_t rx[4] = {0, };                        //create an array of data to be received

		    		    struct spi_ioc_transfer message[2] = {0, };         //setup spi transfer data structure

		    		    message[0].tx_buf = (unsigned long)readcommand;
		    		    message[0].rx_buf = (unsigned long)NULL;
		    		    message[0].len = sizeof(readcommand);
		    		    message[0].cs_change = 1;  //working commented to check other cs message[0].cs_change = 0;

		    		    message[1].tx_buf = (unsigned long)NULL;
		    		    message[1].rx_buf = (unsigned long)rx;
		    		    message[1].len = sizeof(rx);
		    		    message[1].cs_change = 1;
		    		    gpio_set_value_spi(Cs, LOW);
		    		    ret = ioctl(fd, SPI_IOC_MESSAGE(2), &message);      //spi check if sent
		    		    if (ret < 1)
		    		       pabort("can't send spi message");

		    		    for (unsigned int j = 0; j < sizeof(rx); j++) {         //prints returned array on the screen
		    		       if (!(j % 32))
		    		          puts("");
		    		       printf("%.2X ", rx[j]);
		    		    }
		    		    puts("");

		    		    puts("");
		    		    gpio_set_value_spi(Cs, HIGH);
		    		    switch(rx[0])
		    		    {
		    		    printf("\nManufactrer:");
		    		    case 0x20 :
		    		    	 printf("Macron");
		    		    	///\ TargetMemory.mf=macron_target;
		    		    	break;
                                     case 0x62 :
					printf("ON Semi...");
		    		    }
		    		    switch(rx[1])
		    		   	    		    {
		    		    printf("\nMemory Type:");
		    		   	    		    case 0x80 :
                                                            case 0x6:
		    		   	    		    			memorystatus=1;
		    		   	    		    ///\TargetMemory.mstatus=flash_target;


		    		   	    		     printf("Flash");
								
		    		   	    		    	break;
								//case 0x13:
								//memorystatus=1;
									
		    		   	    		    }
		    		    switch(rx[2])
		    		   	    		    {
		    		    printf("\nCapacity:");
		    		   	    		    case 0x14 :
		    		   	    		     printf("1-MByte\n");
		    		   	    		     memorystatus=0x14;
		    		   	    		///\  TargetMemory.PageSize=256;
		    		   	    		 ///\ TargetMemory.NoOfPages=1024*(1024/TargetMemory.PageSize);
		    		   	    		 ///\ TargetMemory.sectorsize=0x10000;

		    		   	    		    	break;
								case 0x13 :
									printf(" 512 KByte\n\r");
								memorystatus=0x13;
		    		   	    		    }
	if(rx[0]==0xff && rx[1]==0xff && rx[2]==0xff && rx[3]==0xff)
		{printf("\nTarget is not Connected");
		memorystatus=-2;
		}
	if(rx[0]==0x0 && rx[1]==0x0 && rx[2]==0x0 && rx[3]==0x0)
		{
		printf("\nTarget is NVM or unknown");
		memorystatus=2;
		}
	return memorystatus;
	}
int main()///(int argc, char *argv[])



{
   int ret = 0;
   int fd;
//   memset(TempTxData128,0x96,sizeof(TempTxData128));
//////////////////
 fd = open("/dev/spidev1.0", O_RDWR);            //open the spi device
		    		   if (fd < 0)
		    		      pabort("can't open device");

		    		   ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);         //set the spi mode
		    		   if (ret == -1)
		    		      pabort("can't set spi mode");

		    		   ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);         //get the spi mode (test)
		    		   if (ret == -1)
		    		      pabort("can't get spi mode");

		    		   ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);   //set the spi bits per word
		    		   if (ret == -1)
		    		      pabort("can't set bits per word");

		    		   ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);   //get the spi bits per word (test)
		    		   if (ret == -1)
		    		      pabort("can't get bits per word");

		    		   ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);   //set the spi max speed
		    		   if (ret == -1)
		    		      pabort("can't set max speed hz");

		    		   ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);   //get the spi max speed (test)
		    		   if (ret == -1)
		    		      pabort("can't get max speed hz");

//		    		   puts("");
//		    		   printf("The spi mode is set to: %d\n", mode);      //output successful settings to the terminal
//		    		   printf("Bits per word: %d\n", bits);
//		    		   speed=10000;
//		    		   printf("Max speed is set to: %d Hz (%d KHz) (%d MHz)\n", speed, speed/1000, speed/1000000);
//
	 int x=  ReadFlashID_Cs(fd,P9_41);
	   close(fd);
return x;
}
