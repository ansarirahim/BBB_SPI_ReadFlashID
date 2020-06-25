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
#define ENABLE_CS 0
#define DISABLE_CS 1
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
tByte FlashReadCmd[4];
tByte NVM16ReadCmd[3];
//tByte TempTxData128[128];
tByte Rxd256[256];
tByte Rxd[2048];
struct spi_ioc_transfer messageRead[2],message16Read[2],message16Read256[2];
struct spi_ioc_transfer messageWREN[1];// = {0, };
tByte WriteEnableCmd[1];
struct spi_ioc_transfer messageData[1];
struct spi_ioc_transfer messageWriteCmd[1] = {0, };
tByte WriteCmd[4];
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 10000000;                  //need to be speeded up when working
static uint16_t delay = 5;
//changed to AT25512 write cycle time
unsigned int Spi_address=0;
tByte TempTxData128[128];
tByte TempTxNvmData128[128];
tByte Write16Cmd[3];
tByte TempByteBuffer[256];
char TemCharBuffer[256];
struct spi_ioc_transfer messageWrite16CmdFF[2] = {0, };
struct spi_ioc_transfer messageWrite16CmdData[2] = {0, };
static void Init_16messageRead256()
{
	 	 	 	   message16Read256[0].tx_buf = (unsigned long) NVM16ReadCmd;
		    	   message16Read256[0].rx_buf = (unsigned long)NULL;
		    	   message16Read256[0].len = sizeof(NVM16ReadCmd);
		    	   message16Read256[0].cs_change = 1;//0 working for P9_17

		    	   message16Read256[1].tx_buf = (unsigned long)NULL;
		    	   message16Read256[1].rx_buf = (unsigned long)Rxd256;
		    	   message16Read256[1].len = sizeof(Rxd256);
		    	   message16Read256[1].cs_change = 1;

		    	   NVM16ReadCmd[0] =READ;
//		    	   (readcommand+1)= &(Spi_address>>16);//
//		    	  	    		  (readcommand+2) =&(Spi_address>>8);//
//		    	  	    		  (readcommand+3) =&(Spi_address);

}
///////////////////////////////
static void Init_messageRead()
{
	 	 	 	   messageRead[0].tx_buf = (unsigned long)FlashReadCmd;
		    	   messageRead[0].rx_buf = (unsigned long)NULL;
		    	   messageRead[0].len = sizeof(FlashReadCmd);
		    	   messageRead[0].cs_change = ENABLE_CS;//0 working for P9_17

		    	   messageRead[1].tx_buf = (unsigned long)NULL;
		    	   messageRead[1].rx_buf = (unsigned long)Rxd256;///Rxd
		    	   messageRead[1].len = sizeof(Rxd256);
		    	   messageRead[1].cs_change = ENABLE_CS;

		    	   FlashReadCmd[0] =READ;
//		    	   (readcommand+1)= &(Spi_address>>16);//
//		    	  	    		  (readcommand+2) =&(Spi_address>>8);//
//		    	  	    		  (readcommand+3) =&(Spi_address);

}

static void Init_16messageRead()
{
	 	 	 	   message16Read[0].tx_buf = (unsigned long) NVM16ReadCmd;
		    	   message16Read[0].rx_buf = (unsigned long)NULL;
		    	   message16Read[0].len = sizeof(NVM16ReadCmd);
		    	   message16Read[0].cs_change = 1;//0 working for P9_17

		    	   message16Read[1].tx_buf = (unsigned long)NULL;
		    	   message16Read[1].rx_buf = (unsigned long)Rxd;
		    	   message16Read[1].len = sizeof(Rxd);
		    	   message16Read[1].cs_change = ENABLE_CS;

		    	   NVM16ReadCmd[0] =READ;
//		    	   (readcommand+1)= &(Spi_address>>16);//
//		    	  	    		  (readcommand+2) =&(Spi_address>>8);//
//		    	  	    		  (readcommand+3) =&(Spi_address);

}
////////////////////////
static void Init_messageWREN(){

	        //setup spi transfer data structure

	messageWREN[0].tx_buf = (unsigned long)WriteEnableCmd;      //send the write enable command
	messageWREN[0].rx_buf = (unsigned long)NULL;
	messageWREN[0].len =1;// sizeof(WriteEnableCmd);
	messageWREN[0].cs_change = ENABLE_CS;


}

static void Init_WriteCommand() {
	//     //setup spi transfer data structure

	messageWriteCmd[0].tx_buf = (unsigned long)WriteCmd;   //send the write command and address
	messageWriteCmd[0].rx_buf = (unsigned long)NULL;
	messageWriteCmd[0].len = sizeof(WriteCmd);
	messageWriteCmd[0].cs_change = ENABLE_CS;
}


static void Init_WriteCommand16FF() {
	//     //setup spi transfer data structure
	  Write16Cmd[0]=WRITE;
	messageWrite16CmdFF[0].tx_buf = (unsigned long)Write16Cmd;   //send the write command and address
	messageWrite16CmdFF[0].rx_buf = (unsigned long)NULL;
	messageWrite16CmdFF[0].len = sizeof(Write16Cmd);
	messageWrite16CmdFF[0].cs_change = 1;

	messageWrite16CmdFF[1].tx_buf = (unsigned long)TempTxData128;   //send the write command and address
	messageWrite16CmdFF[1].rx_buf = (unsigned long)NULL;
	messageWrite16CmdFF[1].len = sizeof(TempTxData128);
	messageWrite16CmdFF[1].cs_change = 1;

}
static void Init_WriteCommand16Data() {
	//     //setup spi transfer data structure
	  Write16Cmd[0]=WRITE;
	messageWrite16CmdData[0].tx_buf = (unsigned long)Write16Cmd;   //send the write command and address
	messageWrite16CmdData[0].rx_buf = (unsigned long)NULL;
	messageWrite16CmdData[0].len = sizeof(Write16Cmd);
	messageWrite16CmdData[0].cs_change = 1;

	messageWrite16CmdData[1].tx_buf = (unsigned long)TempTxNvmData128;   //send the write command and address
	messageWrite16CmdData[1].rx_buf = (unsigned long)NULL;
	messageWrite16CmdData[1].len = sizeof(TempTxNvmData128);
	messageWrite16CmdData[1].cs_change = 1;

}
struct spi_ioc_transfer messageWrite16Cmd[1] = {0, };
static void Init_Write16Command() {
	//     //setup spi transfer data structure

	messageWrite16Cmd[0].tx_buf = (unsigned long)WriteCmd;   //send the write command and address
	messageWrite16Cmd[0].rx_buf = (unsigned long)NULL;
	messageWrite16Cmd[0].len = sizeof(Write16Cmd);
	messageWrite16Cmd[0].cs_change = 1;
}
char ReadStatusRegCmd[1] = {RDSR,};
uint8_t StatusRegValues[2];
struct spi_ioc_transfer messageStatusReg[2];// = {0, };
static void Init_ReadStatusReg()
		{

	messageStatusReg[0].tx_buf = (unsigned long) ReadStatusRegCmd;
	messageStatusReg[0].rx_buf = (unsigned long)NULL;
	messageStatusReg[0].len = sizeof(ReadStatusRegCmd);
	messageStatusReg[0].cs_change = 1;

	 messageStatusReg[1].tx_buf = (unsigned long)NULL;
	 messageStatusReg[1].rx_buf = (unsigned long)StatusRegValues;
	messageStatusReg[1].len = sizeof(StatusRegValues);
	messageStatusReg[1].cs_change = 1;


		}
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
void initLCD_cd()
{

				system(Export_GPIO60);

			system(DIRECTION_SET_GPIO60);


			system(Set_GPIO60);//l595
			usleep(10000);
}
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
		    		    message[0].cs_change = ENABLE_CS;//1;  //working commented to check other cs message[0].cs_change = 0;

		    		    message[1].tx_buf = (unsigned long)NULL;
		    		    message[1].rx_buf = (unsigned long)rx;
		    		    message[1].len = sizeof(rx);
		    		    message[1].cs_change =ENABLE_CS;// 1;
		    		    ///gpio_set_value_spi(Cs, LOW);
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
		    		  ///  gpio_set_value_spi(Cs, HIGH);
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
   //memset(TempTxData128,0x96,sizeof(TempTxData128));
   //initLCD_cd();
 Init_16messageRead256();
Init_16messageRead();
 Init_messageRead();
Init_WriteCommand();
Init_Write16Command();
Init_messageWREN();
Init_ReadStatusReg();
Init_WriteCommand16FF();
Init_WriteCommand16Data();
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
	 int x=  ReadFlashID_Cs(fd,P9_17);//P9_15);
	   close(fd);
return x;
}
