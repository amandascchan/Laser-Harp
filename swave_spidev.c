// file: swave_panda.c
// produces 500Hz square wave on Pandaboard GPIO pin 32 or on an attached spi D/A chip using /dev/spidev1.0
// you can run it without the D/A chip as long as you have /dev/spidev1.0
// 29NOV2012  wb666greene@gmail.com
 
/* compile using:  
	gcc -o swave_spidev swave_spidev.c -lrt -Wall  
*/

// uncommnet to compile on Beaglebone, otherwise builds for Pandaboard
#define BEAGLEBONE


/*
	The starting point was the Realtime "Hello World" program from:
	https://rt.wiki.kernel.org/index.php/RT_PREEMPT_HOWTO
	
	With the Pandaboard GPIO stuff from:
	http://www.ozbotz.org/a-simplereal-time-application-on-pandaboard/
	
	I then added my spidev code and some timing tests from my application to illustrate the issue.
*/



/*
??? Why is spidev so slow and apparently pre-emptable?  Real-time patches appear to make it worse.



Some 5 min sample run results:


panda@PandaES:~$ uname -a
Linux PandaES 3.4.0-rt17+ #2 SMP PREEMPT RT Thu Nov 15 13:51:22 CST 2012 armv7l armv7l armv7l GNU/Linux

I setup this system following the instructions from here:
http://hbrobotics.org/wiki/index.php?title=Installing_and_Configuring_Ubuntu_on_the_PandaBoard

And then enabled the spidev by editing .config and modifying board-omap4panda.c uisng the guidelines:
http://www.omappedia.com/wiki/PandaBoard_SPI



Using GPIO:

panda@PandaES:~$ ./swave_spidev
Using GPIO 32.
 Not running with realtime priority!
Interval Max(mS): 4.0 @N=187436    Min: 0.0 @N=187438   Total Samples: 300000
 gpio time  Max(uS): 2.5   	Min: 1.1   
Intervals +50% over N: 4  0.0%  -50% under N: 10  0.0%
drops, N intervals >= 2 mS: 3  0.0%

panda@PandaES:~$ sudo ./swave_spidev
Using GPIO 32.
 Using realtime priority.
Interval Max(mS): 1.0 @N=5178    Min: 1.0 @N=48343   Total Samples: 300000
 gpio time  Max(uS): 25.1  	Min: 5.5   
Intervals +50% over N: 0  0.0%  -50% under N: 0  0.0%
drops, N intervals >= 2 mS: 0  0.0%
( this is effectively perfect performance! )


Using SPIDEV:

panda@PandaES:~$ ./swave_spidev spi
Using spidev.
 Not running with realtime priority!
Interval Max(mS): 16.8 @N=124700    Min: 0.2 @N=124716   Total Samples: 300000
 spidev time  Max(uS): 16800.0	Min: 149.4 
Intervals +50% over N: 355  0.1%  -50% under N: 1788  0.6%
drops, N intervals >= 2 mS: 280  0.1%

panda@PandaES:~$ sudo ./swave_spidev spi
[sudo] password for panda: 
Using spidev.
 Using realtime priority.
Interval Max(mS): 24.6 @N=87437    Min: 0.2 @N=58088   Total Samples: 300000
 spidev time  Max(uS): 24620.2	Min: 165.8 
Intervals +50% over N: 1615  0.5%  -50% under N: 15330  5.1%
drops, N intervals >= 2 mS: 1547  0.5%


Note:  at 24Mhz spi clock it should only take 8 uS to shift out the 192 bits per every mS to feed the D/A chip.
Even at 100% overhead per 24-bit spi message, the observed times are excessive by two orders of magnitude!


Running cyclictest on this system:

panda@PandaES:~/rt-tests$ sudo ./cyclictest -p 90 -t1 -n -l 1000000
# /dev/cpu_dma_latency set to 0us
policy: fifo: loadavg: 0.29 0.28 0.31 1/126 15826          

T: 0 (15823) P:90 I:1000 C:1000000 Min:      9 Act:   11 Avg:   12 Max:      26
*/


/*
On Pandaboard ES without real time patches:

panda@pandaES:~$ uname -a
Linux pandaES 3.4.0-1485-omap4 #7+spidev SMP PREEMPT Wed Aug 22 18:06:27 CDT 2012 armv7l armv7l armv7l GNU/Linux


panda@pandaES:~$ sudo ./swave_spidev
[sudo] password for panda: 
Using GPIO 32.
 Using realtime priority.
Interval Max(mS): 1.2 @N=167181    Min: 0.8 @N=167182   Total Samples: 300000
 gpio time  Max(uS): 30.5  	Min: 0.0   
Intervals +50% over N: 0  0.0%  -50% under N: 0  0.0%
drops, N intervals >= 2 mS: 0  0.0%
( We could live with this if spidev had this performance! )


panda@pandaES:~$ sudo ./swave_spidev spi
Using spidev.
 Using realtime priority.
Interval Max(mS): 9.3 @N=254779    Min: 0.1 @N=74740   Total Samples: 300000
 spidev time  Max(uS): 9308.0	Min: 30.5  
Intervals +50% over N: 39  0.0%  -50% under N: 84  0.0%
drops, N intervals >= 2 mS: 21  0.0%
( the Min ~30 uS spidev time is what I expeceted as "normal" )


cyclictest on this system:

panda@pandaES:~/rt-tests$ sudo ./cyclictest -p 90 -t1 -n -l 1000000
# /dev/cpu_dma_latency set to 0us
policy: fifo: loadavg: 0.39 0.34 0.36 1/240 2611          

T: 0 ( 2606) P:90 I:1000 C:1000000 Min:      0 Act:   17 Avg:   15 Max:     192
*/

#include<time.h>
#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<sys/ioctl.h>
#include<linux/spi/spidev.h>
#include<getopt.h>
#include<unistd.h>
#include<linux/types.h>
#include<fcntl.h>
#include<string.h>
#include<sched.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>


#define MAX_SAFE_STACK (8*1024)
void stack_prefault(void) {

        unsigned char dummy[MAX_SAFE_STACK];

        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}


#define NSEC_PER_SEC    1000000000


static inline void tsnorm(struct timespec *ts) {
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}


// signal handler function, which just tries to provide a graceful exit for short test runs via ctrl-C exit
int QUIT=0;

void handler(int signal){
	QUIT=1;
}


// functions & variables to setup spidev ioctrl for our D/A device
// nice "feature" of spidev is no errors if no hardware attached as long as /dev/spidev1.0 exists
// need chmod 666 on /dev/spidev1.0 to run as non-root user!
#ifndef BEAGLEBONE
char *spi_device = "/dev/spidev1.0";		// Pandaboard		
#else
//char *spi_device = "/dev/spidev2.0";		// for Beaglebone, seems to change with kernel version :(
char *spi_device = "/dev/spidev1.0";		// 3.8.13-bone32 with BB-SPI0-01-00A0.dts
#endif

int dev1;						// handle to opened spidev
uint32_t speed = 24000000;		// 48000000 max SPI for Panda board, but 24000000 seems to be max shown in spidev dmesg output
// magic numbers for AD5362 dac register data writes
uint8_t dac0=200,dac1=201,dac2=202,dac3=203,dac4=208,dac5=209,dac6=210,dac7=211;
// 8 sets of 3 bytes of data for the dac registers which are big endian data
struct spi_ioc_transfer msg[8];	
uint8_t lead[24];

// function to setup for ioctl of userspace SPIDEV driver
void SPIDEV_transferInitialize(struct spi_ioc_transfer *msg){
	int i;	
	
	// setup sequence of 8 bulk transfers of 3 bytes each
	for(i=0;i<8;i++){
		msg[i].rx_buf = (unsigned long)NULL;
		msg[i].len = 3;
		msg[i].speed_hz = speed;
		msg[i].delay_usecs = 0;
		msg[i].bits_per_word = 8;
		msg[i].cs_change = 1;
		msg[i].pad = 0;
	}

	msg[0].tx_buf = (unsigned long)&lead[0];
	msg[1].tx_buf = (unsigned long)&lead[3];
	msg[2].tx_buf = (unsigned long)&lead[6];
	msg[3].tx_buf = (unsigned long)&lead[9];
	msg[4].tx_buf = (unsigned long)&lead[12];
	msg[5].tx_buf = (unsigned long)&lead[15];
	msg[6].tx_buf = (unsigned long)&lead[18];
	msg[7].tx_buf = (unsigned long)&lead[21];
	
	//initialize channel arrays first byte to be dac register select magic number
	lead[0] = dac0;
	lead[3] = dac1;
	lead[6] = dac2;
	lead[9] = dac3;
	lead[12] = dac4;
	lead[15] = dac5;
	lead[18] = dac6;
	lead[21] = dac7;
}

// function to open SPIDEV
int SPIDEV_open(char *spi_device){
	uint8_t mode = SPI_MODE_1;
	uint8_t lsb = 0;
	int dev1;
	int ret;

	dev1 = open(spi_device,O_RDWR);
	if(dev1 < 0) {
		printf("Couldn't open dev1\n");
		return -1;
	}

	ret = ioctl(dev1,SPI_IOC_WR_MODE,&mode);
	if(ret == -1) { 
		printf("mode not set\n"); 
		return -1;
	}
	ret = ioctl(dev1,SPI_IOC_WR_MAX_SPEED_HZ,&speed);
	if(ret == -1) { 
		printf("write speed not set\n"); 
		return -1;
	}
	ret = ioctl(dev1,SPI_IOC_WR_LSB_FIRST,&lsb);
	if(ret == -1) { 
		printf("bit order not set\n"); 
		return -1;
	}
	return dev1;
}


int main( int argc, char** argv )
{
    struct timespec t;
    struct sched_param param;
    int interval=1000000;  // 1000000ns = 1000us = 1 mS interval, ==> 500Hz square wave
	int idiff, maxidiff=0, minidiff=1000000000, passcount=0, mincount=-1, maxcount=-1;
	int spidiff, spimax=0, spimin=1000000000;
	int over=0, over2=0, under=0;
	struct timespec now, prev, enter, leave;
	int do_spidev=0;
	int16_t sample;
	uint8_t *byteptr = NULL;
    int fd;
    char buffer[32];
    unsigned char value = 0;
    int i,j, ret;
    char *ONEstring = "1";
    char *ZEROstring = "0";

	if(argc>=2){
		do_spidev=1;	// flag loop to use SPIDEV instead of GPIO bit
		if(argc==3) spi_device=argv[2];
		printf("Using spidev: %s\n", spi_device);
	}
	byteptr=(uint8_t *)&sample;
	
	//Setup signal handler for graceful exit
	{
		struct sigaction act;
		act.sa_handler=handler;		// try to catch all signals and set QUIT flag for a graceful exit
		sigemptyset(&act.sa_mask);
		act.sa_flags=0;
		sigaction(SIGINT, &act, 0);
		sigaction(SIGHUP, &act, 0);
		sigaction(SIGQUIT, &act, 0);
		sigaction(SIGTERM, &act, 0);
		sigaction(SIGTSTP, &act, 0);
		sigaction(SIGSEGV, &act, 0);
	}
	
    // Enable GPIO_32, which comes out pin 18 of J6 of the Pandaboard. A list of available GPIOs
    // can be found in the PandaBoard System Reference Manual.
    {	// this may need changes for the Beaglebone, but I believe this comes out pin 25 of J8 on the Beaglebone
#ifdef BEAGLEBONE
		// must change pin mux mode on Beaglebone to make pin become GPIO_32, otherwise its ad0 input
        if ((fd = open("/sys/kernel/debug/omap_mux/gpmc_ad0", O_WRONLY | O_NDELAY, 0)) == 0) {
            printf("Error: Can't open /sys/kernel/debug/omap_mux/gpmc_ad0.\n");
            exit(1);
        }   
        strcpy( buffer, "7" );
        write( fd, buffer, strlen(buffer) );
        close(fd);
#endif
        if ((fd = open("/sys/class/gpio/export", O_WRONLY | O_NDELAY, 0)) == 0) {
            printf("Error: Can't open /sys/class/gpio/export.\n");
            exit(1);
        }   
        strcpy( buffer, "32" );
        write( fd, buffer, strlen(buffer) );
        close(fd);
        if(!do_spidev) printf("Using GPIO 32.\n");
    }

    // Set the direction of the GPIO to "out."
    {	// this may need changes for the Beaglebone
        if ((fd = open("/sys/class/gpio/gpio32/direction", O_WRONLY | O_NDELAY, 0)) == 0) {
            printf("Error: Can't open /sys/class/gpio/gpio32/direction.\n");
            exit(1);
        }
        strcpy( buffer, "out" );
        write( fd, buffer, strlen(buffer) );
        close(fd);
        //printf("Direction set to out.\n");
    }

    // Open the "value" node. We will write to it later.
    {	// this may need changes for the Beaglebone
        if ((fd = open("/sys/class/gpio/gpio32/value", O_WRONLY | O_NDELAY, 0)) == 0) {
            printf("Error: Can't open /sys/class/gpio/gpio32/value.\n");
            exit(1);
        }   
        //printf("Value opened for writing.\n");
    }
    
	// setup SPIDEV transfer
	{
    	dev1 = SPIDEV_open(spi_device);
		if(dev1 < 0) 
			return 1;
		SPIDEV_transferInitialize(msg);    
    }
    
	// if super user -- switch to real-time mode and lock memory pages
	{
		if (geteuid() == 0) {	   
        	memset(&param, 0, sizeof(param));
			param.sched_priority = sched_get_priority_max(SCHED_FIFO);  
        	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                fprintf(stderr, "sched_setscheduler failed! Exiting.\n\n");
                exit(-1);
       		}
        	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
				fprintf(stderr, "mlockall() failed!  Exiting.\n\n");
           		exit(-2);
        	}
        	stack_prefault();
        	fprintf(stderr, " Using realtime priority.\n") ;
		}else {
			fprintf(stderr, " Not running with realtime priority!\n") ;
    	}
	}
	fprintf(stderr, "Square Wave will be generated for 300 seconds (5 minutes) for decent timing statistics.\n");
    clock_gettime(0,&t); // Get current time.
    t.tv_nsec+=interval;
    tsnorm(&t);
	prev.tv_sec=t.tv_sec;
	prev.tv_nsec=t.tv_nsec;
    while (!QUIT && passcount<300000) {		// 300 * 1000 mS = 5 minute run for decent stats
        // wait untill next shot.
        clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);
        clock_gettime(0,&now);
        
        // **** do work ****
        if(!do_spidev){		// this may need changes for the Beaglebone
        	clock_gettime(0,&enter);	// get some timing stats on gpio device time
       		if(value)		// toggle GPIO bit
            	write( fd, ONEstring, 1 );
        	else
           		write( fd, ZEROstring, 1 );
       		value = !value;
        }else{
        	if(value)	// toggle data value between +/- 1/2 full scale to generate square wave 
        		sample=16384;
        	else
        		sample=-16384;
        	value = !value;	
			for(i=0, j=1; i<8; i++, j+=3){
				sample^=0x8000;				// convert to offset binary
				lead[j]=byteptr[1];			// high (MSB) byte into low address of lead array
				lead[j+1]=byteptr[0];		// low (LSB) byte into high addres of lead array
			}
			clock_gettime(0,&enter);		// get some timing stats on spidev time
			// using fwrite(dev1, lead, 24); doesn't fix the latency issue but prevents D/A chip from working because CS stays low for all 192 bits
			ret = ioctl(dev1, SPI_IOC_MESSAGE(8), msg); // 8 denotes how many messages are pointed to by msg array
			if(ret < 1){
				fprintf(stderr, "spidev ioctl error\n");
				break;
			}
        }
		clock_gettime(0,&leave);		// end time of gpio or spidev timer
        // **** end work ****
        
		// do some stats on how accurate our timing is
		{
			spidiff=leave.tv_nsec-enter.tv_nsec;
			if(spidiff < 0)
				spidiff+=1000000000;
			if(spidiff>spimax) spimax = spidiff;
			if(spidiff<spimin) spimin = spidiff;
			idiff=now.tv_nsec-prev.tv_nsec;
			if(idiff < 0)
				idiff+=1000000000;
			if(idiff>maxidiff){ maxidiff = idiff; maxcount=passcount; }
			if(idiff<minidiff && passcount>0){ minidiff = idiff; mincount=passcount; }
			if(idiff > 1500000)  over++;	// actual sample time more than 50% late (1mS=1000000nS)
			if(idiff >= 2000000)  over2++;	// "dropped" samples, presumabley these are followed by very short intervals.
			if(idiff < 500000 && passcount>0) under++;		// actual sample time more than 50% early
			prev.tv_nsec=now.tv_nsec;
			prev.tv_sec=now.tv_sec;
			passcount++;
		}
        // Calculate next shot.
        t.tv_nsec+=interval;
        tsnorm(&t);
	}
   	// print results
	printf("Interval Max(mS): %3.1f @N=%i    Min: %3.1f @N=%i   Total Samples: %d\n",
			maxidiff/1000000.0, maxcount, minidiff/1000000.0, mincount, passcount);
	printf(" %s time  Max(uS): %-6.1f\tMin: %-6.1f\n", do_spidev ? "spidev" : "gpio", spimax/1000.0,spimin/1000.0);
	printf("Intervals +50%% over N: %d  %3.1f%%  -50%% under N: %d  %3.1f%%\n", over, 100.0*over/passcount, under, 100.0*under/passcount);
	printf( "drops, N intervals >= 2 mS: %d  %3.1f%%\n",over2, 100.0*over2/passcount);
	close(dev1);
	close(fd);
   return 0;
}
