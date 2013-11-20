#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h> 
#include <termios.h>    //Enables setting baud rate for RX/TX
#include <signal.h> 	// Defines signal-handling functions (i.e. trap Ctrl-C)
#include <pthread.h>

#define BAUDRATE B9600
#define ARRAY_SIZE 16

int initUART1_TXD();
int initUART2_RXD();
int setSerial();
int readCharSet();
int sendCharSet(unsigned char *msg, int size);
void signal_handler(int sig);

void rGetFirmware();
void rBootFirmware();
void rGetTagProtocol();
void rGetProgram();
void rSetTagProtocol();
void rReadSingle();
void rReadMultiple();
void rSetAntenna();
void rSetRegion();
void rGetAntenna();
void rGetRegion();
void rClearTagBuffer();

int keepgoing = 1;	// Set to 0 when ctrl-c is pressed

pthread_t tid[1];


/****************************************************************
* Thread for Receiving 
****************************************************************/
void* receiveThread(void *arg)
{
    unsigned long i = 0;
    pthread_t id = pthread_self();

    if(pthread_equal(id,tid[0]))
    {
        printf("\n First thread processing\n");
    }

	while(1) 
	{
    	printf("\n Trying to read something\n");
		readCharSet();
			//break;
	}

    return NULL;
}

/****************************************************************
* Main
****************************************************************/
int main(){	
	int err;

	// Set the signal callback for Ctrl-C
	signal(SIGINT, signal_handler);

	initUART1_TXD();
	initUART2_RXD();
	setSerial();
 	err = pthread_create(&(tid[0]), NULL, &receiveThread, NULL);
    if (err != 0)
        printf("\ncan't create thread :[%s]", strerror(err));
    else
        printf("\n Thread created successfully\n");

	//rGetFirmware();
	rBootFirmware();
	rGetProgram();
	sleep(1);
	rGetRegion();
	rSetRegion();
	sleep(1);
	rGetRegion();
	rGetAntenna();
	rSetAntenna();
	sleep(1);
	rGetAntenna();
	rGetTagProtocol();
	rSetTagProtocol();
	sleep(1);
	rGetTagProtocol();
	while(keepgoing){
		rClearTagBuffer();
		sleep(1);
		rGetProgram();
		//sleep(1);
		//rBootFirmware();
        //printf("\n Sending start\n");
		//rGetFirmware();
		//rReadSingle();
		rReadMultiple();
		//rGetProgram();
		usleep(50000);
	}
	return 0;
}

/****************************************************************
* Initiate UART1_TXD
****************************************************************/
int initUART1_TXD(){
	int fd;
	char *uart1_tx = "/sys/kernel/debug/omap_mux/uart1_txd";
	fd = open(uart1_tx, O_WRONLY);
	if (fd < 0) 
	{
		perror("Can't open uart1_txd\n");
		return fd;
	}
	
	write(fd, "00", 2);
	close(fd);
	return 0;
}

/****************************************************************
* Initiate UART2_RXD
****************************************************************/
int initUART2_RXD(){
	int fd;
	char *uart2_rx = "/sys/kernel/debug/omap_mux/spi0_sclk";
	fd = open(uart2_rx, O_WRONLY);
	if (fd < 0) 
	{
		perror("Can't open uart2_rxd\n");
		return fd;
	}
	
	write(fd, "21", 2); //Enable receiver, set to mode 1 (uart2_rxd)
	close(fd);
	return 0;
}

/****************************************************************
* Set serial attributes
****************************************************************/
int setSerial() {
	struct termios Serial;
	int fd;
	// Set the sending port ttyO1
	if ((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0){
		printf("Could not open ttyO1.\n");
		return -1;
	}
	if (tcgetattr(fd, &Serial) != 0){ // Obtain current terminal device settings
		printf("Unable to retrieve port attributes.\n");
		return -1;
	}
	if (cfsetospeed(&Serial, BAUDRATE) < 0){
		printf("Baud rate not successfully set.\n");
		return -1;
	}
	Serial.c_iflag = 0;	//Clear all flags for input modes
	Serial.c_oflag = 0;	//Clear all flags for output modes
	Serial.c_lflag = 0;	//Clear all flags for local modes
	Serial.c_cflag &= ~PARENB;	//Disable parity check
	Serial.c_cc[VMIN] = 0;
	Serial.c_cc[VTIME] = 2;	// Timeout for 0.1 * 2 seconds
	tcsetattr(fd, TCSANOW, &Serial); // Set the modified attributes
	close(fd);

	// Set the receiving port ttyO2
	if ((fd = open("/dev/ttyO2", O_RDWR | O_NOCTTY)) < 0){
		printf("Could not open ttyO1.\n");
		return -1;
	}
	if (tcgetattr(fd, &Serial) != 0){ // Obtain current terminal device settings
		printf("Unable to retrieve port attributes.\n");
		return -1;
	}
	if (cfsetispeed(&Serial, BAUDRATE) < 0){
		printf("Baud rate not successfully set.\n");
		return -1;
	}
	Serial.c_iflag = 0;	//Clear all flags for input modes
	Serial.c_oflag = 0;	//Clear all flags for output modes
	Serial.c_lflag = 0;	//Clear all flags for local modes
	Serial.c_cflag |= CREAD | CS8;	//Enable receiver; 8-bit, no parity, 1 stop bit
	Serial.c_cflag &= ~PARENB;	//Disable parity check
	Serial.c_cc[VMIN] = 1;
	Serial.c_cc[VTIME] = 2;	// Timeout for 0.1 * 2 seconds
	tcsetattr(fd, TCSANOW, &Serial); // Set the modified attributes
	close(fd);

	return 0;
}

/****************************************************************
* Read a set of characters
****************************************************************/
int readCharSet() {
	char byte_in[ARRAY_SIZE];
	struct termios Serial;
	int fd, len, i;
	for(i = 0; i < ARRAY_SIZE; i++) {
		byte_in[i] = 0;
	}
	if ((fd = open("/dev/ttyO2", O_RDWR | O_NOCTTY)) < 0){
		printf("Could not open ttyO2.\n");
		return -1;
	}	
//	if (tcgetattr(fd, &Serial) != 0){ // Obtain current terminal device settings
//		printf("Unable to retrieve port attributes.\n");
//		return -1;
//	}
//	Serial.c_cflag &= ~PARENB;	//Disable parity check
//	tcsetattr(fd, TCSANOW, &Serial); // Set the modified attributes
	len = read(fd, byte_in, ARRAY_SIZE); //Read ttyO2 port, stores data into byte_in

//	for(i = 0; i< ARRAY_SIZE; ) {
//		len = read(fd, byte_in + i, ARRAY_SIZE - i);
//		if (0 < len) 
//			i += len;
//		else if (len == 0)
//			break;
//	}

	for (i = 0; i < ARRAY_SIZE; i++) {
		printf("%X ", byte_in[i]);
	}
	printf("\n%d\n", len);
	close(fd);

	if(len > 0) {
		return 1;
	}
	return 0;
}

/****************************************************************
* Send a set of 5 characters without parity check
****************************************************************/
int sendCharSet(unsigned char *msg, int size) {
	struct termios Serial;
	//int size = strlen(&msg);	//Note: This may cause error when msg==0!
	int fd;
	if ((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0){
		printf("Could not open ttyO1.\n");
		return -1;
	}	
	if (tcgetattr(fd, &Serial) != 0){ // Obtain current terminal device settings
		printf("Unable to retrieve port attributes.\n");
		return -1;
	}
	Serial.c_cflag &= ~PARENB;	//Disable parity check
	tcsetattr(fd, TCSANOW, &Serial); // Set the modified attributes
	write(fd, msg, size);
	close(fd);

	return 0;
}
/****************************************************************
* Signal_handler
****************************************************************/
// Callback called when SIGINT is sent to the process (Ctrl-C)
void signal_handler(int sig) {
	printf( "Ctrl-C pressed, cleaning up and exiting..\n" );
	keepgoing = 0;
}

/****************************************************************
* Get Firmware Version (03h)
*****************************************************************/
void rGetFirmware() {
	unsigned char msg[]={0xFF, 0x00, 0x03, 0x1D, 0x0C};
	sendCharSet(msg, sizeof(msg));
	printf("Get firmware\n");
}

/****************************************************************
* Boot Firmware (04h)
*****************************************************************/
void rBootFirmware() {
	unsigned char msg[]={0xFF, 0x00, 0x04, 0x1D, 0x0B};
	sendCharSet(msg, sizeof(msg));
	printf("Boot firmware\n");
	sleep(2);
}

/****************************************************************
* Get Current Program (0Ch)
*****************************************************************/
void rGetProgram() {
	unsigned char msg[]={0xFF, 0x00, 0x0C, 0x1D, 0x03};
	printf("Get program\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Get Antenna (61h)
*****************************************************************/
void rGetAntenna() {
	unsigned char msg[]={0xFF, 0x01, 0x61, 0x00, 0xBD, 0XBD};
	printf("Get antenna\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Get Tag Protocol (63h)
*****************************************************************/
void rGetTagProtocol() {
	unsigned char msg[]={0xFF, 0x00, 0x63, 0x1D, 0x6C};
	printf("Get tag protocol\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Get Region (67h)
*****************************************************************/
void rGetRegion() {
	unsigned char msg[]={0xFF, 0x00, 0x67, 0x1D, 0x68};
	printf("Get region\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Set Tag Protocol (93h)
*****************************************************************/
void rSetTagProtocol() {
	unsigned char msg[]={0xFF, 0x02, 0x93, 0x00, 0x05, 0x51, 0x7D};
	printf("Set tag protocol\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Set Region (97h)
*****************************************************************/
void rSetRegion() {
	unsigned char msg[]={0xFF, 0x01, 0x97, 0x01, 0x4B, 0xBC};
	printf("Set region\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Set Antenna (91h)
*****************************************************************/
void rSetAntenna() {
	unsigned char msg[]={0xFF, 0x02, 0x91, 0x01, 0x01, 0x70, 0x3B};
	printf("Set antenna\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Read Single (21h)
*****************************************************************/
void rReadSingle() {
	unsigned char msg[]={0xFF, 0x02, 0x21, 0x03, 0xE8, 0xD5, 0x09};
	printf("Read single tag\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Read Multiple (22h)
*****************************************************************/
void rReadMultiple() {
	unsigned char msg[]={0xFF, 0x02, 0x22, 0x03, 0xE8, 0xE5, 0x6A};
	printf("Read multiple tags\n");
	sendCharSet(msg, sizeof(msg));
}

/****************************************************************
* Clear Tag Buffer (2Ah)
*****************************************************************/
void rClearTagBuffer() {
	unsigned char msg[]={0xFF, 0x00, 0x2A, 0x1D, 0x25};
	printf("Clear tag buffer\n");
	sendCharSet(msg, sizeof(msg));
}
