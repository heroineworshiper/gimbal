// Programmer for Feiyu over UART




#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/serial.h>
#include "arm_linux.h"


#define BAUD B115200
#define CAPTURE_FILE "terminal.cap"
#define PAGE_SIZE 1024

FILE *capture = 0;


// Returns the FD of the serial port
static int init_serial(char *path, int baud)
{
	struct termios term;

// Initialize serial port
	int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printf("init_serial %d: path=%s: %s\n", __LINE__, path, strerror(errno));
		return -1;
	}
	
	if (tcgetattr(fd, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}


/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, baud);
	cfsetospeed(&term, baud);
//	term.c_iflag = IGNBRK;
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
//	term.c_cflag &= ~(PARENB | PARODD | CRTSCTS | CSTOPB | CSIZE);
//	term.c_cflag |= CS8;
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	if(tcsetattr(fd, TCSANOW, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	return fd;
}

void print_char(unsigned char c)
{
	fputc(c, capture);
	fflush(capture);
	
	printf("%c", c);
	fflush(stdout);
}

// Read a character
unsigned char read_char(int fd)
{
	unsigned char c;
	int result;
	do
	{
		result = read(fd, &c, 1);
	} while(result <= 0);
	return c;
}

// Send a character
void write_char(int fd, unsigned char c)
{
	int result;
	do
	{
		result = write(fd, &c, 1);
	} while(!result);
}

void write_buffer(int fd, unsigned char *data, int size)
{
	int result = 0;
	int bytes_sent = 0;
	do
	{
		bytes_sent = write(fd, data + result, size - result);
		if(bytes_sent > 0) result += bytes_sent;
	}while(result < size);
}

void read_buffer(int fd, unsigned char *data, int size)
{
	int result = 0;
	int bytes_read = 0;
	do
	{
		bytes_read = read(fd, data + result, size - result);
		if(bytes_read > 0) result += bytes_read;
	} while(result < size);
}

// wait for prompt
void wait_uart(int serial_fd)
{
	while(1)
	{
		unsigned char c = read_char(serial_fd);


		print_char(c);
//printf("main %d: %c%c%c%c\n", __LINE__, code[0], code[1], code[2], code[3]);


		if(c == '>')
		{
			return;
		}
	}
}


int main(int argc, char *argv[])
{
	int write_program = 0;
	char *filename = "";
	int passthrough = 0;
	int i, j;
	int start_address = PROGRAM_START;
	int size = 0;
	unsigned char *data = 0;
	
	if(argc > 1)
	{
		for(i = 1 ; i < argc; i++)
		{
			if(!strcmp(argv[i], "-p"))
			{
				passthrough = atoi(argv[i + 1]);
				i++;
			}
			else
			{
				write_program = 1;
				filename = argv[i];
			}
		}
	}

//	printf("main %d: passthrough=%d\n", __LINE__, passthrough);


	int serial_fd = init_serial("/dev/ttyUSB0", BAUD);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB1", BAUD);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB2", BAUD);
	if(serial_fd < 0) return 1;


	capture = fopen(CAPTURE_FILE, "w");
	if(!capture) 
	{
		printf("Couldn't open capture file %s\n", CAPTURE_FILE);
	}




	if(passthrough > 0 || write_program)
	{
		printf("Passing through %d times\n", passthrough);

// do all passthrough stages
// 1 extra to enter bootloader mode
		for(i = 0; i < passthrough + write_program; i++)
		{
// wait for the bootloader prompt
			char code[4] = { 0, 0, 0, 0 };
			while(1)
			{
				unsigned char c = read_char(serial_fd);
				code[0] = code[1];
				code[1] = code[2];
				code[2] = code[3];
				code[3] = c;

				print_char(c);


				if(code[0] == 'B' &&
					code[1] == 'O' &&
					code[2] == 'O' &&
					code[3] == 'T')
				{
// Send code to activate bootloader if passing through or programming this board
					write_char(serial_fd, 'b');
					break;
				}
			}

// wait for response
			wait_uart(serial_fd);

// send code to pass through
			if(i < passthrough)
			{
				write_char(serial_fd, 'p');
			}
		}
	}

	if(write_program)
	{
		FILE *fd = fopen(filename, "r");
		if(!fd)
		{
			printf("main %d: couldn't open %s\n", __LINE__, filename);
			return 1;
		}

		fseek(fd, 0, SEEK_END);
		size = ftell(fd);
		fseek(fd, 0, SEEK_SET);

		data = malloc(size);
		fread(data, size, 1, fd);
		fclose(fd);

		printf("main %d: writing %s address 0x%x size %d\n", 
			__LINE__,
			filename,
			start_address,
			size);


		for(i = 0; i < size; i += PAGE_SIZE)
		{
// send command to write flash
			write_char(serial_fd, 'w');
			int address = start_address + i;
			write_buffer(serial_fd, (unsigned char*)&address, 4);
			int fragment = PAGE_SIZE;
			if(fragment + i > size)
			{
				fragment = size - i;
			}
			write_buffer(serial_fd, (unsigned char*)&fragment, 4);

// wait for prompt after debug message
			wait_uart(serial_fd);
			write_buffer(serial_fd, data + i, fragment);

// wait for prompt
			wait_uart(serial_fd);

// read back
			unsigned char read_data[PAGE_SIZE];
			write_char(serial_fd, 'r');
			write_buffer(serial_fd, (unsigned char*)&address, 4);
			write_buffer(serial_fd, (unsigned char*)&fragment, 4);
// wait for prompt after debug message
			wait_uart(serial_fd);

			for(j = 0; j < fragment; j++)
			{
				read_data[j] = read_char(serial_fd);
//				printf("main %d wanted=0x%02x got=0x%02x\n", __LINE__, data[i + j], read_data[j]);
				
				if(read_data[j] != data[i + j])
				{
					printf("files differ\n");
					exit(1);
				}
			}
//			read_buffer(serial_fd, read_data, fragment);

// wait for prompt
			wait_uart(serial_fd);
		}


// start program
		write_char(serial_fd, 'g');
	}

	
// go into terminal mode
	struct termios info;
	tcgetattr(fileno(stdin), &info);
	info.c_lflag &= ~ICANON;
	info.c_lflag &= ~ECHO;
	tcsetattr(fileno(stdin), TCSANOW, &info);

	unsigned char test_buffer[32];
	fd_set rfds;
	while(1)
	{
		FD_SET(serial_fd, &rfds);
		FD_SET(0, &rfds);
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;
		int result = select(serial_fd + 1, 
			&rfds, 
			0, 
			0, 
			&timeout);

		if(FD_ISSET(serial_fd, &rfds))
		{

			unsigned char c = read_char(serial_fd);
			print_char(c);
		}

// send input from console
		if(FD_ISSET(0, &rfds))
		{
			int i;
			int bytes = read(0, test_buffer, sizeof(test_buffer));
			
			for(i = 0; i < bytes; i++)
			{
				char c = test_buffer[i];
				if(c < 0xa)
					printf("0x%02x ", c);
				else
					printf("%c", c);

				fflush(stdout);

				if(c == 0xa)
				{
					write_char(serial_fd, 0xd);
// delay to avoid overflowing a 9600 passthrough
					usleep(10000);
					write_char(serial_fd, 0xa);
				}
				else
					write_char(serial_fd, c);


// delay to avoid overflowing a 9600 passthrough
				usleep(10000);
			}
		}
	}
}





























