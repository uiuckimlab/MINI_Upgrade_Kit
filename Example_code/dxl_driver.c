#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/select.h>
#include <termios.h>
#include <stropts.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <time.h>
#include "dxl_driver.h"
#include "dynamixel_sdk.h"

// XL320 Parameters
#define ADDR_320_TORQUE 24
#define ADDR_320_LED 25
#define ADDR_320_D 27
#define ADDR_320_I 28
#define ADDR_320_P 29
#define ADDR_320_POS_GOAL 30
#define ADDR_320_POS_PRESENT 37

// 320 Data Length
#define LEN_320_TORQUE 1
#define LEN_320_D 1
#define LEN_320_I 1
#define LEN_320_P 1
#define LEN_320_POS_GOAL 2
#define LEN_320_POS_PRESENT 2

// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel

uint8_t dxl_addparam_result = False; // AddParam result

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#elif defined(_WIN32) || defined(_WIN64)
	return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}

char getKey()
{
	if (kbhit())
	{
		return getch();
	}
	return '\0';
}

void write1(int port_num, int DXL_ID[], int DXL_num, int ADDR, int Command)
{
	for (int i = 0; i < DXL_num; i++)
	{
		if (DXL_ID[i] != 0)
		{
			write1ByteTxRx(port_num, 2, DXL_ID[i], ADDR, Command);
		}
	}
}

void write2(int port_num, int DXL_ID[], int DXL_num, int ADDR, int CMD)
{
	for (int i = 0; i < DXL_num; i++)
	{
		if (DXL_ID[i] != 0)
		{
			write2ByteTxRx(port_num, 2, DXL_ID[i], ADDR, CMD);
		}
	}
}

void write4(int port_num, int DXL_ID[], int DXL_num, int ADDR, int CMD)
{
	for (int i = 0; i < DXL_num; i++)
	{
		if (DXL_ID[i] != 0)
		{
			write4ByteTxRx(port_num, 2, DXL_ID[i], ADDR, CMD);
		}
	}
}

void ind_addr(int port_num, int DXL_ID[], int DXL_num, int ADDR_IND, int ADDR_DIR, int LEN_DATA)
{

	for (int j = 0; j < LEN_DATA; j++)
	{
		for (int i = 0; i < DXL_num; i++)
		{
			if (DXL_ID[i] != 0)
			{
				write2ByteTxRx(port_num, 2, DXL_ID[i], ADDR_IND + 2 * j, ADDR_DIR + j);
			}
		}
	}
}

void addparam_read(int groupread_num, int DXL_ID[], int DXL_num)
{
	for (int i = 0; i < DXL_num; i++)
	{
		dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID[i]);
		if (dxl_addparam_result != True)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID[i]);
		}
	}
}

void rmparam_read(int groupread_num, int DXL_ID[], int DXL_num)
{
	for (int i = 0; i < DXL_num; i++)
	{
		if (DXL_ID[i] != 0)
		{
			groupSyncReadRemoveParam(groupread_num, DXL_ID[i]);
			// if (dxl_addparam_result != True)
			// {
			// fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID[i]);
			// // return 0;
			// }
		}
	}
}

void addparam_write(int groupwrite_num, int ID, int VALUE, int LEN_DATA)
{
	if (ID != 0)
	{
		groupSyncWriteAddParam(groupwrite_num, ID, VALUE, LEN_DATA);
		// if (dxl_addparam_result != True)
		// {
		// 	fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed\n", ID);
		// 	//   return 0;
		// }
	}
}
int *dxl_scan(int port_num)
{
	static int ID[13];

	for (int j = 0; j < 13; j++)
	{
		ID[j] = 0;
	}

	int i = 0;
	for (int id = 1; id < 13; id++)
	{
		pingGetModelNum(port_num, 2.0, id);
		if ((getLastTxRxResult(port_num, 2.0)) == COMM_SUCCESS)
		{
			ID[i] = id;
			i++;
		}
		// printf("%d      ", ID[id-1]);
	}

	return ID;
}
