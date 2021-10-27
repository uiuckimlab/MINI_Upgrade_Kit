#ifndef DXL_DRIVER_H
#define DXL_DRIVER_H
int getch();

int kbhit(void);

char getKey();

void write1(int port_num, int DXL_ID[], int DXL_num, int ADDR, int Command);

void write2(int port_num, int DXL_ID[], int DXL_num, int ADDR, int CMD);

void write4(int port_num, int DXL_ID[], int DXL_num, int ADDR, int CMD);

void ind_addr(int port_num, int DXL_ID[], int DXL_num, int ADDR_IND, int ADDR_DIR, int LEN_DATA);

void addparam_read(int groupread_num, int DXL_ID[], int DXL_num);

void addparam_write(int groupwrite_num, int ID, int VALUE, int LEN_DATA);

int *dxl_scan(int port_num);

#endif
