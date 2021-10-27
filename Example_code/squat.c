#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "dxl_driver.h"
#include "dynamixel_sdk.h" // Uses Dynamixel SDK library
#include "Kinematics.h"

// XL320 Address
#define ADDR_320_ReturnDelay 5
#define ADDR_320_Shutdown 18
#define ADDR_320_TORQUE 24
#define ADDR_320_LED 25
#define ADDR_320_D 27
#define ADDR_320_I 28
#define ADDR_320_P 29
#define ADDR_320_POS_GOAL 30
#define ADDR_320_POS_PRESENT 37
#define ADDR_320_VOLT 45

// 320 Data Length
#define LEN_320_ReturnDelay 1
#define LEN_320_MODE 1
#define LEN_320_TORQUE 1
#define LEN_320_D 1
#define LEN_320_I 1
#define LEN_320_P 1
#define LEN_320_POS_GOAL 2
#define LEN_320_POS_PRESENT 2
#define LEN_320_VOLT 1

// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel

#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0" // Check which port is being used on your controller

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque

#define ESC_ASCII_VALUE 0x1b
#define PI 3.1412
#define BILLION 1000000000L
#define MILLION 1000000L

// Generation cubic trajectory between two points
float cubic2points(float t, float t_f, float p_0, float p_1)
{
    float a = 2 * (p_0 - p_1) / (t_f * t_f * t_f);
    float b = -3 * (p_0 - p_1) / (t_f * t_f);
    float d = p_0;
    return (a * t * t * t + b * t * t + d);
}

int main()
{

    int DXL_pos[50000][16] = {
        0,
    };
    int motor_command[50000][16] = {
        0,
    };

    int DXL_ID[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    int DXL_POS[16] = {512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512};
    int DXL_ID_arm[6] = {1, 2, 3, 4, 5, 6};
    int DXL_ID_leg[10] = {7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    int DXL_num = 16;
    int input = 0;

    double t = 0.0, t_elapsed = 0.0, t_cubic = 0.0;
    int t_init = 5.0;
    float t_loop1 = 0.0, t_loop2 = 0.0;
    long t_sleep = 0;
    uint64_t diff1, diff2;
    long tick = 0;
    int tick2 = 0, tick3 = 0;
    struct timespec t0, t_start, t_end1, t_end2;
    float x_RH = 0.0, y_RH = 0.0, z_RH = 0.0, x_LH = 0.0, y_LH = 0.0, z_LH = 0.0;
    float x_RF = 0.0, y_RF = 0.0, z_RF = 0.0, Roll_RF = 0.0, Pitch_RF = 0.0, x_LF = 0.0, y_LF = 0.0, z_LF = 0.0, Roll_LF = 0.0, Pitch_LF = 0.0;
    float x_RH0 = 0.0, y_RH0 = 0.0, z_RH0 = 0.0, x_LH0 = 0.0, y_LH0 = 0.0, z_LH0 = 0.0;
    float x_RF0 = 0.0, y_RF0 = 0.0, z_RF0 = 0.0, Roll_RF0 = 0.0, Pitch_RF0 = 0.0, x_LF0 = 0.0, y_LF0 = 0.0, z_LF0 = 0.0, Roll_LF0 = 0.0, Pitch_LF04;

    int t_array[100000] = {
        0,
    };

    // Initialize PortHandler Structs
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    int port_num = portHandler(DEVICENAME);
    closePort(port_num);
    // Initialize PacketHandler Structs
    packetHandler();

    // Open port
    if (openPort(port_num))
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (setBaudRate(port_num, BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    printf("Press any key to continue! (or press ESC to quit!)\n");

    write1(port_num, DXL_ID, DXL_num, ADDR_320_TORQUE, TORQUE_DISABLE);
    write1(port_num, DXL_ID, DXL_num, ADDR_320_ReturnDelay, 50);  
    write1(port_num, DXL_ID, DXL_num, ADDR_320_VOLT, 50);
    write1(port_num, DXL_ID, DXL_num, ADDR_320_TORQUE, TORQUE_ENABLE);

    write1(port_num, DXL_ID, DXL_num, ADDR_320_P, 15); // 0 ~ 254
    write1(port_num, DXL_ID, DXL_num, ADDR_320_I, 15); // 0 ~ 254
    write1(port_num, DXL_ID, DXL_num, ADDR_320_D, 0); // 0 ~ 254

    // Initialize XM430 Groupsyncwrite/read instance
    int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_320_POS_GOAL, LEN_320_POS_GOAL);
    // int groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_320_POS_PRESENT, LEN_320_POS_PRESENT);
    int groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_320_VOLT, LEN_320_VOLT);

    addparam_read(groupread_num, DXL_ID, DXL_num);

    int dxl_comm_result = COMM_TX_FAIL;  // Communication result
    uint8_t dxl_addparam_result = False; // AddParam result
    uint8_t dxl_getdata_result = False;  // GetParam result

            // Pose Initializa
        printf("Go to Initial Pose\n");
            x_RH = 0.0;
            y_RH = -174.0+90.0;
            z_RH = -12.0-90.0;

            x_LH = 0.0;
            y_LH = 174.0-90.0;
            z_LH = -12.0-90.0;
            
            
            x_RF = 16.0-10;
            y_RF = -33.0;
            z_RF = -196.0+10;

            
            x_LF = 16.0-10;
            y_LF = 33.0;
            z_LF = -196.0+10;
            
            Pitch_LF = -2*PI/180;
            Pitch_RF = -2*PI/180;


        IK_RH(groupwrite_num, x_RH, y_RH, z_RH);
        IK_LH(groupwrite_num, x_LH, y_LH, z_LH);
        IK_RF(groupwrite_num, x_RF, y_RF, z_RF, Roll_RF, Pitch_RF);
        IK_LF(groupwrite_num, x_LF, y_LF, z_LF, Roll_LF, Pitch_LF);
        
            // Syncwrite all
        groupSyncWriteTxPacket(groupwrite_num);
        groupSyncWriteClearParam(groupwrite_num);
        sleep(1);
        write1(port_num, DXL_ID_arm, 6, ADDR_320_P, 55); // 0 ~ 254
        write1(port_num, DXL_ID_arm, 6, ADDR_320_I, 10); // 0 ~ 254
        write1(port_num, DXL_ID_arm, 6, ADDR_320_D, 1); // 0 ~ 254

        write1(port_num, DXL_ID_leg, 10, ADDR_320_P, 120); // 0 ~ 254
        write1(port_num, DXL_ID_leg, 10, ADDR_320_I, 15); // 0 ~ 254
        write1(port_num, DXL_ID_leg, 10, ADDR_320_D, 10); // 0 ~ 254
        printf("Press Any Key to Continue\n");  
        getchar(); 



    clock_gettime(CLOCK_MONOTONIC, &t0);

    // While Loop
    while (1)
    {
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        printf("%d     Loop: %2.3f   Elapsed Time: %3.3f   \n", tick3, t_loop2, t_elapsed);

        /////////////////////////////////////////// Keyboard Input /////////////////////////////////////////////////////////////////////

        input = getKey();

        if (input == ESC_ASCII_VALUE)
        {
            break;
        }

        // groupSyncReadTxRxPacket(groupread_num);
        // if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        //     printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));

        // // Syncread present position
        // for (int i = 0; i < DXL_num; i++)
        // {
        //     if (DXL_ID[i] != 0)
        //     {
        //         // Check if groupsyncread data of Dyanamixel is available
        //         dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL_ID[i], ADDR_320_POS_PRESENT, LEN_320_POS_PRESENT);
        //         if (dxl_getdata_result != True)
        //         {
        //             fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID[i]);
        //             return 0;
        //         }
        //         DXL_pos[tick][i] = groupSyncReadGetData(groupread_num, DXL_ID[i], ADDR_320_POS_PRESENT, LEN_320_POS_PRESENT);
        //         printf("DXL%d = %d   ", i + 1, DXL_pos[tick][i]);
        //     }
        // }


        // Syncread present Voltage
        // for (int i = 0; i < DXL_num; i++)
        // {
        //     if (DXL_ID[i] != 0)
        //     {
        //         // Check if groupsyncread data of Dyanamixel is available
        //         dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL_ID[i], ADDR_320_VOLT, LEN_320_VOLT);
        //         if (dxl_getdata_result != True)
        //         {
        //             fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID[i]);
        //             return 0;
        //         }
        //         DXL_pos[tick][i] = groupSyncReadGetData(groupread_num, DXL_ID[i], ADDR_320_VOLT, LEN_320_VOLT);
        //         printf("DXL%d = %d   ", i + 1, DXL_pos[tick][i]);
        //     }
        // }
        // printf("\n");
    
        // Motion Generation
            // x_RH = 0.0;
            // y_RH = -174.0+110.0;
            // z_RH = -12.0-100.0;

            // x_LH = 0.0;
            // y_LH = 174.0-110.0;
            // z_LH = -12.0-100.0;
            
            
            x_RF0 = 16.0-10;
            y_RF0 = -33.0;
            z_RF0 = -196.0+10;           
            x_LF0 = 16.0-10;
            y_LF0 = 33.0;
            z_LF0 = -196.0+10;

            x_RF = x_RF0 + 0.0;
            x_LF = x_LF0 + 0.0;
            z_RF = z_RF0 + 40   -20.0 * (1 - cos(2 * PI * 0.5 * (t_elapsed-t_init)));
            z_LF = z_LF0 + 40   -20.0 * (1 - cos(2 * PI * 0.5 * (t_elapsed-t_init)));
            Pitch_LF = -2*PI/180;
            Pitch_RF = -2*PI/180;
 

        IK_RH(groupwrite_num, x_RH, y_RH, z_RH);
        IK_LH(groupwrite_num, x_LH, y_LH, z_LH);
        IK_RF(groupwrite_num, x_RF, y_RF, z_RF, Roll_RF, Pitch_RF);
        IK_LF(groupwrite_num, x_LF, y_LF, z_LF, Roll_LF, Pitch_LF);

        // Syncwrite all
        groupSyncWriteTxPacket(groupwrite_num);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));

        groupSyncWriteClearParam(groupwrite_num);

        if (tick < 50000)
        {
            t_array[tick] = t_elapsed * 1000;
        }
        tick++;  // For general tick
        tick2++; // For rescan timer
        tick3++; // For Profile V & A

        clock_gettime(CLOCK_MONOTONIC, &t_end1);

        diff1 = MILLION * (t_end1.tv_sec - t_start.tv_sec) + (t_end1.tv_nsec - t_start.tv_nsec) / 1000; //usec
        t_loop1 = diff1 * 1e-3;                                                                         // msec
        t_sleep = 8000.0 - (float)t_loop1 * 1000;                                                       // Loop time == 20ms

        if (t_sleep > 0 && t_sleep < 50000)
        {
            usleep(t_sleep - 60);
        }

        clock_gettime(CLOCK_MONOTONIC, &t_end2);
        t_elapsed = (double)((t_end2.tv_sec - t0.tv_sec) * 1000000 + (t_end2.tv_nsec - t0.tv_nsec) / 1000) / 1000000;
        diff2 = MILLION * (t_end2.tv_sec - t_start.tv_sec) + (t_end2.tv_nsec - t_start.tv_nsec) / 1000; //usec
        t_loop2 = diff2 * 1e-3;

        // end of while loop
    }

    write1(port_num, DXL_ID, DXL_num, ADDR_320_TORQUE, TORQUE_DISABLE);

    // Close port
    closePort(port_num);
    return 0;
}
