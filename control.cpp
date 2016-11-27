#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <cmath>
#include <stdio.h>

extern "C"
{
    #include "gopigo.h"
}

using namespace std;
using namespace cv;

#define WRITE_BUF_SIZE 5
#define READ_BUF_SIZE 32

int send_cmd(char cmd,char v1,char v2,char v3)
{
    w_buf[0]=1;
    w_buf[1]=cmd;
    w_buf[2]=v1;
    w_buf[3]=v2;
    w_buf[4]=v3;

    // run cmd
    ssize_t ret = write(fd, w_buf, WRITE_BUF_SIZE);

    if (ret != WRITE_BUF_SIZE) {
        if (ret == -1) {
            printf("Error writing to GoPiGo (errno %i): %s\n", errno, strerror(errno));
        }
        else {
            printf("Error writing to GoPiGo\n");
        }
        return ret;
    }
    // run for 100 ms
    pi_sleep(100);
    // stop
    w_buf[0]=1;
    w_buf[1]=stop_cmd;
    ret = write(fd, w_buf, WRITE_BUF_SIZE);
    if (ret != WRITE_BUF_SIZE) {
        if (ret == -1) {
            printf("Error stop executing (errno %i): %s\n", errno, strerror(errno));
        }
        else {
            printf("Error stop executing GoPiGo\n");
        }
        return ret;
    }
    return 1;
}


int main( int argc, const char** argv )
{

    if(init()==-1)
        exit(1);
    led_on(1);

    // init control
    set_left_speed(50);
    set_right_speed(50);

    for(;;)
    {
        // user input
        char input[2];    /* Store 2 input characters */
        printf("Enter cmd: ");
        scanf(" %s", &input);
        switch (input[0])
        {
            case 'w': case 'W':
                fwd();
                break;
            case 's': case 'S':
                bwd();
                break;
            case 'e': case 'E':
                increase_speed();
                break;
            case 'g': case 'G':
                decrease_speed();
                break;
            case 'x': case 'X':
                stop();
                break;
            case 'n': case 'N':
                //left_rot();
                send_cmd(left_rot_cmd,0,0,0);
                break;
            case 'm': case 'M':
                // right_rot();
                send_cmd(right_rot_cmd,0,0,0);
                break;
            default:
                printf("invalid command...\n");
                break;
        }

    }

    return 0;
}

