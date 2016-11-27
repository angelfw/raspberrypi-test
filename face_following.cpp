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
    pi_sleep(1000);
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
    CvCapture* capture = 0;
    Mat frame, frameCopy, image, frame_gray;
    Point prev_pos(-1, -1);
    Point curr_pos(0, 0);
    Point target_pos(320, 240);
    int dx = 0;
    
    String face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml";
    CascadeClassifier face_cascade;
    if( !face_cascade.load( face_cascade_name ) )
    { 
        printf("Error loading %s\n", face_cascade_name.c_str()); 
        return -1; 
    };
    
        // Buffer for data being read/ written on the i2c bus
    if(init()==-1)
        exit(1);
    led_on(1);
    
    // init control    
    set_left_speed(50);
    set_right_speed(50);
    
    //capture = cvCaptureFromCAM( CV_CAP_ANY ); //0=default, -1=any camera, 1..99=your camera
    if( !capture )
    {
        cout << "No camera detected" << endl;
    }

    cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );

    if( capture )
    {
        cout << "In capture ..." << endl;
        for(;;)
        {
            IplImage* iplImg = cvQueryFrame( capture );
            frame = cvarrToMat(iplImg);
            target_pos = Point(frame.cols/2.0, frame.rows/2.0);
            cout << "target location: " << target_pos << endl;
            
            if( frame.empty() )
                break;
            if( iplImg->origin == IPL_ORIGIN_TL )
                frame.copyTo( frameCopy );
            else
                flip( frame, frameCopy, 0 );
                
            // Detect faces
            vector<Rect> faces;
            vector<int> rejectLevels;
            vector<double> levelWeights;
            cvtColor( frame, frame_gray, CV_BGR2GRAY );
            equalizeHist( frame_gray, frame_gray );

            face_cascade.detectMultiScale( frame_gray, faces, rejectLevels, levelWeights, \
                                           1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(120, 120), Size(250, 250), true);
            if (faces.empty()) {
                curr_pos = prev_pos;
            } else {
                curr_pos = Point(faces[0].x + faces[0].width*0.5, faces[0].y + faces[0].height*0.5);
            }
            dx = curr_pos.x - target_pos.x;
            cout << "distance: " << dx << endl;
            
            if (!faces.empty() && abs(dx) > 10) {
                
            }
            
            for( size_t i = 0; i < faces.size() && i < 1; i++ )
            {
                rectangle( frame, faces[i], Scalar( 255, 0, 255 ), 4, 8, 0 );
                // cout << "(width, height) = " << faces[i].width << " " << faces[i].height << endl;
            }

            CvMat CvMatframe(frame);
            cvShowImage( "result", &CvMatframe );
            
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

            if( waitKey( 10 ) >= 0 )
            break;
        }
        // waitKey(0);
    }

    cvReleaseCapture( &capture );
    cvDestroyWindow( "result" );

    return 0;
}

