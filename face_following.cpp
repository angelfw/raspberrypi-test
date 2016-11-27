#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <stdio.h>
#include <sys/stat.h>

extern "C"
{
    #include "gopigo.h"
}

using namespace std;
using namespace cv;

#define WRITE_BUF_SIZE 5
#define READ_BUF_SIZE 32
#define max_vel 30

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

    int init_pid = 0, init_count = 0;
    double kp = 0.25, ki = 0.005, kd = 0.0;
    double dp, di, dd;
    double ut;
    int vel;
    double prev_t, curr_t;
    int prev_dx, prev_ut;

    int img_counter = 0;

    string face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml";
    CascadeClassifier face_cascade;
    if( !face_cascade.load( face_cascade_name ) )
    {
        printf("Error loading %s\n", face_cascade_name.c_str());
        return -1;
    };

    struct stat sb;
    string out_dir = "./result";
    if (!(stat(out_dir.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) {
        const int dir_err = mkdir(out_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (-1 == dir_err)
        {
            printf("Error creating directory!n");
            exit(1);
        }
    }
    else
    {
        printf("Output dir: %s\n", out_dir.c_str());
    }

        // Buffer for data being read/ written on the i2c bus
    if(init()==-1)
        exit(1);
    led_on(1);

    // init control
    set_left_speed(max_vel);
    set_right_speed(max_vel);

    capture = cvCaptureFromCAM( CV_CAP_ANY ); //0=default, -1=any camera, 1..99=your camera
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
            // cout << "target location: " << target_pos << endl;

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

            face_cascade.detectMultiScale( frame_gray, faces, \
                                           1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(120, 120), Size(250, 250));

            //face_cascade.detectMultiScale( frame_gray, faces, rejectLevels, levelWeights, \
                                           //1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(120, 120), Size(250, 250), true);
            //cout << "debug: " << rejectLevels.size() << " " << levelWeights.size() << endl;
            //for (size_t i = 0; i < rejectLevels.size() ; i ++ ) {
                //cout << rejectLevels[i] << " " << levelWeights[i] << endl;
            //}

            if (faces.empty()) {
                curr_pos = prev_pos;
            } else {
                curr_pos = Point(faces[0].x + faces[0].width*0.5, faces[0].y + faces[0].height*0.5);
            }
            dx = curr_pos.x - target_pos.x;
            cout << "curr location: " << curr_pos.x << " distance: " << dx << endl;

            if (!faces.empty() && abs(dx) > 10) {
                // simple PID controller
                time_t curr_t = time(0);
                if (!init_pid) {
                    prev_t = curr_t;
                    prev_dx = dx;
                    prev_ut = 0;
                    if (init_count < 2) {
                        init_count ++;
                    } else {
                        init_pid = 1;
                    }

                } else {
                    cout << "prev_t, curr_t: " << prev_t << " " << curr_t << endl;

                    // PID
                    dp = kp * dx;
                    di = ki * (dx + prev_dx);
                    //dd = kd * ((dx - prev_dx)/(curr_t-prev_t));
                    dd = 0;

                    ut = dp + di + dd;
                    //vel = min(max_vel, int(ut/(curr_t-prev_t)));
                    vel = min(max_vel, int(ut));
                    cout << "dp, di, dd: " << dp << " " << di << " " << dd << endl;
                    cout << "ut, vel, curr_t: " << ut << " " << vel << " " << curr_t << endl;

                    // UPDATE
                    prev_dx = dx;
                    prev_ut = ut;
                    prev_t = curr_t;

                    set_left_speed(abs(vel));
                    set_right_speed(abs(vel));
                    if (abs(vel)<10) {
                        stop();
                    } else if (vel < 0) {
                        left_rot();
                    } else {
                        right_rot();
                    }
                    cout << "velocity: " << vel << endl;

                }

            } else {
                init_pid = 0;
                init_count = 0;
                stop();
            }

            for( size_t i = 0; i < faces.size() && i < 1; i++ )
            {
                rectangle( frame, faces[i], Scalar( 255, 0, 255 ), 4, 8, 0 );
                // cout << "(width, height) = " << faces[i].width << " " << faces[i].height << endl;
            }

            CvMat CvMatframe(frame);
            cvShowImage( "result", &CvMatframe );

            ostringstream ss;
            ss << out_dir << "/image" << setfill('0') << img_counter << ".jpg";
            string filename(ss.str());
            imwrite( filename.c_str(), frame );

            img_counter ++;

            if( waitKey( 10 ) >= 0 )
            break;
        }
        // waitKey(0);
    }

    cvReleaseCapture( &capture );
    cvDestroyWindow( "result" );

    return 0;
}

