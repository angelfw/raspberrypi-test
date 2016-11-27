#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;


int main( int argc, const char** argv )
{
    CvCapture* capture = 0;
    Mat frame, frameCopy, image, frame_gray;
    
    
    String face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml";
    CascadeClassifier face_cascade;
    if( !face_cascade.load( face_cascade_name ) )
    { 
        printf("Error loading %s\n", face_cascade_name.c_str()); 
        return -1; 
    };

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
            
            if( frame.empty() )
                break;
            if( iplImg->origin == IPL_ORIGIN_TL )
                frame.copyTo( frameCopy );
            else
                flip( frame, frameCopy, 0 );
                
            // Detect faces
            std::vector<Rect> faces;
            cvtColor( frame, frame_gray, CV_BGR2GRAY );
            equalizeHist( frame_gray, frame_gray );
            
            face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(120, 120), Size(250, 250));
            for( size_t i = 0; i < faces.size(); i++ )
              {
                rectangle( frame, faces[i], Scalar( 255, 0, 255 ), 4, 8, 0 );
                cout << "(width, height) = " << faces[i].width << " " << faces[i].height << endl;
            }

            CvMat CvMatframe(frame);
            cvShowImage( "result", &CvMatframe );

            if( waitKey( 10 ) >= 0 )
            break;
        }
        // waitKey(0);
    }

    cvReleaseCapture( &capture );
    cvDestroyWindow( "result" );

    return 0;
}

