// Standard C/C++ headers

#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// OpenCV headers

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/videoio.hpp> // Needed for OpenCV 3.1, does not exist in 2.4

// NOTE: to make this code work on the GigaByte Brix GB-BSI7H, I had to modify OpenCV as follows:
// in file opencv/modules/videoio/src/cap_v4l.cpp line 249
// changed DEFAULT_V4L_BUFFERS from 4 to 3
// otherwise the v4l driver would complain about failing to allocate memory at 4224x3156 resolution

using namespace std;

#define SHOW_VIDEO		0	// 1 to display video in a window until ESC key is pressed, does not work with MULTI_THREADS
#define WRITE_VIDEO		0	// 1 to record a video
#define WRITE_IMAGES	1	// 1 to record a sequence of still images
#define MULTI_THREADS	1	// 1 to record images and write files on separate threads
#define MAX_FRAMES		10	// maximum number of frames in still image sequence and/or video; ignored if SHOW_VIDEO is 1
#define MAX_CAMERAS		10	// maximum number of cameras

int gNumCameras = 1;
int gFrameWidth = 640;
int gFrameHeight = 480;

struct timeval gStartTime = { 0 };

void CaptureFrame ( cv::VideoCapture *cameras[], cv::VideoWriter *videoWriters[], int count[], int i )
{
	cv::Mat3b	frame;
    char		filename[256] = { 0 };
	int			n = 0;

	printf ( "Camera %d = %x\n", i, (unsigned int) cameras[i] );

    for ( n = 0; n < MAX_FRAMES; n++ )
    {
		*cameras[i] >> frame;

#if WRITE_VIDEO
		if ( videoWriters[i] != NULL )
			videoWriters[i]->write(frame);
#endif

#if WRITE_IMAGES
		sprintf ( filename, "Image%d%03d.png", i, count[i] );
		cv::imwrite ( filename, frame );
#endif
		count[i]++;
		printf ( "Camera %d took image %d\n", i, n );
    }
}

int main ( int argc, char *argv[] )
{
    if ( argc < 4 )
    {
        printf ( "usage: %s n w h\n", argv[0] );
        printf ( "n: number of cameras (max=%d)\n", MAX_CAMERAS );
        printf ( "w: frame width\n" );
        printf ( "h: frame height\n\n" );
        exit ( 0 );
    }

    sscanf ( argv[1], "%d", &gNumCameras );
    sscanf ( argv[2], "%d", &gFrameWidth );
    sscanf ( argv[3], "%d", &gFrameHeight );

	if ( gNumCameras > MAX_CAMERAS )
		return ( -1 );

    double fps = 0, dWidth = 0, dHeight = 0;
    struct timeval endTime = { 0 };
    cv::Size frameSize ( gFrameWidth, gFrameHeight );
    cv::VideoWriter *videoWriters[ MAX_CAMERAS ] = { 0 };
    cv::VideoCapture *cameras[ MAX_CAMERAS ] = { 0 };
	thread *threads[ MAX_CAMERAS ] = { 0 };
    int count[ MAX_CAMERAS ] = { 0 };
    cv::Mat3b frame;
    
    //initialize and allocate memory to load the video stream from camera 

    memset ( videoWriters, 0, sizeof ( cv::VideoWriter * ) * gNumCameras );
    memset ( cameras, 0, sizeof ( cv::VideoCapture * ) * gNumCameras );
    memset ( threads, 0, sizeof ( thread * ) * gNumCameras );

    char filename[256] = { 0 };

    for ( int i = 0; i < gNumCameras; i++ )
    {
       cameras[i] = new cv::VideoCapture ( i );
       if ( cameras[i]->isOpened() )
           printf ( "Opened camera %d.\n", i );
       else
       {
           printf ( "Failed to open camera %d.\n", i );
           exit ( 0 );
       }

       //camera1.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
       cameras[i]->set(CV_CAP_PROP_FRAME_WIDTH, gFrameWidth);
       cameras[i]->set(CV_CAP_PROP_FRAME_HEIGHT, gFrameHeight); 
       fps = cameras[i]->get(CV_CAP_PROP_FPS);
       dWidth = cameras[i]->get(CV_CAP_PROP_FRAME_WIDTH);
       dHeight = cameras[i]->get(CV_CAP_PROP_FRAME_HEIGHT);
       cout << "Camera " << i << " Frame Size = " << dWidth << "x" << dHeight << endl;
       cout << "Camera " << i << " Frame Rate = " << fps << endl;
#if SHOW_VIDEO
		sprintf ( filename, "Camera %d", i );
	    cv::namedWindow ( filename, CV_WINDOW_NORMAL );
#endif

#if WRITE_VIDEO
       sprintf ( filename, "Video%d.avi", i );
       videoWriters[i] = new cv::VideoWriter(filename, CV_FOURCC('M','J','P','G'), 5, frameSize, true );
#endif
	}

    // Can't do anything if no cameras opened.

    if ( cameras[0] == NULL )
        exit ( 0 );

    gettimeofday ( &gStartTime, NULL );

#if MULTI_THREADS
    for ( int i = 0; i < gNumCameras; i++ )
	{
		threads[i] = new thread ( CaptureFrame, cameras, videoWriters, count, i );
		printf ( "threads[%d] = %x\n", i, (unsigned int) threads[i] );
	}

	int total_frames = 0;
	while ( total_frames < MAX_FRAMES * gNumCameras )
	{
		total_frames = 0;
		for ( int i = 0; i < gNumCameras; i++ )
			total_frames += count[i];

		usleep ( 1 );
	}

#else

    memset ( count, 0, sizeof ( int ) * gNumCameras );

#if SHOW_VIDEO
	while ( cv::waitKey ( 1 ) != 27 )	// loop until ESC key is pressed.
#else
    while ( count[0] < MAX_FRAMES )
#endif
    {
        // grab and retrieve each frames of the video sequentially 
        
        for ( int i = 0; i < gNumCameras; i++ )
        {
            if ( cameras[i] == NULL )
                continue;

            *cameras[i] >> frame;
            count[i]++;
			printf ( "Camera %d took frame %d\n", i, count[i] );

#if SHOW_VIDEO
			sprintf ( filename, "Camera %d", i );
            cv::imshow ( filename, frame );
#endif

#if WRITE_VIDEO
            if ( videoWriters[i] != NULL )
                videoWriters[i]->write ( frame );
#endif

#if WRITE_IMAGES
            sprintf ( filename, "Image%d%03d.png", i, count[i] );
            cv::imwrite ( filename, frame );
#endif
        }
    }
    
#endif

    gettimeofday ( &endTime, NULL );

    double total_time = (    endTime.tv_sec +    endTime.tv_usec / 1.0e6 )
                      - ( gStartTime.tv_sec + gStartTime.tv_usec / 1.0e6 );

    printf ( "Total time = %.3f sec\n", total_time );
    for ( int i = 0; i < gNumCameras; i++ )
	{
		cameras[i]->release();
        printf ( "Frame rate %d = %.3f FPS\n", i, count[i] / total_time );
	}

	cv::destroyAllWindows();
    return 0;
}
