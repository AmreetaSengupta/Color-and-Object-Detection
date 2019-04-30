
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

#define HRES 640
#define VRES 480
#define NUM_THREADS (2)

#define GPIO_DIR 	"/sys/class/gpio"

timer_t blink_timer_id;
int gpio_export(uint32_t pin);
int gpio_dir(uint32_t pin, const char *dir);
int gpio_set_value(uint32_t pin, uint32_t val);
int gpio_get_value(uint32_t pin);
int gpio_blink(uint32_t pin);
void blink_timer_init(uint32_t pin);
void blink_timer_handle(union sigval sv);

void ccw();
void cw();
void stop();
/*
CvCapture* capture;
    
void *image_detect(void *)
{
	while(1)
	{
		cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
		IplImage* frame;
		Mat gray;
		vector<Vec3f> circles;
		frame=cvQueryFrame(capture);

        Mat mat_frame(frame);
        medianBlur(mat_frame,mat_frame, 3);
        cvtColor(mat_frame, gray, COLOR_BGR2HSV);
        Mat lower_red_hue_range;
		Mat upper_red_hue_range;
		inRange(gray, Scalar(38, 100, 100), Scalar(50, 150, 255), lower_red_hue_range);
		inRange(gray, Scalar(51,100, 100), Scalar(75, 150, 255), upper_red_hue_range);

		// Combine the above two images
		Mat red_hue_image;
		addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
        GaussianBlur(red_hue_image,red_hue_image, Size(9,9), 2, 2);
        //Find Circles
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 30, 0,0 );

        printf("circles.size = %d\n", circles.size());
	    //Draw detected circles
        for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          if(radius>50)
          {
          // circle center
          circle( mat_frame, center, 3, Scalar(255,255,0), -1, 8, 0 );
          // circle outline
          circle( mat_frame, center, radius, Scalar(255,255,0), 3, 8, 0 );
          }
	    }
     
      if(!frame) break;
	
	  cvShowImage("Capture Example", frame);
	  cvNamedWindow("Threshold lower image", CV_WINDOW_AUTOSIZE);
	  imshow("Threshold lower image", lower_red_hue_range);
	 cvNamedWindow("Threshold upper image", CV_WINDOW_AUTOSIZE);
	  imshow("Threshold upper image", upper_red_hue_range);
	  cvNamedWindow("Combined threshold images", CV_WINDOW_AUTOSIZE);
	  imshow("Combined threshold images", red_hue_image);
	  cvNamedWindow("Detected red circles on the input image", CV_WINDOW_AUTOSIZE);
	  imshow("Detected red circles on the input image", mat_frame);

        char c = cvWaitKey(10);
        if( c == 27 ) break;
	}
	
}

void *motor_working(void *)
{
	while(1)
	{
		
	}
}
*/
int main()
{
		while(1)
	{
	cw();
	}
	
	/*
	int rt_max_prio, rc, i;
	
	
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(0);
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(argv[1]);

	pthread_t threads[NUM_THREADS]; //Thread Descriptors
	pthread_attr_t rt_sched_attr[NUM_THREADS];
	struct sched_param rt_param;
	struct sched_param main_param;

    capture = (CvCapture *)cvCreateCameraCapture(0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);

	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	main_param.sched_priority=rt_max_prio; //Set scheduling priority as maximum (99)
   	sched_setscheduler(getpid(), SCHED_FIFO, &main_param); //Set scheduling policy as SCHED_FIFO

	rt_param.sched_priority=rt_max_prio-1; //Set priority of 98 to task 1

	for(i=0;i<NUM_THREADS;i++)
	{
		pthread_attr_init(&rt_sched_attr[i]);
		rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO); //Set SCHED_FIFO
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param);
	}

	rc = pthread_create(&threads[0],   // pointer to thread descriptor
                      &rt_sched_attr[0],     // thread attributes object                      
                      image_detect, // thread function entry point
                      (void *)0 // parameters to pass in
                       );

	if (rc) //Error checking if thread 1 has been created
	perror("pthread create");

	rc = pthread_create(&threads[1],   // pointer to thread descriptor
						  &rt_sched_attr[1],     // thread attributes object                      
						  motor_working, // thread function entry point
						  (void *)0 // parameters to pass in
					   );

	if (rc) //Error checking if thread 2 has been created
	perror("pthread create");



	for(i=0;i<NUM_THREADS;i++)
	pthread_join(threads[i], NULL);
	cvReleaseCapture(&capture);
	

	for(i=0;i<NUM_THREADS;i++)
	{
		if(pthread_attr_destroy(&rt_sched_attr[i]) != 0)
		perror("attr destroy");
	}
	cvDestroyWindow("Capture Example");
	
	
	return 0; */
} 

int gpio_export(uint32_t pin)
{
	FILE *fp;

	fp = fopen(GPIO_DIR "/export", "w");
	if(fp == NULL)
		return -1;

	fprintf(fp, "%d",pin);

	fclose(fp);
	return 0;
} 

int gpio_dir(uint32_t pin, const char *dir)
{
	FILE *fp;
	char path[50];

	snprintf(path, 50, GPIO_DIR "/gpio%d/direction",pin);

	fp = fopen(path, "w");
	if(fp == NULL)
		return -1;

	fprintf(fp, "%s",dir);

	fclose(fp);
	return 0;
} 

int gpio_set_value(uint32_t pin, uint32_t val)
{
	FILE *fp;
	char path[50];

	snprintf(path, 50, GPIO_DIR "/gpio%d/value",pin);

	fp = fopen(path, "w");
	if(fp == NULL)
		return -1;

	fprintf(fp, "%d",val);

	fclose(fp);
	return 0;
} 
int gpio_get_value(uint32_t pin)
{
	FILE *fp;
	char path[50];
	uint32_t val;

	snprintf(path, 50, GPIO_DIR "/gpio%d/value",pin);

	fp = fopen(path, "r");
	if(fp == NULL)
		return -1;

	val = (uint32_t)fgetc(fp);
	val -= 48;

	fclose(fp);
	return val;
} 
void ccw()
{
	//In1
	gpio_export(161);
	gpio_dir(161,"out");
	gpio_set_value(161, 0);
	
	//In2
	gpio_export(163);
	gpio_dir(163,"out");
	gpio_set_value(163, 1);
	
	//PWM
	gpio_export(57);
	gpio_dir(57,"out");
	//gpio_set_value(57, 1);
}

void cw()
{
	//In1
	gpio_export(161);
	gpio_dir(161,"out");
	gpio_set_value(161, 1);
	
	//In2
	gpio_export(163);
	gpio_dir(163,"out");
	gpio_set_value(163, 0);
	
	//PWM
	//gpio_export(57);
	//gpio_dir(57,"out");
	//gpio_set_value(57, 1);
	gpio_export(57);
	gpio_dir(57,"out");
	//gpio_set_value(57, 1);
	gpio_blink(57);
}

void stop()
{
	//In1
	gpio_export(161);
	gpio_dir(161,"out");
	gpio_set_value(161, 0);
	
	//In2
	gpio_export(163);
	gpio_dir(163,"out");
	gpio_set_value(163, 0);
	
	//PWM
	gpio_export(57);
	gpio_dir(57,"out");
//	gpio_set_value(161, 1);
}

int gpio_blink(uint32_t pin)
{
	blink_timer_init(pin);
} 

void blink_timer_init(uint32_t pin)
{
	struct sigevent sev;
	struct itimerspec trigger;

	memset(&sev, 0, sizeof(struct sigevent));
    memset(&trigger, 0, sizeof(struct itimerspec));

    sev.sigev_notify = SIGEV_THREAD;
	sev.sigev_notify_function = blink_timer_handle;
	sev.sigev_value.sival_int = pin;

	timer_create(CLOCK_REALTIME, &sev, &blink_timer_id);

	trigger.it_interval.tv_sec = 1;
	trigger.it_value.tv_sec = 1;

	timer_settime(blink_timer_id, 0, &trigger, NULL);
}


void blink_timer_handle(union sigval sv)
{	
	uint32_t led;
	
	led = gpio_get_value(sv.sival_int);
	printf(" timer %d\n\r",led);
	gpio_set_value(sv.sival_int, !led);
}
