
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <semaphore.h>
#include <syslog.h>
#include <sys/time.h>
#include <errno.h>
int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE;


using namespace cv;
using namespace std;

#define HRES 640
#define VRES 480
#define NUM_THREADS (3)
#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
#define DELAY_TICKS (1)
#define ERROR (-1)
#define OK (0)

#define GPIO_DIR 	"/sys/class/gpio"

void *Sequencer(void *threadp);
void *image_detect(void *)
int gpio_export(uint32_t pin);
int gpio_dir(uint32_t pin, const char *dir);
int gpio_set_value(uint32_t pin, uint32_t val);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

void cw1();
void cw2();
void stop1();
void stop2();
void left();
void right();
void straight();

CvCapture* capture;

int right_flag= 0;
int left_flag = 0;

void *sequencer(void *)
{
	
}

void *image_detect(void *)
{
	int difference;
	 struct timespec start_time = {0, 0};
    struct timespec finish_time = {0, 0};
    struct timespec thread_dt = {0, 0};
	while(1)
	{
		cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
		IplImage* frame;
		Mat gray;
		vector<Vec3f> circles;
		  clock_gettime(CLOCK_REALTIME, &start_time);
		frame=cvQueryFrame(capture);

        Mat mat_frame(frame);
        medianBlur(mat_frame,mat_frame, 3);
        cvtColor(mat_frame, gray, COLOR_BGR2HSV);
        Mat lower_red_hue_range;
		Mat upper_red_hue_range;
		inRange(gray, Scalar(2, 100, 100), Scalar(7, 150, 255), lower_red_hue_range);
		inRange(gray, Scalar(150,100, 100), Scalar(170, 150, 255), upper_red_hue_range);

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
          printf("center x=%d y=%d \n",cvRound(circles[i][0]),cvRound(circles[i][1]));
          difference=320-cvRound(circles[i][0]);
          printf("difference %d \n",difference);
          if(difference < -30)
          {
			  right_flag = 1;
		  }
          else if(difference > 30)
          {
			  left_flag =1;
		  }
		  
         
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
	/*  imshow("Threshold lower image", lower_red_hue_range);
	  cvNamedWindow("Threshold upper image", CV_WINDOW_AUTOSIZE);
	  imshow("Threshold upper image", upper_red_hue_range);
	  cvNamedWindow("Combined threshold images", CV_WINDOW_AUTOSIZE);
	  imshow("Combined threshold images", red_hue_image);
	  cvNamedWindow("Detected red circles on the input image", CV_WINDOW_AUTOSIZE);
	  imshow("Detected red circles on the input image", mat_frame); */

        char c = cvWaitKey(10);
        if( c == 27 ) break;
          clock_gettime(CLOCK_REALTIME, &finish_time);
		delta_t(&finish_time, &start_time, &thread_dt);
		printf("\nimage processing thread WCET is %ld sec, %ld msec (%ld microsec)\n", thread_dt.tv_sec, (thread_dt.tv_nsec / NSEC_PER_MSEC), (thread_dt.tv_nsec / NSEC_PER_MICROSEC));
	}
	
}

void *motor_working(void *)
{
	while(1)

	{
		 struct timespec start_time = {0, 0};
    struct timespec finish_time = {0, 0};
    struct timespec thread_dt = {0, 0};
    
     clock_gettime(CLOCK_REALTIME, &start_time);
		if(right_flag == 1)
		{
			right();
			right_flag = 0;
		}
		else if(left_flag == 1)
		{
			left();
			left_flag=0;
		}
		else 
		{
			straight();
		}
		clock_gettime(CLOCK_REALTIME, &finish_time);
		delta_t(&finish_time, &start_time, &thread_dt);
		printf("\n motor thread WCET is %ld sec, %ld msec (%ld microsec)\n", thread_dt.tv_sec, (thread_dt.tv_nsec / NSEC_PER_MSEC), (thread_dt.tv_nsec / NSEC_PER_MICROSEC));
	}
}

int main()
{
	int rt_max_prio, rc, i;
	//CvCapture* capture = (CvCapture *)cvCreateCameraCapture(0);
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(argv[1]);

	pthread_t threads[NUM_THREADS]; //Thread Descriptors
	pthread_attr_t rt_sched_attr[NUM_THREADS];
	struct sched_param rt_param[NUM_THREADS];
	struct sched_param main_param;
	
	if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
	if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }

    capture = (CvCapture *)cvCreateCameraCapture(0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);

	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	main_param.sched_priority=rt_max_prio; //Set scheduling priority as maximum (99)
   	sched_setscheduler(getpid(), SCHED_FIFO, &main_param); //Set scheduling policy as SCHED_FIFO

	rt_param[0].sched_priority=rt_max_prio-1; //Set priority of 98 to task 1
	rt_param[1].sched_priority=rt_max_prio-2; //Set priority of 98 to task 1
	rt_param[2].sched_priority=rt_max_prio-3; //Set priority of 97 to task 2

	for(i=0;i<NUM_THREADS;i++)
	{
		pthread_attr_init(&rt_sched_attr[i]);
		rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO); //Set SCHED_FIFO
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
	}
	
	
	rc = pthread_create(&threads[0],   // pointer to thread descriptor
						  &rt_sched_attr[0],     // thread attributes object                      
						  sequencer, // thread function entry point
						  (void *)0 // parameters to pass in
					   );

	if (rc) //Error checking if thread 2 has been created
	perror("pthread create");

	rc = pthread_create(&threads[1],   // pointer to thread descriptor
                      &rt_sched_attr[1],     // thread attributes object                      
                      image_detect, // thread function entry point
                      (void *)0 // parameters to pass in
                       );

	if (rc) //Error checking if thread 1 has been created
	perror("pthread create");


	rc = pthread_create(&threads[2],   // pointer to thread descriptor
						  &rt_sched_attr[2],     // thread attributes object                      
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
	
	
	return 0; 
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
	
	void cw1()
	{
		//AIn1
		gpio_export(17);
		gpio_dir(17,"out");
		gpio_set_value(17, 1);
		
		//AIn2
		gpio_export(27);
		gpio_dir(27,"out");
		gpio_set_value(27, 0);
		
		//PWMA
		gpio_export(22);
		gpio_dir(22,"out");
		gpio_set_value(22, 1);
		//gpio_blink(22);
	}
	
	void stop1()
	{
		//AIn1
		gpio_export(17);
		gpio_dir(17,"out");
		gpio_set_value(17, 0);
		
		//AIn2
		gpio_export(27);
		gpio_dir(27,"out");
		gpio_set_value(27, 0);
		
		//PWMA
		gpio_export(22);
		gpio_dir(22,"out");
		gpio_set_value(22, 1);
		//gpio_blink(22);
	}
	
	void cw2()
	{
		//BIn1
		gpio_export(26);
		gpio_dir(26,"out");
		gpio_set_value(26, 1);
		
		//BIn2
		gpio_export(19);
		gpio_dir(19,"out");
		gpio_set_value(19, 0);
		
		//PWMB
		gpio_export(13);
		gpio_dir(13,"out");
		gpio_set_value(13, 1);
		//gpio_blink(22);
	}
	
	void stop2()
	{
		//BIn1
		gpio_export(26);
		gpio_dir(26,"out");
		gpio_set_value(26, 0);
		
		//BIn2
		gpio_export(19);
		gpio_dir(19,"out");
		gpio_set_value(19, 0);
		
		//PWMB
		gpio_export(13);
		gpio_dir(13,"out");
		gpio_set_value(13, 1);
		//gpio_blink(22);
	}
	
	void left()
	{
		cw1();
		stop2();
	}
	
	void right()
	{
		cw2();
		stop1();
	}
	
	void straight()
	{
		cw1();
		cw2();
	}
	

int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  if(dt_sec >= 0)
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
  else
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }

  return(1);
}
