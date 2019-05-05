


#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <time.h>

#include <fcntl.h>           
#include <sys/stat.h>        

#include <sys/syscall.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <semaphore.h>
#include <syslog.h>
#include <sys/time.h>
#include <errno.h>

#define handle_error(msg) \
			{ perror(msg); \
				exit(1); }

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
#define TRUE (1)
#define FALSE (0)
#define ON_TIME (100000000)


#define GPIO_DIR 	"/sys/class/gpio"

timer_t blink_timer_id;
timer_t blink_timer_id1;
int pwm_gen(uint32_t pin);
int gpio_blink_off(uint32_t pin);
void blink_timer_init(uint32_t pin);
void blink_timer_handle(union sigval sv);
void *Sequencer(void *threadp);
void *image_detect(void *);
int pin_export(uint32_t pin);
int pin_dir(uint32_t pin, const char *dir);
int write_value(uint32_t pin, uint32_t val);
int read_value(uint32_t pin);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);
void set_signal_handler(void);
void signal_handler(int signo, siginfo_t *info, void *extra) ;
void blink_timer_init1();
void blink_timer_handle1(union sigval sv);
void timeset();
int rc;

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
int straight_flag = 0;
sem_t semS1, semS2;
pthread_mutex_t lock;
pthread_t threads[NUM_THREADS]; //Thread Descriptors
int e1;
int timeflag=0;

void *sequencer(void *)
{
    struct timespec delay_time = {0,300000000}; // delay for 350 msec
    struct timespec remaining_time;
    double residual;
    int rc, delay_cnt=0;
  struct timespec start_time = {0, 0};
    struct timespec finish_time = {0, 0};
    struct timespec thread_dt = {0, 0};
  

    do
    {
        delay_cnt=0; residual=0.0;
clock_gettime(CLOCK_REALTIME, &start_time);
        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

       // seqCnt++;


         sem_post(&semS1);

        sem_post(&semS2);
clock_gettime(CLOCK_REALTIME, &finish_time);
		delta_t(&finish_time, &start_time, &thread_dt);
		printf("\nSequencer thread WCET is %ld sec, %ld msec (%ld microsec)\n", thread_dt.tv_sec, (thread_dt.tv_nsec / NSEC_PER_MSEC), (thread_dt.tv_nsec / NSEC_PER_MICROSEC));
   
       
        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer release all sub-services @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    } while(e1);

    sem_post(&semS1); 
    sem_post(&semS2); 
     
    pthread_exit((void *)0);
}

void *image_detect(void *)
{
	int difference;
	 struct timespec start_time = {0, 0};
    struct timespec finish_time = {0, 0};
    struct timespec thread_dt = {0, 0};
	while(e1)
	{
		 sem_wait(&semS1);
		 rc = pthread_mutex_lock(&lock);
		if(rc!= 0)
			handle_error("Mutex lock");
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
		inRange(gray, Scalar(75, 100, 100), Scalar(100, 150, 255), lower_red_hue_range);
		inRange(gray, Scalar(101,100, 100), Scalar(130, 150, 255), upper_red_hue_range);

		// Combine the above two images
		Mat red_hue_image;
		addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
        GaussianBlur(red_hue_image,red_hue_image, Size(9,9), 2, 2);
        //Find Circles
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 30, 0,0 );

       // printf("circles.size = %d\n", circles.size());
	    //Draw detected circles
        for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
       //   printf("center x=%d y=%d \n",cvRound(circles[i][0]),cvRound(circles[i][1]));
          difference=320-cvRound(circles[i][0]);
        //  printf("difference %d \n",difference);
          if(difference < -120)
          {
			  right_flag = 1;
			//  printf(" right flag is 1\n");
		  }
          else if(difference > 120)
          {
			  left_flag =1;
			 // printf(" left flag is 1\n");
		  }
		  else
		  {
			  straight_flag = 1;
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
	

        char c = cvWaitKey(10);
        if( c == 27 ) break;
          clock_gettime(CLOCK_REALTIME, &finish_time);
		delta_t(&finish_time, &start_time, &thread_dt);
		//printf("\nimage processing thread WCET is %ld sec, %ld msec (%ld microsec)\n", thread_dt.tv_sec, (thread_dt.tv_nsec / NSEC_PER_MSEC), (thread_dt.tv_nsec / NSEC_PER_MICROSEC));
		rc = pthread_mutex_unlock(&lock);
		if(rc!= 0)
			handle_error("Mutex unlock");
	}
	 pthread_exit((void *)0);
}

void *motor_working(void *)
{
	while(e1)

	{
		  sem_wait(&semS2);
		  
		 struct timespec start_time = {0, 0};
    struct timespec finish_time = {0, 0};
    struct timespec thread_dt = {0, 0};
    rc = pthread_mutex_lock(&lock);
		if(rc!= 0)
			handle_error("Mutex lock");
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
		else if(straight_flag == 1)
		{
			straight();
			straight_flag = 0;
		
			
		}
		else
		{
			stop1();
			stop2();
		}
		clock_gettime(CLOCK_REALTIME, &finish_time);
		delta_t(&finish_time, &start_time, &thread_dt);
		//printf("\n motor thread WCET is %ld sec, %ld msec (%ld microsec)\n", thread_dt.tv_sec, (thread_dt.tv_nsec / NSEC_PER_MSEC), (thread_dt.tv_nsec / NSEC_PER_MICROSEC));
		rc = pthread_mutex_unlock(&lock);
		if(rc!= 0)
			handle_error("Mutex unlock");
	}
	 pthread_exit((void *)0);
}

int main()
{
	
	int rt_max_prio, i;
	//CvCapture* capture = (CvCapture *)cvCreateCameraCapture(0);
    //CvCapture* capture = (CvCapture *)cvCreateCameraCapture(argv[1]);
	e1=1;
	
	pthread_attr_t rt_sched_attr[NUM_THREADS];
	struct sched_param rt_param[NUM_THREADS];
	struct sched_param main_param;
	
	set_signal_handler();
	rc = pthread_mutex_init(&lock, NULL);
	if(rc!= 0)
		handle_error("Mutex init");
	
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
	rc = pthread_mutex_destroy(&lock);
	if(rc == -1)
		handle_error("mutex destroy");
	
	
	return 0; 
} 

	int pin_export(uint32_t pin)
	{
		FILE *fp;
	
		fp = fopen(GPIO_DIR "/export", "w");
		if(fp == NULL)
			return -1;
	
		fprintf(fp, "%d",pin);
	
		fclose(fp);
		return 0;
	} 
	
	int pin_dir(uint32_t pin, const char *dir)
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
	
	int write_value(uint32_t pin, uint32_t val)
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
	
	int read_value(uint32_t pin)
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

	void cw1()
	{
		//AIn1
		pin_export(17);
		pin_dir(17,"out");
		write_value(17, 1);
		
		//AIn2
		pin_export(27);
		pin_dir(27,"out");
		write_value(27, 0);
		
		//PWMA
		pin_export(22);
		pin_dir(22,"out");
		//gpio_set_value(22, 1);
		pwm_gen(22);
	}
	
	void stop1()
	{
		//AIn1
		pin_export(17);
		pin_dir(17,"out");
		write_value(17, 0);
		
		//AIn2
		pin_export(27);
		pin_dir(27,"out");
		write_value(27, 0);
		
		//PWMA
		pin_export(22);
		pin_dir(22,"out");
		//gpio_set_value(22, 1);
		pwm_gen(22);
	}
	
	void cw2()
	{
		//BIn1
		pin_export(26);
		pin_dir(26,"out");
		write_value(26, 1);
		
		//BIn2
		pin_export(19);
		pin_dir(19,"out");
		write_value(19, 0);
		
		//PWMB
		pin_export(13);
		pin_dir(13,"out");
		//gpio_set_value(13, 1);
		pwm_gen(13);
	}
	
	void stop2()
	{
		//BIn1
		pin_export(26);
		pin_dir(26,"out");
		write_value(26, 0);
		
		//BIn2
		pin_export(19);
		pin_dir(19,"out");
		write_value(19, 0);
		
		//PWMB
		pin_export(13);
		pin_dir(13,"out");
		//gpio_set_value(13, 1);
		pwm_gen(13);
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

int pwm_gen(uint32_t pin)
{

	blink_timer_init(pin);
} 

int gpio_blink_off(uint32_t pin)
{
	timer_delete(blink_timer_id);
	write_value(pin, 0);

	return 0;
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

	trigger.it_interval.tv_sec = 0;
	trigger.it_value.tv_sec = 0;
	trigger.it_interval.tv_nsec = ON_TIME;
	trigger.it_value.tv_nsec = ON_TIME;

	timer_settime(blink_timer_id, 0, &trigger, NULL);

}
void blink_timer_init1()
{
	struct sigevent sev;
	struct itimerspec trigger;
	memset(&sev, 0, sizeof(struct sigevent));
    memset(&trigger, 0, sizeof(struct itimerspec));

    sev.sigev_notify = SIGEV_THREAD;
	sev.sigev_notify_function = blink_timer_handle1;

	timer_create(CLOCK_REALTIME, &sev, &blink_timer_id1);

	

}
void blink_timer_handle(union sigval sv)
{	
	uint32_t led;
	static uint32_t count=0;

	led = read_value(sv.sival_int);
	if(count == 0)
	{
		write_value(sv.sival_int,0);
	}
	else
	{
		write_value(sv.sival_int,1);
	}
	count++;
	if (count == 5)
	{
		count = 0;
	}
	
}
void blink_timer_handle1(union sigval sv)
{	
	if(timeflag==1)
	{
		printf("handler");
		fflush(stdout);
	stop1();
	stop2();
	timeflag=0;
	}
}

void timeset()
{
		struct itimerspec trigger;
	trigger.it_interval.tv_sec = 0;
	trigger.it_value.tv_sec = 0;
	trigger.it_interval.tv_nsec = 1000000000;
	trigger.it_value.tv_nsec = 1000000000;

	timer_settime(blink_timer_id, 0, &trigger, NULL);
}

void set_signal_handler(void)
{
	struct sigaction action;
 
    action.sa_flags = SA_SIGINFO; 
    action.sa_sigaction = signal_handler;
 
    if (sigaction(SIGINT, &action, NULL) == -1)
		handle_error("SIGINT: sigaction")
}

void signal_handler(int signo, siginfo_t *info, void *extra) 
{	
	timer_delete(blink_timer_id);
	stop1();
	stop2();
	sem_close(&semS1);
	sem_close(&semS2);
	e1=0;
	pthread_cancel(threads[0]);
	pthread_cancel(threads[1]);
	pthread_cancel(threads[2]);

}
