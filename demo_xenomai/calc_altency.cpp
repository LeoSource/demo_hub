#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <rtdm/gpio.h>
#include <signal.h>
#include <alchemy/task.h>
#include <sys/mman.h>
#include <alchemy/timer.h>
#include <math.h>
#include <alchemy/sem.h>

unsigned int nsamples=10000;
unsigned long long wr[10000];
unsigned long long re[10000];
RT_SEM sem1;
RT_SEM sem2;
RT_TASK task;
unsigned long long average;
RT_TASK task0,task1,task2;
RTIME USEC = 1000llu;
// LED闪烁
void dem0(void *arg) 
{
    RTIME now;
    int fd,ver,ret,low=0;
    unsigned long long  i=0;
    ver=1;
    fd=open("/dev/rtdm/pinctrl-bcm2835/gpio22",O_TRUNC|O_RDWR,S_IRUSR|S_IWUSR);//O_TRUNC:新写入的覆盖以前的值,O_RDWR:设置此引脚为可读可写模式，并获得文件句柄.S_IRUSR|S_IWUSR:用户具有读与写的权限.
    ret=ioctl(fd, GPIO_RTIOC_DIR_OUT| GPIO_RTIOC_DIR_IN, &low); //将管口设置为输入输出模式 ,并初始化为低电平 
	
    rt_task_sleep(1e9);
    rt_task_set_periodic(NULL, TM_INFINITE, 1e5);//100us的周期
     
    while(i<nsamples) //循环写入
    {
        ver=~ver;
    
        RT_TASK_INFO curtaskinfo;
        rt_task_inquire(NULL,&curtaskinfo);
        now = rt_timer_read()/1.0;
        ret=write(fd, &ver, sizeof(ver));   //点灯
        //rt_printf(" 第 %llu 次循环 --  时间： %.5f ns\n",i,now/1.0);//实时任务中用rt_printf
        
        wr[i]=now/1.0;
        i=i+1;
        
        rt_task_wait_period(NULL);
    }
	rt_sem_v(&sem1);
}
void dem1(void *arg) 
{
	rt_task_sleep(1e9);
	RTIME now; 
	int fd,ret,value;
    unsigned long long count=0;

    fd=open("/dev/rtdm/pinctrl-bcm2835/gpio24",O_RDONLY);
    int xeno_trigger=GPIO_TRIGGER_EDGE_RISING | GPIO_TRIGGER_EDGE_FALLING;//上升与下降沿同时触发
    ret=ioctl(fd, GPIO_RTIOC_IRQEN, &xeno_trigger);//使相应的端口与中断联系起来,并启用中断

    //read函数的等待被告知是否发生中,没发生则等待,发生了则传回数据
    ret=read(fd, &value, sizeof(value));
	now = rt_timer_read()/1.0;
    //rt_printf(" %d\n",value);
    if(ret)
	{   
        re[count]=now/1.0;
        count=count+1;
        //rt_printf(" %d\n",count);//打印中断发生次数
	}
}
unsigned long long calc_average_time(unsigned int number,RTIME *time_values)
{	
    rt_sem_p(&sem2,TM_INFINITE);
	unsigned long long add=0;
	unsigned int i=0;
	
	while(i< (number-1))
	{
        add=add+time_values[i];
        i=i+1;
	}
	add=add/(number-1);
	return add;		
}
void calc_time_diffs(RTIME *time_diff)
{	
	rt_sem_p(&sem1,TM_INFINITE);
	unsigned long long int i=0;
	while(i<(nsamples-1))
	{
        time_diff[i]=(re[i]-wr[i])/1000;
        //printf(" 第 %llu 个数 --  时间： %.5f us\n",i,time_diff[i]/1.0);//非实时内核用printf,
        i=i+1;
	}
	rt_sem_v(&sem2);
}
void write_RTIMES(char * filename, unsigned int number_of_values,
    RTIME *time_values)
{
    unsigned int n=0;
    FILE *file;
    file = fopen(filename,"w");
    while (n<number_of_values) {
        fprintf(file,"%u,%llu\n",n,time_values[n]);
        n++;
    } 
    fclose(file);
}
void doWork(void *arg)
{
    printf("Start working \n");
    while (1) {
        rt_task_sleep(200*USEC);
        printf(".");  
        rt_timer_spin(500*USEC); 
    }
}
int main(int argc, char* argv[])
{
  	int i=0;
	RTIME *time_diff;
	
	rt_sem_create(&sem1,"caculate",0,S_FIFO);
	rt_sem_create(&sem2,"average",0,S_FIFO);
	rt_task_create(&task0, "led_blink", 0, 50, 0);//写周期函数
	rt_task_start(&task0, &dem0, 0);
	
	rt_task_create(&task1, "interrupt", 0, 80, 0);//中断读取函数
	rt_task_start(&task1, &dem1, 0);

	rt_task_create(&task2, "Worker", 0, 20, 0);
	rt_task_start(&task2, &doWork, 0);    

    time_diff = calloc(nsamples, sizeof(RTIME));
    calc_time_diffs(time_diff);
    write_RTIMES("time_diff.csv",(nsamples-2),time_diff);
    average=calc_average_time(nsamples,time_diff);
    printf("Average time difference %llu\n", average);
       
	pause();
}