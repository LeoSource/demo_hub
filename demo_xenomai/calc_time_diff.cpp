#include <stdio.h>
#include<stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <sys/mman.h>
#include <alchemy/timer.h>
#include <math.h>
#include <alchemy/sem.h>

unsigned int nsamples=10000;
unsigned long long value[10000];
RT_SEM sem1;
RT_SEM sem2;//设置信号量以阻塞Linux程序
RT_TASK task;
unsigned long long average;
// function to be executed by task

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
unsigned long long calc_average_time(unsigned int number,RTIME *time_values)//计算平均值
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
void calc_time_diffs(RTIME *time_diff)//计算差值抖动
{	
	rt_sem_p(&sem1,TM_INFINITE);
	unsigned long long int i=0;
	while(i<(nsamples-1))
	{
        time_diff[i]=(value[i+1]-value[i])/1000;
        //printf(" 第 %llu 个数 --  时间： %.5f us\n",i,time_diff[i]/1.0);//非实时内核用printf,
        i=i+1;
	}
	rt_sem_v(&sem2);
     
}
void task_function(void *arg)//实时周期任务
{   
    RTIME now; 
    unsigned long long  i=0;
    
    rt_task_set_periodic(NULL, TM_INFINITE, 1e5);//100us的周期

   //开始任务循环
    while (i < (nsamples)) 
    {
        now = rt_timer_read()/1.0;
        RT_TASK_INFO curtaskinfo;
        rt_task_inquire(NULL,&curtaskinfo);
        //rt_printf(" 第 %llu 次循环 --  时间： %.5f ns\n",i,now/1.0);//实时任务中用rt_printf
        value[i]=now/1.0;
        i=i+1;
        rt_task_wait_period(NULL);
    }
    rt_sem_v(&sem1);
    return;
}

int main(int argc, char* argv[])
{      
    char str[10];
	RTIME *time_diff;
	rt_sem_create(&sem1,"caculate",0,S_FIFO);
	rt_sem_create(&sem2,"average",0,S_FIFO);
    sprintf(str,"task");
    rt_task_create(&task, str, 0, 50, 0);
    rt_task_start(&task, &task_function, 0);
    
    time_diff = calloc(nsamples, sizeof(RTIME));
    calc_time_diffs(time_diff);
    write_RTIMES("time_diff.csv",(nsamples-2),time_diff);
    average=calc_average_time(nsamples,time_diff);
    printf("Average time difference %llu\n", average); 
    
    pause();
}