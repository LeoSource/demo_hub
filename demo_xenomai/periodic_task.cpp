#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <sys/mman.h>
#include <alchemy/timer.h>
#include <math.h>

#define NTASKS 3
RT_TASK task[NTASKS];
// function to be executed by task

void task_function(void *arg)
{   RTIME now; 
    int i=1;
    
   int num = * (int *)arg;
   float period[]={5e6,2e7,3e7};
   //rt_printf("The arg is %d \n", num);
   
    rt_task_set_periodic(NULL, TM_INFINITE, period[num]);
   
   //开始任务循环
  while (1) {
    /*
      Do your thing here
     */
    
    RT_TASK_INFO curtaskinfo;
    rt_task_inquire(NULL,&curtaskinfo);
    rt_printf(" 第 %d 次循环 %s 任务 -- 循环时间： %.6f ms\n",i,curtaskinfo.name,(rt_timer_read()-now)/1000000.0);
    i=i+1;
   now = rt_timer_read();
   rt_task_wait_period(NULL);
   
  }
  return;
}

int main(int argc, char* argv[])
{
     int i;
     char  str[20] ;
     char  name[]={'A','B','C','D','E'};
     mlockall(MCL_CURRENT|MCL_FUTURE);//锁定内存以避免此程序的内存交换
      
     for(i=0; i < NTASKS; i++) 
      {
       //printf("start task  : %c\n",name[i]);
       sprintf(str,"task%d",i);
       rt_task_create(&task[i], str, 0, 50, 0);
       rt_task_start(&task[i], &task_function, &i);
       }
       
       pause();
}