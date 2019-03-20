/*
                                           *******************
********************************************* C SOURCE FILE *********************************************
**                                         *******************                                         **
**                                                                                                     **
**  Project             :    Preemptive/Cooperative RTOS                                               **
**  Filename            :    04_rtos.c                                                                 **
**  Version             :    1.0                                                                       **
**  Date                :    February 7th, 2019                                                        **
**  Target Platform     :    EK-TM4C123GXL Evaluation Board                                            **
**  Target uC           :    TM4C123GH6PMI                                                             **
**  IDE                 :    Code Composer Studio v7.4.0                                               **
**  System Clock        :    40 MHz                                                                    **
**  UART Baud Rate      :    115200                                                                    **
**                                                                                                     **
*********************************************************************************************************
**                                                                                                     **
**  Author : Hariharan Gopalakrishnan under the guidance of  Dr Jason H Losh , University of Texas at  **
**           Arlington.                                                                                **
**                                                                                                     **
**                                                                                                     **
**                                                                                                     **
**  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR                         **
**  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                           **
**  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE                        **
**  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                             **
**  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,                      **
**  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE                      **
**  SOFTWARE.                                                                                          **
**                                                                                                     **
**  Copyright (c) : 2019 , Hariharan Gopalakrishnan , Dr Jason H Losh                                  **
**  All rights reserved.                                                                               **
*********************************************************************************************************


// RTOS Framework - Spring 2019
// J Losh

// Student Name: Hariharan Gopalakrishnan
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

// Program has been formatted for viewing in Teraterm or VT100 based terminals

/*
 * Terminal command syntax:
 * 1) pi on/off
 * 2) schedule on/off
 * 3) rtos on/off
 * 4) reboot
 * 5) pidof <process_name>
 * 6) kill <pid>
 * 7) ipcs
 * 8) ps
 * 9) & <process_name>
 *
 */


//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Green:  PE2
// Yellow: PE3
// Orange: PE4
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
extern void ResetISR();
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>

#define MIN(x, y) (((x) < (y)) ? (x) : (y)) // Min of two priorities, used in priority inheritance

// REQUIRED: correct these bit-banding references for the off-board LEDs

#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    uint16_t currentUser;                  // last user of semaphore
    char sname[16];                        // name of semaphore
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource, *semaphoreptr; // Pointers to semaphore structure

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define MAX_Args 3         // max arguments
#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread
uint32_t sum = 0;   //Sum of cpu-time of all running tasks

//svc
#define svc_yield 1
#define svc_sleep 2
#define svc_wait  3
#define svc_post 4
#define svc_delete 5
uint8_t svc_number;

//shell
uint8_t argc;     // Argument count and general purpose variables.
uint8_t pos[MAX_Args];  // Position of arguments in input buffer.
char *commands[9] = {"pi","schedule","rtos","reboot","pidof","kill","ipcs","ps","&"}; // Currently available commands.
#define Buffer_Max 80  //Max size of input buffer
char str[20];
char input[Buffer_Max];      //Shell input buffer

//General purpose pointers
void *a;
void *sp_system;     // system stack pointer

//RTOS control bits
bool schedule = true;
bool pi = true;
bool rtos;

//Escape sequences for text-color  //Reference : https://en.wikipedia.org/wiki/ANSI_escape_code

#define bred "\033[22;91m"
#define bgreen "\033[22;92m"
#define bblue "\033[22;94m"
#define gray "\033[22;37m"

//Escape sequences for background-color

#define bgblack "\033[22;40m"

//Resets the color settings

#define Color_end "\033[0m"

//Clear terminal

#define clear0 "\033[0;J"
#define clear1 "\033[1;J"



struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *orig_pid;                // to identify task which got killed
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    uint8_t skip_count;            // For priority scheduling
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    uint32_t instime;              // Current cpu-usage time
    uint32_t iirtime;              // Averaged cpu-time
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
    // REQUIRED: initialize systick for 1ms system timer
    NVIC_ST_RELOAD_R |= 0x00009C3F;     // 1 millisecond i.e  N = 40,000  clock pulses ...loading N-1 ; 1khz timer
    NVIC_ST_CURRENT_R |= 0; //  Any value will clear it + count bit in CTRL_R

    //Initializing WideTimer 5 for 1Hz periodic interrupts to calculate cpu-usage

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;  //Turn on widetimer 5 in sysctrl block
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN; // Disabe widetimer for configuration
    WTIMER5_CFG_R = 4; // 32-bit mode
    WTIMER5_TAMR_R = TIMER_TAMR_TAMR_PERIOD; //Periodic mode
    WTIMER5_TAILR_R = 0x02625A00;  // 1 Hz timer.
    WTIMER5_IMR_R = TIMER_IMR_TATOIM;  // Interrupt for time-out events
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96); //Enable interrrupt in NVIC block

}


// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);

        // Priority Scheduler
        if(schedule && ok)
        {
            if(tcb[task].skip_count > 0)
            {
                tcb[task].skip_count--;
                ok &= false;

            }
            else
            {
               tcb[task].skip_count = tcb[task].currentPriority + 8;  // Adding bias of 8 because highest priority is -8
            }
        }
    }
    return task;
}

void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    _fn fn;
    sp_system = (void *)__get_MSP();          // Save system stack pointer
    taskCurrent = rtosScheduler();            // Call scheduler
    tcb[taskCurrent].state = STATE_READY;     // Mark task 0 as ready
    __set_MSP((uint32_t)tcb[taskCurrent].sp); // Initialize sp with the sp of task 0
    fn = (_fn)tcb[taskCurrent].pid;           // Initialize fn pointer with the pid of task 0
    tcb[taskCurrent].instime++;               // Increment cputime of task 0
    NVIC_ST_CTRL_R |= 0x00000007;                 // Enable systick Isr
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;          // turn-on widetimer 5 counter
    (*fn)();                                  // Call task 0
    // Add code to initialize the SP with tcb[task_current].sp;
}

bool createThread(_fn fn, char name[], int priority)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent re-entrance)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].orig_pid = fn;
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            tcb[i].skip_count = priority + 8;
            strncpy(tcb[i].name,name, sizeof(tcb[i].name)-1); //Reference : http://libslack.org/manpages/snprintf.3.html
            tcb[i].name[sizeof(tcb[i].name) - 1] = '\0';
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
  __asm(" SVC #5");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t k;
    for(k = 0; k < taskCount; k++)
    {
        if(tcb[k].pid == fn)
            break;
    }
    tcb[k].priority = priority;
    tcb[k].currentPriority = priority;
}

struct semaphore* createSemaphore(uint8_t count,char nam[])
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        strncpy(pSemaphore->sname,nam, sizeof(pSemaphore->sname)-1); //Include name of semaphore in structure
        pSemaphore->sname[sizeof(pSemaphore->sname) - 1] = '\0';
    }
    return pSemaphore;
}

// Function to store value in r0 onto a C variable
uint32_t getr0(void)
{
    __asm(" MOV r0,r0");
    __asm(" BX LR");
    return 0;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm(" SVC #1");

}


// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #2");

}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC #3");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #4");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
if(rtos == true)
        NVIC_INT_CTRL_R |= 0x10000000;      // Request task switch every 1ms in preemption mode
uint8_t cnt;
for(cnt = 0 ; cnt < taskCount; cnt++)   // Decrement tick from all delayed tasks
{
     if(tcb[cnt].state == 3)
     {
         if(tcb[cnt].ticks > 0)
             tcb[cnt].ticks--;
         else
           tcb[cnt].state = STATE_READY;
     }
}

}


// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
__asm(" ADD sp,#24");   // Compensate for change is sp produced by registers saved by hardware (r0-3,r12,pc,lr)
__asm(" PUSH {r4-r11}"); // Save registers r4-11
tcb[taskCurrent].sp = (void *)__get_MSP();  // Save sp in task structure
taskCurrent = rtosScheduler();           // Call next task
__set_MSP((uint32_t)sp_system);          // Move to system sp
if(tcb[taskCurrent].state == STATE_READY)          // READY tasks
{
__set_MSP((uint32_t)tcb[taskCurrent].sp); // Set sp as the taskcurrent's sp
__asm(" ADD sp,#8");
__asm(" POP {r4-r11}");
__asm(" SUB sp,#24");                     // Move to original value of sp when program entered the ISR
}
else if(tcb[taskCurrent].state == 1)      // Un-Run tasks
{
   tcb[taskCurrent].state = STATE_READY;
 __set_MSP(((uint32_t)tcb[taskCurrent].sp - 8)); // Set sp as sp of unrun task

 // Seeding stack to make it look it was interrupted

 // Push registers r0-3,xPSR,LR,PC,SP

__asm(" SUB sp,#0x24");                             // Push r0-r3
__asm(" ADD sp,#0x18");                             // Move sp to pc of assumed interrupted thread (sp + 0x18)
a = tcb[taskCurrent].pid;                          // Store the task pid in pc
__asm(" STR r0,[sp]");                             // Store thread's pid into pc
__asm(" ADD sp,#0x4");                             // Move sp to xPSR of assumed interrupted thread
__asm(" MOV r0,#0x01000000");
__asm(" orr r0,#0x0000200");
__asm(" STR r0,[sp]");                            // Store in xPSR  ( Thumb state + Thread mode)
__set_MSP(((uint32_t)tcb[taskCurrent].sp - 8));   // Set sp as sp of unrun task
__asm(" SUB sp,#0x24");                           // Pop registers
}

tcb[taskCurrent].instime++;

__asm(" mov lr,#0xFF000000");                      // LR has EXC_RETURN for return to thread mode
__asm(" orr lr,lr,#0x00FF0000");
__asm(" orr lr,lr,#0x0000FF00");
__asm(" orr lr,lr,#0x000000F9");
__asm(" BX LR");
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
     uint32_t svc_number,arg1;
     uint8_t i,j;
     void *temp;
     __asm(" ADD sp,#0x20");  // Compensate for local variables used
     __asm(" LDR  r0,[sp,#0x18]"); // Move sp to point to pc of interrupted kernal fn
     __asm(" LDRH r0,[r0,#-2]");    // Store half word(svc number)in r0
     __asm(" BIC  r0,r0,#0xFF00");  // Mask first two bytes
      svc_number = getr0();
     __asm(" LDR r0,[sp,#0x24]"); // Move the stack pointer to get the value of first argument passed onto the stack and store in r0
      arg1 = getr0();   // arg1 contains either sleep ticks in ms or pointer to semaphore structure in use or pid of task to be deleted

     // SVC handler
     switch(svc_number)
     {
     case svc_yield:
         NVIC_INT_CTRL_R |= 0x10000000;  // Request task switch
         break;
     case svc_sleep:
         tcb[taskCurrent].ticks = arg1;
         tcb[taskCurrent].state = STATE_DELAYED;
         NVIC_INT_CTRL_R |= 0x10000000;
         break;
     case svc_wait:
          semaphoreptr = (struct semaphore*)arg1;
          temp = (&semaphores); // Get the pointer to semaphore structure
          arg1 = arg1 - (uint32_t)temp; // Pointer to the semaphore passed in svc
          arg1 = arg1 / 41;     // Semaphore index
          if(semaphoreptr->count > 0) // Semaphore available
          {
             semaphoreptr->count--;
             semaphoreptr->currentUser = taskCurrent; // Note last user of semaphore
             tcb[taskCurrent].semaphore = NULL;

          }
          else
          {
              semaphoreptr->processQueue[semaphoreptr->queueSize] = taskCurrent; // Add Current task to process queue
              tcb[taskCurrent].state = STATE_BLOCKED;
              tcb[taskCurrent].semaphore = (void *)arg1; //Record blocking semaphore
              semaphoreptr->queueSize++;
              NVIC_INT_CTRL_R |= 0x10000000;

              // Priority Inheritance

              if(pi) // Allow lengthfn to be temporarily be hoisted to highest priority incase of PI.
              {
                  if(tcb[semaphoreptr->currentUser].priority > tcb[taskCurrent].priority)
                  {
                      tcb[semaphoreptr->currentUser].currentPriority = MIN(tcb[semaphoreptr->currentUser].priority,tcb[taskCurrent].priority);
                      tcb[semaphoreptr->currentUser].skip_count = tcb[semaphoreptr->currentUser].currentPriority + 8;

                  }
              }
          }
           break;
     case svc_post:
              semaphoreptr = (struct semaphore*)arg1;
              semaphoreptr->count++;

             if(tcb[taskCurrent].currentPriority != tcb[taskCurrent].priority) // Incase PI happened, restore priority to original value
             {
                 tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
                 tcb[taskCurrent].skip_count = tcb[taskCurrent].currentPriority + 8;
             }

              if(semaphoreptr->count == 1)
              {
                  if(semaphoreptr->queueSize > 0)
                  {

                      tcb[semaphoreptr->processQueue[0]].state = STATE_READY; // Mark waiting task as ready
                      semaphoreptr->count--;
                      semaphoreptr->currentUser = semaphoreptr->processQueue[0];
                      for(i=0;i<((semaphoreptr->queueSize)-1);i++) // Move other waiting task up the queue.
                      {
                          semaphoreptr->processQueue[i] = semaphoreptr->processQueue[i+1];

                      }
                      semaphoreptr->processQueue[semaphoreptr->queueSize] = 0;
                      semaphoreptr->queueSize--;
                  }
              }
              break;
     case svc_delete:
         for(i = 0 ; i < MAX_TASKS ; i++ )
         {
             if(tcb[i].pid == (void *)arg1) // Find pid of task to be deleted
                 break;
         }
         if(tcb[i].state == STATE_BLOCKED) // Remove task from semaphore wait queues
         {
             for(j = 0 ; j < semaphores[(uint32_t)tcb[i].semaphore].queueSize;j++)
             {
                 if(semaphores[(uint32_t)tcb[i].semaphore].processQueue[j] == i)
                     break;
             }
             semaphores[(uint32_t)tcb[i].semaphore].queueSize--;
             for(;j<semaphores[(uint32_t)tcb[i].semaphore].queueSize;j++)
             {
                 semaphores[(uint32_t)tcb[i].semaphore].processQueue[j] = semaphores[(uint32_t)tcb[i].semaphore].processQueue[j+1];
             }

         }
         tcb[i].state = STATE_INVALID;
         tcb[i].pid = 0;
         taskCount--;
         break;
     }
     __asm(" mov lr,#0xFF000000");
     __asm(" orr lr,lr,#0x00FF0000");
     __asm(" orr lr,lr,#0x0000FF00");
     __asm(" orr lr,lr,#0x000000F9");
     __asm(" BX LR");
}

// ISR to perform IIR filtering on cpu-time of tasks (1 Hz)

void wideTimer5Isr()
{
static bool firstUpdate = true;
uint8_t b;
sum = 0;
// Filter co-efficient is 0.8
if (firstUpdate)
{
    for(b = 0 ; b < taskCount ; b++)
    {
       tcb[b].iirtime = tcb[b].instime; // Let initial value be first sample
       sum += tcb[b].iirtime;
       tcb[b].instime = 0;
    }
firstUpdate = false;
}
else
{
    for(b = 0 ; b < taskCount ; b++)
    {
        tcb[b].iirtime = (uint32_t)(tcb[b].iirtime * 0.2 + tcb[b].instime * 0.8); // IIR filter
        sum += tcb[b].iirtime;
        tcb[b].instime = 0;
    }
}
WTIMER5_ICR_R |= TIMER_ICR_TATOCINT;
}
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Initialize Hardware
void initHw()
{
  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart
               SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

            // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
               SYSCTL_GPIOHBCTL_R = 0;

            // Enable GPIO port F peripherals
               SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOA;

            // Configure LEDs

               GPIO_PORTF_DIR_R |= 0x04;
               GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
               GPIO_PORTF_DEN_R |= 0x04;

               GPIO_PORTE_DIR_R |= 0x1E;
               GPIO_PORTE_DR2R_R |= 0x1E;
               GPIO_PORTE_DEN_R |= 0x1E;

            // Configure PBs

               GPIO_PORTA_DEN_R |= 0x7C;
               GPIO_PORTA_PUR_R |= 0x7C;

            // Configure UART0 pins

               SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
               GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
               GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
               GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

            // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)

               UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
               UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
               UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
               UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
               UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
               UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                              // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    return (((((GPIO_PORTA_DATA_R << 2) & 0x1F0)>>2)^(0x7C))>>2);   // Reading only bits 2-6 of GPIO A register, memory mapped onto the address bus
}

//-----------------------------------------------------------------------------
// UART subroutines
//-----------------------------------------------------------------------------

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while ((UART0_FR_R & UART_FR_TXFF)!=0) //Blocking but can yield while it's blocking
    {
        yield();
    }
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while ((UART0_FR_R & UART_FR_RXFE)!=0)  //Blocking but can yield while it's blocking
    {
        yield();
    }
    return UART0_DR_R & 0xFF;
}


// Function to get a command from user through UART interface //Reference: My 5314 project Fall-2018 (LCR-meter)

void getsUart0()
{
     uint8_t c;
     uint8_t count = 0;
l1:  c = getcUart0();
     putcUart0(c);
     if ((c == 0x8) & (count == 0))  // Checking for backspace and if it is the first entry.
         goto l1;

     else if ((c == 0x8) & (count > 0))
        {
         count --;
         goto l1;
        }
     else
     {
         if(c == 0x0D) //  Enter indicates end of command entry.
         {
          input[count] = '\0';
          return;
         }
         else
         {
            if(c < 0x20)  // Check for printable characters.
                goto l1;
            else
               {
                input[count++] = c;
                if (count > Buffer_Max)     // Checking if buffer is full.
                {
                    putsUart0(bred"\nStop!Buffer has overflowed!Try again\r\n\r\n");
                    waitMicrosecond(1000);
                    return;
                }
                else
                    goto l1;
               }
         }
     }
}

// Function to pass the pid of the thread to be killed into destroythread kernal fn
void kill()
{
    uint8_t i;
    bool c = 0;
    _fn fn;
    for(i = 0;i<taskCount;i++)
    {
        snprintf(str,sizeof str,"%p",tcb[i].pid);//Reference : http://libslack.org/manpages/snprintf.3.html
        if(strcasecmp(&input[pos[1]],str)==0)
        {
            c = 1;
            break;
        }

    }
    if(c == 1 && tcb[i].pid!=0)
    {
    fn = (_fn)tcb[i].pid;
    destroyThread(fn);
    }
    else
        putsUart0(bred"Task doesn't exist\r\n");
}

// Function to move cursor to a particular coordinate on the terminal

void moveCursor(uint8_t i , uint8_t j)
{
    char l[20];
    snprintf(l, sizeof l, "%s%d%s%d%s","\033[",i,";",j,"H"); //Reference : http://libslack.org/manpages/snprintf.3.html
    putsUart0(l);
}

// Function to display the pid of a process

void pidThread()
{
    uint8_t r;
    bool c = 0;
    for(r = 0 ; r < MAX_TASKS ; r++)
    {
        if(strcasecmp(&input[pos[1]],tcb[r].name)==0)
        {
        c = 1;
        break;
        }
    }
    if(c)
    {
    snprintf(str,sizeof str,"%p",tcb[r].pid); //Reference : http://libslack.org/manpages/snprintf.3.html
    putsUart0(bgreen);
    putsUart0(str);
    putsUart0("\r\n");
    }
    else
        putsUart0(bred"Thread doesn't exist\r\n");
}

// Function to display semaphore status

void ipcs()
{
putsUart0("    Name     | Count |    User    |    Waiting-Task   \r\n");
putsUart0("-------------|-------|------------|-------------------\r\n");
uint8_t i;
for( i = 0 ; i < MAX_SEMAPHORES-1 ; i++)
{
    putsUart0(semaphores[i].sname);
    putsUart0(" \t");
    snprintf(str,sizeof str,"%d",semaphores[i].count);//Reference : http://libslack.org/manpages/snprintf.3.html
    putsUart0(str);
    putsUart0(" \t");
    if(semaphores[i].currentUser == 0)
        putsUart0(" None \t\t");
    else
    {
        putsUart0(tcb[semaphores[i].currentUser].name);
        putsUart0(" \t");
    }
    if(semaphores[i].processQueue[0] == 0)
        putsUart0(" None \t");
    else
    {
        putsUart0(tcb[semaphores[i].processQueue[0]].name);
        putsUart0(" \t");
    }
    putsUart0("\r\n");
}
}

// Function to display status of all running processes (% cpu updated every second)

void ps()
{
   uint32_t cpu = 0;
   uint32_t cpu1 = 0;
   putsUart0("    Name     |     PID     |    Priority    |    %CPU      \r\n");
   putsUart0("-------------|-------------|----------------|--------------\r\n");
   uint8_t i = 0;
   for(i = 0 ; (i < taskCount); i++)
   {
       if(tcb[i].pid == 0)
           continue;
       putsUart0(tcb[i].name);
       putsUart0("     \t");
       snprintf(str,sizeof str,"%p",tcb[i].pid); //Reference : http://libslack.org/manpages/snprintf.3.html
       putsUart0(str);
       putsUart0("    \t   ");
       snprintf(str,sizeof str,"%d",tcb[i].priority);
       putsUart0(str);
       putsUart0("    \t");
       if(rtos)
       {
           cpu1 = (tcb[i].iirtime*100) / sum;
           snprintf(str,sizeof str,"%d",cpu1);
           putsUart0(str);

       }
       else
       {
       cpu = (tcb[i].iirtime*1000) / sum; // Scaled cpu-time
       snprintf(str,sizeof str,"%d.%.2d",cpu / 10,cpu % 10); //Reference : https://stackoverflow.com/questions/18465943/printf-how-to-insert-decimal-point-for-integer.
       putsUart0(str);
       }
       putsUart0(" \r\n");
   }
}

// Function to restore a killed process

void startThread()
{
   uint8_t i;
   bool fnd = false;
   for(i = 0 ; i < MAX_TASKS ; i++)
   {
       if(strcasecmp(&input[pos[1]],tcb[i].name)==0)
       {
           fnd = true;
           break;
       }
   }
   if(fnd)
       createThread((_fn)tcb[i].orig_pid,tcb[i].name,0);    // Restore task //createThread has re-entrance preventon
   else
       putsUart0(bred"Thread doesn't exist");

}

// Parse input command from user into arguments //Reference: My 5314 project Fall-2018 (LCR-meter)


void parseInput()
{
uint8_t j = 0;
uint8_t i = 0;
uint8_t len = strlen(input);
if(isspace(input[0])) // Checking for space or punctuation in first entry
    input[0] = '\0';                        // Replacing all delimiters with null.
else if(isalpha(input[0]))                  // Check for alphabets
{
    argc++;                                 // Update argc and pos
    pos[j++] = 0;

}
else
{
    argc++;
    pos[j++] = 0;

}

for(i=1;i<len;i++)     // Loop to parse the entries after the first entry.
{
if(isspace(input[i]))
{
    input[i] = '\0';
if(isalpha(input[i+1]))
{   argc++;
    pos[j++] = i+1;

}
else if(isdigit(input[i+1]))
{
    argc++;
    pos[j++] = i+1;

}
else
    continue;
}
}
}


// Function to process the input buffer

void isCommand() //Reference: My 5314 project Fall-2018 (LCR-meter)
{
    uint8_t i;
    bool ch = false;
    for(i=0;i<9;i++)
    {
    if(strcasecmp(&input[pos[0]],commands[i])==0)
    {
        switch(i+1)
        {
        case 1:
            if(strcasecmp(&input[pos[1]],"on")==0)
            {
                pi = 1;
                return;
             }
            else if(strcasecmp(&input[pos[1]],"off")==0)
            {
                pi = 0;
                return;
            }
            else
                putsUart0(bred"Invalid argument for priority inheritance\r\n");
            break;
        case 2:
            if(strcasecmp(&input[pos[1]],"on")==0)
            {
                schedule = 1;
                return ;
            }
            else if(strcasecmp(&input[pos[1]],"off")==0)
            {
                 schedule = 0;
                 return ;
            }
            else
                 putsUart0(bred"Invalid argument for priority scheduling\r\n");
            break;
        case 3:
            if(strcasecmp(&input[pos[1]],"on")==0)
            {
                rtos = 1;
                schedule = 0;
                return;
            }
            else if(strcasecmp(&input[pos[1]],"off")==0)
            {
                rtos = 0;
                ResetISR();
                return;
            }
            else
                putsUart0(bred"Invalid argument for preemption\r\n");
            break;
        case 4:
            rtos = 0;
            schedule = 0;
            ResetISR();
            break;
        case 5:
            pidThread();
            ch = true;
            break;
        case 6:
            kill();
            ch = true;
            break;
        case 7:
            ipcs();
            ch = true;
            break;
        case 8:
            ps();
            ch = true;
            break;
        case 9:
            startThread();
            ch = true;
            break;
        default:
            putsUart0(bred"Invalid Command!\r\n");
            break;
        }
     }
    }
    if(ch == false)
        putsUart0(bred"Command not supported!! Try again !! \r\n");  // Operation not supported by program
    return;
}
//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}


void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);

        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);

        }

        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}
void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void shell()
{
    putsUart0(clear1);
    moveCursor(0,0);
    putsUart0(bgblack);
    while (true)
    {
        argc = 0;
        putsUart0(gray"Enter a command!!\r\n\n"bgreen);
        putsUart0(bblue);
        getsUart0();
        parseInput();
        putsUart0(bgreen"\r\n\r\n");
        isCommand();
        putsUart0("\r\n");
        yield();
        // REQUIRED: add processing for the shell commands through the UART here
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    bool ok;
    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1,"keyPressed");
    keyReleased = createSemaphore(0,"keyReleased");
    flashReq = createSemaphore(5,"flashReq");
    resource = createSemaphore(1,"resource");


    // Add required idle processes at lowest priority
     ok =  createThread(idle, "Idle", 7);
    // Add other processes
     ok &= createThread(lengthyFn, "Lengthyfn", 4);
     ok &= createThread(flash4Hz, "Flash4Hz", 0);
     ok &= createThread(oneshot, "Oneshot", -4);
     ok &= createThread(readKeys, "Readkeys", 4);
     ok &= createThread(debounce, "Debounce", 4);
     ok &= createThread(important, "Important", -8);
     ok &= createThread(uncooperative, "Uncoop", 2);
     ok &= createThread(shell, "Shell", 0);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}

