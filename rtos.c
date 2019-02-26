// RTOS Framework - Spring 2019
// J Losh

// Student Name: Hariharan Gopalakrishnan
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

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

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
//#include <string.h>
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
    uint16_t currentUser;
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread
uint8_t svc_number;
bool rtos = true;
bool pi = true;
void *a;
void *sp_system;
#define svc_yield 1
#define svc_sleep 2
#define svc_wait  3
#define svc_post 4

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    uint16_t skip_count;           // For priority scheduling
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
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
    NVIC_ST_RELOAD_R |= 39999;     // 1 millisecond i.e  N = 40,000  clock pulses ...loading N-1 ; 1khz timer
    NVIC_ST_CURRENT_R |= 0x01; //  Any value will clear it + count bit in CTRL_R
    NVIC_ST_CTRL_R  |= 0x07;  //   System clock + with interrupt + Multi-shot mode.
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
        if(ok)
        {
           if(tcb[task].state == 1 || rtos == false)
               return task;
           else
           {
               if(tcb[task].skip_count == 0)
               {
                  tcb[task].skip_count = tcb[task].currentPriority + 8;
                  return task;
               }
               else
                  tcb[task].skip_count--;
           }
           ok = false;
         }

     }
    //return task;
}

void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    _fn fn;
    sp_system = (void *)__get_MSP();
    taskCurrent = rtosScheduler();
    tcb[taskCurrent].state = STATE_READY;
    __set_MSP((uint32_t)tcb[taskCurrent].sp);
    fn = (_fn)tcb[taskCurrent].pid;
    (*fn)();
    // Add code to initialize the SP with tcb[task_current].sp;
}

bool createThread(_fn fn, char name[], int priority)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list                 //Add name to task list if not full?
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent re-entrance)        // found = true if name already in list ?
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
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
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
/*void destroyThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}
*/
struct semaphore* createSemaphore(uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;

    }
    return pSemaphore;
}

void* getr0(void)
{
    __asm(" MOV r0,r0");
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
uint16_t cnt ; // Since idle will never be delayed
//2 bytes for 8 tasks.
// Find all delayed tasks
//Reduce the ticks of all delayed tasks
for(cnt = 1 ; cnt < MAX_TASKS ; cnt++)     //For all tasks
{
     if(tcb[cnt].state == 3)
     {
         if(tcb[cnt].ticks == 0)
         {
           tcb[cnt].state = 2;
           continue;
         }
         tcb[cnt].ticks--;
      }
}

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
__asm(" ADD sp,#24");
__asm(" PUSH {r4-r11}");
tcb[taskCurrent].sp = (void *)__get_MSP();
taskCurrent = rtosScheduler();
__set_MSP((uint32_t)sp_system);

if(tcb[taskCurrent].state == 2)
{
__set_MSP((uint32_t)tcb[taskCurrent].sp);
__asm(" ADD sp,#8");
__asm(" POP {r4-r11}");
__asm(" SUB sp,#24");
}
else if(tcb[taskCurrent].state == 1)
{
   tcb[taskCurrent].state = 2;
 __set_MSP(((uint32_t)tcb[taskCurrent].sp - 8));
__asm(" SUB sp,#0x24"); //Now it is an exception handler
__asm(" MOV r0,#0x00000000");
__asm(" ADD sp,#0x14");
__asm(" STR r0,[sp]");
__asm(" ADD sp,#0x4");
a = tcb[taskCurrent].pid;
__asm(" STR r0,[sp]");
__asm(" ADD sp,#0x4");
__asm(" MOV r0,#0x41000000");
__asm(" orr r0,#0x00000200");
__asm(" STR r0,[sp]");
__set_MSP(((uint32_t)tcb[taskCurrent].sp - 8));
__asm(" SUB sp,#0x24");

}

__asm(" mov lr,#0xFF000000");
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
     void *temp; uint8_t i;
     __asm(" ADD sp,#0x20");  //Update number as per number of variables used
     __asm(" LDR  r0,[sp,#0x18]");
     __asm(" LDRH r0,[r0,#-2]");
     __asm(" BIC  r0,r0,#0xFF00");
      svc_number = (uint32_t)getr0();
     __asm(" LDR r0,[sp,#0x24]");
      arg1 = (uint32_t)getr0();
     //__asm(" LDR r0,[sp,#0x48]");
     //  arg2 = (uint32_t)getr0();
     switch(svc_number)
     {
     case svc_yield:
         NVIC_INT_CTRL_R = 0x10000000;
         break;
     case svc_sleep:
         tcb[taskCurrent].ticks = arg1;
         tcb[taskCurrent].state = STATE_DELAYED;
         NVIC_INT_CTRL_R = 0x10000000;
         break;
     case svc_wait:
          temp = (&semaphores);
          arg1 = arg1 - (uint32_t)temp;
          arg1 = arg1 / 28;
          if(semaphores[arg1].count > 0)
          {
             semaphores[arg1].count--;
             semaphores[arg1].currentUser = taskCurrent;

          }
          else
          {
             if(semaphores[arg1].processQueue[(semaphores[arg1].queueSize)-1] != taskCurrent) // Improve this.
             {
                 semaphores[arg1].processQueue[semaphores[arg1].queueSize] = taskCurrent;
                 tcb[taskCurrent].state = STATE_BLOCKED;
                 tcb[taskCurrent].semaphore = (void *)arg1;
                 semaphores[arg1].queueSize++;
                 NVIC_INT_CTRL_R = 0x10000000;
             }
             if(tcb[taskCurrent].state == STATE_BLOCKED && pi == true)
             {
                 if(tcb[semaphores[arg1].currentUser].priority > tcb[taskCurrent].priority)
                 {
                     tcb[semaphores[arg1].currentUser].currentPriority = MIN(tcb[semaphores[arg1].currentUser].priority,tcb[taskCurrent].priority);

                 }
             }

          }
           break;
     case svc_post:
              temp = (&semaphores);
              arg1 = arg1 - (uint32_t)temp;
              arg1 = arg1 / 28;
              semaphores[arg1].count++;
              tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
              if(semaphores[arg1].count == 1)
              {
                  if(semaphores[arg1].queueSize > 0)
                  {
                      tcb[semaphores[arg1].processQueue[0]].state = 2;
                      semaphores[arg1].queueSize--;
                      for(i=0;i<((semaphores[arg1].queueSize)-1);i++)
                      {
                          semaphores[arg1].processQueue[i] = semaphores[arg1].processQueue[i+1];

                      }
                      semaphores[arg1].processQueue[semaphores[arg1].queueSize] = 0;

                  }
              }
              break;
     }
     __asm(" mov lr,#0xFF000000");
     __asm(" orr lr,lr,#0x00FF0000");
     __asm(" orr lr,lr,#0x0000FF00");
     __asm(" orr lr,lr,#0x000000F9");
     __asm(" BX LR");
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
               SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOA;

            // Configure LEDs

               GPIO_PORTF_DIR_R = 0x04;
               GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
               GPIO_PORTF_DEN_R = 0x04;

               GPIO_PORTE_DIR_R = 0x1E;
               GPIO_PORTE_DR2R_R = 0x1E;
               GPIO_PORTE_DEN_R = 0x1E;

            // Configure PBs

               GPIO_PORTA_DEN_R = 0x7C;
               GPIO_PORTA_PUR_R = 0x7C;

            // Configure UART0 pins

               SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
               GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
               GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
               GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

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
    return (((((GPIO_PORTA_DATA_R << 2) & 0x1F0)>>2)^(0x7C))>>2);
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
            //destroyThread(flash4Hz);

        }
        if ((buttons & 16) != 0)
        {
            //setThreadPriority(lengthyFn, 4);

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
/*
void shell()
{
    while (true)
    {
        // REQUIRED: add processing for the shell commands through the UART here
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
*/
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
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);


    // Add required idle processes at lowest priority // Order was changed
    ok =  createThread(idle, "Idle", 7);
    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 4);
    ok &= createThread(important, "Important", -8);
    ok &= createThread(flash4Hz, "Flash4Hz", 0);
    ok &= createThread(oneshot, "OneShot", -4);
    ok &= createThread(readKeys, "ReadKeys", 4);
    ok &= createThread(debounce, "Debounce", 4);
    ok &= createThread(uncooperative, "Uncoop", 2);
    //ok &= createThread(shell, "Shell", 0);


    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
