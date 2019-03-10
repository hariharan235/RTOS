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
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>
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

extern void ResetISR();
// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    uint16_t currentUser;
    char sname[16];
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
#define MAX_Args 3
#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread
uint8_t svc_number;
uint8_t argc;     // Argument count and general purpose variables.
uint8_t pos[MAX_Args];  // Position of arguments in input buffer.
char type [MAX_Args];
char* commands[8] = {"pi","schedule","rtos","reboot","pid","kill","ipcs","ps"}; // Currently available commands.
bool schedule;
bool pi;
bool rtos;
void *a;
void *sp_system;
#define svc_yield 1
#define svc_sleep 2
#define svc_wait  3
#define svc_post 4
#define svc_delete 5

//Escape sequences for text-color

#define black "\033[22;30m"
#define red "\033[22;31m"
#define green "\033[22;32m"
#define blue "\033[22;34m"
#define gray "\033[22;37m"
#define yellow "\033[01;33m"
#define white "\033[01;37m"

//Escape sequences for background-color

#define bgblack "\033[22;40m"
#define bgred "\033[22;41m"
#define bggreen "\033[22;42m"
#define bgblue "\033[22;44m"
#define bggray "\033[22;47m"
#define bgyellow "\033[01;43m"
#define bgwhite "\033[01;47m"

//Resets the color settings

#define Color_end "\033[0m"

//Clear terminal

#define clear0 "\033[0;J"
#define clear1 "\033[1;J"
#define clear2 "\033[2;J"
#define clearline "\033[1;K"

//Cursor Navigation sequences


#define up "\033[1;A"
#define down "\033[1;B"
#define left "\033[1;D"
#define right "\033[1;C"

//Saved cursor positions

#define topleft "\033[1;1H"
#define topright "\033[1;100H"
#define frstcomm "\033[3;3H"

//Report Cursor

#define rep "\033[6n"

#define Buffer_Max 80

char str[20];
char input[Buffer_Max];
char scrollbuffer[4][Buffer_Max];
uint8_t l = 0;// Scroll-back write pointer
uint8_t ctr = 1;
int8_t m = 0; // Scroll-back read pointer
const char *arr_buffer[] = {"[a","[b","[c","[d"};

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
        if(tcb[task].state == STATE_UNRUN)
            break;
        if(schedule)
        {
            if(tcb[task].skip_count == 0)
            {
                tcb[task].skip_count = tcb[task].currentPriority + 8;
                ok &= true;
            }
            else
            {
                tcb[task].skip_count--;
                ok &= false;
            }
        }
    }
    return task;
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
    NVIC_ST_CTRL_R  |= 0x07;  //   System clock + with interrupt + Multi-shot mode.
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
            tcb[i].skip_count = priority + 8;
            strncpy(tcb[i].name,name, sizeof(tcb[i].name)-1);
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
    uint16_t k;
    for(k = 0; k < MAX_TASKS; k++)
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
        strncpy(pSemaphore->sname,nam, sizeof(pSemaphore->sname)-1);
        pSemaphore->sname[sizeof(pSemaphore->sname) - 1] = '\0';
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
uint16_t cnt = 0 ; // Since idle will never be delayed
//2 bytes for 8 tasks.
// Find all delayed tasks
//Reduce the ticks of all delayed tasks

for(cnt = 0 ; cnt < MAX_TASKS ; cnt++)     //For all tasks
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
if(rtos == true)
    NVIC_INT_CTRL_R |= 0x10000000;
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
     void *temp; uint8_t i,j;
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
         NVIC_INT_CTRL_R |= 0x10000000;
         break;
     case svc_sleep:
         tcb[taskCurrent].ticks = arg1;
         tcb[taskCurrent].state = STATE_DELAYED;
         NVIC_INT_CTRL_R |= 0x10000000;
         break;
     case svc_wait:
          temp = (&semaphores);
          arg1 = arg1 - (uint32_t)temp;
          arg1 = arg1 / 41;  //Change here
          if(semaphores[arg1].count > 0)
          {
             semaphores[arg1].count--;
             semaphores[arg1].currentUser = taskCurrent;

          }
          else
          {
             //if(semaphores[arg1].processQueue[(semaphores[arg1].queueSize)-1] != taskCurrent) // Improve this.
             //{
                 semaphores[arg1].processQueue[semaphores[arg1].queueSize] = taskCurrent;
                 tcb[taskCurrent].state = STATE_BLOCKED;
                 tcb[taskCurrent].semaphore = (void *)arg1;
                 semaphores[arg1].queueSize++;
                 NVIC_INT_CTRL_R |= 0x10000000;
             //}
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
              arg1 = arg1 / 41;  //Change here
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
     case svc_delete:
         for(i = 0 ; i < MAX_TASKS ; i++ )
         {
             if(tcb[i].pid == (void *)arg1)
                 break;
         }
         if(tcb[i].state == STATE_BLOCKED)
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
    return (((((GPIO_PORTA_DATA_R << 2) & 0x1F0)>>2)^(0x7C))>>2);
}

//-----------------------------------------------------------------------------
// UART subroutines
//-----------------------------------------------------------------------------

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while ((UART0_FR_R & UART_FR_TXFF)!=0)
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

/*void scrollback()  //FIFO format
{
if(ctr<=l)
{
m = l-ctr;
strcpy(input,scrollbuffer[m]);
}
else
{
strcpy(input,scrollbuffer[l-1]);
ctr = 1;
}
}
*/
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
          if(l>4)
              l = 0;
          //strcpy(scrollbuffer[l],input);
          l++;
          return;
         }
         else
         {
            if(c < 0x20)  // Check for printable characters.
                goto l1;
            else
               {
                input[count++] = c; // All entries to the input buffer are converted to lower-case


                    if(strstr(input,arr_buffer[0])!=NULL)
                    {  if(l!=0)
                       {
                        //scrollback();
                        ctr++;
                        return;
                       }
                       else
                       {
                           count--;
                           goto l1;
                       }
                     }

                if (count > Buffer_Max)     // Checking if buffer is full.
                {
                    putsUart0("\nStop!Buffer has overflowed!Try again\r\n\r\n");
                    waitMicrosecond(1000);
                    return;
                }
                else
                    goto l1;
               }
         }
     }
}

void kill()
{
    uint8_t i;
    bool c = 0;
    _fn fn;
    for(i = 0;i<taskCount;i++)
    {
        if(strcasecmp(&input[pos[1]],tcb[i].name)==0)
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
        putsUart0("Task doesn't exist");
}

void moveCursor(uint8_t i , uint8_t j)
{
    char l[20];
    snprintf(l, sizeof l, "%s%d%s%d%s","\033[",i,";",j,"H");
    putsUart0(l);

}
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
    snprintf(str,sizeof str,"%d%s%d%d%d%d%p",0,"x",0,0,0,0,tcb[r].pid);
    putsUart0(str);
    }
    else
        putsUart0("Thread doesn't exist");
}

void ipcs()
{
//moveCursor(1,1);
//putsUart0(clear0);
putsUart0("    Name     | Count |    User    |    Waiting-Task   \r\n");
putsUart0("-------------|-------|------------|-------------------\r\n");
uint8_t i;
for( i = 0 ; i < MAX_SEMAPHORES-1 ; i++)
{
    putsUart0(semaphores[i].sname);
    putsUart0(" \t");
    snprintf(str,sizeof str,"%d",semaphores[i].count);
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



void parseInput()
{
uint8_t j = 0;
uint8_t k = 0;
uint8_t i = 0;
uint8_t len = strlen(input);
if(isspace(input[0]) | (ispunct(input[0]))) // Checking for space or punctuation in first entry
    input[0] = '\0'; // Replacing all delimiters to null.
else if(isalpha(input[0])) // Check for alphabets
{
    argc++;                  // Update argc , pos and type.
    pos[j++] = 0;
    type[k++] = 'a';
}
else
{
    argc++;
    pos[j++] = 0;
    type[k++] = 'n';
}

for(i=1;i<len;i++)     // Loop to parse the entries after the first entry.
{
if(isspace(input[i]) || (ispunct(input[i])))
{
    input[i] = '\0';
if(isalpha(input[i+1]))
{   argc++;
    pos[j++] = i+1;
    type[k++] = 'a';
}
else if(isdigit(input[i+1]))
{
    argc++;
    pos[j++] = i+1;
    type[k++] = 'n';
}
else
    continue;
}
}
}

int isCommand()
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
    if(strcasecmp(&input[pos[0]],commands[i])==0) // Change it to strncasecmp
    {
        switch(i+1)
        {
        case 1:
            if(strcasecmp(&input[pos[1]],"on")==0)
            {
                pi = 1;
                return 0;
             }
            else if(strcasecmp(&input[pos[1]],"off")==0)
            {
                pi = 0;
                return 0;
            }
            else
                putsUart0("Invalid argument for priority inheritance");
            break;
        case 2:
            if(strcasecmp(&input[pos[1]],"on")==0)
            {
                schedule = 1;
                return 0;
            }
            else if(strcasecmp(&input[pos[1]],"off")==0)
            {
                 schedule = 0;
                 return 0;
            }
            else
                 putsUart0("Invalid argument for priority scheduling");
            break;
        case 3:
            if(strcasecmp(&input[pos[1]],"on")==0)
            {
                rtos = 1;
                return 0;
            }
            else if(strcasecmp(&input[pos[1]],"off")==0)
            {
                rtos = 0;
                ResetISR();
            }
            else
                putsUart0("Invalid argument for preemption");
            break;
        case 4:
            ResetISR();
            break;
        case 5:
            pidThread();
            break;
        case 6:
            kill();
            break;
        case 7:
            ipcs();
            break;
        /*case 8:
            if(argc-1>= Min_Args[2]) // Checking if minimum argument criteria is met.
                return 'A';          // Auto
            else
                break;
                */
        default:
            putsUart0("Invalid Command!\r\n\r\n"); // Operation not supported by program
        }
     }
    }
    return 1;
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
    moveCursor(1,1);
    while (true)
    {
        argc = 0;
        putsUart0("Enter something!!\r\n\n"); // Create menu
        getsUart0();
        //putsUart0(clearline);
        parseInput();
        putsUart0("\r\n");
        isCommand();
        putsUart0("\r\n");
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


    // Add required idle processes at lowest priority // Order was changed
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
