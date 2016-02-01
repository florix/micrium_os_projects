/*
*********************************************************************************************************
*               ASSIGNMENT 1. Andrea Floridia, S224906
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "fsl_interrupt_manager.h"
#include "fsl_gpio_common.h"

#include <stdint.h>

#include  <math.h>
#include  <lib_math.h>
#include  <cpu_core.h>

#include  <app_cfg.h>
#include  <os.h>

#include  <fsl_os_abstraction.h>
#include  <system_MK64F12.h>
#include  <board.h>

#include  <bsp_ser.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                      LOCAL MACRO
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                                TIMER_START
*
* Description : This macro is used to start and initialize the low-power timer 0(LPTMR0). These are the
*               the actions that the macro performs once it is espanded:
*               - select the 4Mhz clock:
*                       MCG->C2 |= 1;                           
*                       MCG->SC = 0; 
*               - enable the clock for the low-power timer
*                       SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK; 
*               - reset all registers:
*                       LPTMR0->CSR = 0; 
*                       LPTMR0->PSR = 0; 
*                       LPTMR0->CMR = 0; 
*               - set the TFC bit in CSR register:
*                       LPTMR0->CSR |= LPTMR_CSR_TFC_MASK; 
*               - set the PBYP bit in PSR register:
*                       LPTMR0->PSR |= LPTMR_PSR_PBYP_MASK; 
*               - set the TEN bit in CSR register(this action starts the timer):
*                       LPTMR0->CSR |= LPTMR_CSR_TEN_MASK
*
* Argument(s) : none.
*
* Return(s)   : none.
*
*
* Note(s)     : none.
*********************************************************************************************************
*/
#define	TIMER_START     MCG->C2 |= 1; MCG->SC = 0; SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK; LPTMR0->CSR = 0; LPTMR0->PSR = 0; LPTMR0->CMR = 0; LPTMR0->CSR |= LPTMR_CSR_TFC_MASK; LPTMR0->PSR |= LPTMR_PSR_PBYP_MASK; LPTMR0->CSR |= LPTMR_CSR_TEN_MASK

/*
*********************************************************************************************************
*                                                TIMER_STOP
*
* Description : This macro is used to stop the timer.
*               The actions that the macro performs once it is espanded:
*               - clear the TEN bit in CSR register(disable the counter):
*                       LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK        
* Argument(s) : none.
*
* Return(s)   : none.
*
*
* Note(s)     : none.
*********************************************************************************************************
*/
#define TIMER_STOP      LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK

/*
*********************************************************************************************************
*                                                TIMER_READ
*
* Description : This macro is used to read the counter value stored in CNR register.
*               The actions that the macro performs once it is espanded:
*               - clear the CNR register:
*                       LPTMR0->CNR = 0;
*               - read the value (x should be a 32-bit integer variable):
*                       (x) = LPTMR0->CNR
* Argument(s) : x is the variable where should be stored the read value.
*
* Return(s)   : none.
*
*
* Note(s)     : none.
*********************************************************************************************************
*/
#define TIMER_READ(x)   LPTMR0->CNR = 0; (x) = LPTMR0->CNR



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB          AppTaskStartTCB;
static  CPU_STK         AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB          TaskPTB9TCB;
static  CPU_STK         TaskPTB9Stk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB          TaskTriggerTCB;
static  CPU_STK         TaskTriggerStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB          TaskRGBHandlerTCB;                                 
static  CPU_STK         TaskRGBHandlerStk[APP_CFG_TASK_START_STK_SIZE];     



static  OS_SEM          Sem1;                         /* semaphore used to control task PTB9 execution */

static  uint32_t        old_value = 0;                /* used to detect the trasition in the Echo signal */

static  CPU_INT32U      milliseconds;                 /* how many milliseconds task RGBHandler should be suspended (depending on the blinking rate)*/
static  uint32_t        selected_LED = 0;             /* which LED should blink */    
static  uint8_t         system_state = 0;             /* keeps track of the system state. system_state = 0 means that distance from the closest object is < 10 cm, 1 otherwise */
static  uint8_t         LED_state = 0;                /* keeps track of the RGB state, range from 0 (IDLE) to 11 ( distance from the closest object > 200 cm */
static  uint32_t        counter_value = 0;            /* used to read the content of the CNR register of LPTMR*/

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart      (void  *p_arg);
static  void  TaskTrigger       (void  *p_arg);
static  void  BSP_PTB9_int_hdlr (void );
static  void  TaskPTB9          (void  *p_arg);
static  void  TaskRGBHandler    (void  *p_arg);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This the main standard entry point.
*
* Note(s)     : none.
*********************************************************************************************************
*/

int  main (void)
{
    OS_ERR   err;

#if (CPU_CFG_NAME_EN == DEF_ENABLED)
    CPU_ERR  cpu_err;
#endif

    hardware_init();
    GPIO_DRV_Init(switchPins, ledPins);


#if (CPU_CFG_NAME_EN == DEF_ENABLED)
    CPU_NameSet((CPU_CHAR *)"MK64FN1M0VMD12",
                (CPU_ERR  *)&cpu_err);
#endif

    OSA_Init();                                                 /* Init uC/OS-III.                                        */
    
    INT_SYS_InstallHandler(PORTB_IRQn, BSP_PTB9_int_hdlr);
    
    OSSemCreate( &Sem1, "Semaphore 1", 0, &err );               /* Create semaphore Sem1. Note that the initial value is 0 */

    OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                   */
                 "App Task Start",
                  AppTaskStart,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);
    

    OSA_Start();                                                /* Start multitasking (i.e. give control to uC/OS-III). */

    while (DEF_ON) {                                            /* Should Never Get Here                                */
        ;
    }
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : initializes the hardware and create the others tasks. 
*               
*
* Argument(s) : p_arg   is the argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Notes       : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    OS_ERR      err;

    (void)p_arg;                                                /* See Note #1                                          */


    CPU_Init();                                                 /* Initialize the uC/CPU Services.                      */
    Mem_Init();                                                 /* Initialize the Memory Management Module              */
    Math_Init();                                                /* Initialize the Mathematical Module                   */

    BSP_Ser_Init(115200u);

    /* create the others tasks */
    OSTaskCreate(&TaskTriggerTCB,                              
                 "App Task Trigger",
                  TaskTrigger,
                  0u,
                  3u,   
                 &TaskTriggerStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);

    
    OSTaskCreate(&TaskPTB9TCB,                              
                 "App Task ptb9",
                  TaskPTB9,
                  0u,
                  APP_CFG_TASK_START_PRIO,                      /* NOTE: this task has the highest priority because is responsibile for measuring the distance of the object */
                 &TaskPTB9Stk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);
    
    OSTaskCreate(&TaskRGBHandlerTCB,                             
                 "App Task rgb handler",
                  TaskRGBHandler,
                  0u,
                  3u,
                 &TaskRGBHandlerStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);
    
   
  
    /* this task is used only for initialization, so it can be deleted */
    OSTaskDel((OS_TCB *)0, &err);
}


/*
*********************************************************************************************************
*                                          RGB HANDLER
*
* Description :  This task handles the RGB LED. RGB can be configured using two global variables,
*                selected_LED(which LED should blink) and milliseconds (blinking rate).
*               
*
* Argument(s) : p_arg   is the argument passed  by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Notes       : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static void TaskRGBHandler (void  *p_arg) 
{
    OS_ERR      err;
    

    (void)p_arg;  
    
    while (DEF_ON) {
      
      GPIO_DRV_TogglePinOutput( selected_LED );
      
      OSTimeDlyHMSM(0u, 0u, 0u, milliseconds, OS_OPT_TIME_HMSM_STRICT, &err);
      
    }
}


/*
*********************************************************************************************************
*                                          TRIGGER
*
* Description : it triggers the PTB23 pin of the board, connected to the TRIGGER pin of the sensor. 
*               It generate a pulse of 10 milliseconds every 200 milliseconds
*               
*
* Argument(s) : p_arg   is the argument passed  by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Notes       : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static void TaskTrigger (void  *p_arg)
{
    OS_ERR      err;

    (void)p_arg;                                                
    
    
    while (DEF_ON) {
     
      GPIO_DRV_TogglePinOutput( outPTB23 );
      
      OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
      
      GPIO_DRV_TogglePinOutput( outPTB23 );
      
      OSTimeDlyHMSM(0u, 0u, 0u, 200u, OS_OPT_TIME_HMSM_STRICT, &err);    
      
    }
}

/*
*********************************************************************************************************
*                                          ECHO HANDLER
*
* Description : it handles the ECHO coming from the sensor. Once this task is called, it remains blocked
*               on a semaphore. The task is unblocked by an ISR that use the system call OSSemPost on Sem1
*               , when the falling edge of the echo signal occurs.
*               
*
* Argument(s) : p_arg   is the argument passed  by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Notes       : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static void TaskPTB9 (void  *p_arg)
{
    OS_ERR      os_err;
    CPU_TS      os_ts;
    char        tmp[80];                /* used for debug the system */
    float       distance;
    
    (void)p_arg;
    
    while (DEF_ON) {   
        
        
        /* wait on the semaphore Sem1 */
        OSSemPend(&Sem1, 0,OS_OPT_PEND_BLOCKING,&os_ts, &os_err); 
        
        /* conversion to obtain the distance: note multipling for 10^6 is necessary to convert from second to microsecond */
        /* NOTE: uncomment the following lines to debug the system */
        /*
        sprintf( tmp, "Distance  = %f cm \n\r", (float)(((1000000.0*(counter_value))/( 4*1000000 ))/58) ); 
        
        APP_TRACE_DBG(( tmp ));
        */
        distance = (float)(((1000000.0*(counter_value))/( 4*1000000 ))/58);
        
        /* select the proper LED, depending on the distance and the current state */
        if ((distance < 10.0) && (LED_state != 1)) {
          
          if(system_state)
            OSTaskSuspend(&TaskRGBHandlerTCB, &os_err);                 /* suspend the task, blinking is not required at this distance*/
         
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          system_state = 0;
          selected_LED = BOARD_GPIO_LED_RED;
          GPIO_DRV_ClearPinOutput( BOARD_GPIO_LED_RED );
          LED_state = 1;
          
        }
        else if ((distance >= 10.0) && (distance < 25.0) && (LED_state != 2)) {
          
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          
          selected_LED = BOARD_GPIO_LED_RED;
          milliseconds = 200u;                          
          LED_state = 2;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          /* past state was LED_state = 1, so the RGBHandler has been suspended! */
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       /* past state was not LED_state = 1, so the RGBHandler is suspended by the delay sys call */
          
          system_state = 1;
        }
        else if ((distance >= 25.0) && (distance < 50.0) && (LED_state != 3)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          selected_LED = BOARD_GPIO_LED_RED;
          milliseconds = 300u;                          
          LED_state = 3;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 50.0) && (distance < 75.0) && (LED_state != 4)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          selected_LED = BOARD_GPIO_LED_RED;
          milliseconds = 400u;
          LED_state = 4;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 75.0) && (distance < 100.0) && (LED_state != 5)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          
          selected_LED = BOARD_GPIO_LED_RED;
          milliseconds = 500u;
          LED_state = 5;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 100.0) && (distance < 120.0) && (LED_state != 6)) {       
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          
          selected_LED = BOARD_GPIO_LED_BLUE;
          milliseconds = 100u;
          LED_state = 6;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 120.0) && (distance < 140.0) && (LED_state != 7)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          
          selected_LED = BOARD_GPIO_LED_BLUE;
          milliseconds = 200u;
          LED_state = 7;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 140.0) && (distance < 160.0) && (LED_state != 8)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
         
          selected_LED = BOARD_GPIO_LED_BLUE;
          milliseconds = 300u;
          LED_state = 8;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 160.0) && (distance < 180.0) && (LED_state != 9)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          
          selected_LED = BOARD_GPIO_LED_BLUE;
          milliseconds = 400u;
          LED_state = 9;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 180.0) && (distance < 200.0) && (LED_state != 10)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          
          selected_LED = BOARD_GPIO_LED_BLUE;
          milliseconds = 500u;
          LED_state = 10;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }
        else if ((distance >= 200) && (LED_state != 11)) {
          
          
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_RED );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_BLUE );
          GPIO_DRV_SetPinOutput( BOARD_GPIO_LED_GREEN );
          
          
          selected_LED = BOARD_GPIO_LED_GREEN;
          milliseconds = 1000u;
          LED_state = 11;
          
          if (!system_state)
            OSTaskResume(&TaskRGBHandlerTCB, &os_err);          
          else if (system_state)
            OSTimeDlyResume(&TaskRGBHandlerTCB, &os_err);       
          
          system_state = 1;
        }

        
       
    }
  
}

/*
*********************************************************************************************************
*                                          ISR PTB9
*
* Description : this is the ISR associated with PTB9 events. It is called twice. Once for the rising edge 
*               of the signal, and the second for the rising edge of the signal. The ISR calls TIMER_START
*               when the rising edge occurs. When the falling edge occurs, it calls TIMER_READ to read the
*               current content of CNR register and then TIMER_STOP to stop the timer. Finally it calls the 
*               system call OSSemPost on semaphore Sem1 to unblock TaskPTB9.
*               
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Notes       : none.
*********************************************************************************************************
*/

static void BSP_PTB9_int_hdlr( void )
{

  uint32_t      new_value;
  OS_ERR        os_err;
  uint32_t      ifsr;                                                                   /* interrupt flag status register */
  uint32_t      portBaseAddr = g_portBaseAddr[GPIO_EXTRACT_PORT(inPTB9)];
  uint32_t      portPin      = (1 << GPIO_EXTRACT_PIN(inPTB9));
    
  CPU_CRITICAL_ENTER();
  OSIntEnter();                                                                          /* Tell the OS that we are starting an ISR              */
  
  ifsr = PORT_HAL_GetPortIntFlag(portBaseAddr);
  new_value = GPIO_DRV_ReadPinInput( inPTB9 );                                          /* acquire a sample of the current value for comparison */
  if( (ifsr & portPin) )                                                                /* Check whether the pending interrupt is for inPTB9 */
  {  
                
        /* used to detect trasition from high to low and viceversa */
        if ( new_value != old_value && new_value == 1) {                    
          old_value = new_value;
          TIMER_START;
        }   
        else if ( new_value != old_value && new_value == 0) {
            old_value = new_value;
            TIMER_READ(counter_value);
            TIMER_STOP;
            OSSemPost( &Sem1, OS_OPT_POST_1+OS_OPT_POST_NO_SCHED, &os_err );
        }
   
      GPIO_DRV_ClearPinIntFlag( inPTB9 );
  }
  
  CPU_CRITICAL_EXIT();

  OSIntExit();
}


