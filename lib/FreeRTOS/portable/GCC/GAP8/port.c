/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Macro definitions */
/* Control ans Status Registers Reset Values. */
#define portINITIAL_MSTATUS		( 0x1880 )
#define portINITIAL_MEPC		( 0x1c000080 ) /* Reset */
#define portINITIAL_MCAUSE		( 0x00000000 )

/* Internal Functions. */

/* Setup timer to enable Systick interruptions. */
void prvSetupTimerInterrupt( void );

#if portUSING_MPU_WRAPPERS == 1
/* Setup MPU. */
void prvSetupMPU( void ) PRIVILEGED_FUNCTION;

/*
 * Checks to see if being called from the context of an unprivileged task, and
 * if so raises the privilege level and returns false - otherwise does nothing
 * other than return true.
 */
BaseType_t xPortRaisePrivilege( void );

/* Reset privilege level after call to xPortRaisePrivilege(). */
void vPortResetPrivilege( BaseType_t xRunningPrivileged );
#endif //portUSING_MPU_WRAPPERS == 1

/* Scheduler utilities. */

/* Critical sections management. */
void vPortEnter_Critical( void );
void vPortExit_Critical( void );
uint32_t uPortSet_Interrupt_Mask_From_ISR( void );
void vPortClear_Interrupt_Mask_From_ISR( uint32_t irqSet );

/* FreeRTOS Handlers in GAP8_it.c. */


/* Variables. */
volatile uint32_t ulCriticalNesting = 9999ul;

/*
 * ulCriticalNesting : not necessary.
 * void vPortEnter_Critical( void ), void vPortExit_Critical( void )
 * Both functions are not needed. Functions defined in tasks.c,
 * portCRITICAL_NESTING_IN_TCB must be set to 1 in order to track nesting
 * in Task Control Block.
 * Refer to l. 269, 4068, 4098 in tasks.c.
 *
 * If portCRITICAL_NESTING_IN_TCB 1 defined, no need of ulCriticalNesting,
 * neither void vPortEnter_Critical( void ), void vPortExit_Critical( void ) to
 * be defined.
 */

/* ISR Stack of 1kB. */
extern uint8_t __irq_stack_start__;
StackType_t *xISRStack = ( StackType_t * ) &__irq_stack_start__;


/*-----------------------------------------------------------*/
/*
 * See header file for description.
 */
#if portUSING_MPU_WRAPPERS == 1
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack,
				    TaskFunction_t pxCode,
				    void *pvParameters,
				    BaseType_t xRunPrivileged )
#else
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack,
					TaskFunction_t pxCode,
					void *pvParameters )
#endif //portUSING_MPU_WRAPPERS == 1
{
    /* Few bytes on the bottom of the stack. May be useful for debugging. */
    pxTopOfStack--;
    *pxTopOfStack = 0xdeedfeed;

    /* Hardware Loop registers. */
    {
        pxTopOfStack -= 6;
    }
    /* Control and status registers saved if R/W. */
    {
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pxCode; /* MEPC */
    }
    /* General purpose registers saved. sp reg stored in Task Control Block. */
    {
	pxTopOfStack -= 27; /* a1-a7 + t0-t6 +  s0-11 */
	*pxTopOfStack = ( StackType_t ) pvParameters; /* a0 */
	pxTopOfStack -= 1; /* ra */
    }

/*
 *    Stack
 *
 *     LOW
 * ************  <------ pxTopOfStack
 * *  MSTATUS *
 * *----------*
 * *    ra    *
 * *    gp    *
 * *    tp    *
 * *----------*
 * * Param/a0 *
 * *----------*
 * *    a1    *
 * *    --    *
 * *    --    *
 * *    s11   *
 * *----------*
 * *   CSR    *
 * *----------*
 * * HW loop  *
 * *==========*
 * * deedfeed *
 * ************
 *    HIGH
 *
 */
    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Do not implement. */
}
/*-----------------------------------------------------------*/

/* Setup Systick timer to generate tick interrupts. */
void prvSetupTimerInterrupt( void )
{
    /* Systick timer configuration. */
    SysTick->CFG_REG_LO = ( ( 1 << SysTick_CFG_REG_LOW_ENABLE_Pos )
			    | ( 1 << SysTick_CFG_REG_LOW_RESET_Pos )
			    | ( 1 << SysTick_CFG_REG_LOW_IRQE_Pos )
			    | ( 0 << SysTick_CFG_REG_LOW_IEM_Pos )
			    | ( 1 << SysTick_CFG_REG_LOW_CMP_CLR_Pos )
			    | ( 0 << SysTick_CFG_REG_LOW_ONE_SHOT_Pos )
			    | ( 0 << SysTick_CFG_REG_LOW_PRESCALERE_Pos )
			    | ( 0 << SysTick_CFG_REG_LOW_CLKS_Pos )
			    | ( 0 << SysTick_CFG_REG_LOW_PRESCALER_Pos )
			    | ( 0 << SysTick_CFG_REG_LOW_64BIT_Pos )
	);
    /* Start the timer by putting a CMP value. */
    SysTick->CMP_LO = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1ul;
    SysTick->VALUE_LO = 0;
    /* Enable IRQ from Systick timer. */
    NVIC->MASK_IRQ_OR = (0x1 << SysTick_IRQn);
}
/*-----------------------------------------------------------*/

#if portUSING_MPU_WRAPPERS == 1
void prvSetupMPU( void )
{
    uint32_t	base_l2	 = ( 0x1C000000ul );
    uint32_t	base_fc	 = FC_BASE;
    uint32_t	base_apb = SOC_PERI_BASE;
    /* Setup APB Filter. */
    {
	/* Setup APB_FC_TCDM_AREA. 0x1B00 0000 - 0x1B10 0000 */
	MPU->APB_RULE[0] = (
	    ( GAP_MPU_APB_FC_TCDM_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x2000ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );

	base_fc += ( 0x2000ul << 6 );
	MPU->APB_RULE[1] = (
	    ( GAP_MPU_APB_FC_TCDM_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x2000ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );

	/* Setup APB_FC_PERIPH_AREA. 0x1B20 0000 - 0x1B20 4000 */
	base_fc += ( 0x6000ul << 6 );
	MPU->APB_RULE[2] = (
	    ( GAP_MPU_APB_FC_PERIPH_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x100ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );
	/* Setup APB_APB_AREA. 0x1A10 0000 - 0x1A11 0000 */
	MPU->APB_RULE[3] = (
	    ( GAP_MPU_APB_APB_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_apb ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x400ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );
    }

    /* Setup FC TCDM Filter. */
    {
	/* Setup FC_TCDM_AREA. 0x1B00 0000 - 0x1B10 0000 */
	base_fc	= FC_BASE;
	MPU->FC_TCDM_RULE[0] = (
	    ( GAP_MPU_FC_TCDM_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x2000ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );

	base_fc += ( 0x2000ul << 6 );
	MPU->FC_TCDM_RULE[1] = (
	    ( GAP_MPU_FC_TCDM_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x2000ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );

	/* Setup FC_PERIPH_AREA. 0x1B20 0000 - 0x1B20 4000 */
	base_fc += ( 0x6000ul << 6 );
	MPU->FC_TCDM_RULE[2] = (
	    ( GAP_MPU_FC_PERIPH_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x100ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );

	/* Setup FC_CLUSTER_TCDM_AREA. 0x1B20 4000 - 0x1B20 8000 */
	base_fc += ( 0x100ul << 6 );
	MPU->FC_TCDM_RULE[3] = (
	    ( GAP_MPU_FC_CLUSTER_TCDM_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x100ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );

	/* Setup FC_CLUSTER_PERIPH_AREA. 0x1B30 0000 - 0x1B40 0000 */
	base_fc += ( 0x3F00 << 6 );
	MPU->FC_TCDM_RULE[4] = (
	    ( GAP_MPU_FC_CLUSTER_PERIPH_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x2000ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );

	base_fc += ( 0x2000ul << 6 );
	MPU->FC_TCDM_RULE[5] = (
	    ( GAP_MPU_FC_CLUSTER_PERIPH_AREA << MPU_TYPE_RULE_AREA_Pos )
	    | ( ( ( base_fc ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x2000ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );
    }

    /* Setup L2 Filter. */
    {
	/* Setup L2. Size(L2)=512kB=0x80000  0x1C00 0000 - 0x1C08 0000 */
	MPU->L2_RULE[0] = (
	    ( GAP_MPU_L2_L2_AREA << MPU_TYPE_RULE_AREA_Pos)
	    | ( ( ( base_l2 ) >> GAP_MPU_PAGE_SIZE_LOG2 ) << MPU_TYPE_RULE_BASE_Pos )
	    | ( 0x2000ul << MPU_TYPE_RULE_SIZE_Pos )
	    | ( 1 << MPU_TYPE_ENABLE_Pos )
	    );
    }

    /* Enable portMPU. */
    MPU->MPU_ENABLE = ( 1 << MPU_TYPE_ENABLE_Pos );
}
/*-----------------------------------------------------------*/

void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings,
				const struct xMEMORY_REGION * const xRegions,
				StackType_t *pxBottomOfStack,
				uint32_t ulStackDepth )
{
    uint8_t ucIndex = 0;
    uint32_t	base_l2	 = ( 0x1C000000ul );

    /* Store MPU Settings of a task inside its Task Control Block. */
    if( xRegions == NULL )
    {
        xMPUSettings->xRegions[0].ulRegionBaseAddress = base_l2;
        xMPUSettings->xRegions[0].ulRegionSize = 0x2000;
        xMPUSettings->xRegions[0].ucRegionArea = GAP_MPU_L2_L2_AREA;
    }
    else
    {
	for( ucIndex = 0; ucIndex < portNUM_CONFIGURABLE_REGIONS; ucIndex++ )
	{
	    if( xRegions[ ucIndex ].ulLengthInBytes > 0ul )
	    {
		xMPUSettings->xRegions[ ucIndex ].ulRegionBaseAddress =
		    (uint32_t) xRegions[ ucIndex ].pvBaseAddress;
		xMPUSettings->xRegions[ ucIndex ].ulRegionSize =
		    xRegions[ ucIndex ].ulLengthInBytes;
		xMPUSettings->xRegions[ ucIndex ].ucRegionArea =
		    ( xRegions[ ucIndex ].ulParameters & 0x0f);
	    }
	}
    }
}
/*-----------------------------------------------------------*/

BaseType_t xPortRaisePrivilege( void )
{
    return 1;
}
/*-----------------------------------------------------------*/

void vPortResetPrivilege( BaseType_t xRunningPrivileged )
{
    ( void ) xRunningPrivileged;
}
#endif //portUSING_MPU_WRAPPERS == 1
/*-----------------------------------------------------------*/

void vPortEnter_Critical( void )
{
    portDISABLE_INTERRUPTS();
    /* Increment nesting value everytime a critical section is entered. */
    ulCriticalNesting++;
}
/*-----------------------------------------------------------*/

void vPortExit_Critical( void )
{
    /* Decrement nesting value everytime a critical section is exit. */
    if(ulCriticalNesting > 0)
    {
	ulCriticalNesting--;
	if( ulCriticalNesting == 0 )
	{
	    /* If no more in any critical sections, enable interruptions. */
	    portENABLE_INTERRUPTS();
	}
    }
}
/*-----------------------------------------------------------*/

uint32_t uPortSet_Interrupt_Mask_From_ISR( void )
{
    uint32_t ulIrqMask = 0;
    /* Disable all other interrupts. */
    portDISABLE_INTERRUPTS();
    __asm__ volatile("csrr %0, mstatus":"=r" (ulIrqMask) );
    return ulIrqMask;
}
/*-----------------------------------------------------------*/

void vPortClear_Interrupt_Mask_From_ISR( uint32_t irqSet )
{
    portENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/
