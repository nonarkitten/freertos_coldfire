/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "setjmp.h"
#include "derivative.h"
#include "exceptions.h"
#include "nbits.h"
#include "extern.h"

extern __interrupt void vPortVL1swiHandler(void);
static __interrupt void vPortVpdb0Handler(void);
static __interrupt void vPortVtrap13Handler(void);

__declspec(weak) vectorTableEntryType vector__Vtrap13
		@(VECTOR_TABLE_COPY + GetIntVectAddr(Vtrap13))
		= (vectorTableEntryType)vPortVtrap13Handler;

__declspec(weak) vectorTableEntryType vector__VL1swi  
		@(VECTOR_TABLE_COPY + GetIntVectAddr(VL1swi)) 
		= (vectorTableEntryType)vPortVL1swiHandler;

__declspec(weak) vectorTableEntryType vector__Vpdb0   
		@(VECTOR_TABLE_COPY + GetIntVectAddr(Vpdb0))   
		= (vectorTableEntryType)vPortVpdb0Handler;

#define portINITIAL_FORMAT_VECTOR		( ( StackType_t ) 0x4000 )

/* Supervisor mode set. */
#define portINITIAL_STATUS_REGISTER		( ( StackType_t ) 0x2000)

/* Used to keep track of the number of nested calls to taskENTER_CRITICAL().  This
will be set to 0 prior to the first task being started. */
static uint32_t ulCriticalNesting = 0x9999UL;

/* Used to restore the original system context when the scheduler is ended. */
static jmp_buf xJumpBuf;

#define TICKRATE	(configCPU_CLOCK_HZ / 2 / configTICK_RATE_HZ)
#define PRESCALE	(NBITS(TICKRATE / 65536))
#define MODULO		(TICKRATE / (1 << PRESCALE))

/*-----------------------------------------------------------*/
static __interrupt void vPortVpdb0Handler( void ) 
{
	/* Clear the interrupt. */
	PDB0_SC = PDB_SC_PRESCALER(PRESCALE) | PDB_SC_TRGSEL(0xF) | PDB_SC_PDBEN_MASK | PDB_SC_PDBIE_MASK | PDB_SC_CONT_MASK;

	/* Increment the RTOS tick. */
	if( xTaskIncrementTick() != pdFALSE ) 
	{
		taskYIELD();
	}	
}

static void prvSetupTimerInterrupt( void ) 
{	
	/* MTIM, LPTMR and FTM are used elsewhere, PDB is used.
	 * Change this if your project needs to use the PDB */
	SIM_SCGC3 |= SIM_SCGC3_PDB_MASK;
	PDB0_SC |= PDB_SC_PRESCALER(PRESCALE) | PDB_SC_TRGSEL(0xF) | PDB_SC_PDBIE_MASK| PDB_SC_CONT_MASK;
	PDB0_MOD = (uint32_t)MODULO;                  
	PDB0_IDLY = (uint32_t)MODULO;                  
	PDB0_SC |= (uint32_t)PDB_SC_PDBEN_MASK | PDB_SC_LDOK_MASK;
	PDB0_SC |= (uint32_t)PDB_SC_SWTRIG_MASK;                       
}
/*-----------------------------------------------------------*/

void vConfigureTimerForRunTimeStats( void ) {
	// LPTMR0 rolls over every 2 seconds
	// LPTMR1 rolls over every 36 hours
	LPTMR0_CSR = (uint32_t)0x80UL; // disable and clear timer compare flag
	LPTMR0_PSR = (uint32_t)0x04UL; // set prescalar to bypass using clock 0
	LPTMR0_CMR = (uint32_t)0x0000UL; // we'll ignore this and run in free-running mode

	LPTMR1_CSR = (uint32_t)0x80UL; // disable and clear timer compare flag
	LPTMR1_PSR = (uint32_t)0x78UL; // set prescalar to /65536 using clock 0
	LPTMR1_CMR = (uint32_t)0x0000UL; // we'll ignore this and run in free-running mode
	// no interrupt, free-running counter, and enable
	LPTMR0_CSR = (uint32_t)0x05UL;
	LPTMR1_CSR = (uint32_t)0x05UL;
}
/*-----------------------------------------------------------*/

unsigned long vPortSystemTimer(void) 
{
	LPTMR0_CNR = 1; LPTMR1_CNR = 1;
	return ((LPTMR1_CNR << 16) | LPTMR0_CNR);
}
/*-----------------------------------------------------------*/

StackType_t *pxPortInitialiseStack( StackType_t * pxTopOfStack, TaskFunction_t pxCode, void *pvParameters,  BaseType_t xRunPrivileged ) 
{
	*pxTopOfStack = (StackType_t) 0xDEADBEEF;
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) pvParameters;
	pxTopOfStack--;

	/* Create task with supervisor privileges */
    *pxTopOfStack = (xRunPrivileged) ? 0x40002000UL : 0x40000000UL;
	pxTopOfStack--;

	/* Exception stack frame starts with the return address. */
	*pxTopOfStack = ( StackType_t ) pxCode;
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xa6; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xa5; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xa4; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xa3; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xa2; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xa1; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd7; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd6; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd5; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd4; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd3; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd2; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd1; /*FP*/
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xd0; /*FP*/

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
	extern void vPortStartFirstTask( void );

	if( setjmp( xJumpBuf ) == 0 )
	{
		ulCriticalNesting = 0UL;

		/* Configure a timer to generate the tick interrupt. */
		prvSetupTimerInterrupt();

		/* Start the first task executing. */
		vPortStartFirstTask();
	}

	return pdFALSE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Jump back to the processor state prior to starting the
	scheduler.  This means we are not going to be using a
	task stack frame so the task can be deleted. */
	longjmp( xJumpBuf, 1 );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void ) 
{
	if( ulCriticalNesting == 0UL )
	{
		portDISABLE_INTERRUPTS();
	}
	ulCriticalNesting++;
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void ) 
{
	ulCriticalNesting--;
	if( ulCriticalNesting == 0 ) 
	{
		portENABLE_INTERRUPTS();
	}
}
/*-----------------------------------------------------------*/

asm __declspec(register_abi) void vPortVtrap13Handler(void) {
	/* Presume D0 holds our new IPL and will send it back as well */	
	move.l	d1, -(sp)			// back up D1
			
	move.l	4(sp), d1			// grab the stack frame
	andi.l	#0x0700, d1			
	lsr.l	#8, d1				// mask out and then
	move.l	d1, -(sp)			// save our 'old' IPL
	
	move.l	4(sp), d1			// grab the stack frame (again)
	andi.l	#0xFFFFF8FF, d1		// clear the interrupt bits
	asl.l	#8, d0
	or.l	d0, d1				// mask-in our new IPL
	move.l	d1, 4(sp)			// overwrite our stack frame
	move.l	(sp)+, d0			// recover our old IPL
	move.l	(sp)+, d1			// and the original D1 register
	rte
}

asm __declspec(register_abi) uint32_t ulPortSetIPL(uint32_t d0) {
	trap	#13					// new interrupt level is in D0
}

extern pxCurrentTCB;
#pragma NO_RETURN
asm __declspec(register_abi) void vPortVL1swiHandler(void) {
	// we're in the SSP context not the USP, so we need
	// to save the user state from the supervisor state
    move.l  a0, -(sp)			// get the usp
    move.l	usp, a0

    move.l	(sp)+, -(a0)		// save old a0
    move.l	(sp)+, -(a0)		// save frame
    move.l	(sp)+, -(a0)		// save user pc

    lea.l	-56(a0), a0			// make room for regs
    movem.l d0-d7/a1-a6, (a0)	// save other regs
    
    moveq   #(0x3F ^ configKERNEL_INTERRUPT_PRIORITY), d0
    move.b  d0, 0xFFFFFFDF		// clear forced interrupt

    move.l	a0, -(sp)
    move.l  pxCurrentTCB, a0
    move.l	(sp)+, (a0)			// save the usp

	jsr 	vTaskSwitchContext
	// fall through to the following function
}

asm __declspec(register_abi) void vPortStartFirstTask(void) {
    move.l  pxCurrentTCB, a0
    move.l	(a0), a0			// get the usp

    movem.l	(a0), d0-d7/a1-a6
    lea.l	56(a0), a0			// restore regs

    move.l	(a0)+, -(sp)		// save user pc on ssp frame
    move.l	(a0)+, -(sp)		// save user frame on ssp
    move.l	(a0)+, -(sp)		// restore original a0

    move.l	a0, usp
    move.l	(sp)+, a0

    rte
}
