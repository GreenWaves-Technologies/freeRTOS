/******************************************************************************************
 *
 * Copyright (c) 2018 , GreenWaves Technologies, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of GreenWaves Technologies, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************************/


        .file "port_asm.S"

/*******************************************************************************
	This file contains some macro definitions and functions defined in ASM.
	Functions :
	*xPortStartScheduler
	*vPortYield
	*vPortCtxSwitch
********************************************************************************/

/*******************************************************************************
	EXTERNAL VARIABLES & FUNCTIONS
*******************************************************************************/
        .extern pxCurrentTCB
        .extern prvSetupTimerInterrupt
	.extern ulCriticalNesting
/******************************************************************************/


/*******************************************************************************
	MACRO DEFINITION
*******************************************************************************/

/******************************************************************************/
	.macro DECLARE Routine
	.global \Routine
	.func \Routine
	.type \Routine, %function
	\Routine:
	.endm
/******************************************************************************/

/******************************************************************************/
	.macro portSAVE_CONTEXT

	addi sp, sp, (-35*4)

        /* General purpose registers. */
        sw ra, 0*4(sp)
        sw a0, 1*4(sp)
        sw a1, 2*4(sp)
        sw a2, 3*4(sp)
        sw a3, 4*4(sp)
        sw a4, 5*4(sp)
        sw a5, 6*4(sp)
	sw a6, 7*4(sp)
	sw a7, 8*4(sp)
        sw t0, 9*4(sp)
	sw t1, 10*4(sp)
	sw t2, 11*4(sp)
	sw t3, 12*4(sp)
	sw t4, 13*4(sp)
	sw t5, 14*4(sp)
	sw t6, 15*4(sp)
	sw s0, 16*4(sp)
	sw s1, 17*4(sp)
	sw s2, 18*4(sp)
	sw s3, 19*4(sp)
	sw s4, 20*4(sp)
	sw s5, 21*4(sp)
	sw s6, 22*4(sp)
	sw s7, 23*4(sp)
	sw s8, 24*4(sp)
	sw s9, 25*4(sp)
	sw s10, 26*4(sp)
	sw s11, 27*4(sp)

        /* MSTATUS */
        csrr t0, mstatus
        sw t0, 28*4(sp)

        /* MEPC */
        csrr t0, mepc
        sw t0, 29*4(sp)

        /* HW Loop regs. */
        csrr t0, 0x7B0
        sw t0, 30*4(sp)
        csrr t1, 0x7B1
        sw t1, 31*4(sp)
        csrr t2, 0x7B2
        sw t2, 32*4(sp)
        csrr t0, 0x7B4
        sw t0, 33*4(sp)
        csrr t1, 0x7B5
        sw t1, 34*4(sp)
        csrr t2, 0x7B6
        sw t2, 35*4(sp)

	lw tp, pxCurrentTCB
	sw sp, 0*0(tp)
	.endm
/******************************************************************************/

/******************************************************************************/
	.macro portSAVE_EPC
	csrr t0, mepc
	sw t0, 29*4(sp)
	.endm
/******************************************************************************/

/******************************************************************************/
	.macro portSAVE_RA
	sw ra, 29*4(sp)
	.endm
/******************************************************************************/

/******************************************************************************/
	.macro portRESTORE_CONTEXT

        lw tp, pxCurrentTCB
	lw sp, 0*0(tp)

        /* HW Loop regs. */
        lw t2, 35*4(sp)
        csrw 0x7B6, t2
        lw t1, 34*4(sp)
        csrw 0x7B5, t1
        lw t0, 33*4(sp)
        csrw 0x7B4, t0
        lw t2, 32*4(sp)
        csrw 0x7B2, t2
        lw t1, 31*4(sp)
        csrw 0x7B1, t1
        lw t0, 30*4(sp)
        csrw 0x7B0, t0

        /* MEPC */
	lw t0, 29*4(sp)
	csrw mepc, t0

        /* MSTATUS */
	lw t0, 28*4(sp)
	csrw mstatus, t0

        /* General purpose registers. */
	lw s11, 27*4(sp)
	lw s10, 26*4(sp)
	lw s9, 25*4(sp)
	lw s8, 24*4(sp)
	lw s7, 23*4(sp)
	lw s6, 22*4(sp)
	lw s5, 21*4(sp)
	lw s4, 20*4(sp)
	lw s3, 19*4(sp)
	lw s2, 18*4(sp)
	lw s1, 17*4(sp)
	lw s0, 16*4(sp)
	lw t6, 15*4(sp)
	lw t5, 14*4(sp)
	lw t4, 13*4(sp)
	lw t3, 12*4(sp)
	lw t2, 11*4(sp)
	lw t1, 10*4(sp)
	lw t0, 9*4(sp)
	lw a7, 8*4(sp)
	lw a6, 7*4(sp)
	lw a5, 6*4(sp)
	lw a4, 5*4(sp)
	lw a3, 4*4(sp)
	lw a2, 3*4(sp)
	lw a1, 2*4(sp)
	lw a0, 1*4(sp)
	lw ra, 0*4(sp)

        addi sp, sp, (+35*4)
	mret
	.endm
/******************************************************************************/


/*******************************************************************************
	FUNCTION DEFINITION
*******************************************************************************/

	/* xPortStartScheduler Function. */
	DECLARE xPortStartScheduler
#if portUSING_MPU_WRAPPERS == 1
        .global prvSetupMPU
        jal ra, prvSetupMPU
#endif //portUSING_MPU_WRAPPERS == 1
	jal ra, prvSetupTimerInterrupt
	la t0, ulCriticalNesting
	sw zero, 0*0(t0)
	portRESTORE_CONTEXT
	.endfunc

/******************************************************************************/
