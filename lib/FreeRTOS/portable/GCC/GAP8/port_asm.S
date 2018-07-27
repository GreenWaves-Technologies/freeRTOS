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

/**************************************
	MACRO DEFINITION
***************************************/

/**************************************/
	.macro DECLARE Routine
	.global \Routine
	.func \Routine
	.type \Routine, %function
	\Routine:
	.endm
/*************************************/

/*************************************/
	.macro portSAVE_CONTEXT
	.global pxCurrentTCB
	addi sp, sp, (-30*4)

	sw ra, 0*4(sp)
	sw gp, 1*4(sp)
	sw tp, 2*4(sp)
	sw a0, 3*4(sp)
	sw a1, 4*4(sp)
	sw a2, 5*4(sp)
	sw a3, 6*4(sp)
	sw a4, 7*4(sp)
	sw a5, 8*4(sp)
	sw a6, 9*4(sp)
	sw a7, 10*4(sp)
	sw t0, 11*4(sp)
	sw t1, 12*4(sp)
	sw t2, 13*4(sp)
	sw t3, 14*4(sp)
	sw t4, 15*4(sp)
	sw t5, 16*4(sp)
	sw t6, 17*4(sp)
	sw s0, 18*4(sp)
	sw s1, 19*4(sp)
	sw s2, 20*4(sp)
	sw s3, 21*4(sp)
	sw s4, 22*4(sp)
	sw s5, 23*4(sp)
	sw s6, 24*4(sp)
	sw s7, 25*4(sp)
	sw s8, 26*4(sp)
	sw s9, 27*4(sp)
	sw s10, 28*4(sp)
	sw s11, 29*4(sp)

	lw t0, pxCurrentTCB
	sw sp, 0*0(t0)
	.endm
/*************************************/

/*************************************/
	.macro portSAVE_EPC
	csrr t0, mepc
	sw t0, 30*4(sp)
	.endm
/*************************************/

/*************************************/
	.macro portSAVE_RA
	sw ra, 30*4(sp)
	.endm
/*************************************/

/*************************************/
	.macro portRESTORE_CONTEXT
	.global pxCurrentTCB

	lw t0, pxCurrentTCB
	lw sp, 0*0(t0)

	lw t0, 30*4(sp)
	csrw mepc, t0

	li t0, 0x1880
	csrw mstatus, t0

	lw ra, 0*4(sp)
	lw gp, 1*4(sp)
	lw tp, 2*4(sp)
	lw a0, 3*4(sp)
	lw a1, 4*4(sp)
	lw a2, 5*4(sp)
	lw a3, 6*4(sp)
	lw a4, 7*4(sp)
	lw a5, 8*4(sp)
	lw a6, 9*4(sp)
	lw a7, 10*4(sp)
	lw t0, 11*4(sp)
	lw t1, 12*4(sp)
	lw t2, 13*4(sp)
	lw t3, 14*4(sp)
	lw t4, 15*4(sp)
	lw t5, 16*4(sp)
	lw t6, 17*4(sp)
	lw s0, 18*4(sp)
	lw s1, 19*4(sp)
	lw s2, 20*4(sp)
	lw s3, 21*4(sp)
	lw s4, 22*4(sp)
	lw s5, 23*4(sp)
	lw s6, 24*4(sp)
	lw s7, 25*4(sp)
	lw s8, 26*4(sp)
	lw s9, 27*4(sp)
	lw s10, 28*4(sp)
	lw s11, 29*4(sp)

	addi sp, sp, (+30*4)
	mret
	.endm
/*************************************/


/**************************************
	FUNCTION DEFINITION
**************************************/

	/* xPortStartScheduler Function. */
	DECLARE xPortStartScheduler
	.global prvSetupTimerInterrupt
	.global ulCriticalNesting
	jal ra, prvSetupTimerInterrupt
	la t0, ulCriticalNesting
	sw zero, 0*0(t0)
	portRESTORE_CONTEXT
	.endfunc


	/* vPortYield Function. */
	DECLARE vPortYield
	portSAVE_CONTEXT
	portSAVE_RA
	jal ra, vTaskSwitchContext
	portRESTORE_CONTEXT
	.endfunc
