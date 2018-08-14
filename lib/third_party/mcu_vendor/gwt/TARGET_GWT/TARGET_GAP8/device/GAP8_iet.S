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


        .file "GAP8_iet.S"

/*******************************************************************************
	This file contains the Ecall handler and the Interrupt handler.
*******************************************************************************/

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
	.macro SAVE reg
        .global pxCurrentTCB
	sw ra, 0*4(\reg)
	sw gp, 1*4(\reg)
	sw tp, 2*4(\reg)
	sw a0, 3*4(\reg)
	sw a1, 4*4(\reg)
	sw a2, 5*4(\reg)
	sw a3, 6*4(\reg)
	sw a4, 7*4(\reg)
	sw a5, 8*4(\reg)
	sw a6, 9*4(\reg)
	sw a7, 10*4(\reg)
	sw t0, 11*4(\reg)
	sw t1, 12*4(\reg)
	sw t2, 13*4(\reg)
	sw t3, 14*4(\reg)
	sw t4, 15*4(\reg)
	sw t5, 16*4(\reg)
	sw t6, 17*4(\reg)
	sw s0, 18*4(\reg)
	sw s1, 19*4(\reg)
	sw s2, 20*4(\reg)
	sw s3, 21*4(\reg)
	sw s4, 22*4(\reg)
	sw s5, 23*4(\reg)
	sw s6, 24*4(\reg)
	sw s7, 25*4(\reg)
	sw s8, 26*4(\reg)
	sw s9, 27*4(\reg)
	sw s10, 28*4(\reg)
	sw s11, 29*4(\reg)

        csrr t0, mepc
        sw t0, 30*4(\reg)

        lw t0, pxCurrentTCB
	sw \reg, 0*0(t0)
	.endm
/******************************************************************************/

/******************************************************************************/
	.macro RESTORE reg
        .global pxCurrentTCB

	lw t0, pxCurrentTCB
	lw \reg, 0*0(t0)

	lw t0, 30*4(\reg)
	csrw mepc, t0

	lw ra, 0*4(\reg)
	lw gp, 1*4(\reg)
	lw tp, 2*4(\reg)
	lw a0, 3*4(\reg)
	lw a1, 4*4(\reg)
	lw a2, 5*4(\reg)
	lw a3, 6*4(\reg)
	lw a4, 7*4(\reg)
	lw a5, 8*4(\reg)
	lw a6, 9*4(\reg)
	lw a7, 10*4(\reg)
	lw t0, 11*4(\reg)
	lw t1, 12*4(\reg)
	lw t2, 13*4(\reg)
	lw t3, 14*4(\reg)
	lw t4, 15*4(\reg)
	lw t5, 16*4(\reg)
	lw t6, 17*4(\reg)
	lw s0, 18*4(\reg)
	lw s1, 19*4(\reg)
	lw s2, 20*4(\reg)
	lw s3, 21*4(\reg)
	lw s4, 22*4(\reg)
	lw s5, 23*4(\reg)
	lw s6, 24*4(\reg)
	lw s7, 25*4(\reg)
	lw s8, 26*4(\reg)
	lw s9, 27*4(\reg)
	lw s10, 28*4(\reg)
	lw s11, 29*4(\reg)
	.endm
/******************************************************************************/


/*******************************************************************************
	EXTERNAL VARIABLES & FUNCTIONS
*******************************************************************************/
	.extern _ecall_vector

/******************************************************************************/


/*******************************************************************************
	ECALL HANDLER
*******************************************************************************/
	.section .text, "ax"
	DECLARE ecall_handler
	addi sp, sp, (-9*4)
	/* Save on sp registers used. */
        sw t0, 10*4(sp)
	csrr t0, mepc
        sw t1, 9*4(sp)
	addi t0, t0, 4
        sw ra, 8*4(sp)
	sw t0, 7*4(sp)

	/* Search ecall :  a7 contains ecall number. */
	andi t0, a7, 0xf
	la t1, _ecall_vector
	sll t0, t0, 0x2
	add t1, t1, t0
	lw t0, 0*4(t1)

	/* Execute demanded service. */
	jalr ra, t0

	/* Restore registers used. */
	lw t0, 7*4(sp)
        lw ra, 8*4(sp)
	csrw mepc, t0
        lw t1, 9*4(sp)
        lw t0, 10*4(sp)
	addi sp, sp, (+9*4)
	mret
	.endfunc
/******************************************************************************/


/*******************************************************************************
	IRQ HANDLER
*******************************************************************************/
	/* Default handler, weak func, should be redefined to be overwritten. */
	.weak default_handler
	DECLARE default_handler
	j default_handler
	.endfunc

	/* SysTick_Handler. */
	DECLARE systick_handler
        addi sp, sp, (-30*4)
        SAVE sp
        jal ra, xTaskIncrementTick
        beq a0, zero, _no_switch
#ifdef PREEMPTION
        jal ra, vTaskSwitchContext
#endif
_no_switch:
        RESTORE sp
        addi sp, sp, (+30*4)
        mret
        .endfunc

	/* PendSV_Handler. */
	DECLARE pendSV_handler
	mret
	.endfunc
/******************************************************************************/


/*******************************************************************************
	TO DO : ILL_INS HANDLER
*******************************************************************************/
	DECLARE ill_ins_handler
	j ill_ins_handler
	.endfunc
/******************************************************************************/
