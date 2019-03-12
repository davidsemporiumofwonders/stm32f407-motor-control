.syntax unified
.cpu cortex-m4
.thumb

.global svm_asm
.global fsine
.global fsine_cosine
.global finvtan2
.global fsqrt

//contents of assemly funtions (even seperate files in general) are not visible to the compiler
//use luts
//load constants at startup, how to teel the compiler not to touch them?
//returnin 2 values in c

.section .text.svm_asm,"ax",%progbits
.balign 4
.thumb_func
	svm_asm:
	nop
	/* add best, best , iets
	 * ldr r0, =0x40010018
	 * ldr r1, [pc, index_best]
	 * strh r1, [r0], #4
	 * lsl r1, r1, #16
	 * strh r1, [r0]
	 * bx//?
	 * .4byte 0b01000000010000000100000001010000
	 * .4byte 0b01000000010000000101000001010000
	 * .4byte 0b01000000010000000101000001000000
	 * .4byte 0b01000000010100000101000001000000
	 * .4byte 0b01000000010100000100000001000000
	 * .4byte 0b01000000010100000100000001010000
	 * .pool
	 */

.section .text.fsine,"ax",%progbits
.balign 4
.thumb_func
	fsine:
	nop

.section .text.fsine_cosine,"ax",%progbits
.balign 4
.thumb_func
	fsine_cosine://no linear interpolation for now
	    vcvt.u32.f32 s0,s0,#32
        vmov r0,s0
        lsls.n r1,r0,#1
		mov r1,r0//do that branch/copy differently?
        ite mi
        submi r0, #2^32
        subpl r1, #2^32//check pairing register, condition
		//right shift
        .equ    label, cosine_table
		movw    r2, #:lower16:label//when the table is placed at the top of ram this instruction does nothing
		movt    r2, #:upper16:label
        ldr r3, [r2, r0]
        vmov r3, s0
        ldr r3, [r2, r1]
        vmov r3, s1
        itt cs
        vnegcs.f32 s0,s0
        vnegcs.f32 s1,s1
        it mi
        vnegmi.f32 s1,s1



.section .text.finvtan2,"ax",%progbits
.balign 4
.thumb_func
	finvtan2:
	nop

.section .text.fsqrt,"ax",%progbits
.balign 4
.thumb_func
	fsqrt:
	vsqrt.f32 s0, s0
