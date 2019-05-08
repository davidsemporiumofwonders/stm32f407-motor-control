.syntax unified
.cpu cortex-m4
.thumb

.global svm_correct_current_towards_asm
.global fsine_cosine
.global fix32invtan
.global fsqrt

//contents of assemly funtions (even seperate files in general) are not visible to the compiler
//load constants at startup, how to teel the compiler not to touch them?

.section .text.svm_correct_current_towards_asm,"ax",%progbits
.balign 4
.thumb_func
	svm_correct_current_towards_asm:
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

.section .text.fsine_cosine,"ax",%progbits
.balign 4
.thumb_func
	fsine_cosine://no linear interpolation for now
		push {r2}
	    vcvt.u32.f32 s0,s0,#32
	    .equ    label, cosine_table
		movw    r2, #:lower16:label
		movt    r2, #:upper16:label//keep adresses like this in a fixed register?
        vmov r0,s0
        lsls.n r1,r0,#1
		mov r1,r0//do that branch/copy differently?
        ite mi
        submi r0, #2^32
        subpl r1, #2^32//check pairing register, condition
		lsr r0, #18//right amount?
		lsr r1, #18
        ldr r0, [r2, r0]
        ldr r1, [r2, r1]
        vmov r0, s0//argument order correct?
        vmov r1, s1//move as one?
        pop {r2}
        itt cs
        vnegcs.f32 s0,s0
        vnegcs.f32 s1,s1//does neg also take an extra cyle if consumed immedeately?, this can be doen without fpu
        it mi
        vnegmi.f32 s1,s1

.section .text.fix32invtan,"ax",%progbits
.balign 4
.thumb_func
	fix32invtan:
	nop

.section .text.fsqrt,"ax",%progbits
.balign 4
.thumb_func
	fsqrt:
	vsqrt.f32 s0, s0
