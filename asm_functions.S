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

.section .text.fsine,"ax",%progbits
.balign 4
.thumb_func
	fsine:
	nop

.section .text.fsine_cosine,"ax",%progbits
.balign 4
.thumb_func
	fsine_cosine:
	nop

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
