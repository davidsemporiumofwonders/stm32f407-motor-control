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

.section .text.svm_asm,"ax",%progbits
.balign 4
.thumb_func
	svm_asm:
	nop

.section .text.fsine,"ax",%progbits
.balign 4
.thumb_func
	fsine:// context saving? handling nans and shit, you did fourth order you dipshit
	vcvt.u32.f32 s3,s3,#32
	vmov r3,s3
	vcvt.f32.u32 s3,s3,#32//conv back?
	lsrs.n r3,r3,#31
	it cs//wont need a cycle bc previous is 16 bit
	vsubcs.f32 s3,s0,s3
	vmul.f32 s3,s3,s3//x^2
	vmov.f32 s4,s1//b->s4, remove movs aswell? using mov correct?
	vmls.f32 s4,s3,s2//b-x^2*c, versus vfmls
	vmov.f32 s5,s0//a->s5, fpu needs an extra cycle if result is used right away: so this order
	vmls.f32 s5,s4,s3//a-s4*s3
	//restore r3?
	it ne
	vnegne.f32 s6,s5

.section .text.fsine_cosine,"ax",%progbits
.balign 4
.thumb_func
	fsine_cosine://once again 4th order calculates cosine dipshit
	vcvt.u32.f32 s3,s3,#32
	push {r3}//do this as an str to make use of store buffer? or will this happen anyway
	vmov r3,s3
	vcvt.f32.u32 s3,s3,#32
	vmov.f32 s5,s3//do that branch differently?
	lsls.n r3,r3,#1
	ite mi
	vsubmi.f32 s3,s0,s3
	vsubpl.f32 s5,s0,s5
	vmul.f32 s3,s3,s3//x^2
	vmul.f32 s5,s5,s5//x^2
	vmul.f32 s4,s3,s2//x^2*c
	vmul.f32 s6,s5,s2//x^2*c
	vsub.f32 s4,s1,s4//b-x^2*c
	vsub.f32 s6,s1,s6//b-x^2*c
	vmul.f32 s4,s4,s3
	vmul.f32 s6,s6,s5
	vsub.f32 s4,s0,s4
	vsub.f32 s6,s0,s6
	pop {r3}
	itt cs
	vnegcs.f32 s4,s4
	vnegcs.f32 s6,s6
	it mi
	vnegmi.f32 s6,s6


.section .text.finvtan2,"ax",%progbits
.balign 4
.thumb_func
	finvtan2:
	nop

.section .text.fsqrt,"ax",%progbits
.balign 4
.thumb_func
	fsqrt:
	nop
