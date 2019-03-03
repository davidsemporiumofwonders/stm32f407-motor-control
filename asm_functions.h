#include "costable.h"

struct twofloats{
	float value0;
	float value1;
};

extern void svm_asm(void);
extern float fsine(float i);
extern struct twofloats fsine_cosine(float i);
extern float finvtan2(float i0, float i1);
extern float fsqrt(float i);
