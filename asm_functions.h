#include "costable.h"
#include "atantable.h"

typedef uint32_t ufix32;
typedef uint32_t ufix16;

struct twofloats{
	float value0;
	float value1;
};

extern void svm_asm(void);
extern float fsine(float i);
extern struct twofloats fsine_cosine(float i);
extern float finvtan2(float i0, float i1);
extern ufix32 fix32invtan2(ufix32 i0, ufix32 i1);//single input?
extern float fsqrt(float i);
