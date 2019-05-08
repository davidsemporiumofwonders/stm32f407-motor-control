#include "cos_table.h"
#include "invtan_table.h"

typedef struct sine_cosine_struct{
	float sine;
	float cosine;
} sine_cosine_struct;

extern void svm_correct_current_towards_asm(void);
extern sine_cosine_struct fsine_cosine(float i);
extern ufix32 fix32invtan(ufix32 i);
extern float fsqrt(float i);
