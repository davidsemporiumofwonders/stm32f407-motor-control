#include "stm32f4xx.h"
#include "asm_functions.h"

//todos, questions

//field strength based on calulated speed?
//auto find phase ordering, polartity
//reverse?
//back emf sensing?
//sensorless?, automatic fault detection and switching to sensorless, what type sensors?
//sensor less? -> speed under x? -> just apply a rotating field
//register variables?
//prolly dont need any context saving in the fpu(set on a per function basis?)
//s16 and up need to be preserved use those for variables
//the compiler now pushes d8-d10 can my fpu handle this, have i selected the right one
//see if you can do better than 40khz
//adc setup?
//lower resolution on some adc channels?
//safeties as external interrupt?
//different clock domains?
//backemf sensing on higset priority timer interrupt -> wait for peak and set timer accordingly?
//data/instruction caching in both core and st implementation?
//ideally you would request torque from the controller
//add torque per amp lookup table?
//ethernet or can?
//stacksize?
//interrupt (subroutine calls?) stacking behavior?
//interrupt priorities, turn of nestng globally? (i would be nice if advance tracking could be interrupted though)
//max speed per amp vs max speed per energy?
//peak detection once per x rotations?
//only 4 32 bit instructions of prefetch, 5 waitstates->problem?
//also check core prefetch for sequential 32 bit instructions
//divide by zero handling?
//dsp instuctions!(?)
//stack in ccmram(does the cpu always lock out the dma when stacking)?
//exc_return?(in arm refman)
//all(more) angles(values) in fix 32?

//defines
#define CCMRAM __attribute__((section(".ccmram")))

#define min_err_commutation 1
#define peak_detection_window 1
#define n_samples 1
#define sin60 0.8660254
#define sin120 sin60
#define cos60 0.5
#define cos120 -cos60
#define default_can_adress something
#define prop_delay 0
#define current_per_count 1
#define current_midpoint 50
//#define track_max_speed_per_power
//#define usartout

//typedefs
typedef struct {
	float x;
	float y;
}vector;

typedef struct {
	float mag;
	float ang;
}vector_mag_ang;

typedef struct {
	uint16_t i_a_adc1;
	uint16_t i_a_adc2;
	uint16_t i_b_adc1;
	uint16_t i_b_adc2;
	uint16_t i_f_adc1;
	uint16_t i_f_adc2;
	uint16_t v_rotor_x_adc1;
	uint16_t v_rotor_x_adc2;
	uint16_t v_rotor_y_adc1;
	uint16_t v_rotor_y_adc2;
}regular_adc_conversions;

typedef struct {
	uint16_t temp_adc1;
	uint16_t temp_adc2;
	uint16_t v_dcbus_adc1;
	uint16_t v_dcbus_adc2;
	uint16_t i_dcbus_adc1;
	uint16_t i_dcbus_adc2;
}injected_dc_conversions;

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _sccmram;
extern uint32_t _svtflash;
extern uint32_t _evtflash;


//constants, const gets placed in text! can be changed in linker(relation to literals?)
const float mtpa_lut[][1];//datatype?

const uint32_t rotary_resolver_correction_lut[];//datatype?

const vector switch_state_vectors[] = { {0,1},
										{sin60, cos60},
										{sin120, cos120},
										{0,-1},
										{-sin60, -cos60},
										{-sin120,-cos120} };//index 0 is coil a high, rest off, increasing the index sees the resultant magnetic field progress clockwise

//variables set at init
float max_speed;
float max_current;
float ov_lockout;
float uv_lockout;
float ov_lockout_release;
float uv_lockout_release;//floats?
uint8_t n_polepairs_per_poles_sense_magnet;//pairs!
float torque_per_amp;

uint8_t faultcodes[8];//flash eeprom emulation
uint8_t faultcodes_index;

//dynamic variables
//register float i_a __asm__("s16");//which values do i save in registers?
float i_a;
float i_b;
float i_c;
float angle_advance;
float prev_speed_per_amp;
ufix32 prev_rotor_e_pos;
ufix32 rotor_e_pos;
ufix32 speed;										//be mindful when multiplying fixed point numbers!

volatile float requested_current;
volatile regular_adc_conversions adc_circ_buffer[n_samples] __attribute__ ((aligned (2)));//align on 4 bytes?, array indexing get optimized away when size is 1?

//prototypes
void init_system(void);
void main(void);
void wait(uint32_t ticks);
void calibrate_encoder(void);
ufix32 query_encoder_table(ufix32 pos);
float query_mtpa_table(ufix32 speed, float current);
void process_data(void);
vector_mag_ang calculate_desired_current(void);
void svm_correct_current_towards(vector_mag_ang);
void stator_field_controll(void);
void optimize_advance_angle(void);

extern void TIM2_IRQHandler(void);
extern void TIM3_IRQHandler(void);
extern void TIM5_IRQHandler(void);

void init_system(){
	//setup system clocks
	//enable HSE
	RCC->CR = RCC_CR_HSION & RCC_CR_HSEON;
	//wait till HSE is ready
	while ((RCC->CR & RCC_CR_HSERDY) == 0){}
	//configure art accelerator?
	//configure the Flash Latency cycles and enable prefetch buffer
	FLASH->ACR = FLASH_ACR_LATENCY_5WS & FLASH_ACR_ICEN & FLASH_ACR_ICEN & FLASH_ACR_PRFTEN;
	//configure the ahb, apb prescalers
	RCC->CFGR = RCC_CFGR_PPRE1_DIV2 & RCC_CFGR_PPRE2_DIV2;//delay in completion!
	//configure and enable PLL
	RCC->PLLCFGR = (7<<24) & RCC_PLLCFGR_PLLSRC & (168<<6) & 4;
	RCC->CR |= RCC_CR_PLLON;
	//wait till PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0){}
	//select PLL as system clock source
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	//wait till PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){}

	//copy the data segment into ram
	uint32_t *src, *dst;
	src = &_sidata;
	dst = &_sdata;
	while(dst < &_edata){
		*(dst++) = *(src++);
	}
	//zero the bss segment
	dst = &_sbss;
	while(dst < &_ebss){
		*(dst++) = 0;
	}

	//setup pvd
	PWR->CR = PWR_CR_VOS_0 & PWR_CR_PVDE & (0b110<<5);

	//setup fpu, load freq used constants
	SCB->CPACR = 0b1111<<20;
	//asm volatile("vmov.f32 s5, #0");
	//synch barriers?

	//setup eeprom emulation?

	//configure peripheral clocks
	RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN & RCC_AHB1ENR_CCMDATARAMEN & RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN & RCC_APB1ENR_TIM3EN & RCC_APB1ENR_TIM4EN;
	RCC->APB2ENR = RCC_APB2ENR_TIM1EN & RCC_APB2ENR_USART1EN & RCC_APB2ENR_ADC1EN & RCC_APB2ENR_ADC2EN;

	//copy vector table to ccram
	src = &_svtflash;
	dst = &_sccmram;
	while(src < &_evtflash){
		*(dst++) = *(src++);
	}

	//use vector table in ccram
	SCB->VTOR = &_sccmram;

	//turn on/off peripherals?

	//setup gpio
//	GPIOA->MODER;//function
//	GPIOA->OTYPER;//output type
//	GPIOA->OSPEEDR;
//	GPIOA->PUPDR;//pullups
//	GPIOA->AFR;//alternate function select

#ifdef usartout
	//setup usart
#endif

	//setup ethernet/can

	//negtiote parameters with main controller

	//safety checks

	//setup adc
	ADC->CCR = (0b10 << 16) & (0b10 << 14) & ADC_CCR_DMA & 0b101;//combined regular and injected mode?
	ADC1->CR1 = ADC_CR1_AWDEN & ADC_CR1_AWDSGL & ADC_CR1_SCAN & ADC_CR1_AWDIE & 0;
	ADC1->CR2 = ADC_CR2_CONT & ADC_CR2_ADON;
	ADC1->SMPR1;
	ADC1->HTR;
	ADC1->LTR;
	ADC1->SQR1 = ((sizeof(regular_adc_conversions)/sizeof(uint16_t)/2)<<20);
	ADC1->SQR3;
	//ADC2;

	//setup dma

	//setup encoder?

	//setup tim1 for deadtime
	TIM1->CR1=0;//set deadtime prescaler
	TIM1->CCER = TIM_CCER_CC1E & TIM_CCER_CC1NE & TIM_CCER_CC2E & TIM_CCER_CC2NE & TIM_CCER_CC3E & TIM_CCER_CC3NE;//complentary output polarity?
	TIM1->BDTR = TIM_BDTR_MOE ;//set deadtime
	//init override bits?

	//setup 40khz loop on tim2
	TIM2->CR1 = TIM_CR1_CEN;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->PSC = 10;
	TIM2->CCR1 = 10;

	//setup backemf timer on tim3?
	TIM3->CR1 = TIM_CR1_CEN;
	TIM3->DIER = TIM_DIER_UIE;
	TIM3->EGR = TIM_EGR_UG;
	TIM3->PSC = 10;
	TIM3->CCR1 = 10;

	//setup injected channels timer? on tim4?

#ifdef track_max_speed_per_power
	//setup best advance tracking on tim5?
	//100 herz?
#endif

	//setup interrupts, priorities
	//NVIC->ISER;

	main();
}

void main(){
	while(1){

	}
}

void process_data(){
	//filters and averaging
	uint16_t raw_a = (adc_circ_buffer[0].i_a_adc1 + adc_circ_buffer[0].i_a_adc2) / 2;
	uint16_t raw_b = (adc_circ_buffer[0].i_b_adc1 + adc_circ_buffer[0].i_b_adc2) / 2;
	uint16_t raw_x = (adc_circ_buffer[0].v_rotor_x_adc1 + adc_circ_buffer[0].v_rotor_x_adc2) / 2;
	uint16_t raw_y = (adc_circ_buffer[0].v_rotor_y_adc1 + adc_circ_buffer[0].v_rotor_y_adc2) / 2;
	//decode values
	i_a = ((float)raw_a) * current_per_count - current_midpoint;//do substraction in adc offset register(can you switch around injected and regular)?
	i_b = ((float)raw_b) * current_per_count - current_midpoint;
	rotor_e_pos = fix32invtan2(raw_x, raw_y) * n_polepairs_per_poles_sense_magnet;//does wraparound also work like this on multiplication?
	//adjusts
	rotor_e_pos = query_encoder_table(rotor_e_pos);
	speed = rotor_e_pos - prev_rotor_e_pos;
	prev_rotor_e_pos = rotor_e_pos;
    //rotor_e_pos = rotor_e_pos + speed * prop_delay;//adjust for propogation delay, this can be incoporated in the mpta lut(?), do it at all?
}

void stator_field_controll(){

}

void optimize_advance_angle(){

}

vector_mag_ang calculate_desired_current(){
	vector_mag_ang out;

#ifdef track_max_speed_per_power
	out.ang = (float)rotor_e_pos + angle_advance;
	out.mag = requested_current * torque_per_amp;
#else
	out.ang = (float)rotor_e_pos + query_mtpa_table(speed, requested_current);
	out.mag = requested_current * torque_per_amp;
#endif

	return out;
}

void svm_correct_current_towards(vector_mag_ang ref_current){
	//calculate the third coil current

	i_c = -i_a - i_b;

	//place currents in space
	vector cur_a;
	vector cur_b;
	vector cur_c;

	cur_a.x = 0;
	cur_a.y = i_a;
	cur_b.x = i_b * cos120;
	cur_b.y = i_b * -sin120;
	cur_c.x = i_c * cos120;
	cur_c.y = i_c * sin120;

	//total current vector
	vector cur_total;
	cur_total.x = cur_a.x + cur_b.x + cur_c.x;
	cur_total.y = cur_a.y + cur_b.y + cur_c.y;

	struct twofloats sine_cosine = fsine_cosine(ref_current.ang);
	vector ref;
	ref.x = sine_cosine.value0 * ref_current.mag;
	ref.y = sine_cosine.value1 * ref_current.mag;

	//get error
	vector error;
	error.x = ref.x - cur_total.x;
	error.y = ref.y - cur_total.y;

	//ignore if error is too small?

	//get best action
	uint8_t index_best = 0;
	float best = 0;
	for(uint8_t i = 0; i < sizeof(switch_state_vectors)/sizeof(vector); i++){
		float temp = switch_state_vectors[i].x * error.x + switch_state_vectors[i].y * error.y;
		if(temp > best){
			best = temp;
			index_best = i;
		}
	}

	//pass best to deadtime gen
	uint16_t table0[]={0b0100000001010000, 0b0101000001010000, 0b0101000001000000, 0b0101000001000000, 0b0100000001000000, 0b0100000001010000};
	uint16_t table1[]={0b0100000001000000, 0b0100000001000000, 0b0100000001000000, 0b0100000001010000, 0b0100000001010000, 0b0100000001010000};
	TIM1->CCMR1=table0[index_best];
	TIM1->CCMR2=table1[index_best];
}

void calibrate_encoder(){
	//offload (parts) to pc?
	vector_mag_ang follow_field;
	follow_field.mag = 20;
	while(follow_field.ang < 0.8){
		follow_field.ang += 0.005;
		svm_correct_current_towards(follow_field);// or commutate on timer?
		wait(10);//values!?
		//record data
	}
}

float query_mtpa_table(ufix32 speed, float current){
	return 0;
}

ufix32 query_encoder_table(ufix32 pos){
	return pos;
}

void wait(uint32_t ticks){

}

void TIM2_IRQHandler(){
	//turn off dma
	process_data();
	svm_correct_current_towards(calculate_desired_current());
}

void TIM3_IRQHandler(){
	//dma?
	//wait for peak
	stator_field_controll();
}

#ifdef track_max_speed_per_power
void TIM4_IRQHandler(){
	//interrupt priority?
	//dma?
	optimize_advance_angle();
}
#endif
