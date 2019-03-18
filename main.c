#include "stm32f4xx.h"
#include "asm_functions.h"

//todos, questions

//field strength based on calulated speed?
//auto find phase ordering, polartity
//reverse?
//back emf sensing?
//sensorless?, automatic fault detection and switching to sensorless, what type sensors?
//sensor less? -> speed under x? -> just apply a rotating field
//alu and fpu in parralel?
//register variables?
//prolly dont need any context saving in the fpu(set on a per function basis?)
//s16 and up need to be preserved use those for variables
//the compiler now pushes d8-d10 can my fpu handle this, have i selected the right one
//core coupled memory, different ram banks?
//see if you can do better than 40khz
//interleaved adc conversions, or parralel?
//injected channels for bus measurement?
//anawd to generated interrupt on bus values?
//different clock domains?
//backemf sensing on higset priority timer interrupt -> wait for peak and set timer accordingly?
//data/instruction caching in both core and st implementation?
//ideally you would request torque from the controller
//add torque per amp lookup table?
//ethernet or can?
//stacksize?
//init order still allow optimizations?
//interrupt (subroutine calls?) stacking behavior?
//interrupt priorities, turn of nestng globally? (i would be nice if advance tracking could be interrupted though)
//max speed per amp vs max speed per energy?
//peak detection once per x rotations?
//calculate speed in peak detection?
//only 4 32 bit instructions of prefetch, 5 waitstates->problem?
//also check core prefetch for sequential 32 bit instructions
//io compensation cell?
//divide by zero handling?
//dsp instuctions!(?)
//stack in ram(does the cpu always lock out the dma when stacking)?
//exc_return?(in arm refman)
//mpu?

//defines
#define CCMRAM __attribute__((section(".ccmram")))

#define min_err_commutation 1
#define peak_detection_window 1
#define n_samples_averaging 1
#define sin60 0.8660254
#define sin120 sin60
#define cos60 0.5
#define cos120 -cos60
#define default_can_adress something
#define prop_delay 0
#define current_per_count 1
#define current_midpoint 50
//#define track_max_speed_per_power

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
	uint16_t v_dcbus;
	uint16_t i_dcbus;
	uint16_t i_a;
	uint16_t i_b;
	uint16_t i_f;
	uint16_t v_rotor_x;
	uint16_t v_rotor_y;
}adc_conversions;

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _sccmram;
extern uint32_t _svtflash;
extern uint32_t _evtflash;


//constants, const gets placed in text! can be changed in linker(relation to literals?)
#ifndef track_max_speed_per_power
const float mtpa_lut[][1];//datatype?
#endif

const uint8_t rotary_resolver_correction_lut[];

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
float prev_rotor_e_pos;
float rotor_e_pos;
float speed;
float current_angle_advance;
float prev_speed_per_amp;

volatile float requested_current;
volatile adc_conversions adc_circ_buffer[sizeof(adc_conversions)*n_samples_averaging] __attribute__ ((aligned (2)));//align on 4 bytes?

//prototypes
void init_system(void);
void main(void);
void wait(uint32_t ticks);
void calibrate_encoder(void);
float query_encoder_table(float pos);
float query_mtpa_table(float speed, float current);
void process_data(void);
vector_mag_ang calculate_desired_current(void);
void svm_correct_current_towards(vector_mag_ang);
void stator_field_controll(void);
void optimize_advance_angle(void);

extern void TIM2_IRQHandler(void);
extern void TIM3_IRQHandler(void);
extern void TIM4_IRQHandler(void);

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

	//relocate vector table to ccram
	SCB->VTOR = &_sccmram;

	//turn on/off peripherals?

	//setup gpio
//	GPIOA->MODER;//function
//	GPIOA->OTYPER;//output type
//	GPIOA->OSPEEDR;
//	GPIOA->PUPDR;//pullups
//	GPIOA->AFR;//alternate function select

	//setup usart?
	//setup adc
	ADC1->CR1 = (0b00<<24) & ADC_CR1_AWDIE;//res?, jauto?, single anawd channel(use anawd at all?)

	//setup dma

	//setup encoder?

	//setup tim1 for deadtime
	TIM1->CR1=0;//set deadtime prescaler
	TIM1->CCER = TIM_CCER_CC1E & TIM_CCER_CC1NE & TIM_CCER_CC2E & TIM_CCER_CC2NE & TIM_CCER_CC3E & TIM_CCER_CC3NE;//complentary output polarity?
	TIM1->BDTR = TIM_BDTR_MOE ;//set deadtime

	//setup 40khz loop on tim2
	TIM2->CR1 = TIM_CR1_CEN;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->PSC = 10;
	TIM2->CCR1 = 10;

	//setup stator field timer on tim3?
	TIM3->CR1 = TIM_CR1_CEN;
	TIM3->DIER = TIM_DIER_UIE;
	TIM3->EGR = TIM_EGR_UG;
	TIM3->PSC = 10;
	TIM3->CCR1 = 10;

#ifdef track_max_speed_per_power
	//setup best advance tracking on tim4?
#endif

	//setup ethernet/can

	//negtiote parameters with main controller

	//setup interrupts, priorities
	//NVIC->ISER;

	main();
}

void main(){
	while(1){

	}
}

void process_data(){
	//filters and averaging?
	//decode values
	i_a = ((float)adc_circ_buffer[0].i_a) * current_per_count - current_midpoint;
	i_b = ((float)adc_circ_buffer[0].i_b) * current_per_count - current_midpoint;
	rotor_e_pos = finvtan2(adc_circ_buffer[0].v_rotor_x, adc_circ_buffer[0].v_rotor_y) * n_polepairs_per_poles_sense_magnet;//have arctan output 32bit fixed point keep it there till svm. does wraparound also work like this on multiplication?
	//adjusts
	rotor_e_pos = query_encoder_table(rotor_e_pos);
	speed = rotor_e_pos - prev_rotor_e_pos;
	prev_rotor_e_pos = rotor_e_pos;
    //rotor_e_pos = rotor_e_pos + speed * prop_delay;//adjust for propogation delay, this can be incoporated in the mpta lut(?), do it at all?
}

void stator_field_controll(){
	//no contex saving on fpu? -> do manually here

}

void optimize_advance_angle(){
	//no contex saving on fpu? -> do manually here
	//100 herz?

}

vector_mag_ang calculate_desired_current(){
	vector_mag_ang out;
	out.ang = rotor_e_pos + query_mtpa_table(1,1);
	out.mag = requested_current * torque_per_amp;

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
	//init override bits?
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

float query_mtpa_table(float speed, float current){
	return 0;
}

float query_encoder_table(float pos){
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
	stator_field_controll();
}

#ifdef track_max_speed_per_power
void TIM4_IRQHandler(){
	//interrupt priority?
	//dma?
	optimize_advance_angle();
}
#endif
