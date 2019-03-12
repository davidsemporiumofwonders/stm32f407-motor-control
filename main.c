#include "stm32f4xx.h"
#include "asm_functions.h"

//todos, questions

//field strength based on calulated speed?
//auto find phase ordering, polartity
//reverse?
//back emf sensing?
//sensorless?, automatic fault detection and switching to sensorless, what type sensors?
//alu and fpu in parralel?
//register variables?
//prolly dont need any context saving in the fpu
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
//ethernet or can?
//cleanup ifdefs, only if the roots let the compiler remove uncalled code/variables
//stacksize?
//init order still allow optimizations?

//defines
#define min_mag_commutation 1
#define n_samples_averaging 1
#define sin60 0.8660254
#define sin120 sin60
#define cos60 0.5
#define cos120 -cos60
#define default_can_adress something
#define prop_delay 0
#define current_per_count 1
#define current_count_midpoint 2048
//#define cheat_at_mtpa //make the gross assumption that load is constant and therefore max speed corresponds to max torque

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

//constants, const gets placed in text! can be changed in linker(relation to literals?)
#ifndef cheat_at_mtpa
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
void track_max_speed_per_amp(void);

extern void TIM2_IRQHandler(void);
extern void TIM3_IRQHandler(void);
#ifdef cheat_at_mtpa
extern void TIM4_IRQHandler(void);
#endif

void init_system(){
	//setup system clocks
	//no reset to initial state for now
	//enable HSE
	RCC->CR = RCC_CR_HSEON;
	//wait till HSE is ready
	while ((RCC->CR & RCC_CR_HSERDY) == 0){}
	//configure art accelerator?
	//configure the Flash Latency cycles and enable prefetch buffer
	//FLASH->ACR;
	//configure the HCLK, ahb, apb prescalers
	//RCC->CFGR;
	//configure and enable PLL
	RCC->PLLCFGR = (7<<RCC_PLLCFGR_PLLQ) & (1<<RCC_PLLCFGR_PLLSRC) & (168<<RCC_PLLCFGR_PLLN) & 4;
	RCC->CR = RCC_CR_PLLON;
	//wait till PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0){}
	//select PLL as system clock source
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	//wait till PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){}

	//init ram
	uint32_t *src, *dst;
	//copy the data segment into ram
	src = &_sidata;
	dst = &_sdata;
	if (src != dst){
		while(dst < &_edata){
			*(dst++) = *(src++);
		}
	}
	//zero the bss segment
	dst = &_sbss;
	while(dst < &_ebss){
		*(dst++) = 0;
	}

	//setup debug?
	//setup fpu, load freq used constants ?
	//setup eeprom emulation?
	//configure peripheral clocks
	RCC->AHB1ENR = 1<<22;//enable dma2, gpio clocks,no |=?
	RCC->APB1ENR = 0;//enable timx
	RCC->APB2ENR = 1<<8 | 1;//enable adc1, usart, tim1 clocks

	//turn on/off peripherals
	//setup gpio
	//GPIOA->MODER;
	//RCC->CR;
	//setup adc
	//setup dma
	//setup ethernet
	//setup usart?
	//negtiote parameters with main controller
	//setup encoder?
	//setup tim1 for deadtime
	//setup 40khz loop on tim2
	//setup stator field timer on tim3?
	#ifdef cheat_at_mpta
		//setup best advance tracking
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
	//filters and averaging?
	//decode values
	i_a = (((float)adc_circ_buffer[0].i_a) - current_count_midpoint) * current_per_count;
	i_b = (((float)adc_circ_buffer[0].i_b) - current_count_midpoint) * current_per_count;
	rotor_e_pos = finvtan2(adc_circ_buffer[0].v_rotor_x, adc_circ_buffer[0].v_rotor_y) * n_polepairs_per_poles_sense_magnet;//wraparound?
	//adjusts
	rotor_e_pos = query_encoder_table(rotor_e_pos);
	speed = rotor_e_pos - prev_rotor_e_pos;//wraparound?
	prev_rotor_e_pos = rotor_e_pos;
	rotor_e_pos = rotor_e_pos + speed * prop_delay;//this can be incoporated in the mpta lut(?)
}

void stator_field_controll(){
	//no contex saving on fpu? -> do manually here
	//100 herz?

}

void track_max_speed_per_amp(){
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
	uint16_t table[]={0b0100000001010000, 0b0101000001010000, 0b0101000001000000, 0b0101000001000000, 0b0100000001000000, 0b0100000001010000};
	TIM1->CCMR1=table[index_best];
	//other register!
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

#ifdef cheat_at_mtpa
void TIM4_IRQHandler(){
	//interrupt priority?
	//dma?
	track_max_speed_per_amp();
}
#endif
