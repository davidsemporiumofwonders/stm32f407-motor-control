#include "stm32f4xx.h"
#include "typedefs.h"
#include "asm_functions.h"
#include "motor_controll_luts.h"

//todos, questions

//limit rate or skip on small error?
//field weakening, how to console this with mtpa?
//max speed per amp tracking sould be max acceleration per amp tracking when not at steady speed(trasition?)(?)
//auto find phase ordering, polartity
//reverse?
//back emf sensing?
//sensorless, automatic fault detection and switching to sensorless, what type sensors?
//sensor less? -> speed under x? -> just apply a rotating field
//dont check for sensorless sometimes when doing things for it?
//register variables?
//s16 and up need to be preserved use those for variables, no context saving on fpu
//the compiler now pushes d8-d10 can my fpu handle this, have i selected the right one
//see if you can do better than 40khz
//different clock domains?
//data/instruction caching in both core and st implementation?
//ideally you would request torque from the controller
//add torque per amp lookup table?
//ethernet or can?
//interrupt (subroutine calls?) stacking behavior?
//interrupt priorities, turn of nestng globally?
//max speed per amp vs max speed per energy?, mtpa vs mppa?
//peak detection once per x rotations?
//only 4 32 bit instructions of prefetch, 5 waitstates->problem?
//also check core prefetch for sequential 32 bit instructions
//divide by zero handling?
//dsp instuctions!(?)
//exc_return?(in arm refman)
//do adc averaging?
//fixed point?
//just do a conventional layout of stack and vector table?
//usefullness of dma?
//3 adcs(?)
//try to contain data tapping to one small block
//software fpu stacking?
//stacks on subroutine call?, unnessecary stacking, naked funtions, or inlines(would make some global values locals/reg variables(even as inlines instead of one big function?))
//fpu regs to store table adresses?
//back emf measurement: wont the current/inductance produce a volatge and foul the reading?, what did i have in mind for this?
//derive torque(in cobination with current) from bemf(?)
//consider a torque sensor though
//how to handle non sinusoidial bemf?!

//defines
#define ram_bank_2 __attribute__((section(".ram_bank_2")))
#define align_2 __attribute__ ((aligned (2)))
#define no_wrapper __attribute__((naked))//do this differently(?)

#define min_err_commutation 0//square of current!
#define peak_detection_window 1
#define peak_detection_level 1
#define first_time_peak_detection_window 1
#define n_samples 1
#define default_can_adress 1
#define current_per_count 1
#define current_midpoint 50
//#define adjust_rotor_field
//#define usartout
//#define force_sensorless
//#use_hall

enum {
	i_a_adc1_input = 0,
	i_a_adc2_input = 1,
	i_b_adc3_input = 2,
	i_b_adc1_input = 3,
	v_rotor_x_adc2_input = 4,
	v_rotor_x_adc3_input = 5,
	v_rotor_y_adc1_input = 6,
	v_rotor_y_adc2_input = 7,
}; //adc_reg_channels_vs_inputs

enum {
	v_bemf_adc1 = 8,
	temp_adc2 = 9,
	v_dcbus_adc3 = 10,
}; //adc_inj_channels_vs_inputs

typedef struct {
	uint16_t i_a_adc1;
	uint16_t i_a_adc2;
	uint16_t i_b_adc3;
	uint16_t i_b_adc1;
	uint16_t v_rotor_x_adc2;
	uint16_t v_rotor_x_adc3;
	uint16_t v_rotor_y_adc1;
	uint16_t v_rotor_y_adc2;
	//extra, needed?
}regular_adc_conversions;

typedef struct {
	uint16_t v_bemf_adc1;
	uint16_t temp_adc2;
	uint16_t v_dcbus_adc3;
}injected_adc_conversions;

extern uint32_t _sdata;
extern uint32_t _sidata;
extern uint32_t _eidata;//correctly placed in linker?
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _sccmram;
extern uint32_t _svtflash;
extern uint32_t _evtflash;//provides in linker?

//variables set at init, as variables?
float max_speed;
float max_current;
float ov_lockout;
float uv_lockout;
float ov_lockout_release;
float uv_lockout_release;//floats?
uint8_t n_polepairs_per_pairs_sense_magnet;//pairs!

uint8_t faultcodes[8];//flash eeprom emulation
uint8_t faultcodes_index;

//dynamic variables
//register float i_a __asm__("s16");//which values do i save in registers?
float i_a;
float i_b;
float i_c;
float angle_advance;
float prev_torque_per_amp;
float torque_per_amp;
ufix32 prev_rotor_e_pos;
ufix32 rotor_e_pos;
//somedatatype speed;
bool commutating;
bool sensorless;

volatile float requested_current;
volatile regular_adc_conversions adc_circ_buffer[n_samples] ram_bank_2 align_2;// array indexing get optimized away when size is 1?, deine align atribute?

//prototypes
void no_wrapper init_system(void);//does no_wrapper do what i think it does here?
void main(void);
void copy_mem_section(uint32_t* begin_dest, uint32_t* begin_source, uint32_t* end_source);
void zero_mem_section(uint32_t* begin_dest, uint32_t* end_dest);
void wait(uint32_t ticks);
void calibrate_encoder(void);
ufix32 query_encoder_table(ufix32 pos);
void process_data(void);
vector_mag_ang calculate_desired_current(void);
void svm_correct_current_towards(vector_mag_ang);//inline these functions?, what will i then do for the calibrate encoder function?
void optimize_advance_angle(void);
void write_to_flash(uint32_t* data_start, uint32_t* dest_start, uint16_t length);

extern void ADC_IRQHandler(void);
extern void TIM2_IRQHandler(void);
extern void TIM4_IRQHandler(void);

void no_wrapper init_system(){
	//cant use stack here

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

	//initialize ram
	copy_mem_section(&_sdata, &_sidata, &_eidata);
	//zero out uninitialized variables
	zero_mem_section(&_sbss, &_ebss);

    //bank2 isnt initialized or zeroed out!

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
	RCC->APB2ENR = RCC_APB2ENR_TIM1EN & RCC_APB2ENR_USART1EN & RCC_APB2ENR_ADC1EN & RCC_APB2ENR_ADC2EN;//add others

	//copy vector table to ccram
	copy_mem_section(&_sccmram, &_svtflash, &_evtflash);

	//ccram isnt initialized or zeroed out!

	//use vector table in ccram
	SCB->VTOR = &_sccmram;

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

	//setup adc, basically redo these, order
	ADC->CCR = (0b10 << 16) & (0b10 << 14) & ADC_CCR_DMA & 0b101;//combined regular and injected mode?

	ADC1->CR1 = ADC_CR1_AWDEN & ADC_CR1_AWDSGL & ADC_CR1_SCAN & ADC_CR1_AWDIE & 0;//set anawd on cur a
	ADC1->CR2 = ADC_CR2_CONT & ADC_CR2_ADON;

	ADC1->SMPR1;
	ADC1->HTR;
	ADC1->LTR;
	ADC1->SQR1 = ((sizeof(regular_adc_conversions)/sizeof(uint16_t)/2)<<20);
	ADC1->SQR3;
	ADC1->JSQR;

	ADC2->CR1 = ADC_CR1_AWDEN & ADC_CR1_AWDSGL & ADC_CR1_SCAN & ADC_CR1_AWDIE & 0;//set anawd on temp
	ADC2->CR2 = ADC_CR2_CONT & ADC_CR2_ADON;
	ADC2->SMPR1;
	ADC2->HTR;
	ADC2->LTR;
	ADC2->SQR1 = ((sizeof(regular_adc_conversions)/sizeof(uint16_t)/2)<<20);
	ADC2->SQR3;
	ADC2->JSQR;

	//setup dma, basically redo these
	DMA2_Stream0->PAR = &ADC->CDR;
	DMA2_Stream0->M0AR = &adc_circ_buffer;
	DMA2_Stream0->NDTR = 8;
	DMA2_Stream0->CR = 0b11 << 16;
	DMA2_Stream0->FCR = 0b101;
	DMA2_Stream0->CR = (0b10 << 23) & (0b01 << 13) & (0b10 << 11) & DMA_SxCR_MINC & DMA_SxCR_CIRC;
	DMA2_Stream0->CR = DMA_SxCR_EN;

	//setup encoder?

	//setup tim1 for deadtime
	TIM1->CCER = TIM_CCER_CC1E & TIM_CCER_CC1NE & TIM_CCER_CC2E & TIM_CCER_CC2NE & TIM_CCER_CC3E & TIM_CCER_CC3NE;//complentary output polarity?
	TIM1->BDTR = TIM_BDTR_MOE & 16;
	TIM1->CCMR1 = 0b0100000001000000;
	TIM1->CCMR2 = 0b0100000001000000;

	//setup  peak detect / backemf timer on tim2?
	TIM2->CR1 = TIM_CR1_CEN;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->PSC;
	TIM2->CCR1;

	//setup sensorless/hall position timer on tim 3?

#ifdef track_max_speed_per_power
	//setup best advance tracking on tim4?
	//100 herz?
#endif

	//setup interrupts, priorities
	//NVIC->ISER;

	main();
}

void main(){
	while(1){
		if(commutating){
			process_data();
			svm_correct_current_towards(calculate_desired_current());
			//limit rate?
		}
	}
}

void zero_mem_section(uint32_t* begin_dest, uint32_t* end_dest){
	while(begin_dest < end_dest){
		*(begin_dest++) = 0;
	}
}

void copy_mem_section(uint32_t* begin_dest, uint32_t* begin_source, uint32_t* end_source){
	while(begin_source < end_source){
		*(begin_dest++) = *(begin_source++);
	}
}

void process_data(){
	//add branch for sensorless
	//turn of dma? or give processor higher priority

	//filters and averaging
	uint16_t raw_a = (adc_circ_buffer[0].i_a_adc1 + adc_circ_buffer[0].i_a_adc2) / 2;
	uint16_t raw_b = (adc_circ_buffer[0].i_b_adc1 + adc_circ_buffer[0].i_b_adc3) / 2;
	uint16_t raw_x = (adc_circ_buffer[0].v_rotor_x_adc2 + adc_circ_buffer[0].v_rotor_x_adc3) / 2;
	uint16_t raw_y = (adc_circ_buffer[0].v_rotor_y_adc1 + adc_circ_buffer[0].v_rotor_y_adc2) / 2;

	//decode values
	i_a = ((float)raw_a) * current_per_count - current_midpoint;
	i_b = ((float)raw_b) * current_per_count - current_midpoint;//as vars or just pass as argument?
	rotor_e_pos = fix32invtan(raw_x / raw_y) * n_polepairs_per_pairs_sense_magnet;//does wraparound also work like this on multiplication?,check asm here

	//adjusts
	rotor_e_pos = query_encoder_table(rotor_e_pos);

	//derive others
	//calculate speed, must be filtered: only calculate once every x calls?
}

void optimize_advance_angle(){

}

vector_mag_ang calculate_desired_current(){
	vector_mag_ang out;

	out.ang = (float)rotor_e_pos + angle_advance;
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

	sine_cosine_struct sine_cosine = fsine_cosine(ref_current.ang);
	vector ref;
	ref.x = sine_cosine.sine * ref_current.mag;
	ref.y = sine_cosine.cosine * ref_current.mag;

	//get error
	vector error;
	error.x = ref.x - cur_total.x;
	error.y = ref.y - cur_total.y;

	//ignore if error is too small
	if (error.x*error.x + error.y*error.y < min_err_commutation){
		return;
	}

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
	follow_field.mag = 15;
	while(follow_field.ang < 0.8){
		follow_field.ang += 0.0001;
		process_data();
		svm_correct_current_towards(follow_field);
		//record data
	}
}


ufix32 query_encoder_table(ufix32 pos){// as asm function?
	return pos;
}

void wait(uint32_t ticks){

}

void write_to_flash(uint32_t* data_start, uint32_t* dest_start, uint16_t length){

}

void ADC_IRQHandler(){
	//find out what triggered the interrupt
}

void TIM2_IRQHandler(){
	// lower the amount of calls when not sensorless?

	uint16_t prev_raw_bemf = 2^16;
	//float coils
	TIM1->CCMR1 = 0b0100000001000000;
	TIM1->CCMR2 = 0b0100000001000000;
	wait(10);

	//wait for peak
	while(ADC1->SR & ADC_SR_JEOC){
		uint16_t raw_bemf = ADC1->DR;
		ADC1->CR2 |= ADC_CR2_SWSTART;//bitbanding?, all adcs in synch?
		if(prev_raw_bemf - raw_bemf > peak_detection_level){
			break;//doest this break the upper loop?
		}
		prev_raw_bemf = raw_bemf;
	}

	//adjust postion timer prescaler if sensorless

	//do troque estimation

	//ensure new reg samples are available
	wait(10);//ensure in a different way?
}

#ifdef track_max_speed_per_power
void TIM4_IRQHandler(){
	//interrupt priority?
	//dma?
	optimize_advance_angle();
}
#endif
