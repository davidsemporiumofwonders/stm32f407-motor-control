#include "stm32f4xx.h"
#include "asm_functions.h"

//field strength based on calulated speed
//auto find phase ordering, polartity
//absolute vs relative position?
//reverse?
//can
//(synchronous?)automated adc or software initalized?
//back emf sensing?
//sensorless?, automatic fault detection and switching to sensorless
//alu and fpu in parralel?
//register variables?
//check out automatic stacking behavior of fpu
//checkout sel instruction
//prolly dont need any context saving in the fpu
//s16 and need to be preserved use those for variables
//the compile now pushes d8-d10 can my fpu handle this, have i selected the right one
//(turn on debugging info again later)

#define min_mag_commutation 1
#define n_samples_averaging 1
#define sin120 0.8660254037844
#define cos120 -0.5
#define default_can_adress something
//register float i_a __asm__("s16");

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

volatile adc_conversions adc_circ_buffer[sizeof(adc_conversions)*n_samples_averaging] __attribute__ ((aligned (2)));//align on 4 bytes?

const float mtpa_lut[][1];//datatype?, search algorithm?

const uint8_t rotary_resolver_correction_lut[];//const gets placed in text! can be changed in linker

const vector switch_state_vectors[] = { {0,0},
										{0,0},
										{0,0},
										{0,0},
										{0,0},
										{0,0} };//index 0 is coil a high, rest off, increasing the index sees the resultant magnetic field progress clockwise

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

float i_a;
float i_b;
float i_c;
float prev_rotor_ang;
float rotor_ang;
float speed;
float volatile requested_current;//which values do i save in registers?

//prototypes
void wait(void);
void process_data(void);
vector_mag_ang calculate_desired_current(void);
void svm_correct_current_towards(vector_mag_ang);
void stator_field_controll(void);
float interpolate_mtpa_table(float speed, float current);
extern void TIM2_IRQHandler(void);

void main(){
	//configure clocks
	//turn on/off peripherals
	//setup gpio
	//setup tim1 for deadtime
	//setup 40khz loop
	//setup adc
	//setup fpu, load freq used constants
	//setup can
	//setup dmas
	//setup encoder
	//negtiote parameters with main controller
	while(1){

	}
}

void process_data(){
	//filters and averaging?
	//decode values

	rotor_ang = finvtan2(adc_circ_buffer[0].v_rotor_x, adc_circ_buffer[0].v_rotor_y);
	//corrections rotor vector
	//calculate speed
	//convert mechanical degrees to electrical degrees
	//adjust for delay?

}

void stator_field_controll(){

}

vector_mag_ang calculate_desired_current(){
	vector_mag_ang out;
	out.ang = rotor_ang + interpolate_mtpa_table(1,1);
	out.mag = requested_current * torque_per_amp;

	return out;
}

void svm_correct_current_towards(vector_mag_ang ref_current){
	//calculate the third coil current

	i_c = (float)(-i_a - i_b);

	//place currents in space
	vector cur_a;
	vector cur_b;
	vector cur_c;

	cur_a.x = 0;
	cur_a.y = (float)(i_a);
	cur_b.x = (float)(i_b)*cos120;
	cur_b.y = cur_b.x*-sin120/cos120;
	cur_c.x = i_c*cos120;
	cur_c.y = i_c*sin120;

	//total current vector
	vector cur_total;
	cur_total.x = cur_a.x + cur_b.x + cur_c.x;
	cur_total.y = cur_a.y + cur_b.y + cur_c.y;

	struct twofloats sine_cosine = fsine_cosine(ref_current.ang);
	vector ref;
	ref.x = sine_cosine.value1 * ref_current.mag;//cosine and sine correctly paired?
	ref.y = sine_cosine.value0 * ref_current.mag;

	//get error
	vector error;
	error.x = ref.x - cur_total.x;
	error.y = ref.y - cur_total.y;

	//ignore if error is too small?

	//get best action
	uint8_t index_best = 0;
	float best = 0;
	for(uint8_t i = 0; i < sizeof(switch_state_vectors); i++){
		float temp = switch_state_vectors[i].x*error.x + switch_state_vectors[i].y*error.y < best;
		if(temp > best){
			best=temp;
			index_best=i;
		}
	}

	//pass best to deadtime gen
	//init override bits?
	uint16_t table[]={0b0100000001010000, 0b0101000001010000, 0b0101000001000000, 0b0101000001000000, 0b0100000001000000, 0b0100000001010000};
	TIM1->CCMR1=table[index_best];


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
}

float interpolate_mtpa_table(float speed, float current){
	return 0;
}

void wait(){

}

void TIM2_IRQHandler(){
	process_data();
	svm_correct_current_towards(calculate_desired_current());
}
