#include "stm32f4xx.h"
#include "asm_functions.h"

//field strength based on calulated speed
//auto find phase ordering, polartity
//absolute vs relative position?
//reverse?
//can
//(synchronous?)automated adc or software initalized?
//dummy loops for back emf sensing?
//sensorless?, automatic fault detection and switching to sensorless
//alu and fpu in parralel?
//own math fuctions?
//profiling arm code?
//register variables?
//check out automatic stacking behavior of fpu
//unaligned access
//why am i using so much ram withput code
//are we armv7e-m , checkout sel instruction
//float accuracy
//prolly dont need any context saving in the fpu
//funtion inlining

#define radius_hysteresis_circle 1
#define n_samples_averaging 1
#define sin120 0.8660254037844
#define cos120 -0.5
#define default_can_adress something

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

//cosine table

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
uint8_t n_polepairs;//pairs!, 8 bits?, constant?, technically this number should be npp/npp_sensemagnet
float torque_per_amp;

uint8_t faultcodes[8];//flash eeprom emulation
uint8_t faultcodes_index;

float hall_x_scaling;
float hall_y_scaling;
float i_a;
float i_b;
float i_c;
vector rotor;
float volatile requested_torque;

//prototypes
void wait(void);
void process_data(void);
void svm(void);
void field_controll(void);
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
	//filters?
	//decode values
	//adjust for non linearity?
	//adjust for delay?

}

void field_controll(){

}

void svm(){
	// normalize hall?
	//calculate the third coil current
	float i_c = (float)(-i_a - i_b);

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

	vector_mag_ang rotor_mag_ang;
	rotor_mag_ang.mag=fsqrt(rotor.x*rotor.x + rotor.y*rotor.y);
	rotor_mag_ang.ang=finvtan2(rotor.x,rotor.y);

	//corrections rotor vector
	//calculate speed?
	//convert mechanical degrees to electrical degrees(move these up to data processing?)

	//setup ref vector in xy
	float temp = rotor_mag_ang.mag * requested_torque * torque_per_amp;
	vector ref;
	struct twofloats sine_cosine = fsine_cosine(rotor_mag_ang.ang + mtpa_lut[0][0]);//get rid of the cos(the entire perspective switch really(how conducive are the corrections to this?)) by having the rotation lut return cos a and sin a
	ref.x = sine_cosine.value1*temp;//cosine and sine correctly paired?
	ref.y = sine_cosine.value0*temp;//scaling correct?
	//get error
	vector error;
	error.x = ref.x - cur_total.x;
	error.y = ref.y - cur_total.y;
	//ignore if error is too small?

	uint8_t index_best = 0;
	float best = 0;
	//get best action
	for(uint8_t i = 0; i < sizeof(switch_state_vectors); i++){
		float temp = switch_state_vectors[i].x*error.x + switch_state_vectors[i].y*error.y < best;
		if(temp > best){
			best=temp;
			index_best=i;
		}
	}

	RTC->ALRMAR=index_best;//write to dummy register of now to prevent optimising out
	//pass best to deadtime gen

	//load 32 bit literal(same used to initialize?)
	//load 8 bit imm with on bit set
	//shift betsindex right, update flags
	//shift 8bit imm 8*bestindex left
	//(condition carry flag) or 8bit with 8lsl 8bit
	//or 8bit and 32 bit
	//write 32 bit to registers


	/*
	 * ldr r0, =0b01000000010000000100000001000000
	 * mov r1, #0b10000
	 */
}

float interpolate_mtpa_table(float speed, float current){
	return 0;
}

void wait(){

}


void TIM2_IRQHandler(){
	process_data();
	svm();
}
