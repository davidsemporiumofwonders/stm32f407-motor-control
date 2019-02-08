#include "stm32f4xx.h"
#include "asm_functions.h"

//this project is based around the method described on: http://build-its-inprogress.blogspot.nl/2017/05/vector-hysteresis-foc.html
//my math is NOT(at all!) trustworthy enough to be doing this: evaluate it yourself alswell

//12 o clock is 0 degrees, looking from output side
//coil a is at 0 degrees other coils are named clock wise and placed in 120 degree increments
//positive current on a coil means flowing out of the controller(from the phase connections, obviously not ground wiseguy) and the resultant magnetic field pointing outward from rotor axle
//si units, angles in full revolutions
//[0,0] is on rotor axle, x axis is horizontal(9 to 3 o'clock), y vertical, both axises(is that a word?) are fixed to the stator, incrementing to right and upwards repectively

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

//first complete , then optimize!

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

volatile adc_conversions adc_circ_buffer[sizeof(adc_conversions)*n_samples_averaging] __attribute__ ((aligned (2)));

const float mtpa_lut[][1];//datatype?, enough ram?, search algorithm?

const float rotary_resolver_correction_lut[];//needed?, datatype?

const uint8_t switch_states[] = {1,3,2,6,4,12};

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
float requested_torque;// or pass this like this as arguments?

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
	float calced_cur_c = (float)(-adc_circ_buffer[1].i_a - adc_circ_buffer[1].i_b);

	//place currents in space
	vector cur_a;
	vector cur_b;
	vector cur_c;

	cur_a.x = 0;
	cur_a.y = (float)(adc_circ_buffer[0].i_a);
	cur_b.x = (float)(adc_circ_buffer[0].i_b)*cos120;
	cur_b.y = cur_b.x*-sin120/cos120;
	cur_c.x = calced_cur_c*cos120;
	cur_c.y = calced_cur_c*sin120;

	//total current vector
	vector cur_total;
	cur_total.x = cur_a.x + cur_b.x + cur_c.x;
	cur_total.y = cur_a.y + cur_b.y + cur_c.y;

	//setup rotor vector in mag ang
	vector rotor;

	rotor.x = adc_circ_buffer[0].v_rotor_x * hall_x_scaling;
	rotor.y = adc_circ_buffer[0].v_rotor_y * hall_y_scaling;

	vector_mag_ang rotor_mag_ang;
	rotor_mag_ang.mag=fsqrt(rotor.x*rotor.x + rotor.y*rotor.y);
	rotor_mag_ang.ang=finvtan2(rotor.x,rotor.y);

	//corrections rotor vector
	//calculate speed?
	//convert mechanical degrees to electrical degrees(move these up to data processing?)

	//setup ref vector in xy
	float temp = rotor_mag_ang.mag * requested_torque * torque_per_amp;
	vector ref;
	fsine_cosine(rotor_mag_ang.ang + mtpa_lut[0][0],(float*) &ref.y,(float*) &ref.x);//get rid of the cos(the entire perspective switch really(how conducive are the corrections to this?)) by having the rotation lut return cos a and sin a
	ref.x *= temp;
	ref.y *= temp;//scaling correct?
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

	//pass best to deadtime gen

	//load 32 bit literal(same used to initialize?)
	//load 8 bit imm with on bit set
	//shift betsindex right, update flags
	//shift 8bit imm 8*bestindex left
	//(condition carry flag) or 8bit with 8lsl 8bit
	//or 8bit and 32 bit
	//write 32 bit to registers
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
