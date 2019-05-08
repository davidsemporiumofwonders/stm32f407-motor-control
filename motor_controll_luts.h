//datatypes, sizes, in ram?

#define sin60 0.8660254
#define sin120 sin60
#define cos60 0.5
#define cos120 -cos60

const float mtpa_lut[64][64];

const uint16_t rotary_resolver_correction_lut[128];

//const float current_per_NM[];

const vector switch_state_vectors[] = { {0,1},
										{sin60, cos60},
										{sin120, cos120},
										{0,-1},
										{-sin60, -cos60},
										{-sin120,-cos120} };//index 0 is coil a high, rest off, increasing the index sees the resultant magnetic field progress clockwise
