svm foc motor controll on stm32f407 based around the method described on: http://build-its-inprogress.blogspot.nl/2017/05/vector-hysteresis-foc.html
my math is not really trustworthy enough to be doing this: evaluate it yourself alswell

//conventions:

//12 o clock is 0 degrees, looking from output side
//coil a is at 0 degrees other coils are named clock wise and placed in 120 degree increments
//positive current on a coil means flowing out of the controller(from the phase connections, obviously not ground wiseguy) and the resultant magnetic field pointing outward from rotor axle
//si units, angles in full revolutions
//[0,0] is on rotor axle, x axis is horizontal(9 to 3 o'clock), y vertical, both axises(is that a word?) are fixed to the stator, incrementing to right and upwards repectively
