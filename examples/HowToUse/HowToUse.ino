#include <StateSpaceControl.h>

/*
 * Proper explanation coming soon
 */

StateSpaceController<4,1,2> s;
Matrix<2> sensorRead;

void setup() 
{
  Serial.begin(115200);

	float M = 0.5;
	float m = 0.2;
	float b = 0.1;
	float l = 0.3;
	float I = 0.006;
	float g = 9.81;

  // Fill out the state transition matrix
	s.A(0,1) = 1;
	s.A(1,1) = -(I + m * l * l) * b / (I * (M + m) + M * m * l * l);
	s.A(1,2) = m * m * g * l* l / (I * (M + m) + M * m * l * l);
	s.A(3,1) = -m * l * b / (I * (M + m) + M * m * l * l);
	s.A(3,2) = m * g * l * (M + m) / (I * (M + m) + M * m * l * l);
	s.A(2,3) = 1;

  // the input matrix
	s.B(1) = (I + m * l * l) / (I * (M + m) + M * m * l * l);
	s.B(4) = (m * l) / (I * (M + m) + M * m * l * l);

  // the observation matrix
	s.C(0,0) = 1;
	s.C(1,2) = 1;

  // Now let's have a look at it all
	Serial << s.sys;

  // and the feedback gain matrix
	s.K(0) = -0.9384;
	s.K(1) = -1.5656;
	s.K(2) = 18.0351;
	s.K(3) = 3.3368;

  // Finally, calculate the reference gain
	s.Initialise();
}

void loop() 
{
  // Iterate the feedback loop
	s.Update(sensorRead, 0.05);
}

