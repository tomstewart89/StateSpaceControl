#include <StateSpaceControl.h>

/*
 * This example shows how this library can be used to control the position of a DC motor using a 3rd order state space model. I won't go into the modelling itself 
 * but for more information on that refer to: http://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=ControlStateSpace
 * We'll assume that the motor current, speed and position are all directly measurable so we'll use full state feedback in place of an estimator.
 */

StateSpaceController<3,1,1, FullStateFeedback> s;
Matrix<3> x; // We'll just simulate the motor using the same model we set on the controller so this x will represent that actual state
Matrix<1> y; // and this'll represent the corresponding system output

void setup() 
{
  Serial.begin(115200);

  // Define the parameters of the system where: J = rotor inertia, b = viscous friction coefficient, R = armature resistance, L = armature inductance
  float J = 0.01, b = 0.1, k = 0.01, R = 1, L = 0.5;

	// Now assuming a state of: [position velocity current]^T we can define the motor's dynamics as follows:

  // Define the system matrix using an array. You could also just set it directly on the controller but this way we can take advantage of the array initialisation syntax
  float F[][3] = {{0, 1, 0},
                  {0, -b / J, k / J},
                  {0, -k / L, -R / L}};

	// Define the input matrix
  float G[][1] = {{0},
                  {0},
                  {1 / L}};

	// Define the output matrix
  float H[][3] = {1, 0, 0};

	// Define the control law
  float K[][3] = {12.99, -1};

	// Now set those matrices to their appropriate members of the controller
  s.systemMatrix = F;
  s.inputMatrix = G;
  s.outputMatrix = H;
  s.controlGain = K;

	// Set a target motor position of 2.2 radians
  s.referenceInput(0) = 2.2;

	// Zero out the actual state
  x.Fill(0);

	// Calculate the precompensator
  s.Initialise();
}

void loop() 
{
	// Update the actual state using the model in the controller and the current control input
	x += (s.systemMatrix * x + s.inputMatrix * s.controlInput) * 0.01;

	// Now use the output matrix to convert the actual state into the system output
	y = s.outputMatrix * x;

  // Update the control input. Since we're assuming full state feedback we can pass in the entire state here which removes the need for a state estimator
	s.Update(x, 0.01);

  // Now print out the system output so we can see it settling at 2.2 radians
	Serial << y << '\n';

  delay(100);
}


