#include <StateSpaceControl.h>

/*
 * This example shows how this library can be used to control the position of a DC motor using a 3rd order state space model. 
 * For more information on system modeling refer to: http://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=ControlStateSpace
 * We'll assume that the motor current, speed and position are all directly measurable so we'll use full state feedback in place of an estimator.
 */

float UPDATE_PERIOD = 0.01;

StateSpaceController<3,1> controller;

void setup()
{
  Serial.begin(115200);

  // Define the parameters of the system where: J = rotor inertia, b = viscous friction coefficient, R = armature resistance, L_arm = armature inductance
  float J = 0.01, b = 0.1, k = 0.01, R = 1, L_arm = 0.5;

	// Now assuming a state of: [position velocity current]^T we can define the motor's dynamics as follows:

  // Define the system matrix using an array. You could also just set it directly on the controller but this way we can take advantage of the array initialisation syntax
  controller.model.A << 0, 1, 0,
                  0, -b / J, k / J,
                  0, -k / L_arm, -R / L_arm;

	// Define the input matrix
  controller.model.B << 0,
                        0,
                        1 / L_arm;

	// Define the output matrix
  controller.model.C << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1;

	// Define the control law
  controller.K << 12.99, -1;

	// Set a target motor position of 2.2 radians
  controller.r << 2.2;

  controller.initialise();
}

Matrix<3> x; // We'll just simulate the motor using the same model we set on the controller so this x will represent that actual state
Matrix<3> y; // and this'll represent the corresponding system output

void loop()
{
  // Update the control input. Since we're assuming full state feedback we can pass in the entire state here which removes the need for a state estimator
  controller.update(y, UPDATE_PERIOD);

	// Update the actual state using the model in the controller and the current control input
	x += (controller.model.A * x + controller.model.B * controller.u) * UPDATE_PERIOD;

	// Now use the output matrix to convert the actual state into the system output
	y = controller.model.C * x;

  // Now print out the system output so we can see it settling at 2.2 radians
	Serial << y << '\n';

  delay(UPDATE_PERIOD * 1000);
}
