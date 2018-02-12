#include <StateSpaceControl.h>

/*
   This example shows how this library can be used to balance control the position of an inverted pendulum using a 4th order state space model. I won't go into the modelling itself
   but for more information on that refer to: http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace
   We'll assume that only the cart position and the stick angle are measureable and we'll recover the rest of the state using an estimator
*/


float UPDATE_PERIOD = 0.01;

// Define a cart pole model. I'm planning on including a whole range of different dynamic system models in Models.h, but so far cart pole is the only one. 
CartPoleModel model(0.5, 0.2, 0.1, 0.3, 0.006);

// Define a conroller with estimation and reference tracking
StateSpaceController<4, 1, 2, true, true> controller(model);

void setup()
{
  Serial.begin(115200);

  // Define the control law
  controller.K << -70.7107, -37.8345, 105.5298, 20.9238;

  // Define the estimator gains
  controller.L << 12.61, 0.02,
                  29.51, 2.34,
                  0.02, 19.30,
                  -1.67, 135.98;

  // Are you wondering where these numbers were pulled from? If so, head over to utils/tune_those_gains.ipynb

  controller.initialise();

  // Try to bring the cart to rest at 3.5m from it's home position
  controller.r << 3.5, 0.0;
}

// We'll just simulate the motor using the same model we set on the controller so this x will represent that actual state. 
// To make things a little more difficult, we'll set some initial conditions on the state.
Matrix<4> x(-1.0, 2.0, 0.2, -0.1);
Matrix<2> y; // and this'll represent the corresponding system output

void loop()
{
  controller.update(y, UPDATE_PERIOD);

  // Update the actual state using the model in the controller and the current control input
  x += (controller.model.A * x + controller.model.B * controller.u) * UPDATE_PERIOD;

  // Now use the output matrix to convert the actual state into the system output
  y = controller.model.C * x;

  // Now print out the system output so we can see it settling at {3.5m, 0rads}
  Serial << y << '\n';

  delay(UPDATE_PERIOD * 1000);
}
