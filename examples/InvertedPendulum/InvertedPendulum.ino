#include <StateSpaceControl.h>

/*
 * This example shows how this library can be used to control the position of an inverted pendulum using a 4th order state space model. I won't go into the modelling itself 
 * but for more information on that refer to: http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace
 * We'll assume that only the cart position and the stick angle are measureable and we'll recover the rest of the state using an autonomous estimator
 */

StateSpaceController<4, 1, 2, Autonomous> s;
Matrix<4> x; // We'll just simulate the motor using the same model we set on the controller so this x will represent that actual state
Matrix<2> y; // and this'll represent the corresponding system output

void setup()
{
  Serial.begin(115200);

  // Define the parameters of the system
  float M = 0.5, m = 0.2, b = 0.1, l = 0.3, I = 0.006, g = 9.81;
  float  C = (I * (M + m) + M * m * l * l);

  // Now assuming a state of: [cart_position cart_velocity stick_angle stick_rate]^T we can define the inverted pendulum's dynamics as follows:

  // Define the system matrix using an array. You could also just set it directly on the controller but this way we can take advantage of the array initialisation syntax
  float F[][4] = {{0, 1, 0, 0},
                  {0, -(I + m * l * l) * b / C, m * m * g * l * l / C, 0},
                  {0, 0, 0, 1},
                  {0, -m * l * b / C, m * g * l * (M + m) / C, 0}};

  // Define the input matrix
  float G[][1] = {{0},
                  {(I + m * l * l) / C},
                  {0},
                  {(m * l) / C}};

  // Define the output matrix
  float H[][4] = {{1, 0, 0, 0},
                  {0, 0, 1, 0}};

  // Define the control law
  float K[][4] = {-70.7107, -37.8345, 105.5298, 20.9238};

  // Define the estimator gains
  float L[][2] = {{8.26, -0.1}, {169.92, -4.02},
                  {-0.14, 8.32},{-76.2, 176.04}};

  // Now set those matrices to their appropriate members of the controller
  s.systemMatrix = F;
  s.inputMatrix = G;
  s.outputMatrix = H;

  s.controlGain = K;
  s.estimatorGain = L;
  
  // Set some non-zero initial conditions so the estimator really has to work for it
  float initialConditions[][1] = {1, 2, 0.2, -0.1};
  x = initialConditions;

  // Try to bring the cart to rest at 3.5m from it's home position
  s.referenceInput(0) = 3.5;

  // Calculate the precompensator
  s.Initialise();
}

void loop()
{
  // Update the actual state using the model in the controller and the current control input
  x += (s.systemMatrix * x + s.inputMatrix * s.controlInput) * 0.01;

  // Now use the output matrix to convert the actual state into the system output 
  y = s.outputMatrix * x;

  // Update the control input. Since we can only measure parts of the state we'll pass in what we can observe (y) and let the estimator figure out the rest
  s.Update(y, 0.01);

  // Now print out the system output so we can see it settling at {3.5m, 0rads}
  Serial << y << '\n';

  delay(100);
}

