using namespace BLA;

/*
 * This file defines a Model class to represent a state space model. Model has three template parameters:
 *
 * X := the number of states
 * U := the number of control inputs (inputs to the model, outputs from the controller)
 * Y := the number of outputs (sensor measurements of the system which are fed back to the controller)
 *
 * You can define any state space model by declaring a Model<X,U,Y> and filling out the system matrics, A, B, C & D
 * appropriately. I've also added a couple of subclasses of Model to represent model that are commonly used in state
 * space control, those being DC motor control for servos and Cart Pole for inverted pendulum problems. I'm planning on
 * expanding on this list but in the meantime feel free to PR if you come up with one of your own that might be widely
 * useful!
 */
template <int X, int U, int Y = X>
struct Model
{
    const static int states = X;
    const static int inputs = U;
    const static int outputs = Y;

    Matrix<X, X> A;
    Matrix<X, U> B;
    Matrix<Y, X> C;
    Matrix<Y, U> D;
};

/*
 * This model describes the classic inverted pendulum control problem. In this system, a stick mounted on the top of a
 * cart via passive revolute joint. The task is to keep this upright by by accelerating the cart backwards and forwards.
 * The actual modeling for this came from
 * http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling which defines the state
 * as:
 *
 * x := [cart_position, cart_velocity, stick_angle, stick_angular_rate]^T
 *
 * And it is assumed that 1: a force is directly applied to the cart and 2: that the system is equipped with sensors
 * that can directly measure the cart's displacement and its stick angle.
 *
 * To parameterise the system the following physical measurements are required:
 *
 * M := mass of the cart                       (kg)
 * m := mass of the pendulum                   (kg)
 * b := coefficient of friction for cart       (N/m/s)
 * l := length to pendulum center of mass      (m)
 * I := mass moment of inertia of the pendulum (kg.m^2)
 */
struct CartPoleModel : public Model<4, 1, 2>
{
    CartPoleModel(float M, float m, float b, float l, float I, float g = 9.81)
    {
        float c = (I * (M + m) + M * m * l * l);

        // Define the system matrix
        A = {0, 1, 0, 0, 0, -(I + m * l * l) * b / c, m * m * g * l * l / c,   0,
             0, 0, 0, 1, 0, -m * l * b / c,           m * g * l * (M + m) / c, 0};

        // Define the input matrix
        B = {0, (I + m * l * l) / c, 0, (m * l) / c};

        // Define the output matrix
        C = {1, 0, 0, 0, 0, 0, 1, 0};

        // Define the direct transmission matrix
        D = Zeros<2, 1>();
    }
};

/*
 * This model describes the position of a DC motor based on a voltage input. The actual modeling for this came from
 * http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling which defines the state
 * as:
 *
 * x := [cart_position, cart_velocity, stick_angle, stick_angular_rate]^T
 *
 * And it is assumed that 1: the controller manipulates the voltage supplied to the motor and 2: the motor position,
 * velocity and current are all directly measureable meaning that the system has full state feedback
 *
 * To parameterise the system the following physical measurements are required:
 *
 * J := rotor inertia                       (kg.m^2)
 * b = viscous friction coefficient         (N.m.s)
 * K = back emf constant
 * R = armature resistance                  (ohm)
 * L = armature inductance                  (H)
 */
struct MotorPositionModel : public Model<3, 1>
{
    MotorPositionModel(float J, float b, float k, float R, float L)
    {
        // Define the system matrix
        A = {0, 1, 0, 0, -b / J, k / J, 0, -k / L, -R / L};

        // Define the input matrix
        B = {0, 0, 1 / L};

        // Define the output matrix
        C = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        // Define the direct transmission matrix
        D = Zeros<3, 1>();
    }
};
