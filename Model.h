using namespace BLA;

template<int X, int U, int Y> struct Model
{
    Matrix<X,X> A;
    Matrix<X,U> B;
    Matrix<Y,X> C;
    Matrix<Y,U> D;
};

/*
 * This model describes the classic inverted pendulum control problem. In this system, a stick mounted on the top of a cart via passive revolute joint. The task is to keep this upright by 
 * by accelerating the cart backwards and forwards. The actual modeling for this came from http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
 * which defines the state as:
 * 
 * x := [cart_position, cart_velocity, stick_angle, stick_angular_rate]^T
 * 
 * And it is assumed that 1: a force is directly applied to the cart and 2: that the system is equipped with sensors that can directly measure the cart's displacement and its stick angle.
 * 
 * To parameterise the system the following physical measurements are required:
 * 
 * M := mass of the cart                       (kg)
 * m := mass of the pendulum                   (kg)
 * b := coefficient of friction for cart       (N/m/s)
 * l := length to pendulum center of mass      (m)
 * I := mass moment of inertia of the pendulum (kg.m^2)
 */
struct CartPoleModel : public Model<4,1,2>
{
  CartPoleModel(float M, float m, float b, float l, float I, float g = 9.81)
  {
    float c = (I * (M + m) + M * m * l * l);

    // Now assuming a state of: [cart_position cart_velocity stick_angle stick_rate]^T we can define the inverted pendulum's dynamics as follows:

    // Define the system matrix
    A << 0, 1, 0, 0,
         0, -(I + m * l * l) * b / c, m * m * g * l * l / c, 0,
         0, 0, 0, 1,
         0, -m * l * b / c, m * g * l * (M + m) / c, 0;

    // Define the input matrix
    B << 0,
         (I + m * l * l) / c,
         0,
         (m * l) / c;

    // Define the output matrix
    C << 1, 0, 0, 0,
         0, 0, 1, 0;
  }  
};
