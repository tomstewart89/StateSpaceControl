#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "Model.h"

using namespace BLA;


template<bool ENABLE, int X, int Y = 1> struct EnableMatrixIf
{
  typedef Matrix<X, Y> T;
};


template<int X, int Y> struct EnableMatrixIf<false, X, Y>
{
  typedef Zeros<X, Y> T;
};


template <int X, int U, int Y=X, bool EnableEstimation=false, bool EnableIntegralControl=false, bool EnableReferenceTracking=true>
class StateSpaceController
{
    typename EnableMatrixIf<EnableEstimation, X, X>::T ALC;
    typename EnableMatrixIf<EnableReferenceTracking, U, Y>::T N_bar;

  public:
    // System model
    const Model<X, U, Y> &model;

    // Control variables
    Matrix<X> x_hat; // state estimate
    Matrix<U> u; // control input
    typename EnableMatrixIf<EnableReferenceTracking, Y>::T r; // reference input (assumed to be of the same dimension as the observation y)
    typename EnableMatrixIf<EnableIntegralControl, U>::T w_hat; // estimate of a disturbance / error in the system model (used by the integral controller)

    // Control Gains
    Matrix<U, X> K; // regulator Gain
    typename EnableMatrixIf<EnableEstimation, X, Y>::T L; // estimator gain
    typename EnableMatrixIf<EnableIntegralControl, U, Y>::T I; // integral control gain

    StateSpaceController(const Model<X, U, Y>& _model) : model(_model), x_hat(Zeros<X>()), u(Zeros<U>()), r(Zeros<Y>()), w_hat(Zeros<U>()) { }

    void initialise()
    {
      // If reference tracking is enabled we'll need to precalculate Nbar which maps the reference input to a control input offset
      if (EnableReferenceTracking) {
        auto sys = (model.A || model.B) && (model.C || model.D);

        // Find an inverse for the aggregated matrix
        Matrix <X+U,X+Y> sysInv;

        // case 1: more outputs than inputs - find the left inverse
        if(model.inputs < model.outputs) {
            sysInv = (~sys * sys).Inverse() * ~sys;
        }
        // case 2: more than, or the same number of outputs as inputs - find the right inverse
        else {
            sysInv = ~sys * (sys * ~sys).Inverse();
        }

        // Split it up and multiply it with K to find NBar
        N_bar = K * sysInv.Submatrix(Slice<0, X>(), Slice <X,X+Y> ()) + sysInv.Submatrix(Slice <X,X+U > (), Slice <X,X+Y> ());
      }

      // If estimation is enabled we can also save a bit of processing by precalculating the expression: A - L * C
      if (EnableEstimation) {
        ALC = model.A - L * model.C;
      }
    }

    void update(const Matrix<Y> &y, const float dt=0.0f)
    {
      // If estimation is enabled, update the state estimate
      if (EnableEstimation) {
        x_hat += (ALC * x_hat + model.B * u + L * y) * dt;
      }
      // If not, then we assume that the entire state is fed back to the controller as y (i.e X == Y)
      else {
        static_assert(X == Y || EnableEstimation, "Estimation must be enabled if the state is only partially observed (i.e len(y) != len(x) )");
        x_hat.Submatrix(Slice<0, Y>(), Slice <0, 1>()) = y;
      }

      // Calculate the control input required to drive the state to 0.
      u = -K * x_hat;

      // If reference tracking is enabled then offset the control input to drive the state to the reference input r
      if (EnableReferenceTracking) {
        u += N_bar * r;
      }

      // If integral control is enabled then windup the control input to offset a (presumably) constant disturbance w
      if (EnableIntegralControl) {
        w_hat += I * (y - r) * dt;
        u += w_hat;
      }
    }
};


template<int X, int U, int Y=X> class Simulation
{
public:
  Matrix<X> x;

  const Model<X,U,Y>& model;

  Simulation(const Model<X,U,Y>& _model) : model(_model), x(Zeros<X>()) { }

  Matrix<Y> step(const Matrix<U>& u, const float dt)
  {
      x += (model.A * x + model.B * u) * dt;
      return model.C * x;
  }
};

#endif // STATE_SPACE_CONTROL_H
