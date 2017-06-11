#ifndef STATE_SPACE_BLOCKS_H
#define STATE_SPACE_BLOCKS_H

#include "StateSpaceControl.h"

template<int N> struct SummingJunction : public Block<N>
{
    Input<N> a, b;
    SummingJunction() : a(*this), b(*this) { }
    const Matrix<N> &compute() const { return a() + b(); }
};


template<int N_X, int N_U> struct Gain : public Block<N_U>
{
    Input<N_X> x;
    Matrix<N_U,N_X> K;

    Gain() : x(*this) { }
    const Matrix<N_U> compute() const { return -x() * K; }
};


template<int N_Y, int N_U, int N_X> struct StateEstimator : public Block<N_X>
{
    Matrix<N_X, N_X> F;
    Matrix<N_X, N_U> G;
    Matrix<N_Y,N_X> H;
    Matrix<N_X, N_Y> L;
    Matrix<N_X, N_X> FLH;

    Matrix<N_X> x_hat;
    Matrix<1> t_1; // t-1

    Input<N_U> ut_1; // u{t-1}
    Input<N_Y> y;
    Input<1> t;

    StateEstimator() : ut_1(*this), y(*this), t(*this) { x_hat.Fill(0); }

    bool init() const
    {
        FLH = F - L * H;
        t_1 = t();

        return true;
    }

    // autonomous estimator Fig 7.49(b):  x_dot = (F - L * H) * x_bar + G * u + L * y
    const Matrix<N_X> &compute() const
    {
        float dt =  t()(0) - t_1(0);
        x_hat += (F - L * H * x_hat + G * ut_1() + L * y()) * dt;
        t_1(0) += dt;

        return x_hat;
    }
};

template<int N_Y, int N_U, int N_X> struct SystemModel : public Block<N_Y>
{
    Matrix<N_X, N_X> F;
    Matrix<N_X, N_U> G;
    Matrix<N_Y,N_X> H;

    Matrix<N_X> x;
    Matrix<1> t_1; // t-1

    Input<N_U> u;
    Input<1> t;

    SystemModel() : ut_1(*this), t(*this) { x.Fill(0); }

    bool init() const
    {
        t_1 = t();
    }

    const Matrix<N_X> &compute() const
    {
        float dt =  t()(0) - t_1(0);
        x += (F * x + G * u()) * dt;
        t_1(0) += dt;

        return x_hat;
    }
};


template<int N> struct Value : public Block<N>
{
    Matrix<N_X> val;
    const Matrix<N> &compute() const { return val; }
}


// Takes a reference Block and outputs a state
template<int N_U, int N_Y, int N_X> struct ReferenceTracker : public Block<N_X>
{
    Matrix<N_U,N_Y> N_bar;
    Input<N_U> u;
    Input<N_Y> r;

    Matrix<N_X, N_X> F;
    Matrix<N_X, N_U> G;
    Matrix<N_Y,N_X> H;
    Matrix<N_U,N_X> K;

    ReferenceTracker() : u(*this), r(*this) { }

    bool init() const
    {
        // concatenate the system matrices up into one big block
        auto sys = (F || G) && (H || Zeros<N_Y,N_U>());

        // Find an inverse of that block
        Matrix<N_X+N_Y, N_X+N_Y> sysInv = ~sys * (sys * ~sys).Inverse();

        // Split it up and multiply it with K to find N_bar
        N_bar = K * sysInv.SubMatrix(Slice<0,N_X>(),Slice<N_X,N_X+N_Y>()) + sysInv.SubMatrix(Slice<N_X,N_X+N_U>(),Slice<N_X,N_X+N_Y>());
    }

    const Matrix<N_X> &compute() const
    {
        return N_bar * r() - u();
    }
};

template<int N_R, int N_X> class ErrorTracker : public Block<N_R>
{
    Matrix<N_R,N_X> H_O;
    Matrix<N_R,N_R> I;
    Matrix<N_R> windup;

    Input<N_R> r;
    Input<N_X> x;
    Input<1> t;
    Matrix<1> t_1; // t-1

public:
    ErrorTracker() : r(*this), x(*this), t(*this) { }
    {
        windup.Fill(0);
        t_1 = t();
    }

    const Matrix<N_X> &compute() const
    {
        float dt = t()(0) - t_1;
        windup += (r() - H_O * x) * dt;
        t_1 += dt;

        return r() + windup;
    }
};

#endif // STATE_SPACE_BLOCKS_H