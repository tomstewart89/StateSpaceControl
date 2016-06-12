#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

template<int states, int inputs = 1, int outputs = 1> class StateSpaceController
{
public:
    // state, inputs, outputs and reference
    Matrix<states> xBar;
    Matrix<inputs> u;
    Matrix<outputs> r;

    // The aggregated system matrices aggregated in the form {{A, B},{C, D}}
    Matrix<states+outputs,states+inputs> sys;

    // References to the system matrix
    RefMatrix<states,states> A;
    RefMatrix<states,inputs> B;
    RefMatrix<outputs,states> C;
    RefMatrix<outputs,inputs> D;

    // Gains
    Matrix<inputs,states> K; // control
    Matrix<outputs,outputs> I; // integral control
    Matrix<states,outputs> L; // estimator
    Matrix<inputs,outputs> NBar; // reference

    StateSpaceController() : A(sys,0,0), B(sys,0,states), C(sys,states,0), D(sys,states,states)
    {
        // Zero system matrix
        sys.Fill(0);

        // Zero the state estimate, the control input and the reference input
        xBar.Fill(0);
        u.Fill(0);
        r.Fill(0);

        // Zero the gains
        K.Fill(0);
        L.Fill(0);
        NBar.Fill(0);
        I.Fill(0);
    }

    void Update(Matrix<outputs> &y, float dt)
    {
        // Update the state
        xBar += ((A - L * C) * xBar + B * u + L * y) * dt; // autonomous estimator Fig 7.49(b)

        // Windup the reference input
        r += I * (y - C * xBar);

        // Recalculate the control input
        u =  NBar * r - K * xBar;
    }

    void Initialise()
    {
        // Find an inverse for the system matrix
        Matrix<states + inputs, states + outputs> sysInv = sys.Transpose() * (sys * sys.Transpose()).Inverse();

        // Split it up and multiply it with K to find NBar
        NBar = K * sysInv.Submatrix(Range<states>(0),Range<outputs>(states)) + sysInv.Submatrix(Range<inputs>(states),Range<outputs>(states));
    }
};

#endif // STATE_SPACE_CONTROL_H
