#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

enum EstimationType { FullStateFeedback, Autonomous };

template<int states, int inputs, int outputs, EstimationType type = Autonomous> class StateSpaceController { };

template<int states, int inputs, int outputs> class StateSpaceController<states, inputs, outputs, FullStateFeedback>
{
public:
    //
    Matrix<inputs> controlInput;
    Matrix<outputs> referenceInput;

    // State space matrices
    Matrix<states,states> systemMatrix;
    Matrix<states,inputs> inputMatrix;
    Matrix<outputs,states> outputMatrix;
    Matrix<outputs,inputs> directTransmissionMatrix;

    // Gains
    Matrix<inputs,states> controlGain;
    Matrix<outputs,outputs> integralGain;

    // Things to precalculate
    Matrix<inputs,outputs> precompensator;

    StateSpaceController()
    {
        // Zero out all the matrices
        systemMatrix.Fill(0); inputMatrix.Fill(0); outputMatrix.Fill(0); directTransmissionMatrix.Fill(0);
        controlInput.Fill(0); referenceInput.Fill(0);
        controlGain.Fill(0); integralGain.Fill(0);
    }

    void Initialise()
    {
        Matrix<states+outputs,states+inputs> sys;
        sys.Submatrix(Range<states>(0),Range<states>(0)) = systemMatrix;
        sys.Submatrix(Range<states>(0),Range<inputs>(states)) = inputMatrix;
        sys.Submatrix(Range<outputs>(states),Range<states>(0)) = outputMatrix;
        sys.Submatrix(Range<outputs>(states),Range<inputs>(states)) = directTransmissionMatrix;

        // Find an inverse for the system matrix
        Matrix<states + inputs, states + outputs> sysInv = sys.Transpose() * (sys * sys.Transpose()).Inverse();

        // Split it up and multiply it with K to find NBar
        precompensator = controlGain * sysInv.Submatrix(Range<states>(0),Range<outputs>(states)) + sysInv.Submatrix(Range<inputs>(states),Range<outputs>(states));
    }

    void Update(Matrix<states> &systemState, float dt)
    {
        // Recalculate the control input
        controlInput =  precompensator * referenceInput - controlGain * systemState;

        // Windup the reference input
        referenceInput += integralGain * (referenceInput - outputMatrix * systemState) * dt;
    }
};

template<int states, int inputs, int outputs> class StateSpaceController<states, inputs, outputs, Autonomous>
{
public:
    //
    Matrix<inputs> controlInput;
    Matrix<outputs> referenceInput;
    Matrix<states> stateEstimate;

    // State space matrices
    Matrix<states,states> systemMatrix;
    Matrix<states,inputs> inputMatrix;
    Matrix<outputs,states> outputMatrix;
    Matrix<outputs,inputs> directTransmissionMatrix;

    // Gains
    Matrix<inputs,states> controlGain;
    Matrix<states,outputs> estimatorGain;
    Matrix<outputs,outputs> integralGain;

    // Things to precalculate
    Matrix<inputs,outputs> precompensator;
    Matrix<states,states> systemEstimatorOutput;

    StateSpaceController()
    {
        // Zero out all the matrices
        systemMatrix.Fill(0); inputMatrix.Fill(0); outputMatrix.Fill(0); directTransmissionMatrix.Fill(0);
        stateEstimate.Fill(0); controlInput.Fill(0); referenceInput.Fill(0);
        controlGain.Fill(0); integralGain.Fill(0); estimatorGain.Fill(0);
    }

    void Initialise()
    {
        // Aggregate all the state space matrices into one big matrix
        Matrix<states + outputs, states + inputs> sys;
        sys.Submatrix(Range<states>(0),Range<states>(0)) = systemMatrix;
        sys.Submatrix(Range<states>(0),Range<inputs>(states)) = inputMatrix;
        sys.Submatrix(Range<outputs>(states),Range<states>(0)) = outputMatrix;
        sys.Submatrix(Range<outputs>(states),Range<inputs>(states)) = directTransmissionMatrix;

        // Find an inverse for the aggregated matrix
        Matrix<states + inputs, states + outputs> sysInv = sys.Transpose() * (sys * sys.Transpose()).Inverse();

        // Split it up and multiply it with K to find NBar
        precompensator = controlGain * sysInv.Submatrix(Range<states>(0),Range<outputs>(states)) + sysInv.Submatrix(Range<inputs>(states),Range<outputs>(states));

        // We can save a bit of time by precalculating the (F - L * H) term in the state update equation
        systemEstimatorOutput = systemMatrix - estimatorGain * outputMatrix;
    }

    void Update(Matrix<outputs> &systemOutput, float dt)
    {
        // autonomous estimator Fig 7.49(b):  x_dot = (F - L * H) * x_bar + G * u + L * y
        stateEstimate += (systemEstimatorOutput * stateEstimate + inputMatrix * controlInput + estimatorGain * systemOutput) * dt;

        // Recalculate the control input
        controlInput =  precompensator * referenceInput - controlGain * stateEstimate;

        // Windup the reference input
        referenceInput += integralGain * (referenceInput - outputMatrix * stateEstimate) * dt;
    }
};

#endif // STATE_SPACE_CONTROL_H
