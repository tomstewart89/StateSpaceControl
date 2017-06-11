#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

struct InputBase;

struct BlockBase
{
    static const int MAX_INPUTS = 16;

    mutable int noOfInputs;
    InputBase *inputs[MAX_INPUTS];

    BlockBase() : noOfInputs(0) { }
    virtual bool initialise() const = 0;
};


struct InputBase
{
    const BlockBase *block;

    InputBase() : block(NULL) { }
};


template<int N> struct Block : public BlockBase
{
    enum InitState { Unnitialised, Initialising, Initialised } initState;
    Block() : initState(InitState::Uninitialised) { }
    virtual bool init() { return true; }
    virtual const Matrix<N> &compute() const = 0;

    bool initialise() const
    {
        if(initState == InitState::Initialised)
            return true;

        // if this happens it means there is a recurrence in this graph, meaning things will never be calculated, so fail
        if(initState == InitState::Initialising)
            return false;

        initState = InitState::Initialising;

        // first initialise everything that this block is connected to
        for(int i = 0; i < noOfInputs; ++i)
        {
            // if the input hasn't been connected yet, or if it's block fails to initialise then forget it
            if(!inputs[i]->block || !inputs[i]->block->initialise())
                return false;
        }

        // Lastly if the block's own initialisation fails then bail out
        if(!this->init())
            return false;

        initState = InitState::Initialised;

        return true;
    }
};


template<int N> struct Input : public InputBase
{
    Input(const BlockBase &parent) { parent.inputs[parent.noOfInputs++] = this; }
    void connect(const Block<N> &_block) { block = &_block; }
    const Matrix<N> operator()() const { return static_cast<Input<N>*>(block)->compute(); }
};

#endif // STATE_SPACE_CONTROL_H
