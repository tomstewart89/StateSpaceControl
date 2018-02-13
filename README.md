# StateSpaceControl

This library defines ```StateSpaceController```; a template class which implements a multi-input, multi-output state space feedback controller with reference tracking, state estimation and integral control.

The architecture of the controller implemented by this library is as follows:

![integral_control](https://user-images.githubusercontent.com/2457362/36132608-a5abe1bc-10bb-11e8-95fc-5e877b1851db.png)

If everything in this block diagram makes perfect sense to you then, great! Feel free to carry on to find out how to fill in the system matrices and gains in your controller object. If not, you might like to read through the [What is State Space Control](https://github.com/tomstewart89/StateSpaceControl/blob/master/extras/what_is_state_space_control.md) tutorial to familiarise yourself with some of the core concepts.

## How It Works

### Full State Feedback, No Integral Control

Everything required to work with this library is wrapped up in the ```StateSpaceController``` class. The most simple use case for the ```StateSpaceController``` class is when the system being controlled has full state feedback and doesn't require any integral control. In that case a ```StateSpaceController``` can be instantiated just by supplying the number of states (X) and number of control inputs (U) as template parameters like so:

```
StateSpaceController<3,1> controller;
```
In this case we'll assume that the system has 3 states and 1 control input which incidentally, is that same as the system in the [Motor Position](https://github.com/tomstewart89/StateSpaceControl/tree/master/examples/MotorPosition) example. 

After you've declared a ```StateSpaceController``` we'll need to fill out its system matrices before you can start using it. Based on the template parameters, the ```StateSpaceController``` will declare a ```Model``` containing 4 instances of ```Matrix``` appropriately sized to hold the system matrices which can be accessed via the controller's ```model``` member. The ```Matrix``` class is defined in the [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) library so for more on how to manipulate them refer to the [README](https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/README.md). 

For now we'll just set the (0,0) element of the state transition matrix like so:
```
controller.model.A(0,0) = 5.24;
```
You'll also need to fill out the gain matrix K which can be accessed via the ```K``` member:
```
controller.K(0,1) = 12.92;
```
Once the gains and the system matrices are filled out, the controller object is ready to be used, but first it needs to be initialised so that it can calculate the precompensator as well as a few matrices in the estimator:
```
controller.initialise();
```
Now we're ready to start our control. To specify a setpoint we'll just need to write to the ```controller.r``` member which has the same size as the system output (which in this case is the same size as the state):
```
controller.r(0,0) = 2.2;
```
Next the controller needs to be regularly updated so that it can refresh its commanded control input. To do so, the controller expects sensor readings in the form of a ```Matrix``` of size Y and a float to indicate the time since the last update took place:

```
float dt = DELAY;
Matrix<3> y;

y(0) = 24.43  // (the value of state variable 1)
y(1) = 44.2 // (the value of state variable 2)
y(2) = 234.2 // (the value of state variable 3)

controller.update(y, dt);

```
Finally, we can access the updated control input via the ```controller.u``` member which can then be used in your sketch to command your actuators and control your system:
```
updateSegwayWheelTorques(controller.u);
```

### Including State Estimation & Integral Control

Expanding ```StateSpaceController``` to support estimation and integral control just requires filling in a few more template parameters when the controller object is instantiated. The full set of template parameters (and their defaults) are actually as follows:

```
class StateSpaceController<X, U, Y=X, EnableReferenceTracking=true, EnableEstimation=false, EnableIntegralControl=false>
```
Where Y is the number of system outputs (i.e sensor readings) and the remaining parameters are flags which enable/disable the different blocks in the diagram above.

We'll follow the [Cart Pole](https://github.com/tomstewart89/StateSpaceControl/tree/master/examples/CartPole) example whose system has 4 states, 1 input and 2 outputs which means that its controller's declaration looks like so:

```
StateSpaceController<4,1,2,true,true,true> controller;
```
From there the only extra requirements are that we fill in two additional gain matrices:

```
controller.L;
```
and
```
controller.I;
```

## Utilities

You may have noticed that we have as many as three gain matrices to fill out each with potentially numerous elements. If you've ever tuned a PID controller you'll know that picking good gains is crucial, time consuming and at times requires a lot of guesswork.

Fortunately, state space control includes a formal process for selecting good feedback gains by defining a cost function to balance control effort versus tracking error. I've included an ipython notebook in this library to walk you through the steps involved in those calculations so be sure to check out [Tune Those Gains!](https://github.com/tomstewart89/StateSpaceControl/blob/master/extras/tune_those_gains.ipynb) once you've finished your system modelling.
