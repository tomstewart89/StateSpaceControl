# StateSpaceControl

This library defines ```StateSpaceController```; a template class which implements a multi-input, multi-output state space feedback controller with reference tracking, state estimation and integral control.

The architecture of the controller implemented by this library is as follows:

![integral_control](https://user-images.githubusercontent.com/2457362/36132608-a5abe1bc-10bb-11e8-95fc-5e877b1851db.png)

If everything in this block diagram makes perfect sense to you then, great! Feel free to carry on to find out how to fill in the system matrices and gains in your controller object. If not, you might like to read up on [what is state space control](https://github.com/tomstewart89/StateSpaceControl/blob/master/WhatIsStateSpaceControl.md) to familiarise yourself with some of the core concepts.

## How It Works

### State Space Models

Before creating a state space controller, you'll need to define a model to describe the system that the controller is meant to work with. Specifically, you'll need to define how many states, inputs and outputs that system has. To do this, define a model object like so:

```Model<X,U,Y> model;```

Where ```X``` would be replaced with the number of states, ```U``` with the number of control inputs and ```Y``` with the number of outputs. 

You'll also need to specify how the inputs affect the state and how in turn the state affects the outputs. This can be done by filling in the four matrices defined by model; ```A```, ```B```, ```C``` & ```D```. These matrices are straight out of [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) so you can do all the fancy things with them here as you might have in that library too, including eigen-style assignment like so:

```
model.A << 1, 2, 3,
           4, 5, 6,
           7, 8, 9;
```

Once you've filled in all four of those matrices then you're ready to define a controller. As a sidenote, there's also some predefined models for common dynamic systems in [Model.h](https://github.com/tomstewart89/StateSpaceControl/blob/master/Model.h). At the moment it's just Cart Pole and a DC motor model which are used by the example sketches but I plan to expand on this list in the future. In the meantime, these model also clearly show how to fill in the system matrices so keep them in mind!

### A Simple State Space Controller

A simple state controller controller can be defined like so:

```StateSpaceController<X,U> controller(model);```

Where again ```X``` would be replaced with the number of states, ```U``` with the number of control inputs and ```model``` is some predefined, compatible ```Model<X,U>```.

This controller assumes that the entire state can be directly measured (so called full-state feedback) so it doesn't bother with state estimation. It also doesn't use integral control.

To parameterise this controller, all that needs to be done is to fill out the control gain matrix, ```controller.K``` like so:
```
controller.K << 1, 2, 3, 4;
```
Where 1, 2, 3, 4 would be replaced with appropriately selected feedback gains (more on that [below](https://github.com/tomstewart89/StateSpaceControl/blob/master/README.md#tuning-those-gains)). 

From there, simply initialise the controller like so:
```
controller.initialise();
```
Define a setpoint:
```
controller.r << 2.2;
```
And begin the control loop. Firstly update the controller with an observation ```y```:
```
controller.update(y, dt);
```
Where ```y``` is a matrix populated with ```Y``` sensor readings taken from the actual system. 

Then the updated control input can be accessed like so:
```
controller.u;
```
Which can be sent to the physical system's actuators to drive it to the setpoint.

For a complete example of how to set up a StateSpaceControl with this kind of system refer to the [Motor Position](https://github.com/tomstewart89/StateSpaceControl/blob/master/examples/MotorPosition/MotorPosition.ino) example.

### A State Space Controller With the Works

To define a more advanced controller, you can introduce state estmation and integral control by defining the ```StateSpaceController``` like so:
```
StateSpaceController<X,U,Y,true,true> controller(model);
```
Where yet again ```X``` would be replaced with the number of states, ```U``` with the number of control inputs and ```Y``` with the number of outputs. Additionally, two flags must be set to true, the first represents whether state estimation should be used and the second whether integral control should be used.

To parameterise this controller, this time three matrices, ```K```, ```L```, ```I``` need to be filled in like so:

```
controller.K << 1, 2, 3, 4;

controller.L << 1,
                2,
                3,
                4;
                
controller.I << 1;
```
Where 1, 2, 3, 4 are replaced with carefully selected feedback gains (see [below](https://github.com/tomstewart89/StateSpaceControl/blob/master/README.md#tuning-those-gains) for more)

From there, initialise the controller like so:
```
controller.initialise();
```
Define a setpoint:
```
controller.r << 0.1, 5.2;
```
And begin the control loop:
```
while(true) {
   // Get sensor readings from the plant and store them in y
   controller.update(y, dt);
   // Use controller.u to update the plant's actuators
}
```
For a complete example of how to set up a StateSpaceControl with this kind of system refer to the [Cart Pole](https://github.com/tomstewart89/StateSpaceControl/blob/master/examples/CartPole/CartPole.ino) example.

## Tuning Those Gains

You may have noticed that there are as many as three gain matrices to fill out each with potentially numerous elements. If you've ever tuned a PID controller you'll know that picking good gains is a crucial, time consuming exercise that, at times, requires a lot of guesswork.

Fortunately, state space control includes a formal process for selecting good feedback gains by defining a cost function to balance control effort versus tracking error. I've included an ipython notebook in this library to walk you through the steps involved in those calculations so be sure to check out [Tune Those Gains!](https://github.com/tomstewart89/StateSpaceControl/blob/master/utils/TuneThoseGains.ipynb) once you've finished your system modelling.
