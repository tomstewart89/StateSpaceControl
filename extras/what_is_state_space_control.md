## Hold on, what is State Space Control?

State Space Control is a feedback control algorithm; it calculates the input to apply to a dynamic system in order to maintain the output of that system at a particular setpoint. 

If you're familiar with PID control then at this point state space control must sound like basically the same thing, and to some extent that's true. Like PID, a state space controller measures the output of a system and uses that to calculate the appropriate control input to feed back into it.

Unlike PID control though (at least in most cases) state space control takes into account a model of the system that it's controlling. Defining a model for a given dynamic system can be a little tricky which makes using state space control slightly more complex than a PID controller.

Even still, what state space control lacks in simplicity, it more than makes up for with the fact that it can take into account apriori knowledge of the system's dynamics into its control algorithm. As a result, state space is generally better suited to controlling systems with complex dynamics as well as systems with multiple inputs and outputs. It has a number of other useful properties too, but we can get into those after discussing the fundamentals.

### State Space Models

State space control assumes that a dynamic system maintains some internal set of variables called a "state" (represented by the vector $ \textbf{x} $) which describes the current status of the system. The state, along with the current control input (represented by the vector $ \textbf{u} $) determine how the output of the system (represented by the vector $ \textbf{y} $) will evolve over time. These dynamics are described by the following set of differential equations:

\begin{equation}
\frac{d \boldsymbol{x}}{dt} = A \boldsymbol{x} + B \boldsymbol{u} \\
\boldsymbol{y} = C \boldsymbol{x} + D \boldsymbol{u}
\end{equation}

These equations basically say that the rate of change of the state as well as the system's output are linear combinations of the state and the control input. These linear combinations are defined by four matrices:

* $ A \in \mathbf{R}^{X \;\times\; X} $ is the state transition matrix, which describes the way in which the state would evolve over time in the absence of any control inputs
* $ B \in \mathbf{R}^{X \;\times\; U} $ is the input matrix, which describes the effect of a control input on the state
* $ C \in \mathbf{R}^{Y \;\times\; X} $ is the observation matrix, it describes how the state is mapped onto the sensor measurements
* $ D \in \mathbf{R}^{Y \;\times\; U} $ is the direct transmission matrix, if any of the inputs are immediately reflected in the output, this matrix describes how this mapping should take place (they usually don't and this is left as zeros though)

(note that notation $ \in \mathbf{R}^{N \;\times\; M} $ means that the matrix has N rows and M columns and X, U, Y are the number of states, inputs and outputs respectively)

It's possible to describe any linear time-invariant system by filling in these four matrices.

### Feedback Control

Assuming it's possible to directly measure the entire state (i.e $ \textbf{y} \equiv \textbf{x} $ ) implementing a state space controller is really simple. The state is simply multiplied by a control gain matrix $ K \in \mathbf{R}^{ U \;\times\; X} $ and the result is fed back into the plant (the system model). As a block diagram this looks like so:

![feedback](https://user-images.githubusercontent.com/2457362/36093336-acd4f89a-102d-11e8-9387-dd48d8455e6f.png)

Like PID gains, the $ K $ matrix needs to be carefully tuned for a given system. Depending on the complexity of the system K can be quite large which makes this tuning very difficult. Fortunately state space control includes a formal method for tuning gains to arrive at what is called a Linear Quadratic Regulator. To calculate an LQR for your system head over to the [Tune Those Gains!] python noteobook. 

### Reference Tracking

The control gain in the block diagram above is designed to drive the state to zero. To instead have the system follow a set point, the controller can be modified like so:

![ref_tracking](https://user-images.githubusercontent.com/2457362/36093331-a81be854-102d-11e8-97b9-5b5862cef43a.png)

The matrix $ \bar{N} \in \mathbf{R}^{U \;\times\; Y} $ maps a reference input $ r \in \mathbf{R}^{Y} $ into an offset to the control input $ \textbf{u} $. That offset is just enough to hold the system in place system output reaches the setpoint as well as to cancel out the effect of the $ K $ matrix. The $ \bar{N} $ matrix can be calculated entirely from the system matrices ($ A,B,C,D $) so there's no need for tuning.

### State Estimation

It's rarely the case that the entire state can be directly observed by the controller. If we're only able to partially able to observe the state then an estimator can be used to infer the remaining state variable so that the control gain $ K $ can still do its job. Accordingly, the estimator lives between the output of the system and the control law like so: 

![estimation](https://user-images.githubusercontent.com/2457362/36093334-aa67fab2-102d-11e8-8db7-65e12e5d52ed.png)


An estimator works by maintaining an internal guess of the current state refered to as $ \hat{\textbf{x}} $. It updates that state firstly using the system model like so:

$ \boldsymbol{\hat{x}}(t+1) = \boldsymbol{\hat{x}}(t) + (A \boldsymbol{\hat{x}} + B \boldsymbol{u}) * dt $

Then it calculates the error between the expected system output $ \hat{\textbf{y}} = C \hat{\textbf{x}} $ and the measured output $ \textbf{y} $, then projects that onto the state estimate like so:

$ \boldsymbol{\hat{x}}(t+1) \mathrel{+}\mathrel{\mkern-2mu}= L( \boldsymbol{y} - C \boldsymbol{\hat{x}}) * dt $

Where $ L $ is an estimator gain matrix which maps the state prediction error onto the state. Like the $ K $ matrix, the $ L $ matrix needs to be tuned appropriately for a given system. Fortunately this can also be done without too much guesswork. The process for doing so is described in the [Tune Those Gains!] python notebook included with this library.

### Integral Control

So, how does the I-component from PID control figure into all of this? Strictly speaking in state space control, if the model is parameterised perfectly (and the actual system is perfectly linear) then there should be no need for integral control. However this is almost never the case and as a result a small amount of error can creep into the system. That error is assumed to manifest in the system model as a disturbance $ w $ like so:

\begin{equation}
\frac{d \boldsymbol{x}}{dt} = A \boldsymbol{x} + B \boldsymbol{u} + B_1 w
\end{equation}

To cancel out that $ B_1 w $ component, integral control is used to calculate an offset to the control input in accordance with:

\begin{equation}
\textbf{u}_I = I * \int_0^\infty \textbf{y} - \textbf{r} dt
\end{equation}

Where $ I \in \mathbf{R}^{Y \;\times\; Y} $ is the integral gain matrix. The integral component is then added to the control input alongside the contributions from the reference input and the control law like so:

![integral_control](https://user-images.githubusercontent.com/2457362/36100996-6f4d30d2-104b-11e8-9aec-dd66ff9d441b.png)


# References

This has been a pretty casal introduction to a quite a complex topic so if you're still not feeling comfortable with the concepts at play in state space control, have a look at the following resources:

* [Feedback Control of Dynamic Systems](https://www.amazon.com/Feedback-Control-Dynamic-Systems-7th/dp/0133496597)  - the textbook I used to learn about State Space Control
* [Feedback Systems: An Introduction for Scientists and Engineers](http://www.cds.caltech.edu/~murray/FBSwiki) - a textbook written by the authors of the python control library used in this notebook!
* [Control Tutorials](http://ctms.engin.umich.edu/CTMS/index.php?aux=Home) - an online resource with worked examples of state space modeling and anaylsis (also used in this notebook)

