# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Video Link
https://youtu.be/KcVTLBgQxjA

## Reflection

1. Describe the effect each of the P, I, D components had in your implementation.

I started initially with a P controller with which the car was able to make it up to the bridge. However, on the straight road of the bridge, it started oscillating fervently and finally went off track.

I then enhanced the controller to be a PD controller. This stopped the oscillations on the bridge and the car successfully went around the entire track. However, the car did not stick exactly to the center of the track presumably because of some systemic bias built in to the simulator.

Adding the I component to the controller caused the car to remain mostly at the center of the track.

2. Describe how the final hyperparameters were chosen.

I used the manual tuning approach to settle on the final hyperparameters. As mentioned above, I built the controller incrementally first building the P component, then the D component, and finally the I component. For each of these components I did some manual experiments on the simulator to figure out the best value.

- I used a very small value for the P controller as a large value causes it to oscillate too frequently and go off track.
- The D parameter is much larger
- The I parameter is also quite small


