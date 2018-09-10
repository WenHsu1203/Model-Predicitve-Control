#  **Model Predictive Control (MPC)**

**Model Predictive Control (MPC)**

The goal: Implement a MPC model in C++ to maneuver the vechicle around the track with latency issue.

### Reflection
1. Describing the model in detail, including the state, actuators and update equations.
First I convert the input state data into vehicle space from global frame that the x points to the direction of the car and u is 90 degree counterclockwise to the x-ais. Then use `polyfit()` and `polyeval()` function to calculate the cte and epsi. After that, using cte and epsi to calculate cost which put use of actuators and value gap between sequential actuations into consideration in order to make car drive smoother. Then using the dynamic model from the lecture to update:
_Xt+1 = Xt + Vt × cos(𝛹t) × dt_
_Yt+1 = Yt + Vt × sin(𝛹t) × dt_
_𝛹t+1 = 𝛹t + Vt / Lf × 𝛅t × dt_
_Vt+1 = Vt + at × dt_
_CTEt+1 = 𝒇(Xt) - Yt + Vt × sin(e𝛹t) × dt_
_e𝛹t+1 = 𝛹t - 𝛹des + Vt / Lf × 𝛅t × dt_

2.  Discussing the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values.
     Additionally the student details the previous values tried.
The smaller the dt, the better the result will be. It's like drawing a cruve line by conneting points on that cruve. The more points I have, the more accurate the curve I can draw. On the other hand, it is not always better to have larger N since larger N leads to large computation cost, and since this model has to be real-time, the large computation might casue fatal error in the real world. Another thing needs to be concerned is the multiplication of N and dt should not exceed seconds since it would be not resonable to predict too far from current time, and that's the essenece of MPC.
First I tried (N, dt) = (25, 0.05) and the result is similar to the (10, 0.1) so I chose the later one. 

3. Describing waypoint preprocessing e.g. coordinate transform.
I just change the coordinate to the vehicle space from the world space as I mentioned in the discussion 1.

4. Details on how I deal with 100 ms latency.
The most common way is that using kinematic equations to predict the states for after 100ms before sending them to MPC, and the eqations are the same in the discussion 1, just update 100ms after.
