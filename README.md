# InvertedPendulum

<img src="https://github.com/zanzivyr/InvertedPendulum/blob/main/photos/IMG_1826.JPG" height="320"> [![Inverted Pendulum: Best Trials](https://img.youtube.com/vi/Sc8SkmnBHfo/0.jpg)](https://youtu.be/Sc8SkmnBHfo)

In Spring 2022 I took an engineering project class. The professor allowed me to choose a topic I had been looking into for awhile -- Control Theory. Control seemed to be a landmark subject that would allow me to build more complex robots. To deepen my knowledge I decided to build an Inverted Pendulum, which is one of the fundamental ways to understand Control.

For this project I focused on building a Linear-Quadratic Regulator (LQR) compensator and a Swing-Up controller (via Lyapunov energy function). By the end, I attempted to implement a Kalman Filter, thus turning my controller into a Linear-Quadradic Gaussian (LQG) compensator.

The design of the pendulum is my own, based on others I saw online. I used SolidWorks to model my idea, purchased all components, 3D printed custom pieces, and used an ODrive motor and controller to actuate the robot.

## Presentation

Full write up available here: https://drive.google.com/file/d/1BjKBpGeMnVzuu_CwovKgb8RUEkeBamfX/view?usp=sharing

PowerPoint Presentation available here: https://docs.google.com/presentation/d/1e7umLAcLHKKlPaMdunqyrMWjsbgzsQLA/edit?usp=sharing&ouid=113020445866421200941&rtpof=true&sd=true

### Videos
- Best Trials: https://youtu.be/Sc8SkmnBHfo
- Cart Design 1: https://youtu.be/NO3bNjtjrpo
- Cart Design 2: https://youtu.be/H1BWjyYJvzo
- Motor Testing: https://youtu.be/MVzB0eA6DNc

## Pendulum Design

<img src="https://github.com/zanzivyr/InvertedPendulum/blob/main/photos/pendulum_cad1.png" height="320">  <img src="https://github.com/zanzivyr/InvertedPendulum/blob/main/photos/pendulum_cad2.png" height="320">

Using SolidWorks, I designed a rail, cart, and pendulum. The cart interfaces with the rail in a similar way to a 3D printer. This was passable in the beginning but I think it caused a lot of oscillations when driving the pendulum. After concluding the project I had a chance to look at a lot of industrial CNC machines. These kinds of cart-rail systems are much better, so I will use them next time.

## Controller Selection (PID vs LQR)

The system only has 2 primary sensors: pendulum angle and motor encoder. The secondary sensors are limit switches at the end of the rail for initialization and to prevent damage (though this ended up not working as well as it should have).

Given the two sensors, after initialization, we can track 4 inputs: pendulum angle, pendulum angular velocity, cart position, and cart velocity. Error is found by the difference between the current state of the system and the desired state (pendulum in the upright position).

Due to having multiple inputs and multiple outputs, PID control is not able to be used (it can only handle single in, single out). Instead I chose to use LQR.

## Swing-Up Control

The inverted pendulum is actually a non-linear system. You can see this by thinking about moving around a circle. Given y = cos(x), we know that changing x from 0 to 2PI, y will go from 0 to 1, back to 0, down to -1, then back to 0 --> (0, 1, 0, -1, 0). As we can see this is cyclic, not linear. This is why the inverted pendulum is an interesting control problem.

Stabilizing the pendulum in the up position works for the LQR controller because we are only looking at a very small region where the system effectively behaves as a linear system. If we say that PI is the up position, then we are looking at a region which is maybe PI +/- 0.001. So, for our pendulum to balance, we must start it in a place very close to PI.

### What happens outside of this linearized region?

In all the area outside PI, the pendulum's movement is non-linear. To control this we can move from focusing on the angle of the pendulum to focusing on the energy of the pendulum.

If we instead define the pendulums position (via angle) on its mechanical energy, then we can find a function which can "naturally" drive the pendulum to the up position. This is our Lyapunov energy function. However, there is one caveat: at the absolute down position, the energy function will return 0 and prevent the pendulum from moving. As a result, the pendulum needs a kick to get started.

This is why in the videos you see me tapping the pendulum in some clips. Later on I wrote a function to inject some energy into the system automatically.

## Switching Controllers

During the course of the swing up, at some point the swing up controller must deactivate so that the LQR compensator can activate. The problem is that if there is too much energy in the system when the pendulum is passing through our linearized region, then the LQR compensator will not be able to "catch" the pendulum. The pendulum will blast through the linearized zone and then be back into the swing-up zone which can lead to a "death spin." 

This is how I destroyed my computer. Though I learned a lot on this project, the most important thing I learned is that robots **MUST** be ran in safe areas with nothing valuable in them. Robots have no sensitivity to the world and will attempt to do exactly what you program them to do. Sometimes it leads to disastrous outcomes.

## Oscillations and Kalman Filter

After running hundreds of tests to calibrate the motor and to choose the best LQR gains, I found that there was a lot of noise from my mechanical design and sensors. I believe this unmodeled noise was also highly non-linear in its nature. Furthermore, I believe that my method of "calculating" the pendulum velocity and cart position was incorrect. It turns out that these calculations may only be an approximation of old sensor data (last signal recieved). And even moreso, this out of date approximation is based on the system's mathematical model, not reality.

I think the way to resolve both of these problems is to use a Kalman Filter. The Kalman Filter would find the difference between a proper state estimation (through system dynamics) and the real state (based on sensor data) and drive this error to 0, resulting in a more accurate representation of the system's state. Once this is fed into the LQR compensator, it should remove some of the oscillations that occur when the pendulum is trying to swing up and when it is within the linearized zone but far from PI.

Additionally, changing the mechanical design of the cart to something better (as found in CNC's) would go a long way to remove those oscillations.
