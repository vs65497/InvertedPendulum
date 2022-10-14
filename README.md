# InvertedPendulum

In Spring 2022 I took an engineering research class. The professor allowed me to choose a topic I had been looking into for awhile -- Control Theory. Control seemed to be a landmark subject that would allow me to build more complex robots. To deepen my knowledge I decided to build an Inverted Pendulum, which is one of the fundamental ways to understand Control.

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

Using SolidWorks, I designed a rail, cart, and pendulum. The cart interfaces with the rail in a similar way to a 3D printer. This was passible in the beginning but I think it caused a lot of oscillations when driving the pendulum. After concluding the project I had a chance to look at a lot of industrial CNC machines. These kinds of cart-rail systems are much better, so I will use them next time.

## Controller Selection (PID vs LQR)

The system only has 2 primary sensors: pendulum angle and motor encoder. The secondary sensors are limit switches at the end of the rail for initialization and to prevent damage (though this ended up not working as well as it should have).

Given the two sensors, after initialization, we can track 4 inputs: pendulum angle, pendulum angular velocity, cart position, and cart velocity. Error is found by the difference between the current state of the system and the desired state (pendulum in the upright position).

Due to having multiple inputs and multiple outputs, PID control is not able to be used. Instead I chose to use LQR.

## Swing-Up Control

