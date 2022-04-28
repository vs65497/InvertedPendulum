Cart-Pole Startup Sequence March 17, 2022
UPDATE April 9, 2022
=========================================

You should use the webcam and camera app on Desktop to record trials.

Plugin RPi and Arduino (Mega). 
Can only connect to serial on one device at a time.

Tighten the belt.
Make sure to check the pendulum encoder isnt slipping.

Start with ssh into RPi.

sudo odrivetool

odrv0.reboot()

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
Current should spool up to about 0.8 amps for 3-5 seconds.
Will hear beep from ODrive board.
Cart may jump a couple of times. 
When settled, it will move to the left for ~3 seconds, 
   then to the right for ~3 seconds.
You're now good to go.

quit()

Switch over to the Arduino.

Verify code.
Turn on Serial Monitor. Baudrate: 115200

Start camera. Tell the camera what you're doing with written note.

Upload code.
At this point the cart should jump. Then move to the right at 0.5 velocity.
If it does a major jump and then stops working, redo the entire startup sequence.
It will hit the limit switch, then move to the center of the rail.
It will pause for 5 seconds to wait for the pendulum to settle.
  You can help it settle.
This is the reference point it will use from now on.

Rotate the pendulum counter-clockwise (TO THE RIGHT) up to the apex.

The controller will kick in once it is near the top (apex +/- ~10 deg).

To stop, hit the EMERGENCY STOP.

Make sure the pendulum comes back down on the right side (clockwise).
  There is still a bug where looping the pendulum around at some point will
  cause the force to invert (left becomes right). I'm still not sure exactly why.

----------------------

To change the gain:
===================

Open MATLAB and go to
/MATLAB/cartpole.mlx

Open cartpole.mlx
Scroll down and change the Q and R matricies to your wish.
Run the section.

Copy the K matrix and A-B*K and paste into:
/docs/odrive_setup.txt

Duplicate linalg_Kxxx.h from /activate_controller
Duplicate linalg_Kxxx.h from /swingup_controller

Scroll to bottom and edit K matrix and ABK matrix.

Go back to activate_controller.ino and open in the Arduino IDE.

Change the include file line.
Verify and upload.

You're good to go!
