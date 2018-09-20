The underwater ROV movement is controlled by a modified thrustmaster joystick. 
This code here serves to interface the analog values read from the joystick to digital values in Arduino, and the the values will be processed and convert to movement values (0-255).

The control output system includes 4 pwm controlling 4 separate motors.  

The fundamental concept of how the joystick controls the ROV is that the system’s code convert the analog values retrieved from the joystick to x and y values based on the joystick’s center location. 

Since the analog values received from the joystick are too unstable due to the joystick's sensitivity, the system accounts for the constant varying data first by limiting the maximum and minimum values and then by dividing the each joystick axis into 21 sections, 10 above and to the left of the midpoint section, 10 below and to the right, and one for the equilibrium point. 

Our team achieve this by mapping the analog value in the constrained range (around 120 to 940) from -10 to 10, and setting 0 as the equilibrium point. 

Unlike last year, the xy value is not combining to create a cartesian plane, instead, it separates x as the rotation factor, and y as the linear propulsion factor. 

For the rotation factor, the system will rotate the ROV in the relative direction of the center of joystick with a speed proportional to the magnitude of the joystick movement. 

For linear propulsion factor, the rov will propulse in the y direction of the joystick; simply put, if the joystick location is above half, then it moves forward with magnitude respective to the difference from center, and vice versa for backward. 

To make the transition from linear propulsion state to rotation state, our team have installed a button that when it is pushed, the system will change to the rotation state and when released, it will return back to linear propulsion state as default. 

For the z motion (up and down motion), there will be 2 seperate button exclusively for that control as the joystick only provides 2 variables.  

Our system does not take speed into account for the up and down motion, because usually up and down motion is not a continuous task and does not require as much precisions compared to x and y motion. 
