# First Research Track I assignment - Luca Mosetti

This assignment is based on a simple and portable robot simulator developed by [Student Robotics](https://studentrobotics.org). However, the arena in which the robot moves has been modified specifically for the exercise.

## Installing and runnning
The simulator requires a Python 2.7 installation, the [pygame](https://www.pygame.org/news) library, [PyPyBox2D](https://pypi.org/project/pypybox2d/2.1-r331/) and [PyYAML](https://pypi.org/project/PyYAML/). Once that everything has been installed, in order to run the python script just write:
````
$ python run.py assignment1.py
````

## Exercise requirements
The assignment requires to write a python script in order to make the robot drive counter-clockwise around the circuit while avoiding the golden boxes. Moreover, whenever it is close to a silver box, the robot should grab it and move it behind itself.
<img width="951" alt="arena" src="https://user-images.githubusercontent.com/91455159/141338938-3069965a-e1ac-4962-a53e-8e0a7f54b665.png">

## Logic of the _main()_ code:
The logic that lays behind the main algorithm that has been implemented in order to achieve the exercise's goal is described by the following pseudo-code.

#### Pseudo-code:
````python
function main():
    while(1) do
        closest obstacle position= search_obstacle(forward)
        if(no obstacle too close in the front) do
            closest target position= search_target()
            if(a target has been found) do
                reach_target() to move towards the target 
                move_behind() to grab the target and move it behind
            else
                drive() to move straight forward 
            end if
        else
            distance on the right= search_obstacle(right)
            distance on the left= search_obstacle(left)
            if(right distance is less than left distance) do
                turn_left() to turn left
            else
                turn_right() to turn right
            end if
        end if
    end while
end function
````
In order to maintain the code simple and easy to read, all the necessary sub-tasks such as obstacles detection, targets interaction and motion have been implemented by some separate functions that are called back in the `main()`. The next paragraphs will focus on the explaination of these functions.

## Functions for moving the robot
To drive the robot around the circuit have been developed three functions: `drive()`, `turn()` and `turn_ang()`. The first two are basic methods that are used, respectively, to make the robot move forward and turn in place by providing the wheels' speed and the time interval to maintain it. For instance, the following example shows how to use them to make the robot drive forward for 1 second and then turn in place for the same time interval by setting the motors at half of their power:
````python
speed= 50
dt= 1
drive(speed, dt)
turn(speed, dt)
````
Instead, `turn_ang()` is a more complex function that has been implemented to make the robot turn of a precise angle with respect its current position. This is useful in all those situations, such as moving towards a target, in which it is convenient to have an accurate control on the direction faced by the robot inside the arena.

In order to obtain this behaviour a PID controller can be applied to regulate the speed of the wheels according to the error committed on the turning angle by the system. After some testing, it turned out that just the proportional (P) and derivative (D) actions were sufficient to bring the system to the desired configuration with enough accuracy and without overshooting. In the following, it has been reported the pseudo-code that shows how such PD controller was implemented.

#### _turn_ang()_ pseudo-code:
````python
function turn_ang(ang):
    set point= check_heading() + ang
    initial error= set_point - check_heading()
    step= 0
    
    while(abs(err) is not small enough) do
        save the current time instant through current_milli_time()
        if(step == 0) do
            initialize the error derivate as 0
        else
            err= set_point - check_heading()
            d_err= (err - prevErr)/dt
        end if
        
        speed= Kp*err + Kd*d_err
        turn in place using speed
        
        step++
        prevErr= err
        dt= compute the enlapsed time from the collection of the last error sample
    end while
    
    stop motors
    
end function
````

The algorithm above make use of two further custom functions: `check_heading()` and `current_milli_time()`. The former returns the absolute angular position of the robot (in degrees) inside the map, while the latter is used to get the current system time expressed in milliseconds.

## Functions for detection
To help the robot find tokens and navigate, each token has markers associated to it. The `R.see` method returns a list of all the markers the robot can see, as `Marker` objects. To identify the type of `Marker` and its relative position with respect of the robot, each one has a set of useful attributes. The ones that have been used for the project are the following:
- `info`: it has some attributes itself that describe the marker, the most important one is:
    - `marker_type`: specify the type of the object, which in our case can assume two values: `MARKER_TOKEN_GOLD`, which represents obstacles, or `MARKER_TOKEN_SILVER`, which represents instead targets;
- `dist`: the distance from the centre of the robot to the object (in metres);
- `rot_y`: the rotation about the Y axis of the object with respect the robot (in degrees);

The attributes above are used by the functions `search_obstacle()` and `search_target()` in order to return the pose of the closest `Marker` object in a given direction with respect the current robot configuration. Since the logic that lays behind both the functions is very similar (they differ only for the type of token that is searched), only the pseudo-code of the first one is reported.

#### _search_obstacle()_ pseudo-code:
```` python
function search_obstacle(direction):
        dist= 100
        
        if(direction is "forward") do
            set min_ang and max_ang in order to look forward
        else if(direction is "left")
            set min_ang and max_ang in order to look left
        else if(direction is "right")
            set min_ang and max_ang in order to look right
        end if
        
        for token in R.see() do
            if(token.marker_type is MARKER_TOKEN_GOLD) do
                if(token.dist < dist and min_ang < token.rot_y < max_ang) do
                    dist= token.dist to save the distance of the token
                    ang= token.rot_y to save the angular position of the token
                end if
            end if
        end for
        
        return dist, ang
end function
````

## Functions for interacting with targets
In this last paragraph are presented the functions `reach_target()` and `move_behind()`, that have been developed to reach and move a target behind once it has been detected.

In order to get the robot close to a target, the method `reach_target()` implements a _turn-then-drive_ approach. This means that, once that a target is detected, the robot firstly turns in place to match the correct angle and then drives forward until it is close enough to it.

#### _reach_target()_ pseudo-code:
```` python
function reach_target(target_coordinates):
    dist= get the first element of target_coordinates
    angle= get the second element of target_coordinates
    
    turn_ang(angle)
    while(dist is not small enough) do
        drive()
        update dist
    end while
end function
````

Finally, in order to move the target behind the robot, it has been defined a very simple method that uses the functions `grab()` and `release()`, which are already provided by the simulator library.
#### _move_behind()_ pseudo-code:
```` python
function move_behind():
    R.grab() to grab the target
    turn_ang(180)
    R.release() to release the target
    turn_ang(-180)
end function
````

## Results and future improvements
The code has been tested for quite a few laps (around 10), during which the robot seemed very reliable in driving around the circuit and accomplish the tasks required by the assignment. 

The only abnormal beahviour was sometimes detected when the robot had to align with a target before approaching it. In fact, for particular targets' configurations, the robot, instead of covering the smaller angle to align with them, turns in the opposite direction and covers the bigger one. This behaviour can be adressed to how the `heading` attribute of a `Robot` object is econded. In fact, after having converted it from radians to degrees, the angle varies in the range (-180°, 180°). In particular:

- 0° corresponds to the robot facing directly East;
- 90° corresponds to the robot facing directly South;
- -90° corresponds to the robot facing directly North;
- 180° or -180° corresponds to the robot facing directly West;

Therefore, when the robot has to pass from a positive heading (close to 180°) to a negative one (close to -180°), or viceversa, the turning logic implemented by the function `turn_ang()` makes it turn in such a way to decrese (or increase) its current heading gradually until the final desired orientation is reached. In other terms, according to the function, the heading cannot jump from 180° to a -180° (or viceversa) istantaneously, which results in forcing the robot to cover the bigger angle in the particular situation that we're discussing.

![anomaly](https://user-images.githubusercontent.com/91455159/140932601-d9398874-41e0-452c-b4d0-e9aa45a03589.gif)

Since this problem doesn't happen so frequently and it doesn't compromise the execution of the other tasks, it hasn't been fixed. However, in the future might be solved.




