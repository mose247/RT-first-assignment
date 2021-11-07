# First Research Track I assignment - Luca Mosetti

This assignment is based on a simple and portable robot simulator developed by [Student Robotics](https://studentrobotics.org). However, the arena in which the robot moves has been modified specifically for the exercise.

## Installing and runnning
The simulator requires a Python 2.7 installation, the [pygame](https://www.pygame.org/news) library, [PyPyBox2D](https://pypi.org/project/pypybox2d/2.1-r331/) and [PyYAML](https://pypi.org/project/PyYAML/). Once that everything has been installed, in order to run the python script just write:
````
$ python run.py assignment1.py
````

## Exercise requirements
The assignment requires to write a python script in order to make the robot drive counter-clockwise around the circuit while avoiding the golden boxes. Moreover, whenever it is close to a silver box, the robot should grab it and move it behind itself.

## Logic of the _main()_ code:
The logic that lays behind the main algorithm that has been implemented in order to achieve the exercise's goal is described by the following pseudo-code.

#### Pseudo-code:
````python
function main():
    while(1) do
        ob_coord= search_obstacle(forward)
        if(no obstacle too close in the front) do
            tr_coord= search_target()
            if(a target has been found) do
                reach_target()
                move_behind()
            else
                drive()
            end if
        else
            dist_r= search_obstacle(right)
            dist_l= search_obstacle(left)
            if(dist_r < disr_l) do
                turn_left()
            else
                turn_right()
            end if
        end if
    end while
end function
````
In order to maintain the code simple and easily readable, all the necessary sub-tasks such as obstacles detection, targets interaction and motion have been implemented by some separate functions that are called back in the `main()`. The next paragraphs will focus on the explaination of these functions.

## Functions for moving the robot
To drive the robot around the circuit have been developed three functions: `drive()`, `turn()` and `turn_ang()`. The first two are basic methods that are used, respectively, to make the robot move forward and turn in place by providing the wheels' speed and the time interval to maintain it. For instance, the following example shows how to use them to make the robot drive forward for 1 second and then turn in place for the same time interval by setting the motors at half of their power:
````python
speed= 50
dt= 1
drive(speed, dt)
turn(speed, dt)
````
Instead, `turn_ang()` is a more complex function that has been implemented to make the robot turn of a precise angle with respect its current position. This is useful in all those situations, such as moving towards a target, in which it is convenient to have an accurate control on the direction faced by the robot inside the arena.

In order to obtain this behaviour it has been necessary to implement a PD controller that regulates the speed of the wheels according to the error committed on the turning angle by the system. In the following, it has been reported the pseudo-code that shows how the controller was implemented.

#### _turn_ang()_ pseudo-code:
````python
function turn_ang(ang):
    set_point= check_heading() + ang
    err= set_point - check_heading()
    step= 0
    
    while(abs(err) is not small enough) do
        t= current_milli_time()
        if(step == 0) do
            d_err= 0
        else
            err= set_point - check_heading()
            d_err= (err - prevErr)/dt
        end if
        
        speed= Kp*err + Kd*d_err
        turn in place using speed
        
        step++
        prevErr= err
        dt= current_milli_time() - t
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
            define forward angular range
        else if(direction is "left")
            define left angular range
        else if(direction is "right")
            define right angular range
        end if
        
        for token in R.see() do
            if(token.marker_type is MARKER_TOKEN_GOLD) do
                if(token.dist < dist and min_ang < token.rot_y < max_ang) do
                    dist= token.dist
                    ang= token.rot_y
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
    dist= target_coordinates[0]
    angle= target_coordinates[1]
    
    turn_ang(angle)
    while(dist is not small enough) do
        drive()
        update dist
    end while
end function
````

Finally, in order to move the target behind the robot, it has been defined a simple method that uses the functions `grab()` and `release()` already provided by the simulator library.
#### _move_behind()_ pseudo-code:
```` python
function move_behind():
    grab()
    turn_ang(180)
    release()
    turn_ang(-180)
end function
````






