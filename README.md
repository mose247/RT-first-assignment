# First Research Track I assignment - Luca Mosetti

This assignment is based on a simple and portable robot simulator developed by [Student Robotics](https://studentrobotics.org). However, the arenas in which the robot moves has been modified specifically for exercise.

## Installing and runnning
The simulator requires a Python 2.7 installation, the [pygame](https://www.pygame.org/news) library, [PyPyBox2D](https://pypi.org/project/pypybox2d/2.1-r331/) and [PyYAML](https://pypi.org/project/PyYAML/). Once that everything has been installed, in order to run the python script just write:
````
$ python run.py assignment.py
````

## Exercise requirements
The assignment requires to write a python script in order to make the robot drive counter-clockwise around the circuit while avoiding the golden boxes. Moreover, whenever it is close to a silver box, the robot should grab it and move it behind itself.

## Structure of the _main()_
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
                drive_forward()
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
In order to maintain the code simple and easily readable, all the necessary sub-tasks such as obstacles detection, targets interaction and motion have been implemented by some separate functions that are called back in the `main()`.

## Functions for moving the robot
To drive the robot around the circuit have been developed three functions: `drive()`, `turn()` and `turn_ang()`. The first two are basic methods that are used, respectively, to make the robot move forward and turn in place by providing the wheels' speed and the time interval to maintain it. For instance, the following example shows how to use them to make the robot drive forward for 1 second and then turn in place for the same time interval by setting the motors at half of the motors' power:
````python
speed= 50
dt= 1
drive(speed, dt)
turn(speed, dt)
````
Instead, `turn_ang()` is a more complex function that has been implemented to make the robot turn of a precise angle with respect its current position. This is useful in all those situations, such as moving towards a target, in which it is convenient to have a precise control on the direction faced by the robot inside the arena.

In order to obtain this behaviour it has been necessary to implement a PD controller that regulates the speed of the wheels according to the error committed on the turning angle by the system. Since the function is a bit complex, it is worth to look at the pseudo-code that describes the logic behind it.

#### _turn_ang()_ pseudo-code:
````python
function turn_ang(ang):
    set_point= check_heading + ang
    err= set_point - check_heading
    step= 0
    
    while(abs(err) > epsilon) do
        t= current_milli_time()
        if(step == 0) do
            d_err= 0
        else
            err= set_point - check_heading
            d_err= (err - prevErr)/dt
        end if
        
        speed= Kp*err + Kd*d_err
        set_motors_power(speed)
        
        step++
        prevErr= err
        dt= current_milli_time() - t
    end while
    
    set_motors_power(0)
    
end function
````











