import os
import select
import sys
import termios
import tty

from .settings import STEP_TIME, SPEED_LINEAR_MAX, SPEED_ANGULAR_MAX

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Give Manual Actions to Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


class ManualAction():
    def __init__(self):
        self.action = [0.0, 0.0]
        self.step_time = STEP_TIME

        self.settings = termios.tcgetattr(sys.stdin)

        self.status = 0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

        # print(msg)
        

    def get_key(self, settings):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def print_vels(self, target_linear_velocity, target_angular_velocity):
        # print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        #     target_linear_velocity,
        #     target_angular_velocity))
        pass


    def make_simple_profile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input

        return output


    def constrain(self, input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

    def check_linear_limit_velocity(self, velocity):
        return self.constrain(velocity, -SPEED_LINEAR_MAX, SPEED_LINEAR_MAX)
    
    def check_angular_limit_velocity(self, velocity):
        return self.constrain(velocity, -SPEED_ANGULAR_MAX, SPEED_ANGULAR_MAX)




        
    def get_action(self):
        try:
            key = self.get_key(self.settings)
            if key == 'w':
                self.target_linear_velocity =\
                    self.check_linear_limit_velocity(self.target_linear_velocity + LIN_VEL_STEP_SIZE)
                self.status = self.status + 1
                self.print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == 'x':
                self.target_linear_velocity =\
                    self.check_linear_limit_velocity(self.target_linear_velocity - LIN_VEL_STEP_SIZE)
                self.status = self.status + 1
                self.print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == 'a':
                self.target_angular_velocity =\
                    self.check_angular_limit_velocity(self.target_angular_velocity + ANG_VEL_STEP_SIZE)
                self.status = self.status + 1
                self.print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == 'd':
                self.target_angular_velocity =\
                    self.check_angular_limit_velocity(self.target_angular_velocity - ANG_VEL_STEP_SIZE)
                self.status = self.status + 1
                self.print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == 'l':
                self.target_angular_velocity = 0.0
                self.control_angular_velocity = 0.0
                self.print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif key == ' ' or key == 's':
                self.target_linear_velocity = 0.0
                self.control_linear_velocity = 0.0
                self.target_angular_velocity = 0.0
                self.control_angular_velocity = 0.0
                self.print_vels(self.target_linear_velocity, self.target_angular_velocity)
            else:
                if (key == '\x03'):
                    pass
            
            if self.status == 20:
                # print(msg)
                self.status = 0
            
            self.control_linear_velocity = self.make_simple_profile(
                self.control_linear_velocity,
                self.target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))
            
            self.control_angular_velocity = self.make_simple_profile(
                self.control_angular_velocity,
                self.target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))
            
            self.action = [self.control_linear_velocity / SPEED_LINEAR_MAX, self.control_angular_velocity / SPEED_ANGULAR_MAX]

        except Exception as e:
            print(e)
        
        return self.action