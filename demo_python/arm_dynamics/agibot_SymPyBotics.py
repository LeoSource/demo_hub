import sympy
import numpy
import sympybotics

def create_dynamic_arm_model(arm_name:str):
    rbtdef = sympybotics.RobotDef('XRobot',
        [(      0,        0,     0.048,        'q'),# (alpha, a, d, theta)
            ('-pi/2',        0,         0,   'q-pi/2'),
            (      0,     0.51,         0,   'q+pi/2'),
            (      0,     0.51,      0.11,   'q-pi/2'),
            ('-pi/2',        0,   0.08662,        'q'),
            ( 'pi/2',        0,     0.035,        'q')],
            dh_convention='mdh')
            
    rbtdef.frictionmodel = {'Coulomb', 'viscous'}
    rbtdef.gravityacc = sympy.Matrix([1.1, 2.2, 3.3])
    rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)

    rbt.calc_base_parms()
    rbt.dyn.gen_all()
    return rbt



if __name__ == '__main__':
    rbt = create_dynamic_arm_model('left_arm')