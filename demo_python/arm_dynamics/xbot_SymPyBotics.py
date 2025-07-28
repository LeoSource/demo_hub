import sympy
import numpy
import sympybotics


l_2x,l_2y,m_3,l_3x,l_3y,m_4,l_4x,l_4y,l_5z,m_5,l_5x,l_5y,l_6x,l_6y,l_6z,m_6 = sympy.symbols(
    'l_2x,l_2y,m_3,l_3x,l_3y,m_4,l_4x,l_4y,l_5z,m_5,l_5x,l_5y,l_6x,l_6y,l_6z,m_6')
	
def create_dynamic_robot_model():
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

def parse_joint5(rbt):
    tau_grav5 = sympy.simplify(rbt.dyn.g[4])
    print('\n\nthe gravity torque of joint5 is: ')
    print(tau_grav5)
    # l_5x,l_5y,l_6x,l_6y,l_6z,m_6 = sympy.symbols('l_5x,l_5y,l_6x,l_6y,l_6z,m_6')
    mono5 = [l_5x,l_5y,l_6x,l_6y,l_6z,m_6]
    tau_grav5 = sympy.poly(tau_grav5,mono5)
    for mono in mono5:
        print('\nthe coeff of ',mono,' is: ')
        print(tau_grav5.coeff_monomial(mono))

def parse_joint4(rbt):
    tau_grav4 = sympy.simplify(rbt.dyn.g[3])
    print('\n\nthe gravity torque of joint4 is: ')
    print(tau_grav4)
    # l_4x,l_4y,l_5z,m_5 = sympy.symbols('l_4x,l_4y,l_5z,m_5')
    mono4 = [l_4x,l_4y,l_5x,l_5y,l_5z,m_5,l_6x,l_6y,l_6z,m_6]
    tau_grav4 = sympy.poly(tau_grav4,mono4)
    for mono in mono4:
        print('\nthe coeff of ',mono,' is: ')
        print(tau_grav4.coeff_monomial(mono))

def parse_joint3(rbt):
    tau_grav3 = sympy.simplify(rbt.dyn.g[2])
    print('\n\nthe gravity torque of joint3 is: ')
    print(tau_grav3)
    # l_3x,l_3y,m_4 = sympy.symbols('l_3x,l_3y,m_4')
    mono3 = [l_3x,l_3y,l_4x,l_4y,m_4,l_5x,l_5y,l_5z,m_5,l_6x,l_6y,l_6z,m_6]
    tau_grav3 = sympy.poly(tau_grav3,mono3)
    for mono in mono3:
        print('\nthe coeff of ',mono,' is: ')
        print(tau_grav3.coeff_monomial(mono))

def parse_joint2(rbt):
    tau_grav2 = sympy.simplify(rbt.dyn.g[1])
    print('\n\nthe gravity torque of joint2 is: ')
    print(tau_grav2)
    # l_2x,l_2y,m_3 = sympy.symbols('l_2x,l_2y,m_3')
    mono2 = [l_2x,l_2y,l_3x,l_3y,m_3,l_4x,l_4y,m_4,l_5x,l_5y,l_5z,m_5,l_6x,l_6y,l_6z,m_6]
    tau_grav2 = sympy.poly(tau_grav2,mono2)
    for mono in mono2:
        print('\nthe coeff of ',mono,' is: ')
        print(tau_grav2.coeff_monomial(mono))

if __name__ == '__main__':
    rbt = create_dynamic_robot_model()
    parse_joint5(rbt)
    parse_joint4(rbt)
    parse_joint3(rbt)
    parse_joint2(rbt)