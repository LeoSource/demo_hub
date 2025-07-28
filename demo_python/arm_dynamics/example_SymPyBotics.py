import sympy
import numpy as np
import sympybotics


rbtdef = sympybotics.RobotDef('Example Robot', # robot name
                            [('-pi/2', 0, 0, 'q+pi/2'),  # list of tuples with Denavit-Hartenberg parameters
                            ( 'pi/2', 0, 0, 'q-pi/2')], # (alpha, a, d, theta)
                            dh_convention='standard' # either 'standard' or 'modified'
                            )

rbtdef.frictionmodel = {'Coulomb', 'viscous'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81]) # optional, this is the default value

print(rbtdef.dynparms())

rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)

print(rbt.geo.T[-1])
print(rbt.kin.J[-1])

print('==========')
rbt.calc_base_parms()
print(rbt.dyn.baseparms)