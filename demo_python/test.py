import numpy as np


pos_in_robot = np.array([[0.08654,0.1149,-0.05263],[0.08677,0.11478,-0.06262],
                         [0.08699,0.11466,-0.07262],[0.08699,0.10465,-0.07262],
                         [0.08699,0.09477,-0.07262],[0.08536,0.09552,-0.07231],
                         [0.07925,0.10192,-0.06925],[0.09175,0.10194,-0.0692],
                         [0.09222,0.08879,-0.074],[0.07876,0.08878,-0.07397],
                         [0.07366,0.10805,-0.06482],[0.0973,0.10801,-0.06473],
                         [0.09921,0.08197,-0.07411],[0.07176,0.08198,-0.07403],
                         [0.06912,0.11372,-0.05928],[0.10182,0.11364,-0.05917],
                         [0.10602,0.0751,-0.07279],[0.06501,0.07511,-0.07277],
                         [0.0786,0.08205,-0.07469],[0.10381,0.10189,-0.06593],
                         [0.08552,0.09548,-0.07143]])
pos_in_robot *= 1000
diffs = np.diff(pos_in_robot,axis=0)
distance_in_robot = np.linalg.norm(diffs,axis=1)

pos_in_aimooe = np.array([[-78.5,-2.7,163.5],[-68.5,-2.4,163.2],
                          [-59.3,-2.1,162.8],[-58.5,-2.2,172.5],
                          [-57.6,-2.3,182.3],[-58.2,-3.9,181.5],
                          [-60.8,-7.3,177.6],[-61.5,0.2,177.2],
                          [-55.7,1.1,186.6],[-55.1,-7.8,186.7],
                          [-65,-10.8,173.9],[-66.1,4.1,173.4],
                          [-54.9,6.7,192],[-54.1,-12.8,191.1],
                          [-70.7,-13.4,170.3],[-72.1,6.6,169.6],
                          [-55.4,11.6,197.1],[-53.5,-16.8,196.7],
                          [-53.2,-7.8,191.6],[-63.9,8.4,177.5],
                          [-57.6,-3.4,182]])
diffs = np.diff(pos_in_aimooe,axis=0)
distance_in_aimooe = np.linalg.norm(diffs,axis=1)

diff_distances = np.abs(distance_in_robot - distance_in_aimooe)
print(f'mean distance error: {np.mean(diff_distances):.2f} mm')
print(f'max distance error: {np.max(diff_distances):.2f} mm')


