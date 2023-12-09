import nico_InvKin as NIK
import numpy as np

endpoint = np.array([0.1,0,0.05])

ang = NIK.nico_IK(np.array(endpoint))
print(np.degrees(ang))