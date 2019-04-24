from pydrake.all import (RigidBodyTree, AddFlatTerrainToWorld, RigidBodyPlant,
                         ConstantVectorSource, Simulator, DiagramBuilder,
                         DrakeVisualizer)
from pydrake.all import (RigidBodyFrame, AddModelInstanceFromUrdfFile,
                         getDrakePath, RigidBodyFrame, FloatingBaseType, 
                         PiecewisePolynomial, TrajectorySource, InverseDynamics, Gain, Adder)
from pydrake.lcm import DrakeLcm
import numpy as np
import os

def getKukaUrdfPath(collision_type="none"):
    col_types = {
        "spheres": "spheres_collision",
        "none": "no_collision",
        "polytope": "polytope_collision",
        "primitive": "primitive_collision",
        "visual": "visual"
    }
    drake_models = ["spheres", "none", "polytope", "primitive"]
    try:
        if collision_type in drake_models:
            directory = os.path.join(getDrakePath(), "manipulation", "models",
                                     "iiwa_description", "urdf")
        else:
            directory = "models"
    except RuntimeError as e:
        print "Cannot find path to drake, make sure to export the location:\n"\
              + "export DRAKE_RESOURCE_ROOT=/path/to/kuka-dev-cpp/bazel-kuka-dev-cpp/external"  # noqa
        raise e

    if collision_type in col_types:
        base = "iiwa14_{}.urdf".format(col_types[collision_type])
        return os.path.join(directory, base)
    else:
        raise NameError("Collision type {} is not defined".collision_type)

def addKukaSetup(tree, collision_type="none"):
	world_frame = RigidBodyFrame("world_frame", tree.world(), [0, 0, 0],[0, 0, 0])
	# AddModelInstanceFromUrdfFile("models/kuka_table.urdf",FloatingBaseType.kFixed, world_frame, tree)
	kuka_urdf_path = getKukaUrdfPath(collision_type=collision_type)
	# table_top_frame = tree.findFrame("kuka_table_top")
	# AddModelInstanceFromUrdfFile(kuka_urdf_path, FloatingBaseType.kFixed,table_top_frame, tree)
	AddModelInstanceFromUrdfFile(kuka_urdf_path, FloatingBaseType.kFixed,world_frame, tree)


builder = DiagramBuilder()

# build the experiment system
tree = RigidBodyTree()
AddFlatTerrainToWorld(tree, 100, 10)
addKukaSetup(tree,collision_type='spheres')

# create the diagram
plant = RigidBodyPlant(tree, 0.0001)  # 2000 Hz
deltas = [-0.66667,-0.33333,0.0,0.5,1.0]

for delta in deltas:
	# print "delta: " + str(delta)
	for i in range(tree.get_num_positions()+tree.get_num_velocities()):
		# print "index: " + str(i)
		xk = np.zeros(tree.get_num_positions()+tree.get_num_velocities())
		# xk = np.array([0.3689,0.4607,0.9816,0.1564,0.8555,0.6448,0.3763,0.1909,0.4283,0.4820,0.1206,0.5895,0.2262,0.3846])
		xk[i] += delta
		str_out = ''
		for j in range(tree.get_num_positions()+tree.get_num_velocities()):
			str_out += "%.6f " % xk[j]
		print str_out + "\n"
		kinsol = tree.doKinematics(xk[:tree.get_num_positions()],xk[tree.get_num_positions():])
		massMatrix = tree.massMatrix(kinsol)
		biasTerm = tree.dynamicsBiasTerm(kinsol,{})
		str_out = ''
		for j in range(tree.get_num_positions()):
			for k in range(tree.get_num_positions()):
				str_out += "%.4f " % massMatrix[j][k]
			str_out += "\n"
		print str_out
		str_out = ''
		for j in range(tree.get_num_positions()):
			str_out += "%.4f " % -biasTerm[j]
		print str_out + "\n"