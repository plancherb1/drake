from pydrake.all import (RigidBodyTree, AddFlatTerrainToWorld, RigidBodyPlant,
                         Simulator, DiagramBuilder, DrakeVisualizer,
                         ConstantVectorSource, TrajectorySource,
                         PiecewisePolynomial, InverseDynamicsController, getDrakePath)
from pydrake.lcm import DrakeLcm
import numpy as np

if __name__ == "__main__":
    # create the systems
    directory = os.path.join(getDrakePath(), "manipulation", "models", "iiwa_description", "urdf")
    base = "iiwa14_no_collision.urdf"
    kuka_urdf_path = os.path.join(directory, base)

    tree_1 = RigidBodyTree()
	AddFlatTerrainToWorld(tree_1, 100, 10)
    world_frame_1 = RigidBodyFrame("world_frame", tree_1.world(), [0, 0, 0], [0, 0, 0])
    AddModelInstanceFromUrdfFile(kuka_urdf_path, FloatingBaseType.kFixed, world_frame_1, tree_1)
    
    tree_2 = RigidBodyTree()
	AddFlatTerrainToWorld(tree_2, 100, 10)
    world_frame_2 = RigidBodyFrame("world_frame", tree_1.world(), [0, 0, 0], [0, 0, 0])
    AddModelInstanceFromUrdfFile(kuka_urdf_path, FloatingBaseType.kFixed, world_frame_2, tree_2)
    
    builder = DiagramBuilder()

    # input trajectory
    tspan = [0., TBD]
    ts = np.linspace(tspan[0], tspan[-1], TBD)
    q_des = TBD 
    q_traj = PiecewisePolynomial.Cubic(ts, q_des, np.zeros(7), np.zeros(7))
    x_init = np.vstack((q_traj.value(tspan[0]),q_traj.derivative(1).value(tspan[0])))
    source = builder.AddSystem(TrajectorySource(q_traj, output_derivative_order=1))

    # controller
    kp = 100 * np.ones(7)
    kd = 10 * np.ones(7)
    ki = 0 * np.ones(7)
    controller = builder.AddSystem(InverseDynamicsController(robot=tree_1, kp=kp, ki=ki, kd=kd, has_reference_acceleration=False))
    
    # plant
    plant = RigidBodyPlant(tree_2, 0.0005)
    kuka = builder.AddSystem(plant)

    # visualizer
    lcm = DrakeLcm()
    visualizer = builder.AddSystem(DrakeVisualizer(tree=tree_1, lcm=lcm, enable_playback=True))
    visualizer.set_publish_period(.001)

    # build the diagram
    builder.Connect(source.get_output_port(0), controller.get_input_port(1))
    builder.Connect(controller.get_output_port(0), kuka.get_input_port(0))
    builder.Connect(kuka.get_output_port(0), visualizer.get_input_port(0))
    builder.Connect(kuka.get_output_port(0), controller.get_input_port(0))
    diagram = builder.Build()

    # run the simulator
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(True)
    kuka.set_state_vector(simulator.get_mutable_context(), x_init)
    simulator.StepTo(tspan[-1])