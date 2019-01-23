/*
  run_robobee_traj_optimization.cc

  Objective : Solve trajectory optimization for Robobee dynamics (w/ quaternion)
  Algorithm : Direct collocation based on "Direct Trajectory Optimization Using Nonlinear Programming and Collocation" 1986 AIAA
  Remark : First order piecewise polynomial (C([0,T]))
           Third order piecewise polynomila (C^3([0,T]))
           Collocation at the mid point.

  Cost function : J = \int_{0}^T uT*R*u
  Constraint : 1. Dyamics : \dot{x}=f(x)+g(x)+u for x\in\mathbb{R}^13 and u\in\mathbb{R}^4
               2. Boundary condition : x(0)=x_0 and x(T)=x_T
               3. Bounded terminal time : l_T<=T <=U_T
               4. Thrust lower bound : l_{Thrust} <= u(0) <= u_{Thrust} 

  Direct Collocation Constraint : 1. Dynamics at every knot point
                                  2. Equal interval time for each knot points
                                  3. Boundary constraint for the state
                                  4. Thrust bound constraint at each knot point. 
  
  Author : Nak-seung Patrick Hyun
  Date : 07/25/2018
*/
#include <iostream>
#include <memory>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>

#include <gflags/gflags.h>
#include <iostream>
#include <fstream>

#include "drake/common/find_resource.h"
#include "drake/examples/robobee/robobee_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

#include "drake/examples/robobee/TrajectoryOptPlot.h"


using drake::solvers::SolutionResult;

namespace drake {
namespace examples {
namespace robobee {

typedef trajectories::PiecewisePolynomial<double> PiecewisePolynomialType;

DEFINE_double(simulation_real_time_rate, 1., "Real time rate");

namespace {
DEFINE_double(realtime_factor, 0.15,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  // Issue : Do not define plant as a pointer. Segfault fail when adding TrajectorySource system and connect to Drakevisualizer
  // auto robobee = builder.AddSystem<RobobeePlant<double>>();
  // robobee->set_name("robobee");
  // auto context = robobee->CreateDefaultContext();

//-[0] Get Robobee Plant
  RobobeePlant<double> robobee;
  auto context = robobee.CreateDefaultContext();

  int num_states = robobee.get_num_states();
  int num_input = robobee.get_input_size();
  
//-[0] Set up direct collocation
  
  const int kNumTimeSamples = 100;       // Number of knotpoints including first and last knot
  const double kMinimumTimeStep = 0.0002; // Minimum time step l_T
  const double kMaximumTimeStep = 0.5;  // Maximum time step u_T

//-[0-1] Create direct collocation object

  systems::trajectory_optimization::DirectCollocation dircol(
      &robobee, *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumTimeStep);

  
  // Additional : If one need to add torque constraint.
  // const double kTorqueLimit = 8000;
  // std::cout << "State size: " << robobee.get_num_states() << "\n";
  // std::cout << "dir state size" << context->get_continuous_state().size();
  // dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  // dircol.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);


  auto x = dircol.state(); // See multiple_shooting.h (not actually declared as decision variables in the MathematicalProgram) 
  auto u = dircol.input(); // See multiple_shooting.h (not actually declared as decision variables in the MathematicalProgram) 
  
//-[0-2] Initial configuration in SE(3)
  Eigen::VectorXd x0=Eigen::VectorXd::Zero(13);

  // Position r
  x0(0) = 1.; // x //1.
  x0(1) = 0.; // y
  x0(2) = 0.; // z //0.

  // Orientation q (quaternion)  q = (w, x, y, z)

  double theta0 = 0; //M_PI/4;  // angle of otation
  Eigen::Vector3d v0_q=Eigen::Vector3d:: Zero(3); // Axis of rotation
  v0_q(0) = 0.;
  v0_q(1) = 1.;
  v0_q(2) = 1.; // 1.
  
  // //Creating a unit quaternion q_u = cos(\theta/2) + sin(\theta/2)*\bar{q_v}/||q_v||
  Eigen::VectorXd q = Eigen::VectorXd::Zero(4);
  q(0)= cos(theta0/2);
  double v0_norm; 
  v0_norm = sqrt(v0_q.transpose()*v0_q);

  Eigen::VectorXd v0_normalized = Eigen::VectorXd::Zero(3);
  v0_normalized = v0_q/v0_norm;

  q(1)= sin(theta0/2)*v0_normalized(0);
  q(2)= sin(theta0/2)*v0_normalized(1);
  q(3)= sin(theta0/2)*v0_normalized(2);

  x0(3) =q(0);
  x0(4) =q(1);
  x0(5) =q(2);
  x0(6) =q(3);

  // Angular velocity w
  Eigen::Vector3d w0 = Eigen::Vector3d::Zero(3);
  w0(0)= 0;  // w_x
  w0(1)=  0.;   // w_y
  w0(2)=  0 ; // w_z
  
  x0(10)=w0(0); // w1=1;
  x0(11)=w0(1);// w2=1;
  x0(12)=w0(2);// w3=1;

//-[0-3] Final configuration in SE(3)
  Eigen::VectorXd xf=Eigen::VectorXd::Zero(13);
  
  // Position r
  xf(0) = 0.;  // x
  xf(1) = 0.;  // y
  xf(2) = 0.3; // z

  // Orientation q (quaternion)  q = (w, x, y, z)
  double thetaf = 2*M_PI/1;  // angle of otation
  Eigen::Vector3d vf_q=Eigen::Vector3d:: Zero(3); 
  vf_q(0) = 0.; //1
  vf_q(1) = 2.;
  vf_q(2) = 0.;
  
  // //Creating a unit quaternion q_u = cos(\theta/2) + sin(\theta/2)*\bar{q_v}/||q_v||

  Eigen::VectorXd qf = Eigen::VectorXd::Zero(4);
  qf(0)= cos(thetaf/2);
  double vf_norm; 
  vf_norm = sqrt(vf_q.transpose()*vf_q);

  Eigen::VectorXd vf_normalized = Eigen::VectorXd::Zero(3);
  vf_normalized = vf_q/vf_norm;

  qf(1)= sin(thetaf/2)*vf_normalized(0);
  qf(2)= sin(thetaf/2)*vf_normalized(1);
  qf(3)= sin(thetaf/2)*vf_normalized(2);
  // std::cout << "qf:" << qf <<"\n";

  xf(3) =qf(0);
  xf(4) =qf(1);
  xf(5) =qf(2);
  xf(6) =qf(3);

  // Angular velocity w
  Eigen::Vector3d wf = Eigen::Vector3d::Zero(3);
  wf(0)= 0.;
  wf(1)=  2.;
  wf(2)=  0. ;
  
  xf(10)=wf(0); // w1=1;
  xf(11)=wf(1);// w2=1;
  xf(12)=wf(2);// w3=1;


//-[0-3] Adding constraint to the problem

// 1. Equal time interval constraint
  dircol.AddEqualTimeIntervalsConstraints(); 

// 2. Dynamic constaint is included by creating the object from the plant robobee.

// 3. Boundary Constraint on the state
  dircol.AddBoundingBoxConstraint(x0, x0,
                                dircol.initial_state());  // Initial state constraint
  dircol.AddBoundingBoxConstraint(xf, xf,
                                dircol.final_state());    // Final state constraint

// Remark: other ways to add constraint
// // const Eigen::VectorXd xG(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
// // dircol.AddLinearConstraint(dircol.initial_state() == x0);
// // dircol.AddLinearConstraint(dircol.final_state() == xG);

// 4. Control constraint

  double u_min;
  double u_max;
  u_min = -2.;
  u_max = 2.;
  for(int i=0; i<kNumTimeSamples;i++){

    auto u_indexed = dircol.input(i);
    dircol.AddBoundingBoxConstraint(0,1e10,u_indexed(0)); // Positive Thrust Constraint
    dircol.AddBoundingBoxConstraint(u_min,u_max,u_indexed(1)); // Positive Thrust Constraint
    dircol.AddBoundingBoxConstraint(u_min,u_max,u_indexed(2)); // Positive Thrust Constraint
    dircol.AddBoundingBoxConstraint(u_min,u_max,u_indexed(3)); // Positive Thrust Constraint
    auto x_indexed = dircol.state(i);
    dircol.AddConstraint(x_indexed(3)*x_indexed(3)+x_indexed(4)*x_indexed(4)+
                         x_indexed(5)*x_indexed(5)+x_indexed(6)*x_indexed(6)-1,-0.01,0.01); // Positive Thrust Constraint
  }

//-[0-3] Adding cost to the problem 

  double gain_R = 15.; 
  double gain_Q = 20.;

  Eigen::Matrix4d R = gain_R*Eigen::Matrix4d::Identity();
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(13,13);
  
  Q(10,10)=gain_Q/8;
  Q(11,11)=1;
  Q(12,12)=gain_Q;


  dircol.AddRunningCost( (u.transpose()*R) * u+(x.transpose()*Q) * x);


//-[0-4] Define the guess for the initial state trajectory

  const double timespan_init = 10;
  auto traj_init_x =
      PiecewisePolynomialType::FirstOrderHold({0, timespan_init}, {x0, x0});
  dircol.SetInitialTrajectory(PiecewisePolynomialType(), traj_init_x);

//-[1] Solve Direct collocation

  SolutionResult result = dircol.Solve();

  if (result != SolutionResult::kSolutionFound) {
    std::cerr << "No solution found.\n";
    return 1;
  }  

//-[2] Extract the optimal solution 

  const trajectories::PiecewisePolynomial<double> pp_xtraj = // Optimal state trajectory as piecewise polynomial 
      dircol.ReconstructStateTrajectory();
  const trajectories::PiecewisePolynomial<double> pp_utraj = // Optimal input trajectory as piecewise polynomial 
      dircol.ReconstructInputTrajectory();

// //-[3] Visualization

// //-[3-1]  Adding the Trajectory source as a system to the builder     
//   auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);

// //-[3-2]  Add Drake visualizer & and read the urdf with kQuaternion joint

//   lcm::DrakeLcm lcm;
//   auto tree = std::make_unique<RigidBodyTree<double>>();
//   parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
//       FindResourceOrThrow("drake/examples/robobee/robobee.urdf"),
//       multibody::joints::kQuaternion, tree.get());
  
//   auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);

// // Remark on simulating from the polynomial trajeocty

//   // By default, the simulator triggers a publish event at the end of each time
//   // step of the integrator. However, since this system is only meant for
//   // playback, there is no continuous state and the integrator does not even get
//   // called. Therefore, we explicitly set the publish frequency for the
//   // visualizer.
  
//   publisher->set_publish_period(1.0 / 12000.0);

//   builder.Connect(state_source->get_output_port(),
//                   publisher->get_input_port(0));

//   auto diagram = builder.Build();

// //-[3-3] Construct the simulator 
//   systems::Simulator<double> simulator(*diagram);

//   simulator.set_target_realtime_rate(FLAGS_realtime_factor);
//   simulator.Initialize();
//   simulator.StepTo(pp_xtraj.end_time());

//   std::cout << "Ending time: "<< pp_xtraj.end_time() << "\n";

// //-[3-4] Run simulation
//   simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);

//   simulator.StepTo(pp_xtraj.end_time());


// Exporting the dircol solution.


  // Remark: Useful function to see the solution at once: dircol.PrintSolution(); // Print the solution from MP
  
//-[4] Plotting the trajopt result.

  std::string directory;
  double N =200; // Number of time points for plotting

  directory = "/home/plancher/Desktop/drake/examples/robobee";
  TrajectoryOptPlot(dircol, directory, num_states, num_input, kNumTimeSamples, N);
  
  return 0;
}

}  // namespace
}  // namespace robobee
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::robobee::do_main();
}
