#include <memory>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>

#include <gflags/gflags.h>
#include <iostream>
#include <fstream>

#include "drake/common/find_resource.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

#include "drake/examples/belief/lightDarkPlant.h"

namespace drake {
namespace examples {
namespace belief {

template <typename T>
int do_main() {

//-[0-0] Get Plant and init dircol object
    LightDarkPlant<double> plant;
    auto context = plant.CreateDefaultContext();

    const int kNumTimeSteps = 20;        // Number of knotpoints including first and last knot
    const double kMinimumTimeStep = 0.1;  // Minimum time step l_T
    const double kMaximumTimeStep = 0.1;  // Maximum time step u_T

    systems::trajectory_optimization::DirectCollocation dircol(&plant, *context, kNumTimeSteps, kMinimumTimeStep, kMaximumTimeStep);
  
//-[0-1] Get problem size and initial and final positions
    const int n = plant.get_num_states();
    // const int m = plant.get_input_size();
    auto u = dircol.input();
    auto b = dircol.state();
 
    Matrix<T,3,1> b0;   Matrix<T,2,1> x0;   Matrix<T,2,1> xf;   Matrix<T,3,1> bf;
    b0 << 2, 2, 5;      x0 << 2.5, 0;       xf << 0, 0;         bf << 0, 0, 0;

    Matrix<T,2,1> xfd;  Matrix<T,2,1> xfu;  xfd << -0.1, -0.1;  xfu << 0.1, 0.1;

//-[0-2] Add constraint to the problem -- note dynamic constraint is added automatically
    dircol.AddEqualTimeIntervalsConstraints();                                  // All time steps are equal length
    dircol.AddBoundingBoxConstraint(b0, b0, dircol.initial_state());            // Initial belief state constraint
    dircol.AddBoundingBoxConstraint(xfd, xfu, dircol.final_state().topRows(2));   // Final state constraint
    // Matrix<T,2,1> u_min; u_min << -5, -5; Matrix<T,2,1> u_max; u_max << 5, 5;    // Torque limits
    // for(int i=0; i<kNumTimeSteps; i++){
    //     dircol.AddBoundingBoxConstraint(u_min,u_max,dircol.input(i));
    // }

//-[0-3] Add cost to the problem
    Matrix<T,3,3> Q = DiagonalMatrix<T,3>(0.5,0.5,0);         Matrix<T,2,2> R = DiagonalMatrix<T,2>(0.5,0.5);
    // small on every beleif state delta from goal and on input
    dircol.AddRunningCost(u.transpose()*R*u + (b - bf).transpose()*Q*(b - bf));
    // large on final on variance as well
    auto finalVar = dircol.state().bottomRows(1);     T Qvar = 20;//200;
    dircol.AddFinalCost(finalVar*Qvar*finalVar);

//-[0-4] Set initial guess
    const double timespan_init = 10;
    auto traj_init_x = trajectories::PiecewisePolynomial<T>::FirstOrderHold({0, timespan_init}, {b0, bf});
    dircol.SetInitialTrajectory(trajectories::PiecewisePolynomial<T>(), traj_init_x);

//-[1] Solve Direct collocation
    drake::solvers::SolutionResult result = dircol.Solve();
    if (result != drake::solvers::SolutionResult::kSolutionFound) {
    std::cerr << "No solution found.\n";
    std::cerr << result << std::endl;
    return 1;
    }  

//-[2] Log the optimal solution

    const trajectories::PiecewisePolynomial<double> pp_xtraj = dircol.ReconstructStateTrajectory();
    const trajectories::PiecewisePolynomial<double> pp_utraj = dircol.ReconstructInputTrajectory();
    
    std::ofstream output_file;  output_file.open("/home/plancher/Desktop/drake/examples/belief/data.csv");
    Eigen::VectorXd data;       T totalTime = pp_xtraj.end_time();      T tt;
    
    // output_file << "Final State Trajectory" << std::endl;
    for (int i = 0; i <= kNumTimeSteps; i++) {
        tt = static_cast<double>(i)/kNumTimeSteps * totalTime;   output_file << tt << ',';
        data = pp_xtraj.value(tt);
        for (int j=0; j<n-1; j++){output_file << data[j] << ',';}
        output_file << data[n-1] << std::endl;
    }
    
    // output_file << "Final Input Trajectory" << std::endl;
    // for (int i = 0; i <= kNumTimeSteps-1; i++) {
    //     tt = static_cast<double>(i)/kNumTimeSteps * totalTime;   output_file << tt << ',';
    //     data = pp_utraj.value(tt);
    //     for (int j=0; j<m-1; j++){output_file << data[j] << ',';}
    //     output_file << data[m-1] << std::endl;
    // }
    output_file.close();
  
  return 0;
}

}  // namespace belief
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::belief::do_main<double>();
}
