/*
  TrajectoryOptPlot.cc

  Objective : Plot piecewise polynomial objects collected from DirectCollocation.

  Input : 1. dircol : directcollocation object pointer 
          2. directory : where to save the file
          3. num_states: number of states : Robobee w/ quaternion 13
          4. num_input: number of inputs : Robobee 4
          5. kNumTimeSamples: number of knot points
          6. N : Number of time samples to be plotted. (for visualization purpose) 
  Output : 1. Directory/state_trajopt.txt : Matrix of (Nx (num_states+1)) : First column is the time
           2. Directory/input_trajopt.txt : Matrix of (Nx (num_input+1)) : First column is the time
           3. Directory/time_col_trajopt.txt : Matrix of (kNumTimeSamplesx 1) : Collocation points
           3. Directory/state_col_trajopt.txt : Matrix of (kNumTimeSamplesx (num_states)) : Collocation points
           3. Directory/input_col_trajopt.txt : Matrix of (kNumTimeSamplesx (num_input)) : Collocation points

  Remark : The saved data file can be plotted by calling python script : plot_traj_opt.py
  TODO : Use CallPython to remotely run python script using rpyc.
  
  Author : Nak-seung Patrick Hyun
  Date : 07/25/2018
*/

#include <math.h>
#include <iostream>
#include <fstream>
#include "drake/systems/trajectory_optimization/direct_collocation.h"


namespace drake {
namespace examples {
namespace robobee {


int TrajectoryOptPlot(systems::trajectory_optimization::DirectCollocation& dircol, std::string directory, 
                      int num_states, int num_input, int kNumTimeSamples, double N){
  
  std::string file_name_state ("/state_trajopt.txt");
  std::string file_name_input ("/input_trajopt.txt");
  std::string file_name_time_col_trajopt ("/time_col_trajopt.txt");
  std::string file_name_state_col_trajopt ("/state_col_trajopt.txt");
  std::string file_name_input_col_trajopt ("/input_col_trajopt.txt");
  
  file_name_state = directory + file_name_state;
  file_name_input = directory + file_name_input;
  file_name_time_col_trajopt = directory + file_name_time_col_trajopt;
  file_name_state_col_trajopt = directory + file_name_state_col_trajopt;
  file_name_input_col_trajopt = directory + file_name_input_col_trajopt;
  // std::cout << file_name_state;

  const trajectories::PiecewisePolynomial<double> pp_xtraj = // Optimal state trajectory as piecewise polynomial 
      dircol.ReconstructStateTrajectory();
  const trajectories::PiecewisePolynomial<double> pp_utraj = // Optimal input trajectory as piecewise polynomial 
      dircol.ReconstructInputTrajectory();


  std::ofstream output_file;
  output_file.open(file_name_state);

  if (!output_file.is_open()) {
      std::cerr << "Problem opening solution output file.\n";
  }

  double T =pp_xtraj.end_time();
  double tt;

  Eigen::VectorXd x_opt;
  for (int i = 0; i <= N; i++) {
    tt = i/N * T;
    x_opt = pp_xtraj.value(tt);
    output_file << tt << '\t';    
    
    for (int j=0; j<num_states-1; j++){
      output_file << x_opt[j] << '\t';
    }

    output_file << x_opt[num_states-1] << std::endl;
  }
  
  output_file.close();

  output_file.open(file_name_input);
  if (!output_file.is_open()) {
      std::cerr << "Problem opening solution output file.\n";
  }
  
  Eigen::VectorXd u_opt;
  for (int i = 0; i <= N; i++) {
    tt = i/N * T;
    u_opt = pp_utraj.value(tt);
    output_file << tt << '\t';    
    
    for (int j=0; j<num_input-1; j++){
      output_file << u_opt[j] << '\t';
    }

    output_file << u_opt[num_input-1] << std::endl;
  }
  
  output_file.close();


  output_file.close();
  

//[4-2] Get the optimal state and input for all the knot points
  
  Eigen::VectorXd times_col = dircol.GetSampleTimes(); // Get the collocation time
  Eigen::VectorXd state_col(13);                       
  Eigen::VectorXd input_col(4);                         

  output_file.open(file_name_time_col_trajopt);
  if (!output_file.is_open()) {
      std::cerr << "Problem opening solution output file.\n";
  }
  
  for (int j=0; j<kNumTimeSamples; j++){
      output_file << times_col[j] << '\t';
  }

  output_file.close();

   output_file.open(file_name_state_col_trajopt);
  if (!output_file.is_open()) {
      std::cerr << "Problem opening solution output file.\n";
  }
  for (int i=0; i<kNumTimeSamples; i++){
    state_col = dircol.GetSolution(dircol.state(i));
    for (int j=0; j<num_states-1; j++){
        output_file << state_col[j] << '\t';
    }
    output_file << state_col[num_states-1] << std::endl;
  }
  output_file.close();

  output_file.open(file_name_input_col_trajopt);
  if (!output_file.is_open()) {
      std::cerr << "Problem opening solution output file.\n";
  }
  for (int i=0; i<kNumTimeSamples; i++){
    input_col = dircol.GetSolution(dircol.input(i));
    for (int j=0; j<num_input-1; j++){
        output_file << input_col[j] << '\t';
    }
    output_file << input_col[num_input-1] << std::endl;
  }
  output_file.close();

return 0;

}  // TrajectoOptPlot

}  // namespace robobee
}  // namespace examples
}  // namespace drake
