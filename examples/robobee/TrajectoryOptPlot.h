#pragma once

#include <string>
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace examples {
namespace robobee {

typedef trajectories::PiecewisePolynomial<double> PiecewisePolynomialType;

int TrajectoryOptPlot(systems::trajectory_optimization::DirectCollocation& dircol, std::string directory, 
                      int num_states, int num_input, int kNumTimeSamples, double N);
/// The Quadrotor - an underactuated aerial vehicle. This version of the
/// Quadrotor is implemented to match the dynamics of the plant specified in
/// the `quadrotor.urdf` model file.
//-[4] Plotting the trajopt result.

}  // namespace robobee
}  // namespace examples
}  // namespace drake
