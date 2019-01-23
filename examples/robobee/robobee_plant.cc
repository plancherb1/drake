#include "drake/examples/robobee/robobee_plant.h"

#include <memory>
#include "drake/common/default_scalars.h"
#include "drake/math/gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::Matrix3d;

namespace drake {
namespace examples {
namespace robobee {

namespace {
/*  Matrix3d default_moment_of_inertia() {
  return (Eigen::Matrix3d() <<  // BR
          0.0023, 0, 0,  // BR
          0, 0.0023, 0,  // BR
          0, 0, 0.0040).finished();
}
}  // namespace

template <typename T>
RobobeePlant<T>::RobobeePlant()
    : RobobeePlant(0.5,    // m (kg)
                     0.175,  // L (m)
                     default_moment_of_inertia(),
                     1.0,    // kF
                     0.0245  // kM
                     ) {}
*/                     

Matrix3d default_moment_of_inertia() {
  return (Eigen::Matrix3d() <<  // BR
                0.00142, 0, 0,  // BR
          0,    0.00134, 0,  // BR
          0, 0, 0.00045).finished();
}
}  // namespace

template <typename T>
RobobeePlant<T>::RobobeePlant()
    : RobobeePlant(81.0,    // m (kg)
                     default_moment_of_inertia() ) {}

template <typename T>
RobobeePlant<T>::RobobeePlant(double m_arg, const Matrix3d& I_arg)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<robobee::RobobeePlant>{}),
      g_{9.81}, m_(m_arg), I_(I_arg) {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareContinuousState(kStateDimension);
  this->DeclareVectorOutputPort(systems::BasicVector<T>(kStateDimension),
                                &RobobeePlant::CopyStateOut);
}

template <typename T>
template <typename U>
RobobeePlant<T>:: RobobeePlant(const RobobeePlant<U>& other)
    : RobobeePlant<T>(other.m_, other.I_) {}

template <typename T>
RobobeePlant<T>::~RobobeePlant() {}

template <typename T>
void RobobeePlant<T>::CopyStateOut(const systems::Context<T> &context,
                                     systems::BasicVector<T> *output) const {
  output->set_value(
      context.get_continuous_state_vector().CopyToVector());
}

template <typename T>
void RobobeePlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
  // Get the input value characterizing each of the 4 rotor's aerodynamics.
  const Vector4<T> u = this->EvalVectorInput(context, 0)->get_value();

  // For each rotor, calculate the Bz measure of its aerodynamic force on B.
  // Note: B is the rob boobeedy and Bz is parallel to each rotor's spin axis.
  // u= [F_T, tau_1, tau2, tau3] : Robobee input Thrust and torque in the body frame
  const Vector4<T> uF_Bz = u;

  // Compute the net aerodynamic force on B (from the 4 rotors), expressed in B.
  const Vector3<T> Faero_B(0, 0, m_*uF_Bz(0));

  // Compute the Bx and By measures of the moment on B about Bcm (B's center of
  // mass) from the 4 rotor forces.  These moments arise from the cross product
  // of a position vector with an aerodynamic force at the center of each rotor.
  // For example, the moment of the aerodynamic forces on rotor 0 about Bcm
  // results from Cross( L_* Bx, uF_Bz(0) * Bz ) = -L_ * uF_Bz(0) * By.
  const T Mx = uF_Bz(1);
  const T My = uF_Bz(2);

  // For rotors 0 and 2, get the Bz measure of its aerodynamic torque on B.
  // For rotors 1 and 3, get the -Bz measure of its aerodynamic torque on B.
  // Sum the net Bz measure of the aerodynamic torque on B.
  // Note: Rotors 0 and 2 rotate one way and rotors 1 and 3 rotate the other.
  const Vector4<T> uTau_Bz = u;
  const T Mz = uTau_Bz(3);

  // Form the net moment on B about Bcm, expressed in B. The net moment accounts
  // for all contact and distance forces (aerodynamic and gravity forces) on B.
  // Note: Since the net moment on B is about Bcm, gravity does not contribute.
  // const Vector3<T> Tau_B(Mx, My, Mz);
  const Vector3<T> Tau_B(Mx, My, Mz);
  //std::cout << "Tau_B: " << Tau_B(2) <<"\n";
  // Calculate local celestial body's (Earth's) gravity force on B, expressed in
  // the Newtonian frame N (a.k.a the inertial or World frame).
  const Vector3<T> Fgravity_N(0, 0, -m_ * g_);

  // Extract quaternion and their time-derivatives (quatDt).
  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();
  // std::cout << "quaternion input: " << state.template segment<4>(3) <<"\n";
  
  // T time_t= context.get_time();

  // Convert to Eigen quaternion
  Vector4<T> quat_vector = state.template segment<4>(3);
  
  T temp = quat_vector(3);
  quat_vector(3)=quat_vector(0);
  quat_vector(0)=quat_vector(1);
  quat_vector(1)=quat_vector(2);
  quat_vector(2)=temp;

  const Eigen::Quaternion<T> quat(quat_vector);
  const Vector3<T> w_BN_B = state.template tail<3>();

  // const Vector3<T> quatDt = state.template segment<3>(9);
  // std::cout << "Segment indexing n,i ? :" << state.template segment<3>(9) << "\n";
  // std::cout << "Segment tail :" << state.template tail<3>() << "\n";
  // Convert rolltch-yaw (rpy) orientation to the R_NB rotation matrix.
  // std::cout << "quaternion:" << quat_vector <<"\n";
  const drake::math::RotationMatrix<T> R_NB(quat);
  // std::cout << "R_NB: "<< R_NB.matrix() << "\n";

  // Calculate the net force on B, expressed in N.  Use Newton's law to
  // calculate a_NBcm_N (acceleration of B's center of mass, expressed in N).
  const Vector3<T> Fnet_N = Fgravity_N + R_NB * Faero_B;
  const Vector3<T> xyzDDt = Fnet_N / m_;  // Equal to a_NBcm_N.

  // Use rpy and rpyDt to calculate B's angular velocity in N, expressed in B.
  //const Vector3<T> w_BN_B = rpy.CalcAngularVelocityInChildFromRpyDt(rpyDt);
  const Vector4<T> quatDt = drake::math::CalculateQuaternionDtFromAngularVelocityExpressedInB(quat, w_BN_B);

// Checking the quaternion and angular velocity (World vs Body)
  // MatrixX<T> Eq = Eigen::MatrixXd::Zero(3,4);
  // Vector4<T> quat_original_vector = state.template segment<4>(3);
  // T q0, q1, q2, q3;
  // q0=quat_original_vector(0);
  // q1=quat_original_vector(1);
  // q2=quat_original_vector(2);
  // q3=quat_original_vector(3);

  // Eq.block(0,0,1,4) <<  -1*q1,    q0,   1*q3,   -1*q2;
  // Eq.block(1,0,1,4) << -1*q2,  -1*q3,     q0,  1*q1;
  // Eq.block(2,0,1,4) << -1*q3,   1*q2,  -1*q1,     q0;
  
  // Vector4<T> quatDt_error;
  
  // quatDt_error = quatDt - 1/2.*Eq.transpose()*w_BN_B; 

  // std::cout << "\n qdot error due to missmatch Eq : \n" << quatDt_error<<"\n";

  
  // To compute α (B's angular acceleration in N) due to the net moment 𝛕 on B,
  // rearrange Euler rigid body equation  𝛕 = I α + ω × (I ω)  and solve for α.


  const Vector3<T> wIw = w_BN_B.cross(I_ * w_BN_B);            // Expressed in B
  const Vector3<T> alpha_NB_B = I_.ldlt().solve(Tau_B - wIw);  // Expressed in B
  // const Vector3<T> alpha_NB_N = R_NB * alpha_NB_B;             // Expressed in N

  // Calculate the 2nd time-derivative of rpy.
  const Vector3<T> wDDt = alpha_NB_B;
      // rpy.CalcRpyDDtFromRpyDtAndAngularAccelInParent(rpyDt, alpha_NB_N);

  // Recomposing the derivatives vector.
  VectorX<T> xDt(13);
  xDt << state.template segment<3>(7), quatDt, xyzDDt, wDDt;
  derivatives->SetFromVector(xDt);
  //t << "\n";
  //std::cout << Faero_B(2) << "\n";
  // std::cout << "=================================\n";
  // std::cout << "state: "<< state << "\n";
  // std::cout << "xDt: "<< xDt << "\n";

  // // Printing the state
  // std::cout << "Time : "<< time_t<< "\n";
  
  // std::cout << "Thrust to weight ratio : "<<Faero_B(2)/(-1*Fgravity_N(2)) << "\n";

  // std::cout << "z Position : "<< state(2) << "\n";
  // std::cout << "Thrust (mN/s^2) : "<< Faero_B(2)*1 << "\n";
  // std::cout << "Accellaration (cm/s^2) : "<< xyzDDt(2)*100 << "\n";
  // // std::cout << "q (rad): "<< state(3) << "\n";
  // // std::cout << "p (rad): "<< state(4) << "\n";
  // // std::cout << "y (rad): "<< state(5) << "\n";
  // std::cout << "Tau_x (mNmm): "<< u(1)*1000 << "\n";
  // std::cout << "Tau_y (mNmm): "<< u(2)*1000 << "\n";
  // std::cout << "Tau_z (mNmm): "<< u(3)*1000 << "\n";

  // std::cout << "=================================\n";
}

// Declare storage for our constants.
template <typename T>
constexpr int RobobeePlant<T>::kStateDimension;
template <typename T>
constexpr int RobobeePlant<T>::kInputDimension;

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const RobobeePlant<double>* robobee_plant,
    Eigen::Vector3d nominal_position) {
  auto quad_context_goal = robobee_plant->CreateDefaultContext();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(13);
  x0.topRows(3) = nominal_position;
  x0(3) =1;

  // Nominal input corresponds to a hover.
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(4);
  u0(0)= robobee_plant->g();
  
//std::cout << "Nominal input: " << u0(0) << u0(1) << u0(2) << u0(3); 

  quad_context_goal->FixInputPort(0, u0);
  robobee_plant->set_state(quad_context_goal.get(), x0);

  // Setup LQR cost matrices (penalize position error 10x more than velocity
  // error).
  Eigen::MatrixXd Q = 1*Eigen::MatrixXd::Identity(13, 13);
  Q.topLeftCorner<3, 3>() = 1 * Eigen::MatrixXd::Identity(3, 3);
  Q(2,2)=1;
  std::cout << Q;
  Eigen::Matrix4d R = 1*Eigen::Matrix4d::Identity();

  return systems::controllers::LinearQuadraticRegulator(
      *robobee_plant, *quad_context_goal, Q, R);
}

}  // namespace robobee
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::robobee::RobobeePlant)
