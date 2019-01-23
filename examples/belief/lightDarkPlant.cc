#include "drake/examples/belief/lightDarkPlant.h"
#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace belief {

    // constructors and destructors
    template <typename T>
    LightDarkPlant<T>::LightDarkPlant() : LightDarkPlant(0) {}

    template <typename T>
    LightDarkPlant<T>::LightDarkPlant(int flag) : systems::LeafSystem<T>(systems::SystemTypeTag<belief::LightDarkPlant>{}), StochasticFlag(flag) {
        this->DeclareInputPort(systems::kVectorValued, kInputDimension);
        this->DeclareContinuousState(kStateDimension);
        this->DeclareVectorOutputPort(systems::BasicVector<T>(kStateDimension),&LightDarkPlant::CopyStateOut);
    }

    template <typename T>
    template <typename U>
    LightDarkPlant<T>:: LightDarkPlant(const LightDarkPlant<U>& other): LightDarkPlant<T>(other.StochasticFlag) {}

    template <typename T>
    LightDarkPlant<T>::~LightDarkPlant() {}

    template <typename T>
    void LightDarkPlant<T>::CopyStateOut(const systems::Context<T> &context, systems::BasicVector<T> *output) const {
      output->set_value(context.get_continuous_state_vector().CopyToVector());
    }

    template <typename T>
    T LightDarkPlant<T>::ComputeNoiseVar(T x) const {
        T delta = 5.0-x;    T sigma = 0.5*delta*delta;  return sigma;
    }

    // template <typename T>
    // T LightDarkPlant<T>::ComputeNoise(const systems::Context<T> &context) const {
    //     T sigma = ComputeNoiseVar(context);
    //     std::default_random_engine gen;
    //     std::normal_distribution<T> d{0,sigma};
    //     return d(gen);
    // }

    template <typename T>
    void LightDarkPlant<T>::DoCalcTimeDerivatives(const systems::Context<T> &context, systems::ContinuousState<T> *derivatives) const {
        // get x and u
        const Matrix<T,2,1> u = this->EvalVectorInput(context, 0)->get_value();
        const Matrix<T,3,1> x = context.get_continuous_state_vector().CopyToVector();
        // compute f(x,u)
        Matrix<T,2,1> qd = x.topRows(2) + u;
        if (this->get_stochastic()){
            std::cout << "UGH we need to figure out how to compute autodiffable noise\n";
            // qd(0,0) += ComputeNoise(context);
            // qd(1,0) += ComputeNoise(context);
        }
        // compute new variance
        T var = x(2,0);    T noiseVar = ComputeNoiseVar(x(0,0));
        T var_new = var - ((var*var) / (var + noiseVar));
        // package
        Matrix<T,3,1> xd;
        xd << qd, var_new;
        derivatives->SetFromVector(xd);
        // testing
        // std::cout << "=================================\n";
        // std::cout << "x:  "<< x << "\n";
        // std::cout << "u:  "<< u << "\n";
        // std::cout << "xd: "<< xd << "\n";
        // std::cout << "=================================\n";
    }

    // Declare storage for our constants.
    template <typename T>
    constexpr int LightDarkPlant<T>::kStateDimension;
    template <typename T>
    constexpr int LightDarkPlant<T>::kInputDimension;

}  // namespace belief
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::belief::LightDarkPlant)
