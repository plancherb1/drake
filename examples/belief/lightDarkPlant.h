#pragma once

#include <memory>
#include <random>
#include <Eigen/Dense>
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"


namespace drake {
namespace examples {
namespace belief {

    using Eigen::Matrix;
    using Eigen::DiagonalMatrix;

    template <typename T>
    class LightDarkPlant final : public systems::LeafSystem<T> {
        public:
            LightDarkPlant();
            LightDarkPlant(int flag);

            /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
            template <typename U>
            explicit LightDarkPlant(const LightDarkPlant<U>&);

            ~LightDarkPlant() override;

            int get_input_size() const { return kInputDimension; }
            int get_num_states() const { return kStateDimension; }

            void set_state(systems::Context<T>* context, const VectorX<T>& x) const {
                context->get_mutable_continuous_state_vector().SetFromVector(x);
            }

            int get_stochastic() const { return StochasticFlag; }

        protected:
            void CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const;

            void DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const override;

            T ComputeNoiseVar(T x) const;

            // T ComputeNoise(const systems::Context<T> &context) const;

            /// Declares that the system has no direct feedthrough from any input to any
            /// output.
            ///
            /// The RobobeePlant is incompatible with the symbolic::Expression scalar
            /// type because it invokes the Cholesky LDLT decomposition, which uses
            /// conditionals in its implementation. Therefore, we must specify sparsity
            /// by hand.
            optional<bool> DoHasDirectFeedthrough(int, int) const override {return false;}

        private:
            // Allow different specializations to access each other's private data.
            template <typename> friend class LightDarkPlant;

            static constexpr int kStateDimension{3};
            static constexpr int kInputDimension{2};
            int StochasticFlag;
    };

}  // namespace belief
}  // namespace examples

// The following code was added to prevent scalar conversion to symbolic scalar
// types. The RobobeePlant makes use of classes that are not compatible with
// the symbolic scalar. This NonSymbolicTraits is explained in
// drake/systems/framework/system_scalar_converter.h.
namespace systems {
namespace scalar_conversion {
    template <>
    struct Traits<examples::belief::LightDarkPlant> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
