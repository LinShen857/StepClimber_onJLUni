#pragma once

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>


namespace JLUni::Algo
{
    struct kalman_r1_s
    {
        float x;
        float P;  // estimation error covariance
        const float Q;  // process noise covariance
        const float R;  // measurement noise covariance
        float K;  // Kalman gain
        float Update(const float measurement_)
        {
            P += Q;  // prediction step: increase uncertainty
            K = P / (P + R);  // compute Kalman gain
            x += K * (measurement_ - x);  // update estimate with measurement
            P *= 1.f - K;  // update uncertainty
            return x;
        }
        void Reset(const float x_new_ = 0.f)
        {
            x = x_new_;  // reset state estimate
            P = 1.f;  // reset covariance to the initial measurement covariance
            K = 0.f;  // reset Kalman gain
        }
    };

    template <typename Model, int StateDim, int InputDim, int MeasurementDim>
    class KalmanFilter
    {
        static_assert(StateDim > 0, "Kalman state dimension must be positive");
        static_assert(InputDim > 0, "Kalman input dimension must be positive");
        static_assert(MeasurementDim > 0, "Kalman measurement dimension must be positive");

    public:
        using StateVector = Eigen::Matrix<float, StateDim, 1>;
        using InputVector = Eigen::Matrix<float, InputDim, 1>;
        using MeasurementVector = Eigen::Matrix<float, MeasurementDim, 1>;
        using StateMatrix = Eigen::Matrix<float, StateDim, StateDim>;
        using ProcessNoiseMatrix = Eigen::Matrix<float, InputDim, InputDim>;
        using MeasurementMatrix = Eigen::Matrix<float, MeasurementDim, MeasurementDim>;
        using StateInputMatrix = Eigen::Matrix<float, StateDim, InputDim>;
        using MeasurementStateMatrix = Eigen::Matrix<float, MeasurementDim, StateDim>;
        using KalmanGainMatrix = Eigen::Matrix<float, StateDim, MeasurementDim>;

        KalmanFilter(const ProcessNoiseMatrix& mat_Q_, const MeasurementMatrix& mat_R_,
            const StateMatrix& mat_P_, const StateVector& vec_x_, const InputVector& vec_u_)
            : mat_Q(mat_Q_), mat_R(mat_R_), mat_P(mat_P_), vec_x(vec_x_), vec_u(vec_u_),
            vec_x_prior(StateVector::Zero()), mat_P_prior(StateMatrix::Zero()),
            mat_K(KalmanGainMatrix::Zero()), vec_z(MeasurementVector::Zero())
        {}

        void Reset(const ProcessNoiseMatrix& mat_Q_, const MeasurementMatrix& mat_R_,
            const StateMatrix& mat_P_, const StateVector& vec_x_, const InputVector& vec_u_)
        {
            mat_Q = mat_Q_;
            mat_R = mat_R_;
            mat_P = mat_P_;
            vec_x = vec_x_;
            vec_u = vec_u_;
            vec_x_prior.setZero();
            mat_P_prior.setZero();
            mat_K.setZero();
            vec_z.setZero();
        }

        /**
         * @brief EKF filter predict step
         * @param dt_ time step
         */
        void Predict(const float dt_)
        {
            vec_x_prior = Model::F(vec_x, vec_u, dt_);
            const StateMatrix mat_F_x = Model::JacobianF2X(vec_x, vec_u, dt_);
            const StateInputMatrix mat_F_w = Model::JacobianF2W(vec_x, vec_u, dt_);
            mat_P_prior = mat_F_x * mat_P * mat_F_x.transpose()
                + mat_F_w * mat_Q * mat_F_w.transpose();
            mat_P_prior = (0.5f * (mat_P_prior + mat_P_prior.transpose())).eval();
            vec_x = vec_x_prior;
            mat_P = mat_P_prior;
        }

        /**
         * @brief EKF filter update step
         * @note prediction should be up to the measurement time
         */
        bool Update(const float dt_)
        {
            constexpr float kMinRelativePivot = 32.0f * Eigen::NumTraits<float>::epsilon();
            const MeasurementStateMatrix mat_H_x = Model::JacobianH2X(vec_x_prior, dt_);
            const MeasurementMatrix mat_H_v = Model::JacobianH2V(vec_x_prior, dt_);
            const MeasurementMatrix effective_measurement_noise =
                (mat_H_v * mat_R * mat_H_v.transpose()).eval();
            const MeasurementMatrix innovation_covariance =
                (mat_H_x * mat_P_prior * mat_H_x.transpose() + effective_measurement_noise).eval();

            mat_K.setZero();
            if (!innovation_covariance.allFinite()) return false;

            const Eigen::LDLT<MeasurementMatrix> decomposition(innovation_covariance);
            if (decomposition.info() != Eigen::Success) return false;

            const auto diagonal = decomposition.vectorD();
            const float max_pivot = diagonal.cwiseAbs().maxCoeff();
            const float min_pivot = diagonal.minCoeff();
            if (!std::isfinite(max_pivot) || !std::isfinite(min_pivot) || max_pivot <= 0.0f ||
                min_pivot <= kMinRelativePivot * max_pivot)
            {
                return false;
            }

            const KalmanGainMatrix cross_covariance = mat_P_prior * mat_H_x.transpose();
            const Eigen::Matrix<float, MeasurementDim, StateDim> solved_gain =
                decomposition.solve(cross_covariance.transpose());
            if (!solved_gain.allFinite()) return false;

            const KalmanGainMatrix candidate_gain = solved_gain.transpose();
            const StateVector candidate_state =
                vec_x_prior + candidate_gain * (vec_z - Model::H(vec_x_prior, dt_));
            const StateMatrix identity_minus_kh =
                StateMatrix::Identity() - candidate_gain * mat_H_x;
            StateMatrix candidate_covariance =
                (identity_minus_kh * mat_P_prior * identity_minus_kh.transpose()
                    + candidate_gain * effective_measurement_noise * candidate_gain.transpose()).eval();
            candidate_covariance =
                (0.5f * (candidate_covariance + candidate_covariance.transpose())).eval();

            if (!candidate_state.allFinite() || !candidate_covariance.allFinite()) return false;

            mat_K = candidate_gain;
            vec_x = candidate_state;
            mat_P = candidate_covariance;
            return true;
        }

        ProcessNoiseMatrix mat_Q;  // process noise covariance
        MeasurementMatrix mat_R;  // measurement noise covariance

        StateMatrix mat_P;  // estimation error covariance
        StateVector vec_x;  // state estimate
        InputVector vec_u;  // control input
        StateVector vec_x_prior;  // prior state estimate
        StateMatrix mat_P_prior;  // prior estimation error covariance
        KalmanGainMatrix mat_K;  // Kalman gain

        MeasurementVector vec_z;  // measurement
    };
}
