#pragma once

#include <cmath>
#include <cstdint>

#include <Eigen/Core>

template <int Dim>
class RLS
{
	static_assert(Dim > 0, "RLS dimension must be positive");

public:
	using Vector = Eigen::Matrix<float, Dim, 1>;
	using Covariance = Eigen::Matrix<float, Dim, Dim>;

	explicit RLS(const float delta_, const float lambda_, const Vector& init_params_)
		: lambda(lambda_), delta(delta_), init_params(init_params_),
		params(Vector::Zero()), covariance(Covariance::Zero()), gain(Vector::Zero()),
		output(0.0f), error(0.0f), update_cnt(0)
	{
		CheckConfig();
		Reset();
	}

	void Reset()
	{
		params = init_params;
		covariance = Covariance::Identity() * delta;
		gain.setZero();
		output = 0.0f;
		error = 0.0f;
		update_cnt = 0;
	}

	const Vector& Update(const Vector& sample_, const float actual_)
	{
		constexpr float kMinDenominator = 1.0e-6f;
		const Vector cov_sample = covariance * sample_;
		const float denominator = lambda + sample_.dot(cov_sample);
		output = sample_.dot(params);
		error = actual_ - output;

		if (!std::isfinite(denominator) || !std::isfinite(error) ||
			std::abs(denominator) <= kMinDenominator)
		{
			return params;
		}

		gain = cov_sample / denominator;
		params += gain * error;
		covariance = ((covariance - gain * sample_.transpose() * covariance) / lambda).eval();
		covariance = (0.5f * (covariance + covariance.transpose())).eval();
		++update_cnt;
		return params;
	}

	const Vector& GetParamsVector() const { return params; }
	const Covariance& GetCovariance() const { return covariance; }
	float GetOutput() const { return output; }
	float GetError() const { return error; }
	uint32_t GetUpdateCnt() const { return update_cnt; }

private:
	void CheckConfig()
	{
		if (!std::isfinite(lambda) || lambda <= 0.0f || lambda > 1.0f) lambda = 1.0f;
		if (!std::isfinite(delta) || delta <= 0.0f) delta = 1.0f;
	}

	float lambda;
	float delta;
	Vector init_params;
	Vector params;
	Covariance covariance;
	Vector gain;
	float output;
	float error;
	uint32_t update_cnt;
};
