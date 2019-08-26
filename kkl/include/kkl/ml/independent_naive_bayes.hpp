#ifndef KKL_INDEPENDENT_NAIVE_BAYES_HPP
#define KKL_INDEPENDENT_NAIVE_BAYES_HPP

#include <memory>
#include <Eigen/Dense>

#include <kkl/math/gaussian.hpp>

namespace kkl {
	namespace ml {

/*******************************************************
 * IndependentNaiveBayes
 * 各次元が独立している仮定でのナイーブベイズ
*******************************************************/
template<typename T>
class IndependentNaiveBayes {
public:
	/*******************************************************
	 * constructor
	*******************************************************/
	IndependentNaiveBayes(int dim)
		: cummulative_pos_weight(0.001),
		cummulative_neg_weight(0.001),
		pos_dist(new math::IndependentGaussianEstimater<T>(dim)),
		neg_dist(new math::IndependentGaussianEstimater<T>(dim))
	{}

	/*******************************************************
	 * constructor
	*******************************************************/
	IndependentNaiveBayes(const std::vector<int>& sub_indices)
		: cummulative_pos_weight(0.001),
		cummulative_neg_weight(0.001),
		sub_indices(sub_indices),
		pos_dist(new math::IndependentGaussianEstimater<T>(sub_indices.size())),
		neg_dist(new math::IndependentGaussianEstimater<T>(sub_indices.size()))
	{}

	/*******************************************************
	 * update
	*******************************************************/
	template<typename W, typename VectorT>
	void update(W label, const VectorT& x) {
		if (!sub_indices.empty()){
			add_impl(label, extractSubFeature(x));
		}
		else {
			add_impl(label, x);
		}
	}

	/*******************************************************
	 * update_all
	*******************************************************/
	template<typename W, typename VectorT>
	void update_all(const std::vector<W>& label, const std::vector<VectorT>& features){
		for (int i = 0; i<label.size(); i++) {
			update(label[i], features[i]);
		}
	}

  /*******************************************************
   * predict the response
  *******************************************************/
  template<typename VectorT>
  int predict(const VectorT& x) const {
    if (!sub_indices.empty()){
      return predict_impl(extractSubFeature(x));
    }
    else {
      return predict_impl(x);
    }
  }

  /*******************************************************
   * predict the response
  *******************************************************/
  template<typename VectorT>
  double predict_real(const VectorT& x) const {
    if (!sub_indices.empty()){
      return predict_real_impl(extractSubFeature(x));
    }
    else {
      return predict_real_impl(x);
    }
  }

public:
	/*******************************************************
   * add data
	*******************************************************/
	template<typename W, typename VectorT>
	void add_impl(W label, const VectorT& x) {
		if (label > 0) {
			cummulative_pos_weight += abs(label);
			pos_dist->add(label, x);
		}
		else {
			cummulative_neg_weight += abs(label);
			neg_dist->add(-label, x);
		}
	}

  template<typename VectorT>
  double predict_real_impl(const VectorT& x) const {
    double sum_pos_neg = cummulative_pos_weight + cummulative_neg_weight;
    double pos_priori = cummulative_pos_weight / sum_pos_neg;
    double neg_priori = cummulative_neg_weight / sum_pos_neg;
    double pos_posterior = pos_priori * (*pos_dist)(x);
    double neg_posterior = neg_priori * (*neg_dist)(x);
    return pos_posterior - neg_posterior;
  }

	/*******************************************************
	 * predict
   * ret : +1 or -1
	*******************************************************/
	template<typename VectorT>
	int predict_impl(const VectorT& x) const {
		double sum_pos_neg = cummulative_pos_weight + cummulative_neg_weight;
		double pos_priori = cummulative_pos_weight / sum_pos_neg;
		double neg_priori = cummulative_neg_weight / sum_pos_neg;
		double pos_posterior = pos_priori * (*pos_dist)(x);
		double neg_posterior = neg_priori * (*neg_dist)(x);
		return pos_posterior > neg_posterior ? 1 : -1;
	}

	/*******************************************************
   * extract a sub feature set
	*******************************************************/
	template<typename VectorT>
	Eigen::Matrix<T, Eigen::Dynamic, 1> extractSubFeature(const VectorT& feature) const {
		Eigen::Matrix<T, Eigen::Dynamic, 1> sub_feature(sub_indices.size());
		for (int i = 0; i < sub_indices.size(); i++) {
			sub_feature[i] = feature[sub_indices[i]];
		}
		return sub_feature;
	}

public:
	std::vector<int> sub_indices;

	double cummulative_pos_weight;
	double cummulative_neg_weight;
	std::unique_ptr<math::IndependentGaussianEstimater<T>> pos_dist;
	std::unique_ptr<math::IndependentGaussianEstimater<T>> neg_dist;
};

	}
}

#endif
