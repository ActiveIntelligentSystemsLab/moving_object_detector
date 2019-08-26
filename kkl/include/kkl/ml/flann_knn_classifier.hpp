/**
 * FlannKNNClassifier.hpp
 * @author koide
 * 15/11/16
 **/
#ifndef KKL_FLANN_KNN_CLASSIFIER_HPP
#define KKL_FLANN_KNN_CLASSIFIER_HPP

#include <deque>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <flann/flann.hpp>

namespace kkl {
	namespace ml {

/***************************************
 * FlannKNNClassifier
 * flann���g����knn���ފ�
***************************************/
template<typename T>
class FlannKNNClassifier {
public:
	typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorT;

	// constructor
	FlannKNNClassifier() {
		index_params.reset(new flann::LinearIndexParams());

		labels.reserve(256);

		min_label = 0;
		max_label = 0;
	}

	// �_�̒ǉ�
	// label : ���x��
	// point : �ǉ�����_
	void addPoint(int label, const VectorT& point){
		labels.push_back(label);
		points.push_back(point);

		if (!index) {
			index.reset(new ::flann::Index<::flann::L2<float>>(eigen2flann(points.back()), *index_params));
		} else {
      index->addPoints(eigen2flann(points.back()), 1);
		}

		min_label = std::min(min_label, label);
		max_label = std::max(max_label, label);
	}

	// ����
	// query : ����
	// k : k�ߖT�_��
	// return : ���胉�x��
	int predict(const VectorT& query, int k = 5) {
		if (!index) {
			std::cerr << "error : knn index is not constructed!!" << std::endl;
			return min_label;
		}

		std::vector<std::vector<int>> indices_;
		std::vector<std::vector<float>> dists_;
		index->knnSearch(eigen2flann(query), indices_, dists_, k, flann::SearchParams(32));

		const auto& indices = indices_.front();
		const auto& dists = dists_.front();

		int label_range = max_label - min_label + 1;
		if (label_range == 1) {
			return min_label;
		}

		std::vector<int> hist(label_range, 0);
		for (int i = 0; i < indices.size(); i++) {
			hist[labels[indices[i]] + min_label] ++;
		}

		int max_voted = min_label + std::distance(hist.begin(), std::max_element(hist.begin(), hist.end()));
		return max_voted;
	}

	// 2�l����
	// query : ����
	// k : k�ߖT�_��
	// min_dist : �ŋߖT�_�܂ł̋���
	// return : label > 1 or not
	bool predictBinary(const VectorT& query, int k = 5, float* min_dist = nullptr) {
		if (!index) {
			std::cerr << "error : knn index is not constructed!!" << std::endl;
			return min_label;
		}

		std::vector<std::vector<int>> indices_;
		std::vector<std::vector<float>> dists_;
		index->knnSearch(eigen2flann(query), indices_, dists_, k, flann::SearchParams(32));

		const auto& indices = indices_.front();
		const auto& dists = dists_.front();

		int pos_neg[2] = { 0, 0 };
		for (int i = 0; i < indices.size(); i++) {
			if (labels[indices[i]] > 0) {
				pos_neg[0]++;
			}
			else {
				pos_neg[1] ++;
			}
		}

		if (min_dist) {
			*min_dist = dists[0];
      std::cout << "min_dist " << *min_dist << std::endl;
		}

		return pos_neg[0] > pos_neg[1];;
	}

	// confidence�t��2�l����
	// query : ����
	// k : k�ߖT�_��
	// min_dist : �ŋߖT�_�܂ł̋���
	// return : label > 1 or not
	double predictBinaryReal(const VectorT& query, int k = 5, float* min_dist = nullptr) {
		if (!index) {
			std::cerr << "error : knn index is not constructed!!" << std::endl;
			return min_label;
		}

		std::vector<std::vector<int>> indices_;
		std::vector<std::vector<float>> dists_;
		index->knnSearch(eigen2flann(query), indices_, dists_, k, flann::SearchParams(32));

		const auto& indices = indices_.front();
		const auto& dists = dists_.front();

		int pos_neg[2] = { 0, 0 };
		for (int i = 0; i < indices.size(); i++) {
			if (labels[indices[i]] > 0) {
				pos_neg[0]++;
			}
			else {
				pos_neg[1] ++;
			}
		}

		if (min_dist) {
			*min_dist = dists[0];
		}

		double sign = pos_neg[0] > pos_neg[1] ? +1.0 : -1.0;
		int half = (k - 1) / 2;
		int range = k - half;
		double confidence = (std::max(pos_neg[0], pos_neg[1]) - half) / static_cast<double>(range);

		return sign * confidence;
	}

	// �C���f�b�N�X�Ɋ܂܂��f�[�^�_��
	size_t size() const { return labels.size(); }

private:
	// Eigen::Matrix �� flann::Matrix
	flann::Matrix<T> eigen2flann(const VectorT& v) {
    flann::Matrix<T> mat((T*)v.data(), 1, v.size());
		return mat;
	}

	int min_label;				// ���x���̍ŏ��l
	int max_label;				// ���x���̍ő�l
	std::vector<int> labels;	// ���x���W��
	std::deque<VectorT> points;	// �_�W��

	// flann
	std::unique_ptr<flann::IndexParams> index_params;
	std::unique_ptr<flann::Index<flann::L2<T>>> index;
};

	}
}

#endif
