#ifndef KALMAN_TRACKER_HPP
#define KALMAN_TRACKER_HPP

// Original author: Kenji Koide
// Modified by Fujimoto

#include <Eigen/Dense>
#include <boost/any.hpp>

#include <ros/ros.h>

#include <kkl/math/gaussian.hpp>
#include <kkl/alg/kalman_filter.hpp>


/**
 * @brief Kalman filter-based tracker with a constant velocity model
 */
class KalmanTracker {
  typedef kkl::alg::KalmanFilter<double, 4, 2, 4> KalmanFilter;
public:
  /**
   * @brief constructor
   * @param id            tracker ID
   * @param time          timestamp
   * @param init_pos      initial position
   * @param associated    associated detection
   */
  KalmanTracker(long id, const ros::Time& time, const Eigen::Vector2d& init_pos, const Eigen::Vector2d& init_vel, boost::any associated = boost::any())
    : id_(id),
      init_time(time),
      last_prediction_time(time),
      last_correction_time(time),
      last_associated(associated)
  {
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 2> control = Eigen::Matrix<double, 4, 2>::Zero();
    Eigen::Matrix<double, 4, 4> measurement = Eigen::Matrix<double, 4, 4>::Identity();

    // これらのノイズは小出さんの値のまま
    // 後で変える
    Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
    process_noise.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity() * 0.003;
    process_noise.bottomRightCorner(2, 2) = Eigen::Matrix2d::Identity() * 0.01;
    Eigen::Matrix4d measurement_noise = Eigen::Matrix4d::Identity() * 0.2;

    Eigen::Vector4d mean = Eigen::Vector4d::Zero();
    mean.head<2>() = init_pos;
    mean.tail<2>() = init_vel;
    // これも後で変えるべき
    Eigen::Matrix4d cov = Eigen::Matrix4d::Identity() * 0.1;

    kalman_filter.reset(new KalmanFilter(transition, control, measurement, process_noise, measurement_noise, mean, cov));
  }
  ~KalmanTracker() {}

  using Ptr = std::shared_ptr<KalmanTracker>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  /**
   * @brief predict the current state
   * @param time    current time
   */
  void predict(const ros::Time& time) {
    double difftime = (time - last_prediction_time).toSec();
    difftime = std::max(0.001, difftime);

    kalman_filter->transitionMatrix(0, 2) = difftime;
    kalman_filter->transitionMatrix(1, 3) = difftime;

    kalman_filter->predict(Eigen::Matrix<double, 2, 1>::Zero());
    last_prediction_time = time;

    last_associated = boost::any();
  }

  /**
   * @brief correct the state with an observation
   * @param time    current time
   * @param pos     observed position
   * @param vel     observed velocity
   * @param associated   associated detection
   */
  void correct(const ros::Time& time, const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, boost::any associated = boost::any()) {
    Eigen::Vector4d observation = Eigen::Vector4d::Zero();
    observation.head<2>() = pos;
    observation.tail<2>() = vel;
    kalman_filter->correct(observation);

    last_correction_time = time;
    last_associated = associated;
  }

public:
  long id() const {
    return id_;
  }

  ros::Duration age(const ros::Time& time) const {
    return (time - init_time);
  }

  const ros::Time& lastCorrectionTime() const {
    return last_correction_time;
  }

  const boost::any& lastAssociated() const {
    return last_associated;
  }

  Eigen::Vector2d position() const {
    return kalman_filter->mean.head<2>();
  }

  Eigen::Vector2d velocity() const {
    return kalman_filter->mean.tail<2>();
  }

  Eigen::Vector4d mean() const {
    return kalman_filter->mean;
  }

  Eigen::Matrix2d positionCov() const {
    return kalman_filter->cov.block<2, 2>(0, 0);
  }

  Eigen::Matrix2d velocityCov() const {
    return kalman_filter->cov.block<2, 2>(2, 2);
  }

  Eigen::Matrix4d cov() const {
    return kalman_filter->cov;
  }

  void setMeasurementNoiseCov(const Eigen::Matrix4d& cov) {
    kalman_filter->setMeasurementNoiseCov(cov);
  }

  double squaredMahalanobisDistance(const Eigen::Vector4d& p) const {
    return kkl::math::squaredMahalanobisDistance<double, 4>(
          kalman_filter->mean,
          kalman_filter->cov,
          p);
  }

private:
  long id_;

  ros::Time init_time;              // time when the tracker was initialized
  ros::Time last_prediction_time;   // tiem when prediction was performed
  ros::Time last_correction_time;   // time when correction was performed

  boost::any last_associated;       // associated detection data

  std::unique_ptr<KalmanFilter> kalman_filter;
};

#endif // KALMANTRACKER_HPP
