#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>



using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


const double MINVAL = 0.001;

inline double PrevZero(double x) {
  return (x > MINVAL)? x : MINVAL;
}

inline double NormDeltaRad(double dx){
  while(dx >    M_PI) dx -= 2* M_PI;
  while(dx < -1*M_PI) dx += 2* M_PI;
  return dx;
}


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(double std_a, double std_yawdd) {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = std_a;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = std_yawdd;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  n_z_radar = 3;
  n_z_lidar = 2;
  lambda_ = 3 - n_aug_;
  
  InitWeight(&weights_);
  InitQR(&Q_, &R_lidar_, &R_radar_);
  Xsig_pred_ = MatrixXd(n_x_, n_aug_*2+1);
  
}


UKF::~UKF() {}

void UKF::InitWeight(VectorXd* w_out){
  VectorXd weights = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }
  *w_out = weights;
}


void UKF::InitQR(MatrixXd* Q_out, MatrixXd* R_lidar_out, MatrixXd* R_radar_out){
  MatrixXd Q  = MatrixXd(n_aug_-n_x_, n_aug_-n_x_);;
  MatrixXd Rl = MatrixXd(n_z_lidar,n_z_lidar);
  MatrixXd Rr = MatrixXd(n_z_radar,n_z_radar);
  
  Q << std_a_*std_a_, 0.000,
       0.000,         std_yawdd_*std_yawdd_;

  Rl << std_laspx_*std_laspx_, 0.0,
        0.0,                   std_laspy_*std_laspy_;
  
  Rr << std_radr_*std_radr_, 0.0,                     0.0,
        0.0,                 std_radphi_*std_radphi_, 0.0,
        0.0,                 0.0,                     std_radrd_*std_radrd_;
  
  *Q_out = Q;
  *R_lidar_out = Rl;
  *R_radar_out = Rr;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (not is_initialized_){
    
    P_ << 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0;
    
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      double radr   = meas_package.raw_measurements_(0);
      double radphi = meas_package.raw_measurements_(1);
      double radrd  = meas_package.raw_measurements_(2);
      
      double px = radr * cos(radphi);
      double py = radr * sin(radphi);
      double vx = radrd * cos(radphi);
      double vy = radrd * sin(radphi);
      double v = sqrt(vx*vx+vy*vy);
      
      x_ << px, py, v, 0.0, 0.0;
    }
    if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);
      
      x_ << px, py, 0.0, 0.0, 0.0;
    }
    x_(0) = PrevZero(x_(0));
    x_(1) = PrevZero(x_(1));
    
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
  }
  
  
  double dt = meas_package.timestamp_ - time_us_;
  dt = dt / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  Prediction(dt);
  
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
    if (not use_radar_){
      return;
    }
    UpdateRadar(meas_package);
  }
  
  if(meas_package.sensor_type_ == MeasurementPackage::LASER){
    if (not use_laser_){
      return;
    }
    UpdateLidar(meas_package);
  }
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  /* Step1+2, Augment Sigma Point and Pred */
  SigmaPointPrediction(&Xsig_pred_, delta_t);
  
  /* Step3, Prediction Mean And Cov      */
  PredictMeanAndCovariance(&x_, &P_, delta_t);
  
}

void UKF::AugmentedSigmaPoints(MatrixXd * X_out){
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(n_aug_-n_x_, n_aug_-n_x_) = Q_;
  
  MatrixXd A = P_aug.llt().matrixL();
  
  Xsig_aug.col(0) = x_aug;
  for(int i=0; i<n_aug_; ++i){
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_)*A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*A.col(i);
  }
  *X_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd *X_out,double delta_t){
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  AugmentedSigmaPoints(&Xsig_aug);
  
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2*n_aug_+1);
  for(int i=0; i<2*n_aug_+1; ++i){
    
    double px    = Xsig_aug(0, i);
    double py    = Xsig_aug(1, i);
    double v     = Xsig_aug(2, i);
    double psi   = Xsig_aug(3, i);
    double psi_d = Xsig_aug(4, i);
    double nu    = Xsig_aug(5, i);
    double nu_d  = Xsig_aug(6, i);
    
    /* Update */
    if( fabs(psi_d) > 0.001){
      Xsig_pred(0, i) = px + (   sin(psi+psi_d*delta_t)-sin(psi))*v/psi_d;
      Xsig_pred(1, i) = py + (-1*cos(psi+psi_d*delta_t)+cos(psi))*v/psi_d;
    }
    else{
      Xsig_pred(0, i) = px + v*cos(psi)*delta_t;
      Xsig_pred(1, i) = py + v*sin(psi)*delta_t;
    }
    Xsig_pred(2, i) = v;
    Xsig_pred(3, i) = psi + psi_d*delta_t;
    Xsig_pred(4, i) = psi_d;
    
    /* Noises */
    Xsig_pred(0, i) += 0.5*delta_t*delta_t*nu*cos(psi) ;
    Xsig_pred(1, i) += 0.5*delta_t*delta_t*nu*sin(psi);
    Xsig_pred(2, i) +=             delta_t*nu;
    Xsig_pred(3, i) += 0.5*delta_t*delta_t*nu_d;
    Xsig_pred(4, i) +=             delta_t*nu_d;
  }
  *X_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(VectorXd* X_out, MatrixXd* P_out, double delta_t){
  VectorXd x_avg = VectorXd(n_x_);
  MatrixXd p_out = MatrixXd(n_x_, n_x_);
  
  x_avg.fill(0.0);
  p_out.fill(0.0);
  
  for(int i=0; i<2*n_aug_+1; ++i){
    x_avg += weights_(i) * Xsig_pred_.col(i);
  }
  
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd deltaX = Xsig_pred_.col(i) - x_avg;
    deltaX(3) = NormDeltaRad(deltaX(3));
    p_out += weights_(i) * deltaX * deltaX.transpose();
  }
  *X_out = x_avg;
  *P_out = p_out;
}



void UKF::PredictRadarMeasurement(VectorXd *Z_out, MatrixXd *Zsig_out, MatrixXd *S_out){
//  MatrixXd Zsig = MatrixXd(n_z_radar, 2*n_aug_+1);
//  VectorXd z_pred    = VectorXd(n_z_radar);
//  VectorXd S         = MatrixXd(n_z_radar, n_z_radar);
  
  MatrixXd Zsig     = MatrixXd(n_z_radar, 2*n_aug_+1);
  VectorXd z_pred   = VectorXd(n_z_radar);
  MatrixXd S         = MatrixXd(n_z_radar, n_z_radar);
  
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for(int i=0; i<2*n_aug_+1; ++i){
    double px    = Xsig_pred_(0, i);
    double py    = Xsig_pred_(1, i);
    double v     = Xsig_pred_(2, i);
    double psi   = Xsig_pred_(3, i);
    
    double rho = PrevZero(sqrt(px*px + py*py));
    
    
    Zsig(0, i) = rho;
    Zsig(1, i) = atan(py / PrevZero(px));
    Zsig(2, i) = ((px*cos(psi)*v)+(py*sin(psi)*v)) / rho;
    
    z_pred += weights_(i)*Zsig.col(i);
  }
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd deltaZ = Zsig.col(i) - z_pred;
    deltaZ(1) = NormDeltaRad(deltaZ(1));
    S += weights_(i) * deltaZ * deltaZ.transpose();
  }
  S += R_radar_;
  
  *Z_out = z_pred;
  *Zsig_out = Zsig;
  *S_out = S;
}

void UKF::PredictLidarMeasurement(VectorXd *Z_out, MatrixXd *Zsig_out, MatrixXd *S_out){
  MatrixXd Zsig = MatrixXd(n_z_lidar, 2*n_aug_+1);
  VectorXd z_pred    = VectorXd(n_z_lidar);
  MatrixXd S         = MatrixXd(n_z_lidar, n_z_lidar);
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for(int i=0; i<2*n_aug_+1; ++i){
    double px    = Xsig_pred_(0, i);
    double py    = Xsig_pred_(1, i);
    Zsig(0, i) = px;
    Zsig(1, i) = py;
    z_pred += weights_(i)*Zsig.col(i);
  }
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd deltaZ = Zsig.col(i) - z_pred;
    S += weights_(i) * deltaZ * deltaZ.transpose();
  }
  S += R_lidar_;
  
  *Z_out = z_pred;
  *Zsig_out = Zsig;
  *S_out = S;
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  MatrixXd Zsig = MatrixXd(n_z_lidar, 2*n_aug_+1);
  VectorXd z_pred    = VectorXd(n_z_lidar);
  MatrixXd S         = MatrixXd(n_z_lidar, n_z_lidar);
  PredictLidarMeasurement(&z_pred, &Zsig, &S);
  VectorXd z = VectorXd(n_z_lidar);
  z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1);
  MatrixXd Tc = MatrixXd(n_x_, z_pred.size());
  Tc.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd dx = Xsig_pred_.col(i) - x_;
    VectorXd dz = Zsig.col(i) - z_pred;
    Tc += weights_(i) * dx * dz.transpose();
  }
  
  UnscentedKalmanFilter(z_pred, z, Tc, S);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  MatrixXd Zsig = MatrixXd(n_z_radar, 2*n_aug_+1);
  VectorXd z_pred    = VectorXd(n_z_radar);
  MatrixXd S         = MatrixXd(n_z_radar, n_z_radar);
  PredictRadarMeasurement(&z_pred, &Zsig, &S);
  VectorXd z = VectorXd(n_z_radar);
  z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);
  MatrixXd Tc = MatrixXd(n_x_, z_pred.size());
  Tc.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd dx = Xsig_pred_.col(i) - x_;
    VectorXd dz = Zsig.col(i) - z_pred;
    dx(3) = NormDeltaRad(dx(3));
    dz(1) = NormDeltaRad(dz(1));
    Tc += weights_(i) * dx * dz.transpose();
  }
  UnscentedKalmanFilter(z_pred, z, Tc, S);
}

void UKF::UnscentedKalmanFilter(VectorXd &z_pred, VectorXd &z, MatrixXd &T, MatrixXd &S){
  MatrixXd K = T * S.inverse();
  VectorXd dz = z - z_pred;
  if(z.size()==3){
    dz(1) = NormDeltaRad(dz(1));
  }
  x_ += K*dz;
  P_ -= K*S*K.transpose();
}
