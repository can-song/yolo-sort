#include "kalman.hpp"

void KalmanFilter::predict(){
    x = A * x;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::predict(Eigen::VectorXf u){
    x = A * x + B * u;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(Eigen::VectorXf z){
    Eigen::MatrixXf PHT = P * H.transpose();
    Eigen::MatrixXf K = PHT * (H*PHT+R).inverse();
    x = x + K * (z - H * x);
    P = P - K * H * P;
}