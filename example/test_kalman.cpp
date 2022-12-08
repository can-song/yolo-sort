#include <random>
#include <vector>
#include <iostream>
#include "kalman.hpp"
using namespace std;

int main(int argv, char **argc){
    default_random_engine e;
    normal_distribution<float> n(0, 1);

    size_t N = 200;
    float q = 5;
    float r = 5;
    // Eigen::VectorXf t(N);
    vector<Eigen::VectorXf> xx(N);
    vector<Eigen::VectorXf> zz(N);
    for(size_t i=0; i<N; ++i){
        xx[i] = Eigen::VectorXf::Ones(1) * (i*0 + n(e) * q);
        zz[i] = Eigen::VectorXf::Ones(1) * (i*0 + n(e) * r);
    }

    KalmanFilter kalman(1, 1);
    Eigen::VectorXf x = Eigen::VectorXf::Zero(1);
    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(1, 1);
    Eigen::MatrixXf H = Eigen::MatrixXf::Identity(1, 1);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(1, 1) * q;
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(1, 1) * r;
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(1, 1) * 10;
    kalman.init(x, A, H, Q, R, P);

    for(size_t i=0; i<N; ++i){
        kalman.predict();
        kalman.update(zz[i]);
        Eigen::VectorXf x = kalman.estimate_state();
        cout << x[0] << ":" << xx[i][0] << ":" << zz[i][0] << ", ";
    }

    return 0;
}