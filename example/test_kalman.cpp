#include <random>
#include <vector>
#include <iostream>
#include <cmath>
#include "kalman.hpp"
#include "matplotlibcpp.h"
using namespace std;
namespace plt = matplotlibcpp;

int main(int argv, char **argc){
    default_random_engine e;
    normal_distribution<float> n(0, 1);

    size_t N = 200;
    float q = 0.01;
    float r = 0.64;
    // Eigen::VectorXf t(N);
    vector<Eigen::VectorXf> xx(N);
    vector<Eigen::VectorXf> yy(N);
    vector<Eigen::VectorXf> zz(N);
    vector<Eigen::VectorXf> uu(N);
    for(size_t i=0; i<N; ++i){
        float t = sin(i*0.1);
        yy[i] = Eigen::VectorXf::Ones(1) * t;
        xx[i] = Eigen::VectorXf::Ones(1) * (t + n(e) * q);
        zz[i] = Eigen::VectorXf::Ones(1) * (t + n(e) * r);
        uu[i] = Eigen::VectorXf::Ones(1) * (cos(i*0.1)*0.1);
    }

    KalmanFilter kalman(1, 1);
    Eigen::VectorXf x = Eigen::VectorXf::Ones(1)*100;
    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(1, 1);
    Eigen::MatrixXf B = Eigen::MatrixXf::Identity(1, 1);
    Eigen::MatrixXf H = Eigen::MatrixXf::Identity(1, 1);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(1, 1) * q;
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(1, 1) * r;
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(1, 1) * 10;
    kalman.init(x, A, B, H, Q, R, P);

    vector<int> t;
    vector<float> u, v, w, y;
    for(size_t i=0; i<N; ++i){
        kalman.predict(uu[i]);
        kalman.update(zz[i]);
        Eigen::VectorXf x = kalman.estimate_state();
        t.push_back(i);
        u.push_back(x[0]);
        v.push_back(xx[i][0]);
        w.push_back(zz[i][0]);
        y.push_back(yy[i][0]);
    }
    // plt::xkcd();
    plt::named_plot("posteri", t, u, "b--");
    plt::named_plot("prior", t, v, "r--");
    plt::named_plot("measure", t, w, "y--");
    plt::named_plot("real", t, y, "g");
    plt::legend();
    // plt::title("posteri estimate state");
    plt::save("kalman.png");
    plt::show();
    plt::close();
    return 0;
}