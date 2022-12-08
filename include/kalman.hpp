#include <Eigen/Core>
#include <Eigen/Dense>

class KalmanFilter{
public:
    KalmanFilter(size_t _dim_x, size_t _dim_z):dim_x(_dim_x),dim_z(_dim_z){}
    KalmanFilter(size_t _dim_x, size_t _dim_z, size_t _dim_u):dim_x(_dim_x),dim_z(_dim_z),dim_u(_dim_u){}
    void init(Eigen::VectorXf xx, 
              Eigen::MatrixXf AA, 
              Eigen::MatrixXf HH, 
              Eigen::MatrixXf QQ, 
              Eigen::MatrixXf RR, 
              Eigen::MatrixXf PP){
        x = xx;
        A = AA;
        H = HH;
        Q = QQ;
        R = RR;
        P = PP;
    }
    void init(Eigen::VectorXf xx, 
              Eigen::MatrixXf AA, 
              Eigen::MatrixXf BB, 
              Eigen::MatrixXf HH, 
              Eigen::MatrixXf QQ, 
              Eigen::MatrixXf RR, 
              Eigen::MatrixXf PP){
        x = xx;
        A = AA;
        B = BB;
        H = HH;
        Q = QQ;
        R = RR;
        P = PP;
    }
    void predict();
    void predict(Eigen::VectorXf u);
    void update(Eigen::VectorXf z);
    Eigen::VectorXf estimate_state(){return x;}
private:
    size_t dim_x;
    size_t dim_z;
    size_t dim_u;
    // x: state
    Eigen::VectorXf x;
    // A: state transition matrix
    Eigen::MatrixXf A;
    // B: control transition matrix
    Eigen::MatrixXf B;
    // H: measurement matrix
    Eigen::MatrixXf H;
    // Q: process noise covariance
    Eigen::MatrixXf Q;
    // R: measurement noise covariance
    Eigen::MatrixXf R;
    // P: posteriori estimate error covariance
    Eigen::MatrixXf P;
};