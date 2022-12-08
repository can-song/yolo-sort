#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>
using namespace std;

int main(int argv, char**argc){
    Eigen::MatrixXf x = Eigen::MatrixXf::Random(3, 0);
    cout << x.size() << endl;
    cout << "rows: " << x.rows() << ", colums: " << x.cols() << endl;
    return 0;
}