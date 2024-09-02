#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
public:
    KalmanFilter(){}

    void init(double _dt, double _p1, double _p2, double _q1, double _q2, double _r){
        dt = _dt;
        p1 = _p1;
        p2 = _p2;
        q1 = _q1;
        q2 = _q2;
        r = _r;

        // 상태 전이 행렬 초기화
        A << 1, dt,
             0, 1;

        // 관측 행렬 초기화
        H << 1, 0;
           
        // 초기 상태 벡터 (위치와 속도)
        x.setZero();

        // 초기 공분산 행렬
        P << p1, 0,
             0, p2;

        // 프로세스 잡음 공분산
        Q << q1, 0,
             0, q2;

        // 측정 잡음 공분산
        R << r;
    }

    void predict() {
        // 상태 예측
        x = A * x; // x_esti -> x_pred
        P = A * P * A.transpose() + Q; // P -> P_pred
    }

    void update(Matrix<double, 1, 1> z) {
        // 잔차 계산
        Matrix<double, 1, 1> y = z - H * x; // 1x1

        // 잔차 공분산 계산
        Matrix<double, 1, 1> S = H * P * H.transpose() + R; // 1x1

        // 칼만 이득 계산
        Matrix<double, 2, 1> K = P * H.transpose() * S.inverse(); // 2x1

        // 상태 업데이트
        x = x + K * y; // x_pred -> x_esti 

        // 공분산 행렬 업데이트
        P = P - K * H * P; // P_pred -> P
    }

    MatrixXd getState() const {
        return x;
    }

private:
    Matrix<double, 2, 2> A; // 상태 전이 행렬
    Matrix<double, 1, 2> H; // 관측 행렬
    Matrix<double, 2, 1> x; // 상태 벡터 (pos, vel)
    Matrix<double, 2, 2> P; // 공분산 행렬
    Matrix<double, 2, 2> Q; // 프로세스 잡음 공분산
    Matrix<double, 1, 1> R; // 측정 잡음 공분산
    double dt;              // 시간 간격
    double p1; 
    double p2; 
    double q1; 
    double q2; 
    double r; 
};

#endif