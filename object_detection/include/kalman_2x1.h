#ifndef KALMAN_FILTER_2x1_H
#define KALMAN_FILTER_2x1_H

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
public:
    KalmanFilter(double dt) : dt(dt) {
        // 상태 전이 행렬 초기화
        A << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

        // 관측 행렬 초기화
        H << 1, 0, 0, 0,
             0, 1, 0, 0;
           
        // 초기 상태 벡터 (위치와 속도)
        x.setZero();

        // 초기 공분산 행렬
        P.setIdentity();

        // 프로세스 잡음 공분산
        Q.setIdentity();
        Q *= 10;

        // 측정 잡음 공분산
        R.setIdentity();
        R *= 0.1;
    }

    void predict() {
        // 상태 예측
        x = A * x; // x_esti -> x_pred
        P = A * P * A.transpose() + Q; // P -> P_pred
    }

    void update(Matrix<double, 2, 1> z) {
        // 잔차 계산
        Matrix<double, 2, 1> y = z - H * x; // 2x1

        // 잔차 공분산 계산
        Matrix<double, 2, 2> S = H * P * H.transpose() + R; // 2x2

        // 칼만 이득 계산
        Matrix<double, 4, 2> K = P * H.transpose() * S.inverse(); // 4x2

        // 상태 업데이트
        x = x + K * y; // x_pred -> x_esti 

        // 공분산 행렬 업데이트
        P = P - K * H * P; // P_pred -> P
    }

    MatrixXd getState() const {
        return x;
    }

private:
    Matrix<double, 4, 4> A; // 상태 전이 행렬
    Matrix<double, 2, 4> H; // 관측 행렬
    Matrix<double, 4, 1> x; // 상태 벡터 (x, y, vx, xy)
    Matrix<double, 4, 4> P; // 공분산 행렬
    Matrix<double, 4, 4> Q; // 프로세스 잡음 공분산
    Matrix<double, 2, 2> R; // 측정 잡음 공분산
    double dt;             // 시간 간격
};

#endif