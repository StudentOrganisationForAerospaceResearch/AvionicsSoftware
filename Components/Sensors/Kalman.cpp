/*
 * File name: Kalman.cpp
 *
 * Contains: Functions for kalman filter
 *
 * Authors: Findlay Brown and Aiden Ballard
 * Language: C++ (english)
 */

#include "Kalman.h"
using namespace std;

Kalman::Kalman(int dim_x, int dim_z, bool lowPass)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
    this->lowPass = lowPass;
}

Kalman::Kalman(int dim_x, int dim_z)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
    lowPass = false;
}

Kalman::Kalman() {}

void Kalman::init(MatrixXf &F_in, MatrixXf &H_in, MatrixXf &Qa_in, VectorXf &Z_in, MatrixXf &R_in, VectorXf &X_init, MatrixXf &P_init)
{
    // set fixed matrices
    F = &F_in;
    H = &H_in;
    Q = *F * Qa_in * F->transpose();
    I = I.Identity(dim_x, dim_x);
    K.Zero(dim_x, dim_z);

    // set state, measurement matrices
    Z = &Z_in;
    R = &R_in;
    X.resize(dim_x);
    X = X_init;
    Xpred = *F * X; // Predict next state
    P.resize(dim_x, dim_x);
    P = P_init;
    Ppred = (*F * P * F->transpose()) + Q; // Predict next covariance
}

void Kalman::init(int dim_xin, int dim_zin, MatrixXf &F_in, MatrixXf &H_in, MatrixXf &Qa_in, VectorXf &Z_in, MatrixXf &R_in, VectorXf &X_init, MatrixXf &P_init)
{
    // set dimensions
    this->dim_x = dim_xin;
    this->dim_z = dim_zin;

    // set fixed matrices
    F = &F_in;
    H = &H_in;
    Q = *F * Qa_in * F->transpose();
    I = I.Identity(dim_x, dim_x);
    K.Zero(dim_x, dim_z);

    // set state, measurement matrices
    Z = &Z_in;
    R = &R_in;
    X.resize(dim_x);
    X = X_init;
    Xpred = *F * X; // Predict next state
    P.resize(dim_x, dim_x);
    P = P_init;
    Ppred = (*F * P * F->transpose()) + Q; // Predict next covariance
}

void Kalman::update()
{
    K = (Ppred * H->transpose()) * ((*H * Ppred * H->transpose() + *R).inverse());      // kalman gain update equation
    X = Xpred + K * (*Z - (*H * Xpred));                                                // state update equation
    P = ((I - K * *H) * Ppred) * ((I - K * *H).transpose()) + (K * *R * K.transpose()); // Covariance update equation
    Xpred = *F * X;                                                                     // Predict next state
    Ppred = (*F * P * F->transpose()) + Q;                                              // Predict next covariance
}

// bool Kalman::lowPassInit(MatrixXd betaX, int dim_n)
// {
    
// }