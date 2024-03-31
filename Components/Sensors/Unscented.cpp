/*
Implementations for unscented kalman filter
See Unscented.h for function help
*/

#include "Unscented.h"
#include <iostream>

KalmanU::KalmanU(int dim_x, int dim_z)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
    k = 3 - dim_x;
}

void KalmanU::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in)
{
    R = &R_in;
    Z = &Z_in;

    linearDynamics = false;

    // Calculate weights
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void KalmanU::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in)
{
    H = &H_in;
    R = &R_in;
    Z = &Z_in;

    linearDynamics = false;

    // Calculate weights
    k = 3 - dim_x;
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void KalmanU::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &F_in, MatrixXf &Qa_in)
{
    F = &F_in;
    R = &R_in;
    Z = &Z_in;

    linearDynamics = true;

    Q = *F * Qa_in * F->transpose();

    // Calculate weights
    k = 3 - dim_x;
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void KalmanU::init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in, MatrixXf &F_in, MatrixXf &Qa_in)
{
    H = &H_in;
    F = &F_in;
    R = &R_in;
    Z = &Z_in;

    linearDynamics = true;

    Q = *F * Qa_in * F->transpose();

    // Calculate weights
    k = 3 - dim_x;
    w0 = k / (dim_x + k);
    w1 = 1 / (2 * (dim_x + k));
    W.setZero((2 * dim_x) + 1);
    W1.setConstant(2 * dim_x, w1);
    W << w0, W1;

    // initalize and predict next state
    X = X0;
    P = P0;
    uTransform();
}

void KalmanU::update()
{
    sMeasurements.setZero(dim_z, (2 * dim_x) + 1);
    for (int i = 0; i < (2 * dim_x) + 1; i++)
    {
        sMeasurements.col(i) = observe(sPoints.col(i));
    }
    Zpred = sMeasurements * W;
    
    measurementProjError.setZero(dim_z, (2 * dim_x) + 1);
    for (int i = 0; i < dim_z; i++)
    {
        measurementProjError.row(i) = (sMeasurements.row(i).array() - Zpred.row(i).value()).matrix();
    }

    WeightedZvariance = (measurementProjError * W.asDiagonal() * measurementProjError.transpose()) + *R; // Compute measurement variance matrix
    XZCovariance = projError * W.asDiagonal() * measurementProjError.transpose();                        // Compute state-measurement cross covariance matrix
    K = XZCovariance * WeightedZvariance.inverse();                                                      // Compute Kalman gain
    X = Xpred + K * (*Z - Zpred);                                                                        // update state prediction
    P = Ppred - (K * WeightedZvariance * K.transpose());                                                 // update Covariance
    uTransform();                                                                                        // make next predictions
}

MatrixXf KalmanU::observe(MatrixXf sigmaX)
{
    // return *H * sigmaX; //linear observation equation default.
    VectorXf Zout(2);
    Zout << sqrt(pow(sigmaX(0), 2) + pow(sigmaX(3), 2)),
            atan(sigmaX(3)/sigmaX(0));
    return Zout;
}

MatrixXf KalmanU::predict(MatrixXf sigmaX)
{
    return *F * sigmaX; // linear system dynamics default.
}

void KalmanU::uTransform()
{
    // Generate sigma points
    sPoints.setZero(dim_x, (2 * dim_x) + 1);
    sPoints.col(0) = X;
    sigma = ((dim_x + k) * P).llt().matrixL();
    for (int i = 1; i < dim_x + 1; i++)
    {
        sPoints.col(i) = X + sigma.col(i - 1);
    }
    for (int i = dim_x + 1; i < (2 * dim_x) + 1; i++)
    {
        sPoints.col(i) = X - sigma.col(i - dim_x - 1);
    }

    // Propigate points
    for (int i = 0; i < (2 * dim_x) + 1; i++)
    {
        sPoints.col(i) = predict(sPoints.col(i));
    }

    // Calculate weighted mean for points
    Xpred = sPoints * W;
    projError.setZero(dim_x, (2 * dim_x) + 1);
    for (int i = 0; i < dim_x; i++)
    {
        projError.row(i) = (sPoints.row(i).array() - Xpred.row(i).value()).matrix();
    }
    if (linearDynamics == true)
    {
        Ppred = projError * W.asDiagonal() * projError.transpose() + Q;
    }
    else
    {
        Ppred = projError * W.asDiagonal() * projError.transpose();
    }
}