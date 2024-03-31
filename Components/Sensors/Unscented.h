/*
Class for unscented Kalman filters with no control input support.
See notes on functions predict() and observe() which usually must be overwritten for implementation
See Kalman.h for additional (IMPORTANT) information.
Note that dim_x and dim_z MUST be defined with the constructor
*/

#ifndef KalmanU_h
#define KalmanU_h

#include "Kalman.h"
//#include "eigen-3.4.0\\eigen-3.4.0\\Eigen\\Cholesky"

class KalmanU : public Kalman
{
public:
    /*
     * Constructor
     * dim_x: number of state variables (size of state vector X)
     * dim_z: number of measurement inputs (size of measurement vector Z)
     */
    KalmanU(int dim_x, int dim_z);


    /* Initalize the Kalman filter
    
    F_in -> State transition matrix (dim_x X dim_x) (For linear system dynamics only)
    H_in -> Observation matrix (dim_z X dim_x) (For linear observation equations only)
    Qa_in -> Process noise covariance initializer, see below (dim_x X dim_x) (For linear system dynamics only)
    Z_in -> Measurement vector. Update passed vector with new values each time step. (dim_z)
    R_in -> Measurement Covariance matrix. Update passed matrix with new values each time step. (dim_z X dim_z)
    X0 -> inital state vector. (dim_x)
    P0 -> inital estimate covariance (dim_x X dim_x)

    Note: Qa should be initialized as s^2[M], where M has a 1 on the main diagonal corresponding to each
    * state variable associated with s^2. For instance, state vector [x,y] where only y is affected by variance has Qa =
    * [0,  0]
    * [0,s^2]
    */
    virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in);

    virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in);

    virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &F_in, MatrixXf &Qa_in);

    virtual void init(VectorXf &X0, MatrixXf &P0, VectorXf &Z_in, MatrixXf &R_in, MatrixXf &H_in, MatrixXf &F_in, MatrixXf &Qa_in);

    /*Update the filter with new measurements
    Note that this reads the variables Z and R passed to init() for new data.*/
    virtual void update();

    /*Set K for filter tuning.
    K is by default set to 3-N, the value generally best for gaussian-distributed data 
    Run this function BEFORE init() has been called. */
    void setK(float k) { this->k = k; };

protected:

    /*Takes state sigma points and transforms them to the measurement space (Ex: returning a Measurement sigma point matrix). Must be overwritten with 
    non-linear observation equations. Uses H matrix passed to init() function by default (useful if observation equations are linear).*/
    MatrixXf observe(MatrixXf sigmaX);

    /*Takes state vector X and calculates state based off of previous state, to be stored in Xpred.  Must be overwritten with
    non-linear system dynamics. Uses F matrix passed to init() by default (useful if system dynamics are linear) 
    X will be a dim_x X 1 matrix, output must also be a dim_x X 1 matrix.*/
    MatrixXf predict(MatrixXf sigmaX);

    /*Function to perform the unscented transform*/
    void uTransform();

    float k; // Tuning parameter for linearization.
    MatrixXf sPoints; //to be assigned an array of sigma points
    MatrixXf sigma; //Decomposed matrix used for finding sigma points
    VectorXf W; //weight matrix / vector for sigma points
    VectorXf W1; //Matrix for filling W
    float w0, w1; //weights for W matrix
    MatrixXf projError; //Matrix for calculating projected covariance
    MatrixXf sMeasurements; //Matrix of measurement predictions from Unscented transform
    VectorXf Zpred; // expected measurement vector - needs to be explicitly defined for UKF
    MatrixXf WeightedZvariance; //weighted measurement variance matrix
    MatrixXf XZCovariance; // state-measuremnt cross covariance matrix
    MatrixXf measurementProjError; //Matrix for calculating weighted measurement variance matrix
    bool linearDynamics;
};
#endif
