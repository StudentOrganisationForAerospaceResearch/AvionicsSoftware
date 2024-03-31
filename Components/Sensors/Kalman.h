/*
 * File name: Kalman.h
 *
 * Contains: Function prototypes
 *
 * Authors: Findlay Brown and Aiden Ballard
 * Language: C++
 * 
 * Class for multivariate Kalman filters with no control input support. 
 * Dimensions for filter MUST be set either with contructor or init().
 * init() must be called before any calls to update().
 * See comments for init() function for important details.
 */

// include path for eigen
// #include <eigen-path>
//#include "eigen-3.4.0\\eigen-3.4.0\\Eigen\\Core"
//#include "eigen-3.4.0\\eigen-3.4.0\\Eigen\\Geometry"
#ifndef Kalman_h
#define Kalman_h

using namespace Eigen;

/* dim_x: number of state variables (size of state vector X)
 * dim_z: number of measurement inputs (size of measurement vector Z)

 * Fixed Matrix Dimensions must be:
 * F - dim_x x dim_x
 * H - dim_z x dim_x
 * Q - dim_x x dim_x (also goes for Qa)
 * R - dim_z x dim_z
 */

class Kalman
{
public:
    /*
     * Constructor
     * dim_x: number of state variables (size of state vector X)
     * dim_z: number of measurement inputs (size of measurement vector Z)
     */
    Kalman(int dim_x, int dim_z, bool lowPass);
    Kalman(int dim_x, int dim_z);
    Kalman();

    /* Initalize the Kalman filter

    F_in -> State transition matrix (dim_x X dim_x)
    H_in -> Observation matrix (dim_z X dim_x)
    Qa_in -> Process noise covariance initializer, see below (dim_x X dim_x)
    Z_in -> Measurement vector. Update passed vector with new values each time step. (dim_z)
    R_in -> Measurement Covariance matrix. Update passed matrix with new values each time step. (dim_z X dim_z)
    X_init -> inital state vector. (dim_x)
    P_init -> inital estimate covariance (dim_x X dim_x)

    Note: Qa should be initialized as s^2[M], where M has a 1 on the main diagonal corresponding to each
    * state variable associated with s^2. For instance, state vector [x,y] where only y is affected by variance has Qa =
    * [0,  0]
    * [0,s^2]
    */
    virtual void init(MatrixXf &F_in, MatrixXf &H_in, MatrixXf &Qa_in, VectorXf &Z_in, MatrixXf &R_in, VectorXf &X_init, MatrixXf &P_init);

    /* Initalize the Kalman filter
    
    dim_xin -> size of state vector
    dim_zin -> size of measurement vector
    F_in -> State transition matrix (dim_x X dim_x)
    H_in -> Observation matrix (dim_z X dim_x)
    Qa_in -> Process noise covariance initializer, see below (dim_x X dim_x)
    Z_in -> Measurement vector. Update passed vector with new values each time step. (dim_z)
    R_in -> Measurement Covariance matrix. Update passed matrix with new values each time step. (dim_z X dim_z)
    X_init -> inital state vector. (dim_x)
    P_init -> inital estimate covariance (dim_x X dim_x)

    Note: Qa should be initialized as s^2[M], where M has a 1 on the main diagonal corresponding to each
    * state variable associated with s^2. For instance, state vector [x,y] where only y is affected by variance has Qa =
    * [0,  0]
    * [0,s^2]
    */
    virtual void init(int dim_xin, int dim_zin, MatrixXf &F_in, MatrixXf &H_in, MatrixXf &Qa_in, VectorXf &Z_in, MatrixXf &R_in, VectorXf &X_init, MatrixXf &P_init);

    /*Update the filter with new measurements
    Note that this reads the variables Z and R passed to init() for new data.*/
    virtual void update();

    /* Returns VectorXf-type state vector for current state*/
    VectorXf getState() {return X;}

    /*Returns VectorXF-type state vector for predicted next state*/
    VectorXf getNextState() {return Xpred;}

    /*Returns MatrixXf-type covariance matrix for current state*/
    MatrixXf getCov() {return P;}

    /*Returns MatrixXf-type covariance matrix for predicted next state*/
    MatrixXf getNextCov() {return Ppred;}

protected:
    /* Initialize the low pass filter with N measurements for callibration. 
    - betaX is a dim_x X dim_n matrix of unfiltered measurements for callibration
    */
    bool lowPassInit();

    /*Low - pass enable*/
    bool lowPass;

    /* Problem Dimensions */
    int dim_x;
    int dim_z;

    /* Fixed Matrices */
    MatrixXf *F;  // State Transition matrix
    MatrixXf *H;  // Mesaurement function
    MatrixXf Q;   // Process Noise Covariance matrix
    MatrixXf I;   // Identity matrix

    /* State Matrices */
    VectorXf X;     // Current State vector
    MatrixXf P;     // Current State Covariance matrix
    VectorXf Xpred; // Predicted state matrix
    MatrixXf Ppred; // Predicted Coviariance matrix
    MatrixXf K;     // Kalman Gain matrix

    /* Measurement Matrices */
    VectorXf *Z; // measurement Matrix
    MatrixXf *R; // measurement Covariance matrix
};
#endif

/* Notes:
-   Note that any system input equations are omitted -
    it is assumed that the system state is independent of actions taken by the filter / the control system that the filter informs
-   Note that Zn has a 'true' value and an observed value - for instance:
    Xn is the position, velocity, acceleration of a car in 3 dimensions and Zn is the (measured) acceleration of the car in 3 dimensions.
    This means that the state update equation, to find the difference in measured acceleration between time step n-1 and time step n,
    must use the "true" value of Zn-1, not the value actually measured at time step n-1. To do this, it uses the state at time n-1 and
    guesses what the 'true' measurements must have been, as in the equation Zn = HXn.
*/
