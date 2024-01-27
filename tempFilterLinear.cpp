// Author: Andrey Dimanchev
// Example of Linear Kalman Filter based on an example from "KALMAN FILTER from the Ground Up" by Alex Becker

#include "tempFilterHPP.hpp"

#include <fstream>
#include <iostream>

using namespace std;

double currentX;
double p1;
double kalmanGain;
double R;
double zMeasurement;

// get initialize state
void TempFilter::init(double p0, double measurement, double q){

    cout << "initialize\n";
    
    predict(measurement);
    extrapolateEstimateVariance(p0, q);
}

// setter for measurement and r
void TempFilter::measure(double r, double zN){
    setR(r);
    setM(zN);
}

// since linear no prediciton model is used hence current measurement is predicted to be the new one 
void TempFilter::predict(double oldX){
    currentX = oldX;
}

// calculate new estimate variance 
void TempFilter::extrapolateEstimateVariance(double p0, double q){
    p1 = p0 + q;
}

// getter for old P
double TempFilter::getP1(){
    return p1;
}

// getter for measurement
double TempFilter::getX(){
    return currentX;
}

// setter for R
void TempFilter::setR(double r){
    R = r;
}

// setter for measurement 
void TempFilter::setM(double zM){
    zMeasurement = zM;
}

// calculate kalman gain
void TempFilter::kalmanGainMethod(double p, double r){
    kalmanGain = (double) (p / (p+r));
}

// getter for kalman gain
double TempFilter::getK(){
    return kalmanGain;
}

// predict new measurement and return it
double TempFilter::calculateXPredict(){
    double xPredict = getX() + kalmanGain * (zMeasurement - getX());
    return xPredict;
}

// calculate new p with kalman gain using the old p
// return the new P 
double TempFilter::calculateNewP(){
    double newP = (1-kalmanGain) * getP1();
    return newP;
}

// contructor for TempFilter
TempFilter::TempFilter() {
}


//initializes the system with initial parameters
int main()
{
                        // define system
    double dt = 5;      // taken every 5 seconds
    double truth = 50;  // truth value
    double q = 0.15;    // process noise variance (assumed to be very accurate)

    double mE = 0.1;    // measurement error

    double iX = 60;     // initial measurement 

    double estV = 10000;// estimate variance is high since guess is imprecise

    double measurements[10] = {50.486,50.963,51.597,52.001,52.518,53.05,53.438,53.858,54.465,55.114};
    double truths[10] = {50.505,50.994,51.493,52.001,52.506,52.998,53.521,54.005,54.5, 54.997};
    double estimates[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int l = sizeof(measurements) / sizeof(measurements[0]);

    TempFilter tf;

    // initialization
    tf.init(estV, iX, q);
    

    for(int i = 0; i < l; i++){
        cout << "\n" << i + 1 << "\n";
        cout << "Measurement: " << measurements[i] << "\n";
        // measure
        tf.measure(0.01, measurements[i]);
        // update
        tf.kalmanGainMethod(tf.getP1(), 0.01);
        double xPredict = tf.calculateXPredict();
        double newP = tf.calculateNewP();
        //predict
        tf.predict(xPredict);       
        tf.extrapolateEstimateVariance(newP, q);

        cout << "Update" << "\n";
        cout << "K: " << tf.getK() << "\n";
        cout << "x(n,n): " << xPredict << "\n";
        cout << "newP: " << newP << "\n";

        estimates[i] = xPredict;

        cout << "Predict" << "\n";
        cout << "x(n+1, n+1): " << tf.getX() << "\n";
        cout << "predictP: " << tf.getP1() << "\n";

        //cout << "Estimate: " << estimates << "\n";

    }

    // print estimates to plot with python file
    for (auto x : estimates) {
        std::cout << x << ", ";
    }

    return 0;
};