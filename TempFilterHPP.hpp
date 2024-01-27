#ifndef TEMP_FILTER_HPP
#define TEMP_FILTER_HPP

class TempFilter
{
    public:
        TempFilter();

        void init(double p0, double measurement, double q);
        void predict(double oldX);
        void extrapolateEstimateVariance(double p0, double q);
        double getP1();
        double getX();
        void measure(double r, double zN);
        void setR(double r);
        void setM(double zM);
        void kalmanGainMethod(double p, double r);
        double getK();
        double calculateXPredict();
        double calculateNewP();
    
    protected:

    private:
    
};

#endif 
