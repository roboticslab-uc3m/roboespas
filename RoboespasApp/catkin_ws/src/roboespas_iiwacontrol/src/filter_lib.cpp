#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

//|----------------| Library that offers 2 classes that allow to filter the noise from data by
//| FILTER LIBRARY | returning the median of the last nsamples.
//|----------------|
// - MovingAverageFilter: Class that saves each sample and return the median of the last nsamples, or the median of all the samples introduced if they are less than nsamples
// - MovingAverageFilterSizVars: Class that creates six objects of the previous class to filter a whole cartesian position without having to create by yourself six different filters, one for each coordinate. All six filters use the same number of samples to calculate the median.
namespace MAFilter
{
    class MovingAverageFilter
    {
        //Class that saves each sample and return the median of the last nsamples, or the median of all the samples introduced if they are less than nsamples
        private:
        int nsamples=1;
        vector<double> values;
        public:
        //Constructor for the class, which just needs the number of samples that will be saved inside the object to calculate the median of them
        MovingAverageFilter(int n_samples)
        {
            nsamples=n_samples;
        }
        //Clear the samples inside the filter
        void Clear()
        {
            values.clear();
        }
        //Grabs a value, saves it in the vector of values inside the filter, deletes a value if the filter already has inside nsamples, and returns the median of all the nsamples of the filter. This introduces a delay, so nsamples shouldn't be too high, as it will introduce a delay of nsamples*tsample/2, but also reduces the noise of the measurement.
        double Filter(double val)
        {
            //Add the new value
            values.push_back(val);
            //Delete the first sample in the vector (which is the oldest one)
            if (values.size()>nsamples) //Limit nsamples
            {
                values.erase(values.begin());
            }
            //Temporal variable used to sort the vector without changing the order of entry to the vector
            vector<double> vals=values;
            sort(vals.begin(), vals.end());
            //Take number in medium position of the vector and return it as answer
            int median_id=round(vals.size()/2);
            double val_filtered;
            val_filtered=vals[median_id];
            return val_filtered;
        }
    };
    class MovingAverageFilterSixVars
    {
        //Class that saves each sample and return the median of the last nsamples, or the median of all the samples introduced if they are less than nsamples. This time each sample has six doubles, instead of just one. It is just a set of 6 one-value filters
        private:
        int nsamples=1;
        //Six independent filters
        MovingAverageFilter filter1 = MovingAverageFilter(1);
        MovingAverageFilter filter2 = MovingAverageFilter(1);
        MovingAverageFilter filter3 = MovingAverageFilter(1);
        MovingAverageFilter filter4 = MovingAverageFilter(1);
        MovingAverageFilter filter5 = MovingAverageFilter(1);
        MovingAverageFilter filter6 = MovingAverageFilter(1);
        public:
        MovingAverageFilterSixVars(int n_samples)
        {
            nsamples=n_samples;
            //All filters will have same nsamples
            filter1=MovingAverageFilter(nsamples);
            filter2=MovingAverageFilter(nsamples);
            filter3=MovingAverageFilter(nsamples);
            filter4=MovingAverageFilter(nsamples);
            filter5=MovingAverageFilter(nsamples);
            filter6=MovingAverageFilter(nsamples);
        }
        void Clear()
        {
            //When clearing the six-val filter just clear all the six filters
            filter1.Clear();
            filter2.Clear();
            filter3.Clear();
            filter4.Clear();
            filter5.Clear();
            filter6.Clear();
        }
        VectorXd Filter(VectorXd val)
        {
            //The filtered vector will be just a composition of the six filtered independent values of the input vector
            VectorXd val_filtered(6);
            val_filtered[0]=filter1.Filter(val[0]);
            val_filtered[1]=filter2.Filter(val[1]);
            val_filtered[2]=filter3.Filter(val[2]);
            val_filtered[3]=filter4.Filter(val[3]);
            val_filtered[4]=filter5.Filter(val[4]);
            val_filtered[5]=filter6.Filter(val[5]);
            return val_filtered;
        }
    };
}
