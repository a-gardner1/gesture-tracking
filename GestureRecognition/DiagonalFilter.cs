using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    /// <summary>
    /// A very simple filter for estimating an n-dimensional constant whose components are all uncorrelated.
    /// </summary>
    class DiagonalFilter
    {
        /// <summary>
        /// Use primarily as a storage device. We are going to control the prediction and correction steps
        /// in order to make things efficient.
        /// Super simplified Kalman filter structure.
        /// </summary>
        struct KFilter
        {
            internal Vector<double> K, PHat, P, R;
            internal Vector<double> xHat, x, z;

            public KFilter(int n, double processNoiseMagnitude, double measurementNoiseMagnitude)
            {
                K = new DenseVector(n);
                PHat = new DenseVector(n).Add(1);
                P = new DenseVector(n).Add(1);
                R = new DenseVector(n).Add(1) * measurementNoiseMagnitude;
                xHat = new DenseVector(n);
                x = new DenseVector(n);
                z = new DenseVector(n);
            }
        }
        KFilter state;
        double processNoiseMagnitude;

        int numDimensions;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="numDimensions"> The number of variables for which covariance is calculated.</param>
        /// <param name="processNoiseMagnitude"></param>
        /// <param name="measurementNoiseMagnitude"></param>
        public DiagonalFilter(int numDimensions, double processNoiseMagnitude, double measurementNoiseMagnitude)
        {
            if (numDimensions <= 0)
            {
                throw new ArgumentException("The number of dimensions must be positive. Provided: " + numDimensions + ".");
            }
            this.numDimensions = numDimensions;
            state = new KFilter(numDimensions, processNoiseMagnitude, measurementNoiseMagnitude);
            this.processNoiseMagnitude = processNoiseMagnitude;
        }

        public void init(Vector<double> initialEstimate)
        {
            state.x = initialEstimate;
        }

        public void step(Vector<double> estimate, double dt, bool isInvalid = false)
        {
            //predict
            state.xHat = state.x.Clone();
            state.PHat = state.P.Add(dt * dt * processNoiseMagnitude);
            //measure
            state.z = estimate;
            //correct
            state.K = state.PHat.PointwiseDivide(state.PHat + state.R);
            state.x = state.xHat + state.K.PointwiseMultiply(state.z - state.xHat);
            state.P = (state.K * (-1)).Add(1); //temporary value
            // use the numerically stable Joseph form
            state.P = state.P.PointwiseMultiply(state.PHat.PointwiseMultiply(state.P)) 
                + state.K.PointwiseMultiply(state.R.PointwiseMultiply(state.K));
            
        }

        public Vector<double> getState(bool aPosteriori = true)
        {
            if(aPosteriori)
            {
                return state.x.Clone();
            }
            else
            {
                return state.xHat.Clone();
            }
        }
    }
}
