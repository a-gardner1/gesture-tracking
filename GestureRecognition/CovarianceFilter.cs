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
    /// A filter for estimating a positive semidefinite covariance matrix. 
    /// Since all matrices involved in the Kalman filter estimation are diagonal, this could be 
    /// significantly sped up in the future.
    /// </summary>
    class CovarianceFilter
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
        public CovarianceFilter(int numDimensions, double processNoiseMagnitude, double measurementNoiseMagnitude)
        {
            if (numDimensions <= 0)
            {
                throw new ArgumentException("The number of dimensions must be positive. Provided: " + numDimensions + ".");
            }
            this.numDimensions = numDimensions;
            state = new KFilter(numDimensions * (numDimensions + 1) / 2, processNoiseMagnitude, measurementNoiseMagnitude);
            this.processNoiseMagnitude = processNoiseMagnitude;
        }

        public void init(Matrix<double> initialEstimate)
        {
            state.x = MathUtility.upperTriangleVectorization(initialEstimate);
        }

        public void step(Matrix<double> estimate, double dt, bool isInvalid = false)
        {
            //predict
            state.xHat = state.x.Clone();
            state.PHat = state.P.Add(dt * dt * processNoiseMagnitude);
            //measure
            state.z = MathUtility.upperTriangleVectorization(estimate);
            //correct
            state.K = state.PHat.PointwiseDivide(state.PHat + state.R);
            state.x = state.xHat + state.K.PointwiseMultiply(state.z - state.xHat);
            Matrix<double> unconstrained = matricize(state.x);
            var evd = unconstrained.Evd();
            // clip eigenvalues
            for(int i = 0; i < evd.EigenValues.Count; ++i)
            {
                evd.EigenValues[i] = Math.Max(evd.EigenValues[i].Real, (double) 0);
            }
            Matrix<double> constrained = evd.EigenVectors.Multiply(evd.D).TransposeAndMultiply(evd.EigenVectors);
            // constrain a posteriori estimate, diagonal Kalman gain, and a posteriori estimated error
            state.x = MathUtility.upperTriangleVectorization(constrained);
            state.K = (state.x - state.xHat).PointwiseDivide(state.z - state.xHat);
            for (int i = 0; i < state.K.Count; ++i )
            {
                if(Double.IsNaN(state.K[i]))
                {
                    state.K[i] = 0;
                }
            }
            state.P = (state.K * (-1)).Add(1); //temporary value
            // use the numerically stable Joseph form
            state.P = state.P.PointwiseMultiply(state.PHat.PointwiseMultiply(state.P)) 
                + state.K.PointwiseMultiply(state.R.PointwiseMultiply(state.K));
            
        }

        /// <summary>
        /// Place the given vector into the upper triangular portion of a new matrix.
        /// </summary>
        /// <param name="upperTriangle"></param>
        /// <returns></returns>
        private Matrix<double> matricize(Vector<double> upperTriangle)
        {
            Matrix<double> result = new DenseMatrix(numDimensions);
            for (int i = 0, k = 0; i < result.ColumnCount; ++i)
            {
                for (int j = 0; j <= i; ++j, ++k)
                {
                    result[j, i] = upperTriangle[k];
                }
            }
            return result;
        }

        /// <summary>
        /// Return the current estimate of the covariance matrix.
        /// </summary>
        public Matrix<double> Covariance
        {
            get
            {
                Matrix<double> result = matricize(state.x);
                for(int i = 0; i < result.RowCount; ++i)
                {
                    for(int j = 0; j <= i; ++j)
                    {
                        result[i, j] = result[j, i];
                    }
                }
                return result;
            }
        }

        /// <summary>
        /// Return the correlation matrix corresponding to the current estimate of the covariance matrix.
        /// </summary>
        public Matrix<double> Correlation
        {
            get
            {
                Vector<double> deviation = Covariance.Diagonal().PointwisePower(0.5);
                return Covariance.PointwiseDivide(deviation.OuterProduct(deviation));
            }
        }
    }
}
