using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    // Provides utility functions for computing certain Kalman filter related quantities.
    abstract class KFUtility
    {
        /// <summary>
        /// Calculate Nth order white noise transition matrix.
        /// </summary>
        /// <param name="order">The order of differentiation, i.e. 1 includes velocity, 2 includes acceleration.</param>    
        /// <param name="dimensionality">The number of dimensions of the motion.</param>   
        /// <param name="dt">The duration of the transition.</param>   
        /// <returns></returns>
        public static Matrix<double> calculateTransitionMatrix(uint order, uint dimensionality, double dt)
        {
            Matrix<double> A = CreateMatrix.DenseIdentity<double>((int)(order + 1));
            double ddt = 1;
            for(int i = 0; i < order; ++i)
            {
                ddt *= (dt / (i + 1));
                for (int j = 0; j < order - i; ++j)
                {
                    A[j, j + i + 1] = ddt;
                    //A[j + 1, j] = ddt; // should not be symmetric
                }
            }
            return A.KroneckerProduct(CreateMatrix.DenseIdentity<double>((int)dimensionality));
        }

        /// <summary>
        /// Computes Q, a white-noise process noise matrix of the given order, 
        /// where 0 = position, 1 = velocity, 2 = acceleration, etc.
        /// </summary>
        /// <param name="order"></param>
        /// <param name="dimensionality"></param>
        /// <param name="dt"></param>
        /// <param name="sigmaSquared"></param>
        /// <returns></returns>
        public static Matrix<double> calculateProcessNoiseMatrix(uint order, uint dimensionality, double dt, double sigmaSquared)
        {
            Vector<double> w = new DenseVector((int)(order + 1));
            w[(int)order] = dt * Math.Sqrt(sigmaSquared); // eh... I'd like to avoid the square root, but..
            for (int i = 1; i < order + 1; ++i)
            {
                w[(int)order - i] = w[(int)order - i + 1] * (dt / (i + 1));
            }
            return w.OuterProduct(w).KroneckerProduct(CreateMatrix.DenseIdentity<double>((int)dimensionality));
        }


    }
}
