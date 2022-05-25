using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics.LinearAlgebra;

namespace GestureRecognition
{

    class KalmanFilter
    {
        class KalmanFilterException : Exception
        {
            public KalmanFilterException()
            {
            }

            public KalmanFilterException(string message)
                : base(message)
            {

            }

            public KalmanFilterException(string message, Exception inner)
                : base(message, inner)
            {

            }
        }

        // See "An Introduction to the Kalman Filter" by Greg Welch and Gary Bishop
        public delegate Vector<double> ArbitraryFunction();
        public ArbitraryFunction transitionFunction;
        public ArbitraryFunction measurementFunction;
        public Matrix<double> A; // transition 
        public Matrix<double> B; // adjustment
        public Matrix<double> Q; //processNoise
        public Matrix<double> H; //translation; translation from measurement variables/space to state variables/space
        public Matrix<double> R; //measurementNoise
        public Matrix<double> PHat; // P_k^-; a priori error covariance
        public Matrix<double> P; // P_k; a posteriori error covariance
        public Matrix<double> K; // Kalman gain
        public Vector<double> xHat; // x_k^-; a priori estimate/ prediction
        public Vector<double> x; // x_k; a posteriori estimate
        public Vector<double> z; // z_k; measurement
        public Vector<double> u; // u_k
        /// <summary>
        /// Identity matrix
        /// </summary>
        public Matrix<double> I; 
        protected bool isUncontrolled;
        /// <summary>
        /// Flags must all be set in order for filter to work; otherwise an exception is thrown. 
        /// Used to make sure no variables are forgotten by the user.
        /// Call all "set*()" functions each predict-correct cycle in order to set safety flags. 
        /// If the filter is uncontrolled, one does not need to call setB() nor setU().
        /// </summary>
        private uint safetyFlags; 

        /**
         * Create the Kalman filter, including persistent storage of variables.
         * */
        public KalmanFilter(int n, int m, int l) 
        {
            transitionFunction = linearTransition;
            measurementFunction = linearMeasurement;
            A = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.CreateIdentity(n);
            if (l > 0) { 
                B = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, l);
                u = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(l);
                isUncontrolled = false;
            }
            else
            {
                isUncontrolled = true;
                setB();
                setU();
            }
            Q = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.CreateIdentity(n);
            H = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(m, n);
            R = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.CreateIdentity(m);
            PHat = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n);
            P = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.CreateIdentity(n);
            K = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, m);
            I = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.CreateIdentity(n);
            xHat = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(n);
            x = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(n);
            z = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(m);
        }

        public Vector<double> linearTransition()
        {
            return isUncontrolled ? (Vector<double>)(A * x) : (Vector<double>)(A * x + B * u);
        }

        public Vector<double> linearMeasurement()
        {
            return H * xHat;
        }

        public void predict()
        {
            checkSetP();
            xHat = transitionFunction();
            PHat = (MathNet.Numerics.LinearAlgebra.Double.Matrix)(A * (P.TransposeAndMultiply(A)) + Q);
        }

        public void correct()
        {
            checkSetC();
            PHat = (MathNet.Numerics.LinearAlgebra.Double.Matrix)((PHat + PHat.Transpose()) * 0.5);
            K = (MathNet.Numerics.LinearAlgebra.Double.Matrix)(PHat.TransposeAndMultiply(H) * (H * PHat.TransposeAndMultiply(H) + R).Inverse());
            x = xHat + K * (z - measurementFunction());
            P = (MathNet.Numerics.LinearAlgebra.Double.Matrix) (I - K * H); //temporary value
            // use the more numerically stable Joseph form
            P = (MathNet.Numerics.LinearAlgebra.Double.Matrix)((P) * PHat.TransposeAndMultiply(P) + K * R.TransposeAndMultiply(K));
            //P = (MathNet.Numerics.LinearAlgebra.Double.Matrix)((P + P.Transpose()) * 0.5);
            resetAll();
        }

        /// <summary>
        /// Calculate and return the log-likelihood of the currently stored observation using the 
        /// current a priori estimate and estimated error covariance. 
        /// In other words, it is consistent with the last call to predict().
        /// 
        /// Returns NaN if the residual covariance H * PHat.TransposeAndMultiply(H) + R is singular.
        /// </summary>
        /// <returns></returns>
        public double getLogLikelihood()
        {
            try
            {
                Vector<double> residual = z - H * xHat;
                Matrix<double> S = H * PHat.TransposeAndMultiply(H) + R;
                return -(residual * ((S).Inverse() * residual) + Math.Log(S.Determinant()) + z.Count * Math.Log(2 * Math.PI)) / 2;
            }
            catch (MathNet.Numerics.NonConvergenceException e)
            {
                return Double.NaN;
            }
        }

        /// <summary>
        /// Signify that the transition matrix A has been set for this time-step.
        /// </summary>
        public void setA()
        {
            safetyFlags = safetyFlags | 1;
        }

        /// <summary>
        /// Signify that the control matrix B has been set for this time-step.
        /// </summary>
        public void setB()
        {
            safetyFlags = safetyFlags | 2;
        }

        /// <summary>
        /// Signify that the control input u has been set for this time-step.
        /// </summary>
        public void setU()
        {
            safetyFlags = safetyFlags | 4;
        }

        /// <summary>
        /// Signify that the measurement matrix H has been set for this time-step.
        /// </summary>
        public void setH()
        {
            safetyFlags = safetyFlags | 8;
        }

        /// <summary>
        /// Signify that the measurement z has been set for this time-step.
        /// </summary>
        public void setZ()
        {
            safetyFlags = safetyFlags | 16;
        }

        /// <summary>
        /// Signify that the process noise covariance Q has been set for this time-step.
        /// </summary>
        public void setQ()
        {
            safetyFlags = safetyFlags | 32;
        }

        /// <summary>
        /// Signify that the measurement noise covariance R has been set for this time-step.
        /// </summary>
        public void setR()
        {
            safetyFlags = safetyFlags | 64;
        }

        /// <summary>
        /// Check that all necessary flags are set for the predict step.
        /// </summary>
        private void checkSetP()
        {
            if ((safetyFlags & 39) != 39)
            {
                throw new KalmanFilterException("A variable required for Kalman filter prediction is not set.");
            }
        }

        /// <summary>
        /// Check that all necessary flags are set for the update step.
        /// </summary>
        private void checkSetC()
        {
            if ((safetyFlags & 127) != 127)
            {
                throw new KalmanFilterException("A variable required for Kalman filter correction is not set.");
            }
        }

        public void resetAll()
        {
            safetyFlags = 0;
            if (isUncontrolled)
            {
                setB();
                setU();
            }
        }
    }
}
