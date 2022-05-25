using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

//using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    /**
     * Arguably the simplest of the possible Kalman filter routes for tracking a marker while estimating
     * velocity and acceleration. 
     * 
     * Since we are assuming non-constant velocity and acceleration, we include a jerk component and let
     * noise reflect our uncertainty in its value.
     * 
     * Initialized with a priori guess of process and measurement covariances, assumed to be spherical.
     * 
     * Replaces missing values with Kalman filter predictions.
     * */
    class SimpleMarkerTracker
    {
        private KalmanFilter filter;

        internal KalmanFilter Filter
        {
            get { return filter; }
        }


        private double QScalar, RScalar; // process and measurement noise intensities
        private bool modelJerk;
        public SimpleMarkerTracker(double Q, double R, bool modelJerk)
        {
            this.modelJerk = modelJerk;
            // create a filter with 12 state variables, 3 observations, and 0 control variables
            filter = new KalmanFilter(modelJerk ? 12 : 9, 3, 0);
            QScalar = Q;
            RScalar = R;
            filter.Q = (DenseMatrix) filter.Q * QScalar; //currently scaled identity
            filter.R = (DenseMatrix) filter.R * RScalar; //currently scaled identity
            calcStationary();
        }

        public void init(Vector z)
        {
            filter.x[0] = z.getX();
            filter.x[1] = z.getY();
            filter.x[2] = z.getZ();
        }

        public Vector predict(double dt)
        {
            // calculate A_t, Q_t, R_t
            calcNonstationary(dt);
            filter.predict();
            return new Vector(filter.xHat[0], filter.xHat[1], filter.xHat[2]);
        }

        public void correct(Vector z)
        {
            // if no measurement taken (if occluded), use prediction as measurement
            if (z.isInvalid())
            {
                filter.z = (DenseVector) (filter.H * filter.xHat);
            }
            else // use measurement 
            {
                filter.z[0] = z.getX();
                filter.z[1] = z.getY();
                filter.z[2] = z.getZ();
            }
            filter.setZ();
            filter.correct();
            if (Double.IsNaN(filter.x.Norm(2)))
            {
                throw new Exception("Numerical error!");
            }
        }

        /**
         * At some point should replace return with read-only version or copy.
         * */
        public MathNet.Numerics.LinearAlgebra.Vector<double> getState()
        {
            return filter.x;
        }

        public Vector getPosition()
        {
            return new Vector(filter.x[0], filter.x[1], filter.x[2]);
        }
        public Vector getVelocity()
        {
            return new Vector(filter.x[3], filter.x[4], filter.x[5]);
        }
        public Vector getAcceleration()
        {
            return new Vector(filter.x[6], filter.x[7], filter.x[8]);
        }

        private void calcStationary()
        {
            filter.H[0, 0] = 1;
            filter.H[1, 1] = 1;
            filter.H[2, 2] = 1;
        }

        /**
         * Calculate quantities that depend upon the time-step.
         * */
        private void calcNonstationary(double dt)
        {
            double dt2 = dt * dt * 0.5;
            double dt3 = dt * dt * dt / 6.0;
            double dt4 = dt2 * dt2;
            filter.A[0, 3] = dt;
            filter.A[1, 4] = dt;
            filter.A[2, 5] = dt;
            filter.A[3, 6] = dt;
            filter.A[4, 7] = dt;
            filter.A[5, 8] = dt;
            filter.A[0, 6] = dt2;
            filter.A[1, 7] = dt2;
            filter.A[2, 8] = dt2;
            if (modelJerk)
            {
                filter.A[3, 9] = dt2;
                filter.A[4, 10] = dt2;
                filter.A[5, 11] = dt2;
                filter.A[0, 9] = dt3;
                filter.A[1, 10] = dt3;
                filter.A[2, 11] = dt3;
            }
            filter.Q[0, 0] = dt4 * QScalar;
            filter.Q[1, 1] = dt4 * QScalar;
            filter.Q[2, 2] = dt4 * QScalar;
            filter.Q[0, 3] = 2 * dt3 * QScalar;
            filter.Q[3, 0] = 2 * dt3 * QScalar;
            filter.Q[1, 4] = 2 * dt3 * QScalar;
            filter.Q[4, 1] = 2 * dt3 * QScalar;
            filter.Q[2, 5] = 2 * dt3 * QScalar;
            filter.Q[5, 2] = 2 * dt3 * QScalar;
            filter.Q[3, 3] = dt2 * QScalar;
            filter.Q[4, 4] = dt2 * QScalar;
            filter.Q[5, 5] = dt2 * QScalar;
            filter.Q[6, 6] = dt * QScalar;
            filter.Q[7, 7] = dt * QScalar;
            filter.Q[8, 8] = dt * QScalar;
            //filter.Q[9, 9] = QScalar;
            //filter.Q[10, 10] = QScalar;
            //filter.Q[11, 11] = QScalar;
            filter.R[0, 0] = RScalar;
            filter.R[1, 1] = RScalar;
            filter.R[2, 2] = RScalar;
            filter.setA();
            filter.setB();
            filter.setQ();
            filter.setR();
            filter.setU();
            filter.setH();
        }
    }
}
