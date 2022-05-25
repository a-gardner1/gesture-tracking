using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.ObjectModel;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    /**
     * Responsible for tracking a rigid pattern of markers in global coordinates, e.g.
     * the pattern on the back of the hand. Should be capable of handling partial and
     * total occlusions of the pattern as well as smoothing out spurious measurements.
     * 
     * Currently implemented with a separate SimpleMarkerTracker per marker in the pattern.
     * */
    class DualPatternTracker : PatternTracker
    {
        KalmanFilter state; // primary Kalman filter; position, velocity, acceleration, orientation
        KalmanFilter weights; // secondary Kalman filter; angular velocity and angular acceleration
        double smallGamma = 0.0001; // the threshold used to determine when an approximation for small rotations is used
        double sigmaA, sigmaAlpha; // noise intensity for linear and angular acceleration
        double R; // measurement noise intensity
        double logLikelihood = 0, mse = 0;
        bool isInGate = true;

        Vector<double> savedX, savedZ;
        Matrix<double> savedPX, savedPZ;

        public override double LogLikelihood
        {
            get { return logLikelihood; }
        }

        public override double MeanError
        {
            get { return mse; }
        }

        public override double SigmaAlpha
        {
            get { return sigmaAlpha; }
            set { sigmaAlpha = value; }
        }

        public override double SigmaA
        {
            get { return sigmaA; }
            set { sigmaA = value; }
        }

        public override double SigmaVicon
        {
            get { return R; }
            set { R = value; calcStationary(); }
        }
        Matrix<double> rho;
        int numEdges;
        Matrix<double> C; // used for linearization of H for weights filter

        public DualPatternTracker(int numMarkers, double sigmaAccel, double sigmaAngAccel, 
            double sigmaMeasure, double smallGammaThreshold = 0.0001)
        {
            numEdges = numMarkers - 1;
            smallGamma = smallGammaThreshold;
            sigmaA = sigmaAccel;
            sigmaAlpha = sigmaAngAccel;
            R = sigmaMeasure;
            state = new KalmanFilter(numMarkers * 3 + 6, numMarkers * 3, 0);
            weights = new KalmanFilter(6, numMarkers * 3 + 3, 0);
            rho = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(3);
            C = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(numMarkers * 3 + 6, 6);
            calcStationary();
        }

        /// <summary>
        /// Initialize the state of the Kalman filters.
        /// </summary>
        /// <param name="pattern"> Used to set the initial state</param>
        public override void init(ReadOnlyCollection<Vector> pattern)
        {
            if (pattern.Count != numEdges + 1)
            {
                throw new Exception("Wrong number of markers in pattern!");
            }
            //ignore invalidity. If it is invalid, it should be zero anyway.
            state.x[0] = pattern[0].getX();
            state.x[1] = pattern[0].getY();
            state.x[2] = pattern[0].getZ();
            for(int i = 1; i < pattern.Count; ++i)
            {
                state.x[6 + 3 * i] = pattern[i].getX() - pattern[0].getX();
                state.x[7 + 3 * i] = pattern[i].getY() - pattern[0].getY();
                state.x[8 + 3 * i] = pattern[i].getZ() - pattern[0].getZ();
            }
        }

        public override void saveState()
        {
            savedX = state.x.Clone();
            savedZ = weights.x.Clone();
            savedPX = state.P.Clone();
            savedPZ = weights.P.Clone();
        }

        public override void revertState()
        {
            if (savedX == null)
            {
                throw new Exception("Cannot revert to saved state. No previous state has been saved!");
            }
            state.x.SetSubVector(0, state.x.Count, savedX);
            weights.x.SetSubVector(0, weights.x.Count, savedZ);
            state.P.SetSubMatrix(0, 0, savedPX);
            weights.P.SetSubMatrix(0, 0, savedPZ);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="measurements"></param>
        /// <param name="dt"></param>
        public override void step(ReadOnlyCollection<Vector> measurements, Vector RHat, double dt)
        {
            //predict
            calcNonstationaryWP(dt);
            weights.predict();
            calcNonstationarySP(dt);
            state.predict();
            //measure
            for (int i = 0; i < measurements.Count; ++i)
            {
                state.z[3 * i] = measurements[i].getX();
                state.z[3 * i + 1] = measurements[i].getY();
                state.z[3 * i + 2] = measurements[i].getZ();
            }
            weights.z.SetSubVector(0, state.z.Count, state.z);
            weights.z[3 * measurements.Count] = RHat.getX();
            weights.z[3 * measurements.Count + 1] = RHat.getY();
            weights.z[3 * measurements.Count + 2] = RHat.getZ();
            state.setZ();
            weights.setZ();
            // if some are visible, then there is an approximate rotation. else there isn't
            calcNonstationaryC(measurements, dt, !RHat.isInvalid());
            logLikelihood = state.getLogLikelihood() + weights.getLogLikelihood();
            mse = Math.Sqrt(Geometry.getSumSquareError(measurements, getPositions().AsReadOnly(), false)) / 2;
            state.correct();
            weights.correct();
            DenseVector residual = new DenseVector(weights.z.Count);
            residual.SetSubVector(0, state.z.Count, state.z - state.H * state.xHat);
            residual.SetSubVector(state.z.Count, 3, weights.z.SubVector(state.z.Count, 3) - weights.H.SubMatrix(3 * (numEdges + 1), 3, 0, 6) * weights.xHat);
            weights.x = (DenseVector) (weights.xHat + weights.K * (residual));
        }

        /**
         * Calculate certain quantities that are time-invariant.
         * */
        private void calcStationary()
        {
            state.R = (MathNet.Numerics.LinearAlgebra.Double.Matrix)(CreateMatrix.DenseIdentity<double>(state.R.RowCount) * R);
            weights.R = (MathNet.Numerics.LinearAlgebra.Double.Matrix)(CreateMatrix.DenseIdentity<double>(weights.R.RowCount) * R);
        }

        private void calcNonstationaryWP(double dt)
        {
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            weights.A.SetSubMatrix(0, 3, I3 * dt);
            weights.setA();
            weights.Q.SetSubMatrix(0, 0, I3 * (dt3 / 3));
            weights.Q.SetSubMatrix(0, 3, I3 * (dt2 / 2));
            weights.Q.SetSubMatrix(3, 0, I3 * (dt2 / 2));
            weights.Q.SetSubMatrix(3, 3, I3 * (dt));
            weights.Q = (MathNet.Numerics.LinearAlgebra.Double.DenseMatrix)(weights.Q * sigmaAlpha);
            weights.setQ();
        }

        /// <summary>
        /// Calculate time-variant quantities (most of them).
        /// </summary>
        /// <param name="measurements"> Latest positions of pattern markers. If a marker is occluded, an invalid Vector should be provided in its place. </param>
        /// <param name="dt"> The length of the time-step. </param>
        private void calcNonstationarySP(double dt)
        {
            calcRho(dt, true);
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt2 * dt2;
            double dt5 = dt4 * dt;
            // generate transition matrices
            state.A.SetSubMatrix(0, 3, I3 * dt);
            state.A.SetSubMatrix(0, 6, I3 * dt2 / 2);
            state.A.SetSubMatrix(3, 6, I3 * dt);
            for(int i = 0; i < numEdges; ++i)
            {
                state.A.SetSubMatrix(9 + 3 * i, 9 + 3 * i, rho);
            }
            state.setA();
            //generate process noise covariances
            state.Q.SetSubMatrix(0, 0, I3 * (dt5 * (sigmaA / 20)));
            state.Q.SetSubMatrix(0, 3, I3 * (dt4 * (sigmaA / 8)));
            state.Q.SetSubMatrix(3, 0, I3 * (dt4 * (sigmaA / 8)));
            state.Q.SetSubMatrix(0, 6, I3 * (dt3 * (sigmaA / 6)));
            state.Q.SetSubMatrix(6, 0, I3 * (dt3 * (sigmaA / 6)));
            state.Q.SetSubMatrix(3, 3, I3 * (dt3 / 3));
            state.Q.SetSubMatrix(3, 6, I3 * (dt2 / 2));
            state.Q.SetSubMatrix(6, 3, I3 * (dt2 / 2));
            state.Q.SetSubMatrix(6, 6, I3 * (dt));
            Matrix<double> I3dt5 = I3 * (dt5 * sigmaAlpha / 20);
            for (int i = 0; i < numEdges; ++i)
            {
                for (int j = 0; j < numEdges; ++j)
                {
                    state.Q.SetSubMatrix(9 + 3 * i, 9 + 3 * j, I3dt5);
                }
            }
            state.setQ();
        }

        private void calcNonstationaryC(ReadOnlyCollection<Vector> measurements, double dt, bool rotationMeasured)
        {
            if (measurements.Count != numEdges + 1)
            {
                throw new Exception("Wrong number of markers in measurements!");
            }
            // rho should already be calculated from calcNonstationarySP
            var gamma = calcRho(dt, true); 
            double dt2 = dt * dt;
            // generate measurement matrices
            state.H.SetSubMatrix(0, 0, measurements[0].isInvalid() ? I3 * 0 : I3);
            //double mag = gamma.Item1.getMagnitude();
            for (int i = 0; i < numEdges; ++i)
            {
                if (measurements[i + 1].isInvalid())
                {
                    state.H.SetSubMatrix(3 + 3 * i, 9 + 3 * i, I3 * 0);
                    state.H.SetSubMatrix(3 + 3 * i, 0, I3 * 0);
                }
                else
                {
                    state.H.SetSubMatrix(3 + 3 * i, 9 + 3 * i, I3);
                    state.H.SetSubMatrix(3 + 3 * i, 0, I3);
                }
                //calculate partial derivatives of rho*e_i with respect to omega and alpha
                Matrix<double> C_i = PatternTracker.calcC_i(new Vector(state.xHat[9 + 3 * i], state.xHat[10 + 3 * i], state.xHat[11 + 3 * i]), gamma.Item1, gamma.Item2, smallGamma);
                C.SetSubMatrix(9 + 3 * i, 0, C_i * dt);
                C.SetSubMatrix(9 + 3 * i, 3, C_i * (dt2 / 2));
            }
            weights.H.SetSubMatrix(0, 0, (MathNet.Numerics.LinearAlgebra.Double.Matrix)(state.H * C));
            if (rotationMeasured)
            {   
                weights.H.SetSubMatrix(3 * (numEdges + 1), 0, dt * I3);
                weights.H.SetSubMatrix(3 * (numEdges + 1), 3, (dt2 / 2) * I3);
            }
            else
            {   //rotation not measured
                weights.H.SetSubMatrix(3 * (numEdges + 1), 0, new DenseMatrix(3,6));
            }
            state.setH();
            weights.setH();
            state.setR();
            weights.setR();
        }

        private Tuple<Vector, Matrix<double>> calcRho(double dt, bool useAPriori)
        {
            double dt2 = dt * dt / 2;
            Vector gamma;
            if (useAPriori)
            {
                gamma = new Vector(weights.xHat[0] * dt + weights.xHat[3] * dt2,
                    weights.xHat[1] * dt + weights.xHat[4] * dt2,
                    weights.xHat[2] * dt + weights.xHat[5] * dt2);
            }
            else
            {
                gamma = new Vector(weights.x[0] * dt + weights.x[3] * dt2,
                    weights.x[1] * dt + weights.x[4] * dt2,
                    weights.x[2] * dt + weights.x[5] * dt2);
            }
            Matrix<double> Gamma = Geometry.getCrossProductMatrix(gamma);
            double mag = gamma.getMagnitude();
            if (mag < smallGamma)
            {
                rho = I3 + Gamma + Gamma * Gamma * 0.5;
            }
            else
            {
                rho = I3 + Gamma * (Math.Sin(mag) / mag) + Gamma * Gamma * ((1 - Math.Cos(mag)) / (mag * mag)); 
            }
            return new Tuple<Vector, Matrix<double>>(gamma, Gamma);
        }

        public override List<Vector> getPositions(bool aPosteriori = true)
        {
            List<Vector> positions = new List<Vector>(new Vector[numEdges + 1]);
            if (aPosteriori)
            {
                positions[0] = new Vector(state.x[0], state.x[1], state.x[2]);
            }
            else
            {
                positions[0] = new Vector(state.xHat[0], state.xHat[1], state.xHat[2]);
            }
            for (int i = 0; i < numEdges; ++i)
            {
                if (aPosteriori)
                {
                    positions[i + 1] = new Vector(state.x[9 + 3 * i], state.x[10 + 3 * i], state.x[11 + 3 * i]);
                }
                else
                {
                    positions[i + 1] = new Vector(state.xHat[9 + 3 * i], state.xHat[10 + 3 * i], state.xHat[11 + 3 * i]);
                }
                positions[i + 1] += positions[0];
            }
            return positions;
        }

        public override Vector getAngularVelocity()
        {
            return new Vector(weights.x[0], weights.x[1], weights.x[2]);
        }

        public override Vector getAngularAcceleration()
        {
            return new Vector(weights.x[3], weights.x[4], weights.x[5]);
        }

        public override string checkForNumericalIssues()
        {
            throw new NotImplementedException();
        }


        public override bool IsInGate
        {
            get { throw new NotImplementedException(); }
        }
    }
}
