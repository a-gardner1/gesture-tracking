using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    class CombinedPatternTracker : PatternTracker
    {
        KalmanFilter state;
        double smallGamma = 0.0001;
        double sigmaA, sigmaAlpha; // noise intensity for linear and angular acceleration
        double logLikelihood = 0, mse = 0;
        double gateSize = 40;
        bool isInGate = true; // whether the latest observation was within the expected area
        double angleScale = 100; // scale of the angular measurements. 1000 = milliradians, .001 = kiloradians

        public override double LogLikelihood
        {
            get { return logLikelihood; }
        }

        public override double MeanError
        {
            get { return mse; }
        }

        Vector<double> savedX;
        Matrix<double> savedP;
        Matrix<double> A; // matrix used for nonlinear transition.

        double R; // measurement noise intensity
        Matrix<double> rho;
        int numEdges;
        Matrix<double> C; // used for linearization of H for weights filter

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

        public override void saveState()
        {
            savedX = state.x.Clone();
            savedP = state.P.Clone();
        }

        public override void revertState()
        {
            if (savedX == null)
            {
                throw new Exception("No prior state exists, cannot revert!");
            }
            state.x.SetSubVector(0, state.x.Count, savedX);
            state.P.SetSubMatrix(0, 0, savedP);
        }

        public CombinedPatternTracker(int numMarkers, double sigmaAccel, double sigmaAngAccel, 
            double sigmaMeasure, double smallGammaThreshold = 0.0001)
        {
            numEdges = numMarkers - 1;
            smallGamma = smallGammaThreshold;
            sigmaA = sigmaAccel;
            sigmaAlpha = sigmaAngAccel;
            R = sigmaMeasure;
            state = new KalmanFilter(3 * numMarkers + 12, 3 * numMarkers + 3, 0);
            A = CreateMatrix.DenseIdentity<double>(3 * numMarkers + 12);
            rho = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(3);
            C = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(numMarkers * 3 + 6, 6);
            state.transitionFunction = transitionFunction;
            calcStationary();
        }

        public override void init(System.Collections.ObjectModel.ReadOnlyCollection<Vector> pattern)
        {
            if (pattern.Count != numEdges + 1)
            {
                throw new Exception("Wrong number of markers in pattern!");
            }
            //ignore invalidity. If it is invalid, it should be zero anyway.
            state.x[0] = pattern[0].getX();
            state.x[1] = pattern[0].getY();
            state.x[2] = pattern[0].getZ();
            for (int i = 1; i < pattern.Count; ++i)
            {
                state.x[6 + 3 * i] = pattern[i].getX() - pattern[0].getX();
                state.x[7 + 3 * i] = pattern[i].getY() - pattern[0].getY();
                state.x[8 + 3 * i] = pattern[i].getZ() - pattern[0].getZ();
            }
        }

        public override void step(System.Collections.ObjectModel.ReadOnlyCollection<Vector> positions, Vector gammaHat, double dt)
        {
            if (NumGatesMissed >= MaxGatesMissed)
            {
                // first we must verify that a complete pattern is present. otherwise, we cannot reinitialize.
                bool allPresent = true;
                foreach (Vector position in positions)
                {
                    allPresent = allPresent && !position.isInvalid();
                }
                if (allPresent)
                {
                    state = new KalmanFilter(3 * positions.Count + 12, 3 * positions.Count + 3, 0);
                    init(positions);
                }
            }
            calcNonstationaryP(dt);
            state.predict();
            //measure
            for (int i = 0; i < positions.Count; ++i)
            {
                state.z[3 * i] = positions[i].getX();
                state.z[3 * i + 1] = positions[i].getY();
                state.z[3 * i + 2] = positions[i].getZ();
            }
            state.z[3 * positions.Count] = gammaHat.getX();
            state.z[3 * positions.Count + 1] = gammaHat.getY();
            state.z[3 * positions.Count + 2] = gammaHat.getZ();
            state.setZ();
            // correct
            calcNonstationaryC(positions, dt, !gammaHat.isInvalid());
            logLikelihood = state.getLogLikelihood();
            mse = Math.Sqrt(Geometry.getSumSquareError(positions, getPositions(false).AsReadOnly(), false)) / 2;
            isInGate = testInGate(positions, gateSize, 2.5);
            if (!isInGate && numGatesMissed < maxGatesMissed)
            {
                state.x.SetSubVector(0, state.x.Count, state.xHat);
                state.P.SetSubMatrix(0, 0, state.PHat);
                ++numGatesMissed;
            }
            else
            {
                numGatesMissed = 0;
                state.correct();
            }
        }
        
        private DenseVector transitionFunction()
        {
            return (DenseVector) (A * state.x);
        }

        private void calcStationary()
        {
            state.R = (Matrix) (CreateMatrix.DenseIdentity<double>(state.z.Count) * R);
        }

        private void calcNonstationaryP(double dt)
        {
            //compute F(t,k)
            var gamma = calcRho(dt, false);
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt2 * dt2;
            double dt5 = dt4 * dt;
            double dt6 = dt3 * dt3;
            // generate transition matrices
            A.SetSubMatrix(0, 0, KFUtility.calculateTransitionMatrix(2, 3, dt));
            state.A.SetSubMatrix(0, 0, KFUtility.calculateTransitionMatrix(2, 3, dt));
            for (int i = 0; i < numEdges; ++i)
            {
                A.SetSubMatrix(9 + 3 * i, 9 + 3 * i, rho);
                state.A.SetSubMatrix(9 + 3 * i, 9 + 3 * i, rho);
                Matrix<double> C_i = PatternTracker.calcC_i(
                    new Vector(state.x[9 + 3 * i], state.x[10 + 3 * i], state.x[11 + 3 * i]),
                    gamma.Item1, 
                    gamma.Item2, 
                    smallGamma);
                C.SetSubMatrix(9 + 3 * i, 0, C_i * (dt / angleScale));
                C.SetSubMatrix(9 + 3 * i, 3, C_i * (dt2 / (2 * angleScale)));
            }
            state.A.SetSubMatrix(0, 9 + 3 * numEdges, C);
            A.SetSubMatrix(9 + 3 * numEdges, 9 + 3 * numEdges, KFUtility.calculateTransitionMatrix(1, 3, dt));
            state.A.SetSubMatrix(9 + 3 * numEdges, 9 + 3 * numEdges, KFUtility.calculateTransitionMatrix(1, 3, dt));
            state.setA();
            //generate process noise covariances
            state.Q.SetSubMatrix(0, 0, KFUtility.calculateProcessNoiseMatrix(2, 3, dt, sigmaA));
            state.Q.SetSubMatrix(9 + 3 * numEdges, 9 + 3 * numEdges, 
                KFUtility.calculateProcessNoiseMatrix(1, 3, dt, sigmaAlpha));
            Matrix<double> I3dt5 = I3 * (dt6 * sigmaAlpha / 36);
            for (int i = 0; i < numEdges; ++i)
            {
                for (int j = 0; j < numEdges; ++j)
                {
                    state.Q.SetSubMatrix(9 + 3 * i, 9 + 3 * j, I3dt5);
                }
                state.Q.SetSubMatrix(9 + 3 * i, 9 + 3 * numEdges, state.Q.SubMatrix(0, 1, 3, 6) * (sigmaAlpha / sigmaA));
                state.Q.SetSubMatrix(9 + 3 * numEdges, 9 + 3 * i, state.Q.SubMatrix(3, 6, 0, 1) * (sigmaAlpha / sigmaA));
            }
            //state.Q.SetSubMatrix(9 + 3 * numEdges, 9 + 3 * numEdges, state.Q.SubMatrix(3, 6, 3, 6));
            state.setQ();
        }

        private void calcNonstationaryC(System.Collections.ObjectModel.ReadOnlyCollection<Vector> measurements,
            double dt, bool rotationMeasured)
        {
            // generate measurement matrix
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
            }
            if (rotationMeasured)
            {
                state.H.SetSubMatrix(3 * (numEdges + 1), state.x.Count - 6, (dt / angleScale) * I3 );
                //state.H.SetSubMatrix(3 * (numEdges + 1), state.x.Count - 3, (dt * dt / 2) * I3);
            }
            else
            {   //rotation not measured
                state.H.SetSubMatrix(3 * (numEdges + 1), state.x.Count - 6, new DenseMatrix(3, 6));
            }
            state.setH();
            state.setR();
        }

        private Tuple<Vector, Matrix<double>> calcRho(double dt, bool useAPriori)
        {
            double dt2 = dt * dt / 2;
            Vector gamma;
            int floor = 3 * numEdges + 9;
            if (useAPriori)
            {
                gamma = new Vector(state.xHat[floor] * dt + state.xHat[floor + 3] * dt2,
                    state.xHat[floor + 1] * dt + state.xHat[floor + 4] * dt2,
                    state.xHat[floor + 2] * dt + state.xHat[floor + 5] * dt2) / angleScale;
            }
            else
            {
                gamma = new Vector(state.x[floor] * dt + state.x[floor + 3] * dt2,
                    state.x[floor + 1] * dt + state.x[floor + 4] * dt2,
                    state.x[floor + 2] * dt + state.x[floor + 5] * dt2) / angleScale;
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
            return new Vector(state.x[9 + 3 * numEdges], state.x[10 + 3 * numEdges], state.x[11 + 3 * numEdges]);
        }

        public override Vector getAngularAcceleration()
        {
            return new Vector(state.x[12 + 3 * numEdges], state.x[13 + 3 * numEdges], state.x[14 + 3 * numEdges]);
        }

        public override string checkForNumericalIssues()
        {
            Vector<System.Numerics.Complex> test = state.P.Evd().EigenValues;
            int minIndex = 0;
            double minimum = test[0].Real, minimum2 = 0;
            for (int i = 1; i < test.Count; ++i)
            {
                if (test[i].Real < minimum)
                {
                    minIndex = i;
                    minimum = test[i].Real;
                }
            }
            test = state.PHat.Evd().EigenValues;
            minimum2 = test[0].Real;
            for (int i = 1; i < test.Count; ++i)
            {
                if (test[i].Real < minimum2)
                {
                    minIndex = i;
                    minimum2 = test[i].Real;
                }
            } 
            return "k(P) = " + state.P.ConditionNumber().ToString("0.##") + "\r\n"
                + "k(S) = " + (state.H * state.PHat.TransposeAndMultiply(state.H) + state.R).ConditionNumber().ToString("0.##") + "\r\n"
                + "smallest eigenvalue(PHat) = " + minimum2.ToString() + "\r\n"
                + "smallest eigenvalue(P) = " + minimum.ToString() + "\r\n" 
                + "trace(PHat) = " + state.PHat.Trace().ToString("0.##") + "\r\n"
                + "trace(P) = " + state.P.Trace().ToString("0.##") + "\r\n"; 
        }

        /// <summary>
        /// Define the gate as an expected radius centered on the origin and a specified
        /// range of axis angles.
        /// </summary>
        /// <param name="positions"></param>
        /// <returns></returns>
        private bool testInGate(System.Collections.ObjectModel.ReadOnlyCollection<Vector> positions,
            double maxOriginDeviation, double maxAngleDeviation)
        {
            // get coordinate to determine axes
            List<Vector> predictedCoordinates = Geometry.getCoordinateSystem(getPositions(false));
            List<Vector> measuredCoordinates = Geometry.getCoordinateSystem(positions);
            if (measuredCoordinates.Count > 0)
            {
                if ((predictedCoordinates[0] - measuredCoordinates[0]).getMagnitude() > maxOriginDeviation)
                {
                    return false;
                }
                else if (predictedCoordinates[1].getAngleBetween(measuredCoordinates[1]) > maxAngleDeviation)
                {
                    // 2.5 chosen as approximately the fastest human angular velocity
                    return false;
                }
                else if (predictedCoordinates[2].getAngleBetween(measuredCoordinates[2]) > maxAngleDeviation)
                {
                    // 2.5 chosen as approximately the fastest human angular velocity
                    return false;
                }
                else if (predictedCoordinates[3].getAngleBetween(measuredCoordinates[3]) > maxAngleDeviation)
                {
                    // 2.5 chosen as approximately the fastest human angular velocity
                    return false;
                }
            }
            // we have passed all of the tests
            return true;
        }

        public override bool IsInGate
        {
            get { return isInGate; }
        }
    }
}
