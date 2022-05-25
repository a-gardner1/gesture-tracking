using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    class RigidPatternTracker : PatternTracker
    {
        KalmanFilter state;
        Matrix<double> edges;
        Vector<double> A; // nonlinear transition function; only used with quaternions
        Vector<double> H; // nonlinear measurement function
        double smallGamma = 0.0001;
        double sigmaA, sigmaAlpha; // noise intensity for linear and angular nth order motion before resetting the filter
        double logLikelihood = 0, mse = 0;
        double angleScale = 1; // scale of the angular measurements. 1000 = milliradians, .001 = kiloradians
        bool useQuaternions = true;
        uint order = 2; // the order of the motion of the filter. 0 = constant, 1 = velocity, 2 = acceleration, ...
        int numLinearStates;
        int numAngularStates;
        int numMeasurements;

        // For testing validity of measurement.
        bool isInGate = true;
        double gateSize = 40;
        
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

        double R; // measurement noise intensity
        Matrix<double> rho;
        int numEdges;

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

        public RigidPatternTracker(int numMarkers, double sigmaAccel, double sigmaAngAccel,
            double sigmaMeasure, double smallGammaThreshold = 0.0001, bool useQuaternions = true, int order = 2)
        {
            if (order < 0)
            {
                order = 2;
            }
            this.order = (uint)order;
            numMeasurements = 3 * (numMarkers + 1);
            numEdges = numMarkers - 1;
            edges = new DenseMatrix(3, numEdges);
            H = new DenseVector(numMeasurements);
            smallGamma = smallGammaThreshold;
            sigmaA = sigmaAccel;
            sigmaAlpha = sigmaAngAccel;
            R = sigmaMeasure;
            numLinearStates = 3 * (order + 1);
            numAngularStates = numLinearStates;
            if (useQuaternions)
            {
                numAngularStates += 1;
            }
            state = new KalmanFilter(numLinearStates + numAngularStates, numMeasurements, 0);
            rho = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(3);
            if (useQuaternions)
            {
                state.transitionFunction = transitionFunction;
            }
            state.measurementFunction = measurementFunction;
            calcStationary();
            this.useQuaternions = useQuaternions;
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
                edges[0, i - 1] = pattern[i].getX() - pattern[0].getX();
                edges[1, i - 1] = pattern[i].getY() - pattern[0].getY();
                edges[2, i - 1] = pattern[i].getZ() - pattern[0].getZ();
            }
            if (useQuaternions)
            {
                state.x[numLinearStates] = 1;
            }
        }

        public override void step(System.Collections.ObjectModel.ReadOnlyCollection<Vector> positions, Vector gammaHat, double dt)
        {
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
            calcNonstationaryC(positions, dt, !gammaHat.isInvalid());
            logLikelihood = state.getLogLikelihood();
            mse = Math.Sqrt(Geometry.getSumSquareError(positions, getPositions().AsReadOnly(), false)) / 2;
            isInGate = testInGate(positions, gateSize, 2.5);
            if (!isInGate && numGatesMissed < maxGatesMissed)
            {
                //calcNonstationaryC(Enumerable.Repeat<Vector>(new Vector(true), positions.Count).ToList().AsReadOnly(), dt, true);
                state.x.SetSubVector(0, state.x.Count, state.xHat);
                state.P.SetSubMatrix(0, 0, state.PHat);
                ++numGatesMissed;
            }
            else
            {
                numGatesMissed = 0;
                state.correct();
            }
            // normalize orientation quaternion
            if(useQuaternions)
            {
                double norm = 0;
                for(int i = 0; i < 4; ++i)
                {
                    norm += state.x[i + numLinearStates] * state.x[i + numLinearStates];
                }
                norm = Math.Sqrt(norm);
                for (int i = 0; i < 4; ++i)
                {
                    state.x[i + numLinearStates] /= norm;
                }
            }
            if (false)
            {
                using (System.IO.StreamWriter writer
                            = new System.IO.StreamWriter("RigidPatternTrackerKFState.csv", true))
                {
                    String stateString = "" + state.x[0];
                    for (int i = 1; i < state.x.Count; ++i)
                    {
                        stateString += "\t" + state.x[i];
                    }
                    writer.WriteLine(stateString);
                }
            }
        }

        private DenseVector transitionFunction()
        {
            return (DenseVector)A;
        }

        private DenseVector measurementFunction()
        {
            return (DenseVector)H;
        }

        private void calcStationary()
        {
            state.R = (Matrix)(CreateMatrix.DenseIdentity<double>(state.z.Count) * R);
        }

        /// <summary>
        /// use a posteriori estimates
        /// </summary>
        /// <param name="dt"></param>
        private void calcNonstationaryP(double dt)
        {
            //compute F(t,k)
            if (Math.Abs(state.x[numLinearStates]) < .001)
            {
                System.Console.Write("");
            }
            // generate linearized transition matrices
            state.A.SetSubMatrix(0, 0, KFUtility.calculateTransitionMatrix(order, 3, dt));
            if (useQuaternions)
            {
                Quaternion lambda = new Quaternion(state.x[numLinearStates],
                    state.x[numLinearStates + 1],
                    state.x[numLinearStates + 2],
                    state.x[numLinearStates + 3]);
                Vector gamma = new Vector();
                double ddt = 1;
                for (int i = 0; i < order; ++i)
                {
                    ddt = ddt * dt / (i + 1);
                    gamma = gamma + new Vector(state.x[numLinearStates + 4 + 3 * i],
                        state.x[numLinearStates + 5 + 3 * i],
                        state.x[numLinearStates + 6 + 3 * i]) * ddt;
                }
                Quaternion qGamma = new Quaternion(gamma);
                state.A.SetSubMatrix(numLinearStates, numLinearStates, QuaternionUtility.rightDeriv(qGamma));
                Matrix<double> dqlambdaDgamma = QuaternionUtility.leftDeriv(lambda) * QuaternionUtility.quaternionizationDeriv(gamma, smallGamma);
                ddt = 1;
                for (int i = 0; i < order; ++i)
                {
                    ddt = ddt * dt / (i + 1);
                    state.A.SetSubMatrix(numLinearStates, numLinearStates + 4 + 3 * i, dqlambdaDgamma * ddt);
                }
                if (order > 0)
                {
                    state.A.SetSubMatrix(numLinearStates + 4, numLinearStates + 4, KFUtility.calculateTransitionMatrix(order - 1, 3, dt));
                }
                A = state.A * state.x;
                A.SetSubVector(numLinearStates, 4, QuaternionUtility.toMathNetVector(qGamma * lambda));
            }
            else
            {
                var gamma = calcRho(false);
                state.A.SetSubMatrix(numLinearStates, numLinearStates, KFUtility.calculateTransitionMatrix(order, 3, dt));
            }
            state.setA();
            //generate process noise covariances
            state.Q.SetSubMatrix(0, 0, KFUtility.calculateProcessNoiseMatrix(order, 3, dt, sigmaA));
            if (useQuaternions)
            {
                Quaternion lambda = new Quaternion(state.x[numLinearStates],
                    state.x[numLinearStates + 1],
                    state.x[numLinearStates + 2],
                    state.x[numLinearStates + 3]);
                Matrix<double> W = new DenseMatrix((int) (3 * (order + 1) + 1), 3);
                double ddt = Math.Sqrt(sigmaAlpha);
                for (int i = 0; i < order; ++i)
                {
                    ddt = ddt * dt / (i + 1);
                    W.SetSubMatrix(W.RowCount - 3 * (i + 1), 0, I3 * ddt);
                }
                ddt = ddt * dt / (order + 1);
                W.SetSubMatrix(0, 0, QuaternionUtility.quaternionizationDeriv(lambda) * I3 * ddt);
                state.Q.SetSubMatrix(numLinearStates, numLinearStates, W.TransposeAndMultiply(W));
                //state.Q.SetSubMatrix(13, 13, KFUtility.calculateProcessNoiseMatrix(1, 3, dt, sigmaAlpha));
            }
            else
            {
                state.Q.SetSubMatrix(numLinearStates, numLinearStates, KFUtility.calculateProcessNoiseMatrix(order, 3, dt, sigmaAlpha));
            }
            state.setQ();
        }

        /// <summary>
        /// Use a priori estimates
        /// </summary>
        /// <param name="measurements"></param>
        /// <param name="dt"></param>
        /// <param name="omegaMeasured"></param>
        private void calcNonstationaryC(System.Collections.ObjectModel.ReadOnlyCollection<Vector> measurements,
            double dt, bool omegaMeasured)
        {

            if (Math.Abs(state.x[numLinearStates]) < .001)
            {
                System.Console.Write("");
            }
            // generate measurement matrix
            H.SetSubVector(0, 3, measurements[0].isInvalid() ? new DenseVector(3) : state.xHat.SubVector(0,3));
            state.H.SetSubMatrix(0, 0, measurements[0].isInvalid() ? I3 * 0 : I3);
            //double mag = gamma.Item1.getMagnitude();
            if (useQuaternions)
            {
                Quaternion lambda = new Quaternion(state.xHat[numLinearStates],
                    state.xHat[numLinearStates + 1],
                    state.xHat[numLinearStates + 2],
                    state.xHat[numLinearStates + 3]);
                for (int i = 0; i < numEdges; ++i)
                {
                    if (measurements[i + 1].isInvalid())
                    {
                        H.SetSubVector(3 + 3 * i, 3, new DenseVector(3));
                        state.H.SetSubMatrix(3 + 3 * i, 0, new DenseMatrix(3));
                        state.H.SetSubMatrix(3 + 3 * i, numLinearStates, new DenseMatrix(3,4));
                    }
                    else
                    {
                        Quaternion edge = new Quaternion(0, edges[0, i], edges[1, i], edges[2, i]);
                        Matrix<double> deriv = QuaternionUtility.conjugateProductOuterDeriv(lambda, edge);
                        // we are only measuring the imaginary part of q.
                        deriv = deriv.SubMatrix(1, 3, 0, 4);
                        Vector newEdge = QuaternionUtility.conjugateProduct(lambda, edge).Vector;
                        H.SetSubVector(3 + 3 * i, 3, state.xHat.SubVector(0, 3) + new DenseVector(newEdge.getVector()));
                        state.H.SetSubMatrix(3 + 3 * i, 0, I3);
                        state.H.SetSubMatrix(3 + 3 * i, numLinearStates, deriv);
                    }
                }
            }
            else
            {
                var lambda = calcRho(true);
                for (int i = 0; i < numEdges; ++i)
                {
                    if (measurements[i + 1].isInvalid())
                    {
                        H.SetSubVector(3 + 3 * i, 3, new DenseVector(3));
                        state.H.SetSubMatrix(3 + 3 * i, 0, new DenseMatrix(3));
                        state.H.SetSubMatrix(3 + 3 * i, numLinearStates, new DenseMatrix(3));
                    }

                    else
                    {
                        Vector<double> edge = new DenseVector(3);
                        edge[0] = edges[0, i];
                        edge[1] = edges[1, i];
                        edge[2] = edges[2, i]; //new DenseVector(edges.SubMatrix(0, 3, i, 1).ToColumnWiseArray());
                        H.SetSubVector(3 + 3 * i, 3, state.xHat.SubVector(0, 3) + rho * edge);
                        state.H.SetSubMatrix(3 + 3 * i, 0, I3);
                        Matrix<double> C_i = PatternTracker.calcC_i(
                            new Vector(edges[0, i], edges[1, i], edges[2, i]),
                            lambda.Item1,
                            lambda.Item2,
                            smallGamma);
                        state.H.SetSubMatrix(3 + 3 * i, numLinearStates, C_i / angleScale);

                    }
                }
            }
            if (order >= 1)
            {
                if (omegaMeasured)
                {
                    if (useQuaternions)
                    {
                        H.SetSubVector(3 * (numEdges + 1), 3, state.xHat.SubVector(numLinearStates + 4, 3) * (dt / angleScale));
                        state.H.SetSubMatrix(3 * (numEdges + 1), numLinearStates + 4, I3 * (dt / angleScale));
                        //state.H.SetSubMatrix(3 * (numEdges + 1), state.x.Count - 3, (dt * dt / 2) * I3);
                    }
                    else
                    {
                        H.SetSubVector(3 * (numEdges + 1), 3, state.xHat.SubVector(numLinearStates + 3, 3) * (dt / angleScale));
                        state.H.SetSubMatrix(3 * (numEdges + 1), numLinearStates + 3, I3 * (dt / angleScale));
                        //state.H.SetSubMatrix(3 * (numEdges + 1), state.x.Count - 3, (dt * dt / 2) * I3);
                    }
                }
                else
                {   //rotation not measured
                    H.SetSubVector(3 * (numEdges + 1), 3, new DenseVector(3));
                    state.H.SetSubMatrix(3 * (numEdges + 1), numLinearStates + (useQuaternions ? 4 : 3), new DenseMatrix(3, 3));
                }
            }
            state.setH();
            state.setR();
        }

        private Tuple<Vector, Matrix<double>> calcRho(bool useAPriori)
        {
            Vector gamma;
            if (useAPriori)
            {
                gamma = new Vector(state.xHat[numLinearStates],
                    state.xHat[numLinearStates + 1],
                    state.xHat[numLinearStates + 2]) / angleScale;
            }
            else
            {
                gamma = new Vector(state.x[numLinearStates],
                    state.x[numLinearStates + 1],
                    state.x[numLinearStates + 2]) / angleScale;
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

        public Quaternion getOrientation(bool aPosteriori = true)
        {
            if(useQuaternions)
            {
                return new Quaternion(state.x[numLinearStates],
                    state.x[numLinearStates + 1],
                    state.x[numLinearStates + 2],
                    state.x[numLinearStates + 3]);
            }
            else
            {
                return new Quaternion(new Vector(state.x[numLinearStates],
                    state.x[numLinearStates + 1],
                    state.x[numLinearStates + 2]));
            }
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
            if (useQuaternions)
            {
                Quaternion lambda;
                if (aPosteriori)
                {
                    lambda = new Quaternion(state.x[numLinearStates],
                        state.x[numLinearStates + 1],
                        state.x[numLinearStates + 2],
                        state.x[numLinearStates + 3]);
                }
                else
                {
                    lambda = new Quaternion(state.xHat[numLinearStates],
                        state.xHat[numLinearStates + 1],
                        state.xHat[numLinearStates + 2],
                        state.xHat[numLinearStates + 3]);
                }
                for (int i = 0; i < numEdges; ++i)
                {
                    Quaternion edge = new Quaternion(0, edges[0, i], edges[1, i], edges[2, i]);
                    positions[i + 1] = QuaternionUtility.conjugateProduct(lambda, edge).Vector;
                    positions[i + 1] += positions[0];
                }
            }
            else
            {
                calcRho(!aPosteriori);
                Matrix<double> rotatedEdges = rho * edges;
                for (int i = 0; i < numEdges; ++i)
                {
                    positions[i + 1] = new Vector(rotatedEdges[0, i], rotatedEdges[1, i], rotatedEdges[2, i]);
                    positions[i + 1] += positions[0];
                }
            }
            return positions;
        }

        public override Vector getAngularVelocity()
        {
            if (order >= 1)
            {
                if (useQuaternions)
                {
                    return new Vector(state.x[numLinearStates + 4],
                        state.x[numLinearStates + 5],
                        state.x[numLinearStates + 6]);
                }
                else
                {
                    return new Vector(state.x[numLinearStates + 3],
                        state.x[numLinearStates + 4],
                        state.x[numLinearStates + 5]);
                }
            }
            else
            {
                return new Vector(true);
            }
        }

        public override Vector getAngularAcceleration()
        {
            if (order >= 2)
            {
                if (useQuaternions)
                {
                    return new Vector(state.x[numLinearStates + 7],
                        state.x[numLinearStates + 8],
                        state.x[numLinearStates + 9]);
                }
                else
                {
                    return new Vector(state.x[numLinearStates + 6],
                        state.x[numLinearStates + 7],
                        state.x[numLinearStates + 8]);
                }
            }
            else
            {
                return new Vector(true);
            }
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
            return "l = <" + state.x[numLinearStates].ToString("0.##") + "\t"
                + state.x[numLinearStates + 1].ToString("0.##") + "\t"
                + state.x[numLinearStates + 2].ToString("0.##")
                + (useQuaternions ? "\t" + state.x[numLinearStates + 3].ToString("0.##") : "")
                + ">" + "\r\n"
                + "k(P) = " + state.P.ConditionNumber().ToString("0.##") + "\r\n"
                + "k(S) = " + (state.H * state.PHat.TransposeAndMultiply(state.H) + state.R).ConditionNumber().ToString("0.##") + "\r\n"
                + "smallest eigenvalue(PHat) = " + minimum2.ToString() + "\r\n"
                + "smallest eigenvalue(P) = " + minimum.ToString() + "\r\n"
                + "trace(PHat) = " + state.PHat.Trace().ToString("0.##") + "\r\n"
                + "trace(P) = " + state.P.Trace().ToString("0.##") + "\r\n";
        }

        /// <summary>
        /// Test whether the observed positions are within the expected gate or area of the state.
        /// Should probably be replaced with a proper hypothesis test at some point.
        /// </summary>
        /// <param name="positions"></param>
        /// <param name="maxOriginDeviation"></param>
        /// <param name="maxAngleDeviation"></param>
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
