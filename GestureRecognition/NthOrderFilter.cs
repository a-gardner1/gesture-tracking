using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    class NthOrderFilter
    {
        protected KalmanFilter state;
        private bool useQuaternions;
        private uint order;
        private uint numDimensions;
        double logLikelihood = 0;

        public double LogLikelihood
        {
            get { return logLikelihood; }
        }

        public uint Order
        {
            get { return order; }
        }

        public uint NumDimensions
        {
            get { return numDimensions; }
        }

        /// <summary>
        /// Use with caution.
        /// </summary>
        public KalmanFilter Filter
        {
            get { return state; }
        }

        int numStates;
        Vector<double> A;
        Matrix<double> I3 = CreateMatrix.DenseIdentity<double>(3),
            I4 = CreateMatrix.DenseIdentity<double>(4),
            Z3 = new SparseMatrix(3),
            Z4 = new SparseMatrix(4);
        double Q, R;

        public double ProcessNoiseMagnitude
        {
            get { return Q; }
            set { Q = value; }
        }
        public double MeasurementNoiseMagnitude
        {
            get { return R; }
            set { R = value; }
        }

        private Vector<double> savedX;
        private Matrix<double> savedP;

        public void saveState()
        {
            savedX = state.x.Clone();
            savedP = state.P.Clone();
        }

        public void revertState()
        {
            if (savedX == null)
            {
                throw new Exception("No prior state exists, cannot revert!");
            }
            state.x.SetSubVector(0, state.x.Count, savedX);
            state.P.SetSubMatrix(0, 0, savedP);
        }

        public NthOrderFilter(bool useQuaternions, int order, double processNoiseMagnitude, double measurementNoiseMagnitude, int numDimensions = 3)
        {
            if(order < 0)
            {
                order = 2;
            }
            if (numDimensions < 0 || useQuaternions)
            {
                numDimensions = 3;
            } 
            this.order = (uint)order;
            this.useQuaternions = useQuaternions;
            this.numDimensions = (uint)numDimensions;
            numStates = numDimensions * (1 + order);
            if(useQuaternions)
            {
                numStates += 1;
            }
            state = new KalmanFilter(numStates, useQuaternions ? 4 : numDimensions, 0);
            if (useQuaternions)
            {
                state.transitionFunction = transitionFunction;
            }
            ProcessNoiseMagnitude = processNoiseMagnitude;
            MeasurementNoiseMagnitude = measurementNoiseMagnitude;
        }

        public void init(Quaternion firstMeasurement)
        {
            if (useQuaternions)
            {
                state.x.SetSubVector(0, 4, QuaternionUtility.toMathNetVector(firstMeasurement));
                if(firstMeasurement.Vector.isInvalid())
                {
                    state.x[0] = 1;
                }
            }
            else if(numDimensions == 3)
            {
                state.x.SetSubVector(0, (int)numDimensions, new DenseVector(firstMeasurement.Vector.getVector()));
            }
            else
            {
                throw new Exception("One can only initialize with a quaternion if useQuaternions is true or if the number of dimensions is equal to 3.");
            }
            calcStationary();
        }

        public void init(Vector<double> firstMeasurement, bool isInvalid = false)
        {
            if (useQuaternions)
            {
                init(new Quaternion(firstMeasurement[0], 
                    isInvalid ? new Vector(true) : new Vector(firstMeasurement[1], firstMeasurement[2], firstMeasurement[3])));
                return;
            }
            else
            {
                state.x.SetSubVector(0, (int)numDimensions, firstMeasurement);
            }
            calcStationary();
        }

        private DenseVector transitionFunction()
        {
            return (DenseVector)A;
        }


        /// <summary>
        /// Embed vectors in a quaternion if not using Quaternions.
        /// Lazy way.
        /// </summary>
        /// <param name="measurement"></param>
        /// <param name="dt"></param>
        public void step(Quaternion measurement, double dt)
        {
            step(useQuaternions ? 
                QuaternionUtility.toMathNetVector(measurement) : new DenseVector(measurement.Vector.getVector()), 
                dt, measurement.Vector.isInvalid());
            if(useQuaternions)
            {
                double norm = 0;
                for (int i = 0; i < 4; ++i)
                {
                    norm += state.x[i] * state.x[i];
                }
                norm = Math.Sqrt(norm);
                for (int i = 0; i < 4; ++i)
                {
                    state.x[i] /= norm;
                }
            }
        }

        public void step(Vector<double> measurement, double dt, bool isInvalid = false)
        {
            calcNonstationaryP(dt);
            state.predict();
            if (isInvalid)
            {
                state.z.Clear();
            }
            else
            {
                state.z.SetSubVector(0, measurement.Count, measurement);
            }
            state.setZ();
            calcNonstationaryC(!isInvalid, dt);
            logLikelihood = state.getLogLikelihood();
            state.correct();
        }

        /// <summary>
        /// Step off of an N-th order measurement
        /// </summary>
        /// <param name="measurement"></param>
        /// <param name="dt"></param>
        public void stepN(Vector<double> measurement, double dt, int N)
        {
            if (N < 1)
            {
                throw new ArgumentException("The order of the measurement must be positive. Given order: " + N + ".");
            }
            calcNonstationaryP(dt);
            state.predict();
            state.z.SetSubVector(0, measurement.Count, measurement);
            state.setZ();
            calcNonstationaryN(N, dt);
            logLikelihood = state.getLogLikelihood();
            state.correct();
        }

        private void calcStationary()
        {
            state.R = (Matrix)(CreateMatrix.DenseIdentity<double>(state.z.Count) * R);
        }

        private void calcNonstationaryP(double dt)
        {
            if(useQuaternions)
            {
                Quaternion lambda = new Quaternion(state.x[0],
                    state.x[1],
                    state.x[2],
                    state.x[3]);
                Vector gamma = new Vector();
                double ddt = 1;
                for (int i = 0; i < order; ++i)
                {
                    ddt = ddt * dt / (i + 1);
                    gamma = gamma + new Vector(state.x[4 + 3 * i],
                        state.x[5 + 3 * i],
                        state.x[6 + 3 * i]) * ddt;
                }
                Quaternion qGamma = new Quaternion(gamma);
                state.A.SetSubMatrix(0, 0, QuaternionUtility.rightDeriv(qGamma));
                Matrix<double> dqlambdaDgamma = QuaternionUtility.leftDeriv(lambda) * QuaternionUtility.quaternionizationDeriv(gamma, .0001);
                ddt = 1;
                for (int i = 0; i < order; ++i)
                {
                    ddt = ddt * dt / (i + 1);
                    state.A.SetSubMatrix(0,  4 + 3 * i, dqlambdaDgamma * ddt);
                }
                if (order > 0)
                {
                    state.A.SetSubMatrix(4,  4, KFUtility.calculateTransitionMatrix(order - 1, 3, dt));
                }
                A = state.A * state.x;
                A.SetSubVector(0, 4, QuaternionUtility.toMathNetVector(qGamma * lambda));
                state.A.SetSubMatrix(4, 4, KFUtility.calculateTransitionMatrix(order - 1, 3, dt));
            }
            else
            {
                state.A.SetSubMatrix(0, 0, KFUtility.calculateTransitionMatrix(order, numDimensions, dt));
            }
            state.setA();
            //generate process noise covariances
            if (useQuaternions)
            {
                Quaternion lambda = new Quaternion(state.x[0],
                     state.x[1],
                     state.x[2],
                     state.x[3]);
                Matrix<double> W = new DenseMatrix((int)(3 * (order + 1) + 1), 3);
                double ddt = Math.Sqrt(Q);
                for (int i = 0; i < order; ++i)
                {
                    ddt = ddt * dt / (i + 1);
                    W.SetSubMatrix(W.RowCount - 3 * (i + 1), 0, I3 * ddt);
                }
                ddt = ddt * dt / (order + 1);
                W.SetSubMatrix(0, 0, QuaternionUtility.quaternionizationDeriv(lambda) * I3 * ddt);
                state.Q.SetSubMatrix(0, 0, W.TransposeAndMultiply(W));
            }
            else
            {
                state.Q.SetSubMatrix(0, 0, KFUtility.calculateProcessNoiseMatrix(order, numDimensions, dt, Q));
            }
            state.setQ();
        }

        private void calcNonstationaryC(bool measured, double dt)
        {
            if (numDimensions == 3)
            {
                state.H.SetSubMatrix(0, 0, measured ? (useQuaternions ? I4 : I3) : (useQuaternions ? Z4 : Z3));
            }
            else
            {
                state.H.SetSubMatrix(0, 0, measured ? CreateMatrix.DenseIdentity<double>((int)numDimensions) : new SparseMatrix((int)numDimensions));
            }
            state.setH();
            state.setR();
        }

        private void calcNonstationaryN(int N, double dt)
        {
            if (numDimensions == 3)
            {
                state.H.SetSubMatrix(0, 3 * N + (useQuaternions ? 1 : 0), I3);
            }
            else
            {
                state.H.SetSubMatrix(0, (int)numDimensions * N, CreateMatrix.SparseIdentity<double>((int)numDimensions));
            }
            state.setH();
            state.setR();
        }

        public DenseVector getNthOrderState(uint n, bool aPosteriori = true)
        {
            Vector<double> x;
            if(aPosteriori) {
                x = state.x;
            }
            else{
                x = state.xHat;
            }
            if (useQuaternions)
            {
                return (DenseVector)x.SubVector((n == 0) ? 0 : (int)(4 + (n - 1) * 3), (n == 0) ? 4 : 3);
            }
            else
            {
                return (DenseVector)x.SubVector((int)(n * numDimensions), (int)numDimensions);
            }
        }

        public Vector<double> getEntireState(bool aPosteriori = true)
        {
            return aPosteriori ? state.x.Clone() : state.xHat.Clone();
        }

    }
}
