using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    abstract class PatternTracker
    {

        protected static Matrix<double> I3 = CreateMatrix.DenseIdentity<double>(3);
        protected int numGatesMissed = 0; // the number of consecutive times that the observation has been outside the expected area
        protected int maxGatesMissed = 30; // the maximum number of consecutive times that the observation can be outside the expected area before resetting the filter
        

        public abstract bool IsInGate
        {
            get;
        }

        public int NumGatesMissed
        {
            get
            {
                return numGatesMissed;
            }
        }

        public int MaxGatesMissed
        {
            get
            {
                return maxGatesMissed;
            }
            set
            {
                maxGatesMissed = value;
            }
        }

        public abstract double MeanError
        {
            get;
        }

        public abstract double LogLikelihood
        {
            get;
        }

        public abstract double SigmaAlpha
        {
            get;
            set;
        }

        public abstract double SigmaA
        {
            get;
            set;
        }

        public abstract double SigmaVicon
        {
            get;
            set;
        }

        public abstract void saveState();

        public abstract void revertState();

        public abstract void init(ReadOnlyCollection<Vector> completePattern);

        public abstract void step(ReadOnlyCollection<Vector> positions, Vector gammaHat, double dt);

        public abstract List<Vector> getPositions(bool aPosteriori = true);

        public abstract Vector getAngularVelocity();

        public abstract Vector getAngularAcceleration();

        public abstract string checkForNumericalIssues();

        protected static Matrix<double> calcC_i(Vector e_i, Vector gamma, Matrix<double> Gamma, double smallGamma)
        {
            double mag = gamma.getMagnitude();
            var C_i = -Geometry.getCrossProductMatrix(e_i);
            Vector<double> e = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(e_i.getVector());
            Vector<double> gam = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(gamma.getVector());
            if (mag < smallGamma)
            {
                C_i = C_i
                    - Gamma * e.OuterProduct(gam) / 3
                    + (gam * e * I3 + gam.OuterProduct(e) - 2 * e.OuterProduct(gam)) / 2
                    - Gamma * Gamma * e.OuterProduct(gam) / 12;
            }
            else
            {
                double mag2 = mag * mag;
                C_i = C_i * (Math.Sin(mag) / mag)
                    + Gamma * e.OuterProduct(gam) * ((mag * Math.Cos(mag) - Math.Sin(mag)) / (mag * mag2))
                    + (gam * e * I3 + gam.OuterProduct(e) - 2 * e.OuterProduct(gam)) * ((1 - Math.Cos(mag)) / (mag2))
                    + Gamma * Gamma * e.OuterProduct(gam) * ((mag * Math.Sin(mag) + 2 * Math.Cos(mag) - 2) / (mag2 * mag2));
            }
            return C_i;
        }
    }
}
