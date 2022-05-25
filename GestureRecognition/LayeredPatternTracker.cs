using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    /// <summary>
    /// Use a rigid pattern tracker and two nth order filters to refine its results.
    /// 
    /// TODO: Make in-gate decisions based upon predictions of the Nth order filters rather than the rigid pattern tracker.
    /// Will require controlling innards of each filter and cause tighter coupling :(.
    /// </summary>
    class LayeredPatternTracker : PatternTracker
    {
        RigidPatternTracker rpt;
        NthOrderFilter linearFilter;
        NthOrderFilter angularFilter;
        double mse;
        bool isInGate;

        #region get/set
        public override bool IsInGate
        {
            get { return rpt.IsInGate; } // temporary solution
        }

        public override double MeanError
        {
            get { return mse; }
        }

        public override double LogLikelihood
        {
            get { return linearFilter.LogLikelihood + angularFilter.LogLikelihood; }
        }

        public override double SigmaAlpha
        {
            get
            {
                return rpt.SigmaAlpha;
            }
            set
            {
                rpt.SigmaAlpha = value;
                angularFilter.ProcessNoiseMagnitude = value;
            }
        }

        public override double SigmaA
        {
            get
            {
                return rpt.SigmaA;
            }
            set
            {
                rpt.SigmaA = value;
                linearFilter.ProcessNoiseMagnitude = value;
            }
        }

        public override double SigmaVicon
        {
            get
            {
                return rpt.SigmaVicon;
            }
            set
            {
                rpt.SigmaVicon = value;
                linearFilter.MeasurementNoiseMagnitude = value;
                angularFilter.MeasurementNoiseMagnitude = value;
            }
        }
        #endregion

        public override void saveState()
        {
            rpt.saveState();
            linearFilter.saveState();
            angularFilter.saveState();
        }

        public override void revertState()
        {
            rpt.revertState();
            linearFilter.revertState();
            angularFilter.revertState();
        }

        public LayeredPatternTracker(int numMarkers, double sigmaAccel, double sigmaAngAccel,
            double sigmaMeasure, bool useQuaternions, 
            int patternOrder, int linearOrder, int angularOrder)
        {
            rpt = new RigidPatternTracker(numMarkers, sigmaAccel, 
                sigmaAngAccel, sigmaMeasure, 
                .0001, useQuaternions, 
                patternOrder);
            linearFilter = new NthOrderFilter(false, linearOrder, sigmaAccel, sigmaMeasure);
            angularFilter = new NthOrderFilter(true, angularOrder, sigmaAngAccel, sigmaMeasure);
        }

        public override void init(System.Collections.ObjectModel.ReadOnlyCollection<Vector> completePattern)
        {
            rpt.init(completePattern);
            Quaternion position = new Quaternion(0, rpt.getPositions()[0]);
            linearFilter.init(position);
            Quaternion orientation = new Quaternion(rpt.getOrientation());
            angularFilter.init(orientation);
        }

        public override void step(System.Collections.ObjectModel.ReadOnlyCollection<Vector> positions, Vector gammaHat, double dt)
        {
            rpt.step(positions, gammaHat, dt);
            mse = Math.Sqrt(Geometry.getSumSquareError(positions, getPositions().AsReadOnly(), true)) / 2;
            Quaternion position = new Quaternion(0, rpt.getPositions()[0]);
            linearFilter.step(position, dt);
            Quaternion orientation = new Quaternion(rpt.getOrientation());
            angularFilter.step(orientation, dt);
        }

        public override List<Vector> getPositions(bool aPosteriori = true)
        {
            List<Vector> positions = rpt.getPositions();
            //subtract out origin to obtain edges
            for(int i = 1; i < positions.Count; ++i)
            {
                positions[i] -= positions[0];
            }
            // replace origin with filtered origin
            DenseVector position = linearFilter.getNthOrderState(0, aPosteriori);
            positions[0] = new Vector(position[0], position[1], position[2]);
            // replace orientation with filtered orientation
            // compose two rotations. First undo the rotation of rpt, then do the rotation of angularFilter
            DenseVector orientation = angularFilter.getNthOrderState(0, aPosteriori);
            Quaternion lambda = new Quaternion(orientation[0],
                orientation[1],
                orientation[2],
                orientation[3]);
            lambda = lambda * rpt.getOrientation().conjugate(); // compose rotations
            for (int i = 1; i < positions.Count; ++i)
            {
                Quaternion edge = new Quaternion(0, positions[i]);
                positions[i] = QuaternionUtility.conjugateProduct(lambda, edge).Vector;
                positions[i] += positions[0];
            }
            return positions;
        }

        public override Vector getAngularVelocity()
        {
            if(angularFilter.Order > 0)
            {
                DenseVector omega = angularFilter.getNthOrderState(1, true);
                return new Vector(omega[0], omega[1], omega[2]);
            }
            else
            {
                return new Vector(true);
            }
        }

        public override Vector getAngularAcceleration()
        {
            if (angularFilter.Order > 1)
            {
                DenseVector alpha = angularFilter.getNthOrderState(2, true);
                return new Vector(alpha[0], alpha[1], alpha[2]);
            }
            else
            {
                return new Vector(true);
            }
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

        public override string checkForNumericalIssues()
        {
            return rpt.checkForNumericalIssues();
        }
    }
}
