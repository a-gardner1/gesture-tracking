using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GestureRecognition
{
    class ExponentialSmootherPatternTracker : PatternTracker
    {
        double alphaP, alphaV, alphaA;
        Vector position, velocity, acceleration; // position, velocity, acceleration
        Vector orientation, omega, alpha; // orientation, angular velocity, angular acceleration
        double stepSize;

        public override bool IsInGate
        {
            get { throw new NotImplementedException(); }
        }

        public override double MeanError
        {
            get { throw new NotImplementedException(); }
        }

        public override double LogLikelihood
        {
            get { throw new NotImplementedException(); }
        }

        public override double SigmaAlpha
        {
            get
            {
                throw new NotImplementedException();
            }
            set
            {
                throw new NotImplementedException();
            }
        }

        public override double SigmaA
        {
            get
            {
                throw new NotImplementedException();
            }
            set
            {
                throw new NotImplementedException();
            }
        }

        public override double SigmaVicon
        {
            get
            {
                throw new NotImplementedException();
            }
            set
            {
                throw new NotImplementedException();
            }
        }

        public override void saveState()
        {
            throw new NotImplementedException();
        }

        public override void revertState()
        {
            throw new NotImplementedException();
        }

        public override void init(System.Collections.ObjectModel.ReadOnlyCollection<Vector> completePattern)
        {
            throw new NotImplementedException();
        }

        public override void step(System.Collections.ObjectModel.ReadOnlyCollection<Vector> positions, Vector gammaHat, double dt)
        {
            //calculate prediction
            double numSteps = dt / stepSize;
            Vector predictedVelocity = velocity + acceleration * numSteps;
            Vector prediction = position + velocity * numSteps;
            // calculate correction
            if(!positions[0].isInvalid()) {
                position = alphaP * positions[0] + (1 - alphaP) * prediction;
            }
            throw new NotImplementedException();
        }

        public override List<Vector> getPositions(bool aPosteriori = true)
        {
            throw new NotImplementedException();
        }

        public override Vector getAngularVelocity()
        {
            throw new NotImplementedException();
        }

        public override Vector getAngularAcceleration()
        {
            throw new NotImplementedException();
        }

        public override string checkForNumericalIssues()
        {
            throw new NotImplementedException();
        }
    }
}
