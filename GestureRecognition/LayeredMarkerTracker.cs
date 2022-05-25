using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    class LayeredMarkerTracker
    {
        NthOrderFilter positionTracker;
        NthOrderFilter motionTracker;
        uint order = 2;

        public LayeredMarkerTracker(int order, double processNoiseMagnitude, double measurementNoiseMagnitude)
        {
            if(order < 1)
            {
                order = 2;
            }
            this.order = (uint)order;
            positionTracker = new NthOrderFilter(false, 0, processNoiseMagnitude, measurementNoiseMagnitude);
            motionTracker = new NthOrderFilter(false, order, processNoiseMagnitude, measurementNoiseMagnitude);
        }

        public void init(Vector initial)
        {
            positionTracker.init(new Quaternion(0,initial));
            motionTracker.init(new Quaternion(0, initial));
        }

        /// <summary>
        /// Give the primary measurement (a position) to the position tracker.
        /// </summary>
        /// <param name="measurement"></param>
        /// <param name="dt"></param>
        public void stepP(Vector measurement, double dt)
        {
            positionTracker.step(new Quaternion(0, measurement), dt);
            motionTracker.step(positionTracker.getNthOrderState(0), dt);
        }

        /// <summary>
        /// Give the primary measurement (a velocity) to the motion tracker.
        /// </summary>
        /// <param name="measurement"></param>
        /// <param name="dt"></param>
        public void stepV(Vector measurement, double dt)
        {
            motionTracker.stepN(new DenseVector(measurement.getVector()), dt, 1);
            positionTracker.step(positionTracker.getNthOrderState(0), dt);
        }


        /**
         * At some point should replace return with read-only version or copy.
         * */
        public MathNet.Numerics.LinearAlgebra.Vector<double> getState()
        {
            return motionTracker.getEntireState();
        }

        public Vector getPosition()
        {
            Vector<double> p = motionTracker.getNthOrderState(0);
            return new Vector(p[0], p[1], p[2]);
        }
        public Vector getVelocity()
        {
            Vector<double> p = motionTracker.getNthOrderState(1);
            return new Vector(p[0], p[1], p[2]);
        }
        public Vector getAcceleration()
        {
            if (order > 1)
            {
                Vector<double> p = motionTracker.getNthOrderState(2);
                return new Vector(p[0], p[1], p[2]);
            }
            else
            {
                return new Vector(true);
            }
        }
    }
}
