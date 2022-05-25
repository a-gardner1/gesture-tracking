using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{

    /**
     * Arguably the simplest Kalman filter based tracker for unlabeled hand markers while
     * estimating velocity and acceleration.
     * 
     * Uses a Kalman filter per marker. At each time-step, performs following process:
     * 
     * 1. Predict locations of tracked markers.
     * 2. Assign measured unlabeled markers to predictions.
     * 3. Correct locations based upon assigned measurements. If no measurement assigned, 
     * use prediction.
     * 
     * */
    class SimpleHandTracker
    {
        private List<SimpleMarkerTracker> trackers = new List<SimpleMarkerTracker>();
        private List<Vector> latestPredictions = new List<Vector>();

        public SimpleHandTracker(int numMarkers, double Q, double R)
        {
            for(int i = 0; i < numMarkers; ++i)
            {
                trackers.Add(new SimpleMarkerTracker(Q, R, true));
                latestPredictions.Add(new Vector());
            }
        }

        public void init()
        {
            foreach (SimpleMarkerTracker smt in trackers)
            {
                smt.init(new Vector());
            }
        }

        public void predict(double dt)
        {
            for(int i = 0; i < trackers.Count; ++i) 
            {
                latestPredictions[i] = trackers[i].predict(dt);
            }
        }

        public void correct(List<Vector> observed)
        {
            // assign predictions to measurements
            DenseMatrix distance = new DenseMatrix(observed.Count, trackers.Count);
            for (int i = 0; i < observed.Count; ++i)
            {
                for (int j = 0; j < trackers.Count; ++j)
                {
                    distance[i, j] = (observed[i] - latestPredictions[j]).getMagnitude();
                }
            }
            // could be pre-allocated. size never changes
            List<int> assignment = Assignment.getAssignment(distance);
            for(int t = 0; t < trackers.Count; ++t)
            {
                if (assignment[t] != -1)
                {
                    trackers[t].correct(observed[assignment[t]]);
                }
                else
                {
                    // give it an invalid vector
                    trackers[t].correct(new Vector(true));
                }
            }
        }

        public List<Vector> getEstimatedState()
        {
            List<Vector> estimate = new List<Vector>();
            for (int i = 0; i < trackers.Count; ++i )
            {
                estimate.Add(trackers[i].getPosition());
            }
            return estimate;
        }
    }
}
