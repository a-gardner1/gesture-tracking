using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{

    /**
     * Arguably the simplest correlated Kalman filter based tracker for unlabeled hand markers while
     * estimating velocity and acceleration.
     * 
     * Uses a Kalman filter per marker. At each time-step, performs following process:
     * 
     * 1. Predict locations of tracked markers.
     * 2. Assign measured unlabeled markers to predictions.
     * 3. Correct locations based upon assigned measurements. If no measurement assigned, 
     * use corrected velocities and accelerations of correlated visible markers to move marker.
     * 4. Update correlation matrix between velocities and accelerations.
     * 
     * */
    class CorrelatedHandTracker
    {
        private List<SimpleMarkerTracker> trackers = new List<SimpleMarkerTracker>();
        //private List<NthOrderFilter[,]> correlationTrackers;
        private List<CovarianceFilter> covarianceFilters;
        //private List<Matrix<double>> covariances;
        private List<Vector> latestPredictions = new List<Vector>();
        //private Matrix<double> vCovariance, aCovariance; //velocity and acceleration covariance
        int numMarkers;
        private int windowSize = 20;
        private List<int> numObservations = new List<int>();
        //private Matrix<double> vWindow, aWindow; // arranged by column
        private long windex = 0; // number of steps taken
        private readonly Vector<double> ONES;
        private double correlationThreshold;
        private Matrix<double> identity6 = CreateMatrix.DiagonalIdentity<double>(6);
        private Matrix<double> correctedH; // the measurement matrix to use when making corrections
        private List<double> gateRadii = new List<double>();

        private double processNoise, measurementNoise;

        public CorrelatedHandTracker(int numMarkers, double Q, double R, int windowSize, double correlationThreshold, int correlationOrder = 1)
        {
            if (correlationOrder < 1 || correlationOrder > 2)
            {
                correlationOrder = 1;
            }
            processNoise = Q;
            measurementNoise = R;
            this.numMarkers = numMarkers;
            covarianceFilters = new List<CovarianceFilter>();
            //correlationTrackers = new List<NthOrderFilter[,]>();
            //covariances = new List<Matrix<double>>();
            for (int o = 0; o < correlationOrder; ++o)
            {
                covarianceFilters.Add(new CovarianceFilter(3 * numMarkers, 100, 100));
                //correlationTrackers.Add(new NthOrderFilter[numMarkers, numMarkers]);
                //covariances.Add(new DenseMatrix(3 * numMarkers));
            }
            for (int i = 0; i < numMarkers; ++i)
            {
                trackers.Add(new SimpleMarkerTracker(Q, R, false));
                latestPredictions.Add(new Vector());
                gateRadii.Add(1000);
                numObservations.Add(0);
                //for (int o = 0; o < correlationOrder; ++o)
                //{
                //    correlationTrackers[o][i, i] = new NthOrderFilter(false, 0, 100, 1, 6);
                //    for (int j = i + 1; j < numMarkers; ++j)
                //    {
                //        correlationTrackers[o][i, j] = new NthOrderFilter(false, 0, 100, 1, 9);
                //        correlationTrackers[o][j, i] = correlationTrackers[0][i, j];
                //    }
                //}
            }
            //vCovariance = new DenseMatrix(3 * numMarkers);
            //aCovariance = new DenseMatrix(3 * numMarkers);
            //vWindow = new DenseMatrix(3 * numMarkers, windowSize);
            //aWindow = new DenseMatrix(3 * numMarkers, windowSize);
            this.windowSize = windowSize;
            ONES = DenseVector.Create(windowSize, 1);
            this.correlationThreshold = correlationThreshold;
            correctedH = new DenseMatrix(3*correlationOrder, 9);
            correctedH.SetSubMatrix(0, 3, CreateMatrix.SparseIdentity<double>(3*correlationOrder));
        }

        /// <summary>
        /// Calculate the covariance of marker velocities and accelerations 
        /// (but not the covariance between any given velocity and acceleration).
        /// </summary>
        private void calculateCovariance(double dt)
        {
            //Vector<double> vLatest = new DenseVector(3 * numMarkers);
            //Vector<double> aLatest = new DenseVector(3 * numMarkers);
            //for (int i = 0; i < trackers.Count; ++i)
            //{
            //    vLatest.SetSubVector(3 * i, 3, trackers[i].getState().SubVector(3, 3));
            //    aLatest.SetSubVector(3 * i, 3, trackers[i].getState().SubVector(6, 3));
            //}
            for (int o = 0; o < covarianceFilters.Count; ++o )
            {
                // calculate current covariance. could potentially be done as a sliding window.
                Vector<double> apparentCovariance = new DenseVector(3 * numMarkers);
                for (int i = 0; i < trackers.Count; ++i)
                {
                    apparentCovariance.SetSubVector(3 * i, 3, trackers[i].getState().SubVector(3*(o+1), 3));
                }
                covarianceFilters[o].step(apparentCovariance.OuterProduct(apparentCovariance), dt);
            }
            //for (int o = 0; o < correlationTrackers.Count; ++o)
            //{
            //    for (int i = 0; i < trackers.Count; ++i)
            //    {
            //        //calculate intra-marker covariance (only unique elements)
            //        Vector<double> a = trackers[i].getState().SubVector(3 * (o + 1), 3);
            //        Matrix<double> aaT = a.OuterProduct(a);
            //        Vector<double> measurement = new DenseVector(6);
            //        measurement[0] = aaT[0, 0];
            //        measurement[1] = aaT[0, 1];
            //        measurement[2] = aaT[1, 1];
            //        measurement[3] = aaT[0, 2];
            //        measurement[4] = aaT[1, 2];
            //        measurement[5] = aaT[2, 2];
            //        correlationTrackers[o][i, i].step(measurement, dt);
            //        measurement = correlationTrackers[o][i, i].getNthOrderState(0);
            //        covariances[o][3 * i, 3 * i] = measurement[0];
            //        covariances[o][3 * i, 3 * i + 1] = measurement[1];
            //        covariances[o][3 * i + 1, 3 * i + 1] = measurement[2];
            //        covariances[o][3 * i + 1, 3 * i + 2] = measurement[3];
            //        covariances[o][3 * i, 3 * i + 2] = measurement[4];
            //        covariances[o][3 * i + 2, 3 * i + 2] = measurement[5];
            //        covariances[o][3 * i + 1, 3 * i] = measurement[1];
            //        covariances[o][3 * i + 2, 3 * i + 1] = measurement[3];
            //        covariances[o][3 * i + 2, 3 * i] = measurement[4];
            //        //calculate inter-marker covariance
            //        for (int j = i + 1; j < trackers.Count; ++j)
            //        {
            //            Vector<double> b = trackers[j].getState().SubVector(3 * (o + 1), 3);
            //            measurement = new DenseVector(a.OuterProduct(b).ToColumnWiseArray());
            //            correlationTrackers[o][i, j].step(measurement, dt);
            //            covariances[o].SetSubMatrix(3 * i, 3 * j, new DenseMatrix(3, 3, correlationTrackers[o][i, j].getNthOrderState(0).ToArray()));
            //            covariances[o].SetSubMatrix(3 * j, 3 * i, covariances[o].SubMatrix(3 * i, 3, 3 * j, 3));
            //        }
            //    }
            //}
               
            //vWindow.SetSubMatrix(0, (int)(windex % windowSize), vLatest.ToColumnMatrix());
            //aWindow.SetSubMatrix(0, (int)(windex % windowSize), aLatest.ToColumnMatrix());
            ////inefficient but correct; estimates always exist regardless of occlusion
            //if (windex >= windowSize)
            //{
            //    Vector<double> mean = vWindow.RowSums() / windowSize;
            //    vCovariance = (vWindow - mean.OuterProduct(ONES));
            //    vCovariance = vCovariance.TransposeAndMultiply(vCovariance);
            //    vCovariance /= windowSize - 1;
            //    mean = aWindow.RowSums() / windowSize;
            //    aCovariance = (vWindow - mean.OuterProduct(ONES));
            //    aCovariance = aCovariance.TransposeAndMultiply(aCovariance);
            //    aCovariance /= windowSize - 1;
            //}
            windex++;
        }

        public void init(List<Vector> initialPositions = null)
        {
            if (initialPositions == null)
            {
                foreach (SimpleMarkerTracker smt in trackers)
                {
                    smt.init(new Vector());
                }
            }
            else
            {
                int i;
                for(i = 0; i < Math.Min(trackers.Count, initialPositions.Count); ++i)
                {
                    trackers[i].init(initialPositions[i]);
                }
                for(; i <trackers.Count; ++i)
                {
                    trackers[i].init(new Vector());
                }
            }
            foreach(CovarianceFilter cf in covarianceFilters)
            {
                cf.init(CreateMatrix.SparseIdentity<double>(3 * numMarkers));
            }
        }

        public void step(List<Vector> observed, double dt)
        {
            predict(dt);
            correct(observed, dt);
        }

        public void predict(double dt)
        {
            for(int i = 0; i < trackers.Count; ++i) 
            {
                latestPredictions[i] = trackers[i].predict(dt);
            }
        }

        public void correct(List<Vector> observed, double dt)
        {
            List<int> assignment = Enumerable.Repeat(-1, trackers.Count).ToList();
            // assign predictions to measurements
            if (observed.Count > 0)
            {
                DenseMatrix distance = new DenseMatrix(observed.Count, trackers.Count);
                for (int i = 0; i < observed.Count; ++i)
                {
                    for (int j = 0; j < trackers.Count; ++j)
                    {
                        distance[i, j] = (observed[i] - latestPredictions[j]).getMagnitude();
                        distance[i, j] = gateRadii[j] < distance[i, j] ? Double.MaxValue : distance[i, j];
                    }
                }
                // could be pre-allocated. size never changes
                assignment = Assignment.getAssignment(distance);
                for (int i = 0; i < trackers.Count; ++i)
                {
                    if (assignment[i] != -1 && distance[assignment[i], i] > gateRadii[i])
                    {
                        assignment[i] = -1;
                    }
                }
            }
            for(int t = 0; t < trackers.Count; ++t)
            {
                if (assignment[t] != -1)
                {
                    trackers[t].correct(observed[assignment[t]]);
                    gateRadii[t] = Math.Max(40, gateRadii[t] / 1.2);
                    ++numObservations[t];
                }
                else
                {
                    gateRadii[t] = Math.Min(1000, gateRadii[t] * 1.2);
                }
            }
            correlatedCorrection(assignment);
            calculateCovariance(dt);
        }

        /// <summary>
        /// Calculates a posteriori estimates of missing markers using 
        /// the latest a posteriori estimates of observed markers. If no
        /// correlated markers are visible, the a priori estimate is used.
        /// </summary>
        /// <param name="assignment"></param>
        private void correlatedCorrection(List<int> assignment)
        {
            List<Matrix<double>> correlations = new List<Matrix<double>>();
            // sums represent total weight of correlated correction per marker component
            List<Vector<double>> sums = new List<Vector<double>>();
            for(int o = 0; o < covarianceFilters.Count; ++o)
            {
                correlations.Add(covarianceFilters[o].Correlation);
                //Vector<double> deviation = covariances[o].Diagonal().PointwisePower(0.5);
                //correlations.Add(covariances[o].PointwiseDivide(deviation.OuterProduct(deviation)));
                sums.Add(new DenseVector(3 * numMarkers));
            }
            //Vector<double> deviations = vCovariance.Diagonal().PointwisePower(0.5);
            //Matrix<double> vCorrelation = vCovariance.PointwiseDivide(deviations.OuterProduct(deviations));
            //deviations = aCovariance.Diagonal().PointwisePower(0.5);
            //Matrix<double> aCorrelation = aCovariance.PointwiseDivide(deviations.OuterProduct(deviations));
            //Vector<double> vSums = new DenseVector(deviations.Count), aSums = new DenseVector(deviations.Count);
            //iterate over missing markers
            for(int t = 0; t < trackers.Count; ++t)
            {
                if (assignment[t] == -1)
                {
                    if (true || numObservations[t] >= windowSize)
                    {
                        // calculate adjusted correlations and norm of correction
                        //Vector nVelocity = new Vector();
                        //Vector nAccel = new Vector();
                        Vector<double> newZ = new DenseVector(3 * correlations.Count);

                        for (int i = 0; i < 3; ++i)
                        {
                            for (int j = 0; j < 3 * trackers.Count; ++j)
                            {
                                int t2 = j / 3;
                                for (int o = 0; o < correlations.Count; ++o)
                                {
                                    //if marker not seen enough or not visible or correlation less than threshold, set correlation to zero
                                    if (numObservations[t2] < windowSize || assignment[t2] == -1
                                        || Math.Abs(correlations[o][3 * t + i, j]) < correlationThreshold
                                        || Double.IsNaN(correlations[o][3 * t + i, j]))
                                    {
                                        correlations[o][3 * t + i, j] = 0;
                                    }
                                    sums[o][3 * t + i] += Math.Abs(correlations[o][3 * t + i, j]);
                                    newZ[i + 3 * o] += trackers[t2].getState()[3 * (o + 1) + i] * correlations[o][3 * t + i, j];
                                }
                                ////if marker not visible or correlation less than threshold, set correlation to zero
                                //if (assignment[t2] == -1 || Math.Abs(vCorrelation[3 * t + i, j]) < correlationThreshold || Double.IsNaN(vCorrelation[3 * t + i, j]))
                                //{
                                //    vCorrelation[3 * t + i, j] = 0;
                                //}
                                //vSums[3 * t + i] += Math.Abs(vCorrelation[3 * t + i, j]);
                                //if (assignment[t2] == -1 || Math.Abs(aCorrelation[3 * t + i, j]) < correlationThreshold || Double.IsNaN(aCorrelation[3 * t + i, j]))
                                //{
                                //    aCorrelation[3 * t + i, j] = 0;
                                //}
                                //aSums[3 * t + i] += Math.Abs(aCorrelation[3 * t + i, j]);
                                //nVelocity[i] += trackers[t2].getVelocity()[i] * vCorrelation[3 * t + i, j];
                                //nAccel[i] += trackers[t2].getAcceleration()[i] * aCorrelation[3 * t + i, j];
                            }
                            for (int o = 0; o < correlations.Count; ++o)
                            {
                                newZ[i + 3 * o] /= sums[o][3 * t + i];
                            }
                            //nVelocity[i] /= vSums[3 * t + i];
                            //nAccel[i] /= aSums[3 * t + i];
                        }
                        Vector<double> oldZ = trackers[t].Filter.z;
                        trackers[t].Filter.z = newZ;
                        //trackers[t].Filter.z[0] = nVelocity[0];
                        //trackers[t].Filter.z[1] = nVelocity[1];
                        //trackers[t].Filter.z[2] = nVelocity[2];
                        //trackers[t].Filter.z[3] = nAccel[0];
                        //trackers[t].Filter.z[4] = nAccel[1];
                        //trackers[t].Filter.z[5] = nAccel[2];
                        trackers[t].Filter.setZ();
                        Matrix<double> oldH = trackers[t].Filter.H;
                        Matrix<double> oldR = trackers[t].Filter.R;
                        trackers[t].Filter.H = correctedH.Clone();
                        //// remove correlations for which we have no information.
                        //// corresponding variables will be updated normally (i.e.
                        //// with no correlation correction) by the Kalman filter
                        //for (int i = 0; i < 3; ++i)
                        //{
                        //    if (vSums[3 * t + i] == 0)
                        //    {
                        //        trackers[t].Filter.H[i, i] = 0;
                        //    }
                        //    if (aSums[3 * t + i] == 0)
                        //    {
                        //        trackers[t].Filter.H[3 + i, 3 + i] = 0;
                        //    }
                        //}
                        // stall components for which we have no correlated information.
                        // corresponding variables will be 'measured' as not moving.
                        for (int i = 0; i < 3; ++i)
                        {
                            for (int o = 0; o < correlations.Count; ++o)
                            {
                                if (sums[o][3 * t + i] == 0)
                                {
                                    trackers[t].Filter.z[3 * o + i] = 0;
                                }
                                else
                                {
                                    trackers[t].Filter.z[3 * o + i] *= 1;
                                }
                            }
                            //if (sums[3 * t + i] == 0)
                            //{
                            //    trackers[t].Filter.z[i] = 0;
                            //}
                            //else
                            //{
                            //    trackers[t].Filter.z[i] *= 1;
                            //}
                            //if (aSums[3 * t + i] == 0)
                            //{
                            //    trackers[t].Filter.z[3 + i] = 0;
                            //}
                        }
                        trackers[t].Filter.R = CreateMatrix.SparseIdentity<double>(3 * correlations.Count) * measurementNoise;
                        trackers[t].Filter.setZ();
                        trackers[t].Filter.correct();
                        trackers[t].Filter.H = oldH;
                        trackers[t].Filter.R = oldR;
                        trackers[t].Filter.z = oldZ;
                    }
                    else // not enough data to estimate covariance. use prediction
                    {
                        trackers[t].correct(new Vector(true));
                    }
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
