using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ILNumerics;
using ILNumerics.Drawing;
using ILNumerics.Drawing.Plotting;

namespace GestureRecognition
{
    class SimpleHandTrackerTest
    {
        private SimpleHandTracker tracker;
        private int numMarkers;
        private ILArray<float> truePositions;
        private ILArray<float> noisyPositions;
        private ILArray<float> filterPositions;
        private ILArray<float> noise; // square errors for noisy measurements
        private ILArray<float> errors; // square errors for predictions
        private List<RandomWalker> markers = new List<RandomWalker>();
        private Random rand;
        private int bufferSize;

        public static void insertVector(ILArray<float> p, Vector v, int column)
        {
            p[0, column] = (float)v[0];
            p[1, column] = (float)v[1];
            p[2, column] = (float)v[2];
        }

        private class RandomWalker
        {
            public Vector position = new Vector(0.5, 0.5, 0.5);
            private Vector velocity = new Vector();
            private Vector accel = new Vector();
            private Vector jerk = new Vector();
            private Random rand;
            private double Q;
            private double R;

            public RandomWalker(Vector initialPosition, int seed, double Q, double R)
            {
                position = initialPosition;
                rand = new Random(seed);
                this.R = Math.Sqrt(R);
                this.Q = Math.Sqrt(Q);
            }

            //Generate normal distribution using Box-Muller transform
            private static double normalDist(double stdDev, Random rand)
            {
                double u = rand.NextDouble();
                double v = rand.NextDouble();
                return stdDev * Math.Sqrt(-2 * Math.Log(u)) * Math.Cos(2*Math.PI*v);
            }

            public Vector step(double dt)
            {
                jerk = new Vector(normalDist(Q, rand), normalDist(Q, rand), normalDist(Q, rand));
                accel = accel + jerk * dt;
                velocity = velocity + accel * dt + jerk * dt * dt / 2;
                position = position + velocity * dt + accel * dt * dt / 2 + jerk * dt * dt * dt / 6;
                //check boundary
                if (position.getX() < 0 || position.getX() > 1)
                {
                    velocity.setX(velocity[0] * -1);
                    //clamp to range [0,1]
                    position.setX(Math.Max(position.getX(), 0));
                    position.setX(Math.Min(position.getX(), 1));
                }
                if (position.getY() < 0 || position.getY() > 1)
                {
                    velocity.setY(velocity[1] * -1);
                    //clamp to range [0,1]
                    position.setY(Math.Max(position.getY(), 0));
                    position.setY(Math.Min(position.getY(), 1));
                }
                if (position.getZ() < 0.01)
                {
                    velocity.setZ(velocity[2] * -1);
                    //clamp to range [0,1]
                    position.setZ(Math.Max(position.getZ(), 0.01));
                }
                Vector noisyPosition = new Vector(position[0] + normalDist(R, rand),
                    position[1] + normalDist(R, rand),
                    position[2] + normalDist(R, rand));
                return noisyPosition;
            }
        }

        /**
         * Initialize the simple hand tracker with a specified buffer size, number of markers, and process
         * and measurement covariance magnitudes. Goes ahead and fills the buffer. To continue the simulation
         * into later time-steps, call step(). Only frames in the buffer are retained for rendering.
         * */
        public SimpleHandTrackerTest(int numFramesPerBuffer, int numMarkers, int seed, double Q, double R)
        {
            this.bufferSize = numFramesPerBuffer;
            this.numMarkers = numMarkers;
            rand = new Random(seed);
            for (int i = 0; i < numMarkers; ++i)
            {
                markers.Add(new RandomWalker(new Vector(rand.NextDouble(), rand.NextDouble(), rand.NextDouble()), rand.Next(), Q, R));
            }
            tracker = new SimpleHandTracker(numMarkers, 1*Q, 1*R);
            tracker.init();
            truePositions = ILMath.zeros<float>(3, numMarkers * bufferSize);
            noisyPositions = ILMath.zeros<float>(3, numMarkers * bufferSize);
            filterPositions = ILMath.zeros<float>(3, numMarkers * bufferSize);
            errors = ILMath.zeros<float>(2, numMarkers * bufferSize);
            noise = ILMath.zeros<float>(2, numMarkers * bufferSize);
            List<double> mse = new List<double>(new double[numMarkers]);
            for (int f = 0; f < bufferSize; ++f)
            {
                double dt = 0.02+rand.NextDouble()*.02; //small random timesteps
                List<Vector> measurements = new List<Vector>();
                for (int i = 0; i < numMarkers; ++i)
                {
                    measurements.Add(markers[i].step(dt));
                    insertVector(truePositions, markers[i].position, i * bufferSize + f);
                    insertVector(noisyPositions, measurements[i], i * bufferSize + f);
                }
                tracker.predict(dt);
                tracker.correct(measurements);
                List<Vector> estimates = tracker.getEstimatedState();
                for (int e = 0; e < estimates.Count; ++e)
                {
                    errors[0, e * bufferSize + f] = (float)f;
                    errors[1, e * bufferSize + f] = (float)Math.Pow((estimates[e] - markers[e].position).getMagnitude(), 2);
                    Vector noisyP = new Vector((double)noisyPositions[0, e * bufferSize + f],
                        (double)noisyPositions[1, e * bufferSize + f],
                        (double)noisyPositions[2, e * bufferSize + f]);
                    noise[0, e * bufferSize + f] = (float)f;
                    noise[1, e * bufferSize + f] = (float)Math.Pow((noisyP - markers[e].position).getMagnitude(), 2);
                    insertVector(filterPositions, estimates[e], e * bufferSize + f);
                    mse[e] = (mse[e] * (f) + Math.Pow((estimates[e] - markers[e].position).getMagnitude(), 1)) / (f+1);
                }
            }
            for(int i = 0; i < mse.Count; ++i)
            {
                System.Console.WriteLine(mse[i]);
            }
        }

        public void step()
        {
            //rotate queues
            truePositions[":", "0:" + (numMarkers * bufferSize - 2)] = truePositions[":", "1:" + (numMarkers * bufferSize - 1)];
            noisyPositions[":", "0:" + (numMarkers * bufferSize - 2)] = noisyPositions[":", "1:" + (numMarkers * bufferSize - 1)];
            filterPositions[":", "0:" + (numMarkers * bufferSize - 2)] = filterPositions[":", "1:" + (numMarkers * bufferSize - 1)];
            errors["1", "0:" + (numMarkers * bufferSize - 2)] = errors["1", "1:" + (numMarkers * bufferSize - 1)];
            noise["1", "0:" + (numMarkers * bufferSize - 2)] = noise["1", "1:" + (numMarkers * bufferSize - 1)];
            double dt = 0.02 + rand.NextDouble() * .02; //small random timesteps
            List<Vector> measurements = new List<Vector>();
            for (int i = 0; i < numMarkers; ++i)
            {
                measurements.Add(markers[i].step(dt));
                insertVector(truePositions, markers[i].position, i * bufferSize + bufferSize-1);
                insertVector(noisyPositions, measurements[i], i * bufferSize + bufferSize - 1);
            }
            tracker.predict(dt);
            tracker.correct(measurements);
            List<Vector> estimates = tracker.getEstimatedState();
            for (int e = 0; e < estimates.Count; ++e)
            {
                errors[1, e * bufferSize + bufferSize - 1] = (float)Math.Pow((estimates[e] - markers[e].position).getMagnitude(), 2);
                Vector noisyP = new Vector((double)noisyPositions[0, e * bufferSize + bufferSize - 1],
                    (double)noisyPositions[1, e * bufferSize + bufferSize - 1],
                    (double)noisyPositions[2, e * bufferSize + bufferSize - 1]);
                noise[1, e * bufferSize + bufferSize - 1] = (float)Math.Pow((noisyP - markers[e].position).getMagnitude(), 2);
                insertVector(filterPositions, estimates[e], e * bufferSize + bufferSize - 1);
            }
        }

        public ILScene generateScene()
        {
            ILScene scene = new ILScene();
            var points = Shapes.Point;
            int length = truePositions.Length;
            int numFrames = length / numMarkers;
            points.Positions.Update(truePositions);
            points.Positions.Update(length, length, noisyPositions);
            points.Positions.Update(2*length, length, filterPositions);
            float step = 0.8f / (float)numMarkers;
            ILPlotCube cube = new ILPlotCube(twoDMode: false);
            for (int i = 0; i < numMarkers; ++i)
            {
                points.Colors.Update(0 + i * numFrames, numFrames, ILMath.repmat(ILMath.array<float>(step * i, step * i, step * i, 1f), 1, numFrames));
                points.Colors.Update(length + i * numFrames, numFrames, ILMath.repmat(ILMath.array<float>(1.0f-step * i, 0f, 0f, 1f), 1, numFrames));
                points.Colors.Update(2 * length + i * numFrames, numFrames, ILMath.repmat(ILMath.array<float>(0f, 1.0f-step * i, 0f, 1f), 1, numFrames));
                points.Color = null; // enable individual colors
                cube.Add(points);
                var tstrip = new ILLineStrip();
                tstrip.Positions = points.Positions;
                tstrip.Colors = points.Colors;
                tstrip.Indices.Update(ILMath.vec<int>(i * numFrames, (i+1) * numFrames - 1));
                var nstrip = new ILLineStrip();
                nstrip.Positions = points.Positions;
                nstrip.Colors = points.Colors;
                nstrip.Indices.Update(ILMath.vec<int>(length + i * numFrames, length + (i+1) * numFrames - 1));
                var fstrip = new ILLineStrip();
                fstrip.Positions = points.Positions;
                fstrip.Colors = points.Colors;
                fstrip.Indices.Update(ILMath.vec<int>(2 * length + i * numFrames, 2 * length + (i+1) * numFrames - 1));
                cube.Add(tstrip);
                cube.Add(fstrip);
                cube.Add(nstrip);
            }
            cube.Plots.Clipping = null;
            cube.Limits.Set(new Vector3(0, 0, cube.Limits.ZMin),
                new Vector3(1, 1, cube.Limits.ZMax));
            scene.Add(cube);
            return scene;
        }
        public ILScene generateErrorPlot()
        {
            ILScene scene = new ILScene();
            int length = truePositions.Length;
            int numFrames = length / numMarkers;
            float step = 0.8f / (float)numMarkers;
            ILPlotCube cube = new ILPlotCube(twoDMode: true);
            for (int i = 0; i < numMarkers; ++i)
            {
                cube.Add(new ILLinePlot(noise[":", "" + i * numFrames + ":" + ((i + 1) * numFrames - 1)]));
                cube.Add(new ILLinePlot(errors[":", "" + i * numFrames + ":" + ((i + 1) * numFrames - 1)]));
            }
            cube.Limits.Set(new Vector3(cube.Limits.XMin, 0, cube.Limits.ZMin),
                new Vector3(cube.Limits.XMax, 0.02, cube.Limits.ZMax));
            scene.Add(cube);
            return scene;
        }
    }
}
