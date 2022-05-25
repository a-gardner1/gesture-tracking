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
    class SimpleMarkerTrackerTest
    {
        private SimpleMarkerTracker tracker;

        private Vector position = new Vector (0.5, 0.5, 0.5);
        private Vector velocity = new Vector();
        private Vector accel = new Vector();
        private Vector jerk = new Vector();
        private Random rand;
        public ILScene scene;


        /**
         * See how well the tracker works at following a random walk.
         * */
        public SimpleMarkerTrackerTest(int numFrames, int seed)
        {
            double dt = .01;
            tracker = new SimpleMarkerTracker(0.1, 1, true);
            tracker.init(new Vector());
            rand = new Random(seed);
            Random rand2 = new Random(seed+1);
            scene = new ILScene();
            ILArray<float> truePositions = ILMath.zeros<float>(3, numFrames);
            ILArray<float> noisyPositions = ILMath.zeros<float>(3, numFrames);
            ILArray<float> filterPositions = ILMath.zeros<float>(3, numFrames);
            for(int i = 0; i < numFrames; ++i) {
                jerk = new Vector((rand.NextDouble() * 2 - 1), (rand.NextDouble() * 2 - 1), (rand.NextDouble() * 2 - 1));
                dt = 0.02 + rand2.NextDouble() * 0.02;
                accel = accel + jerk * dt;
                velocity = velocity + accel * dt + jerk * dt * dt / 2;
                position = position + velocity * dt + accel * dt * dt / 2 + jerk * dt * dt * dt / 6;
                //check boundary
                if(position.getX() < 0 || position.getX() > 1)
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
                truePositions[0, i] = (float)position[0];
                truePositions[1, i] = (float)position[1];
                truePositions[2, i] = (float)position[2];
                tracker.predict(dt);
                Vector noisyPosition = new Vector(position.getX() + 0.01 * (rand.NextDouble() * 2 - 1), position.getY() + 0.01 * (rand.NextDouble() * 2 - 1), position.getZ() + 0.01 * (rand.NextDouble() * 2 - 1));
                noisyPositions[0, i] = (float)noisyPosition[0];
                noisyPositions[1, i] = (float)noisyPosition[1];
                noisyPositions[2, i] = (float)noisyPosition[2];
                tracker.correct(noisyPosition);
                filterPositions[0, i] = (float)tracker.getPosition()[0];
                filterPositions[1, i] = (float)tracker.getPosition()[1];
                filterPositions[2, i] = (float)tracker.getPosition()[2];
                System.Console.WriteLine((tracker.getPosition() - position).getMagnitude());
            }
            var points = Shapes.Point;
            points.Positions.Update(0, numFrames, truePositions);
            points.Positions.Update(numFrames, numFrames, noisyPositions);
            points.Positions.Update(2 * numFrames, numFrames, filterPositions);
            points.Colors.Update(ILMath.repmat(ILMath.array<float>(0f, 0f, 1f, 1f), 1, numFrames));
            points.Colors.Update(numFrames, numFrames, ILMath.repmat(ILMath.array<float>(1f, 0f, 0f, 1f), 1, numFrames));
            points.Colors.Update(2*numFrames, numFrames, ILMath.repmat(ILMath.array<float>(0f, 1f, 0f, 1f), 1, numFrames));
            points.Color = null;
            var tstrip = new ILLineStrip();
            tstrip.Positions = points.Positions;
            tstrip.Colors = points.Colors;
            tstrip.Indices.Update(ILMath.vec<int>(0, numFrames - 1));
            var nstrip = new ILLineStrip();
            nstrip.Positions = points.Positions;
            nstrip.Colors = points.Colors;
            nstrip.Indices.Update(ILMath.vec<int>(numFrames, 2*numFrames - 1));
            var fstrip = new ILLineStrip();
            fstrip.Positions = points.Positions;
            fstrip.Colors = points.Colors;
            fstrip.Indices.Update(ILMath.vec<int>(2*numFrames, 3*numFrames - 1));
            ILPlotCube cube = new ILPlotCube(twoDMode: false);
            cube.Add(points);
            cube.Add(tstrip);
            cube.Add(fstrip);
            cube.Add(nstrip);
            scene.Add(cube);
        }

    }
}
