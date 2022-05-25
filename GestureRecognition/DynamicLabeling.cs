using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    class DynamicLabeling
    {
        private Obsolete.DataReader dataReader;
        private const string dataDir = "C:\\Users\\Drew\\Documents\\Visual Studio 2010\\Projects\\CVPR2014\\CVPR2014\\Data";
        private const string userDir = dataDir + "\\Users";
        private string outDir = dataDir + "\\LabeledDynamic";
        private double dt = 0.01; // all performances in the dataset (as of 6/14/2016) were recorded at 100 Hz
        private bool saveUnlabeled = false; // current implementation is lazy... labeling occurs, but is ignored when it comes time to save

        private List<Vector> previousLocal = null;

        LabeledMarkerScoreGrid grid;

        public DynamicLabeling(bool useNeuralNetForLabeling = true, bool saveUnlabeled = false)
        {
            this.saveUnlabeled = saveUnlabeled;
            if (saveUnlabeled)
            {
                outDir = dataDir + "\\UnlabeledDynamic";
            }
            else
            {
                if (useNeuralNetForLabeling)
                {
                    outDir = dataDir + "\\MLPLabeledDynamic";
                }
                else
                {
                    outDir = dataDir + "\\LabeledDynamic";
                }
            }
            dataReader = new Obsolete.DataReader();
            dataReader.setRootDir(userDir);
            System.Console.WriteLine("Loading dynamic gestures");
            dataReader.loadDynamicGestures(true);
            System.Console.WriteLine("Dynamic gestures loaded!");
            grid = new LabeledMarkerScoreGrid(useNeuralNetForLabeling 
                ? @"..\..\..\NeuralNetworks\KerasLM-G-5-l2-1-L-2-N-50-A-relu-S-2-TS-0.6-0.25-0.15-1.csv" 
                : @"..\..\..\Grids\GridScores-Sph0-Reg-0-Scale-50-BoxC1.csv", 
                useNeuralNetForLabeling);
            grid.load();
            System.Console.WriteLine("Creating directory");
            if (!System.IO.Directory.Exists(outDir))
            {
                System.IO.Directory.CreateDirectory(outDir);
            }
        }

        public void labelDynamicGestures()
        {
            var data = dataReader.getAllDynamicGesturesByUser(true);
            var allWriter = new System.IO.StreamWriter(outDir + "\\all.left.csv", false);
            // for each user
            for(int u = 0; u < data.Count; ++u)
            {
                //for each gesture class
                for(int g = 0; g < data[u].Count; ++g)
                {
                    using (var writer = new System.IO.StreamWriter(outDir + "\\u" + u + "g" + g + ".left.csv", false))
                    {
                        writer.WriteLine(dt);
                        // for each performance of the gesture
                        for (int p = 0; p < data[u][g].Count; ++p)
                        {
                            if (data[u][g][p].Count > 0)
                            {
                                previousLocal = new List<Vector>();
                                // try to provide a consistent labeling based on the score grid and correlated tracker
                                LayeredPatternTracker lpt = new LayeredPatternTracker(4, 1000, 1000, 0.001, true, 0, 1, 1);
                                CorrelatedHandTracker cht = new CorrelatedHandTracker(11, 1000, .001, 20, 0.6);
                                DiagonalFilter priorGridScore = new DiagonalFilter(121, 100, 1);
                                priorGridScore.init(new DenseVector(121));
                                ReadOnlyCollection<Vector> pattern = (new Vector[] {new Vector(true), new Vector(true), new Vector(true), new Vector(true)}).ToList().AsReadOnly();
                                bool ready = false;
                                // filter each frame of the performance
                                for (int f = 0; f < data[u][g][p].Count; ++f)
                                {
                                    updatePattern(pattern, data[u][g][p][f]);
                                    // transition unlabeled markers to filtered local coordinate system
                                    // pass through global coordinate on way
                                    var unfilteredLocal = Geometry.getCoordinateSystem(pattern);
                                    List<Vector> unlabeled;
                                    if (unfilteredLocal.Count > 0)
                                    {
                                        unlabeled = Geometry.transformToGlobal(unfilteredLocal[0], unfilteredLocal[1], unfilteredLocal[2], unfilteredLocal[3],
                                            data[u][g][p][f].markers.AsReadOnly());
                                    }
                                    else 
                                    {
                                        //then the markers were already in global coordinates
                                        unlabeled = data[u][g][p][f].markers;
                                    }
                                    if (!ready) // wait until we actually see the pattern.
                                    {
                                        // NOTE: This method is flawed in that it cannot handle partial patterns.
                                        // A method that can should be defined.
                                        if (!pattern[0].isInvalid() && !pattern[1].isInvalid()
                                            && !pattern[2].isInvalid() && !pattern[3].isInvalid())
                                        {
                                            ready = true;
                                            // indicate the start of a new performance in the file
                                            writer.WriteLine("Start," + data[u][g][p][0].ID);
                                            allWriter.WriteLine("Start," + data[u][g][p][0].ID);
                                            lpt.init(pattern);
                                            var estimatedPattern = Geometry.getCoordinateSystem(lpt.getPositions());
                                            unlabeled = Geometry.transformAndPrune(estimatedPattern[0],
                                                                                   estimatedPattern[1],
                                                                                   estimatedPattern[2],
                                                                                   estimatedPattern[3],
                                                                                   unlabeled.AsReadOnly(), 40, 200);
                                            cht.init(unlabeled);
                                        }
                                    }
                                    else
                                    {
                                        lpt.step(pattern, new Vector(true), dt);
                                        if (!lpt.IsInGate)
                                        {
                                            // treat as false measurement, give pattern back to unlabeledList
                                            for (int i = 0; i < 4; ++i)
                                            {
                                                if (!pattern[i].isInvalid())
                                                {
                                                    unlabeled.Add(pattern[i]);
                                                }
                                            }
                                        }
                                        var estimatedPattern = Geometry.getCoordinateSystem(lpt.getPositions());
                                        unlabeled = Geometry.transformAndPrune(estimatedPattern[0],
                                                                               estimatedPattern[1],
                                                                               estimatedPattern[2],
                                                                               estimatedPattern[3],
                                                                               unlabeled.AsReadOnly(), 40, 200);
                                        cht.step(unlabeled, dt);
                                    }
                                    if(ready) {
                                        var labeled = grid.orderPositions(cht.getEstimatedState().AsReadOnly(),
                                            MathUtility.reshape(priorGridScore.getState().ToColumnMatrix(), 11, 11));
                                        priorGridScore.step(new DenseVector(grid.MostRecentCostMatrix.ToColumnWiseArray()), dt);
                                        // save to file
                                        allWriter.WriteLine(writeLine(writer, lpt, saveUnlabeled ? unlabeled : labeled));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            allWriter.Close();
        }

        private void updatePattern(ReadOnlyCollection<Vector> pattern, Obsolete.Frame f)
        {
            pattern[0].set(f.Origin.getX(),
                f.Origin.getY(),
                f.Origin.getZ(),
                f.Origin.isInvalid());
            pattern[1].set(f.XMarker.getX(),
                f.XMarker.getY(),
                f.XMarker.getZ(),
                f.XMarker.isInvalid());
            pattern[2].set(f.YMarker.getX(),
                f.YMarker.getY(),
                f.YMarker.getZ(),
                f.YMarker.isInvalid());
            pattern[3].set(f.Extra.getX(),
                f.Extra.getY(),
                f.Extra.getZ(),
                f.Extra.isInvalid());
        }

        private string writeLine(System.IO.StreamWriter writer, LayeredPatternTracker lpt, List<Vector> labeled)
        {
            List<Vector> pattern = lpt.getPositions();
            //global coordinate section
            String line = pattern[0].ToString()
                            + "," + pattern[1].ToString()
                            + "," + pattern[2].ToString()
                            + "," + pattern[3].ToString();
            line += "," + lpt.getAngularVelocity();
            line += "," + lpt.getAngularAcceleration();
            //local coordinate section
            if (previousLocal == null || previousLocal.Count == 0)
            {
                previousLocal = Geometry.getCoordinateSystem(pattern);
            }
            List<Vector> transformedPattern = Geometry.transform(previousLocal[0],
                previousLocal[1],
                previousLocal[2],
                previousLocal[3],
                pattern.AsReadOnly());
            line += "," + transformedPattern[0].ToString()
                    + "," + transformedPattern[1].ToString()
                    + "," + transformedPattern[2].ToString()
                    + "," + transformedPattern[3].ToString();
            line += "," + Geometry.transform(new Vector(),
                previousLocal[1],
                previousLocal[2],
                previousLocal[3],
                new List<Vector>(new Vector[] { lpt.getAngularVelocity() }).AsReadOnly())[0];
            line += "," + Geometry.transform(new Vector(),
                previousLocal[1],
                previousLocal[2],
                previousLocal[3],
                new List<Vector>(new Vector[] { lpt.getAngularAcceleration() }).AsReadOnly())[0];
            for (int i = 0; i < labeled.Count; ++i)
            {
                line += "," + labeled[i].ToString();
            }
            writer.WriteLine(line);
            return line;
        }
    }
}
