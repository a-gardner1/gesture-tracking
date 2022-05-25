using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using ILNumerics;
using ILNumerics.Drawing;
using ILNumerics.Drawing.Plotting;
using System.Collections.ObjectModel;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    public partial class DPTTestForm : Form
    {
        FilteredDataStream dataStream = new FilteredDataStream(0.1, 0.0001);
        Queue<List<Vector>> history = new Queue<List<Vector>>();
        int historyLimit = 50000; // enough for 1000 seconds of data at 50 Hz; if negative or zero, unlimited
        PatternTracker dpt;
        CorrelatedHandTracker cht;
        DualPatternTrackerTest.RandomRigidPattern rrp;
        static bool useNeuralNetForLabeling = true;
        static LabeledMarkerScoreGrid grid = new LabeledMarkerScoreGrid(useNeuralNetForLabeling 
            ? @"..\..\..\NeuralNetworks\KerasLM-G-5-l2-1-L-2-N-50-A-relu-S-2-TS-0.6-0.25-0.15-1.csv"
            : @"..\..\..\Grids\GridScores-Sph0-Reg-0-Scale-50-BoxC1.csv", 
            useNeuralNetForLabeling);
        ILMarkerTrajectorySet ilmts;
        ILArray<float> groups;
        bool ready = false;
        bool initialized = false; // is the tracker initialized?
        uint bufferSize = 1;
        bool isSimulation = false, isRecording = false, isDynamicRecording = false;
        int randSeed;
        double dt = 0.02;
        string recordingSource = "";
        System.IO.StreamReader playback;
        Vector gammaHat = new Vector(true);
        List<Vector> previousMeasurement;
        double time = 0;
        int displayOptions = 31;
        List<double> observedErrors;
        DiagonalFilter priorGridScore;

        public DPTTestForm(bool simulate = false, int seed = 1)
        {
            InitializeComponent();
            isSimulation = simulate;
            randSeed = seed;
            grid.load();
            initialize(randSeed);
            step();
        }

        private void initialize(int seed)
        {
            int patternsize = 4;
            time = 0;
            ready = false;
            initialized = false;
            if (isSimulation)
            {
                patternsize = 4;
                rrp = new DualPatternTrackerTest.RandomRigidPattern(patternsize - 1, seed, 100, 1, .1);
                dt = 0.02;
            }
            else if (isRecording)
            {
                try
                {
                    if (isDynamicRecording)
                    {
                        if (playback == null)
                        {
                            playback = new System.IO.StreamReader(recordingSource);
                        }
                        dt = 0.01; // all dynamic gesture recordings were at 100 Hz.
                    }
                    else
                    {
                        playback = new System.IO.StreamReader(recordingSource);
                        dt = Double.Parse(playback.ReadLine());
                    }
                }
                catch (Exception e)
                {
                    MessageBox.Show("Error occurred trying to open " + recordingSource + ". Returning to simulation. " + e.Message);
                    isRecording = false;
                    isSimulation = true;
                    initialize(seed);
                    return;
                }
            }
            else
            {
                dataStream.initiate();
            }
            observedErrors = new List<double>();
            double initialQ_1 = 1000, initialQ_2 = 100, initialR = .001;
            //dpt = new DualPatternTracker(patternsize, initialQ_1, initialQ_2, initialR, 0.0001);
            //dpt = new CombinedPatternTracker(patternsize, initialQ_1, initialQ_2, initialR, 0.0001);
            //dpt = new RigidPatternTracker(patternsize, initialQ_1, initialQ_2, initialR, 0.0001, true, 0);
            dpt = new LayeredPatternTracker(patternsize, initialQ_1, initialQ_2, initialR, true, 0, 1, 1);
            cht = new CorrelatedHandTracker(11, initialQ_1, initialR*1, 20, 0.3);
            SigmaAText.Text = "" + initialQ_1;
            SigmaAlphaText.Text = "" + initialQ_2;
            SigmaViconText.Text = "" + initialR;
            groups = ILMath.zeros<float>(1, 2 * patternsize + 33);
            groups[":", "0:" + (patternsize - 1)] = 100;
            groups[":", "" + patternsize + ":" + (2 * patternsize - 1)] = 1;
            for(int i = 0; i < 11; ++i)
            {
                groups[":", "" + (2 * patternsize + i)] = 6 * i + 30;
                groups[":", "" + (2 * patternsize + 11 + i)] = 6 * i + 30;
                groups[":", "" + (2 * patternsize + 22 + i)] = 1;
            }
            List<Vector> fillerPositions = new List<Vector>(new Vector[2 * patternsize + 33]);
            for (int i = 0; i < fillerPositions.Count; ++i)
            {
                fillerPositions[i] = new Vector(true);
            }
            ilmts = new ILMarkerTrajectorySet(bufferSize, fillerPositions, groups, Colormaps.Jet);
            priorGridScore = new DiagonalFilter(121, 100, 1);
            priorGridScore.init(new DenseVector(121));
            //step();
        }

        private void step()
        {
            if (isSimulation)
            {
                rrp.step(dt);
            }
            else if(!isRecording)
            {
                dataStream.getFrame();
            }
            ReadOnlyCollection<Vector> pattern, unlabeledList;
            if (isReady())
            {
                if (isSimulation)
                {
                    pattern = rrp.measure(true).AsReadOnly();
                    unlabeledList = new List<Vector>().AsReadOnly();
                }
                else if (isRecording)
                {
                    var frame = playbackOneFrame();
                    pattern = frame.Item1;
                    unlabeledList = frame.Item2;
                }
                else
                {
                    pattern = dataStream.LHPat;
                    unlabeledList = dataStream.GlobalPositions;
                }
                if (!initialized)
                {
                    dpt.init(pattern);
                    List<Vector> initUnlabeled = Geometry.prune(pattern[0], unlabeledList, 40, 200);
                    if (initUnlabeled.Count < 11)
                    {
                        initUnlabeled.AddRange(Enumerable.Repeat<Vector>(pattern[0], 11 - initUnlabeled.Count));
                    }
                    cht.init(initUnlabeled);
                    //cht.init();
                    previousMeasurement = dpt.getPositions();
                    List<Vector> latestPositions = getLatestPositions(pattern, unlabeledList);
                    ilmts = new ILMarkerTrajectorySet(bufferSize, latestPositions, groups, Colormaps.Jet);
                    initialized = true;
                }
                else
                {
                    dt = (isSimulation || isRecording) ? dt : dataStream.Dt;
                    dpt.MaxGatesMissed = (int) (1 / dt);
                    time += dt;
                    int numSteps = 1;
                    for (int i = 0; i < numSteps; ++i)
                    {
                        dpt.step(pattern, observeRotation(pattern, dt / numSteps), dt / numSteps);
                        previousMeasurement = dpt.getPositions();
                        for (int m = 0; m < pattern.Count; ++m)
                        {
                            if (!pattern[m].isInvalid())
                            {
                                previousMeasurement[m].set(pattern[m][0], pattern[m][1], pattern[m][2], false);
                            }
                        }
                    }
                    if (!dpt.IsInGate)
                    {
                        // treat as false measurement, give pattern back to unlabeledList
                        List<Vector> ul = unlabeledList.ToList();
                        for (int i = 0; i < 4; ++i)
                        {
                            if (!pattern[i].isInvalid())
                            {
                                ul.Add(pattern[i]);
                            }
                        }
                        unlabeledList = ul.AsReadOnly();
                    }
                    List<Vector> localCoordinates = Geometry.getCoordinateSystem(dpt.getPositions());
                    //cht.step(Geometry.transformAndPrune(localCoordinates[0],
                    //    localCoordinates[1],
                    //    localCoordinates[2],
                    //    localCoordinates[3],
                    //    unlabeledList, 40, 200), dt);
                    cht.step(Geometry.prune(localCoordinates[0],
                        unlabeledList, 40, 200), dt);
                    List<Vector> latestPositions = getLatestPositions(pattern, unlabeledList);
                    ilmts.step(latestPositions);
                    if (!isRecording)
                    {
                        if (historyLimit > 0 && history.Count >= historyLimit)
                        {
                            history.Dequeue();
                        }
                        List<Vector> frameInfo = new List<Vector>(pattern);
                        frameInfo.AddRange(unlabeledList);
                        history.Enqueue(frameInfo);
                    }
                }
            }
        }

        private List<Vector> getLatestPositions(ReadOnlyCollection<Vector> pattern,
            ReadOnlyCollection<Vector> unlabeledList)
        {
            List<Vector> latestPositions = new List<Vector>();
            List<Vector> estimatedPattern = dpt.getPositions();
            List<Vector> localCoordinates = Geometry.getCoordinateSystem(estimatedPattern);
            // estimated pattern
            if ((displayOptions & 2) == 2)
            {
                latestPositions.AddRange(estimatedPattern);
            }
            else
            {
                latestPositions.AddRange(Enumerable.Repeat<Vector>(estimatedPattern[0], 4));
            }
            // measured pattern
            if ((displayOptions & 1) == 1)
            {
                latestPositions.AddRange(pattern);
            }
            else
            {
                latestPositions.AddRange(Enumerable.Repeat<Vector>(estimatedPattern[0], 4));
            }
            // estimated unlabeled
            if ((displayOptions & 8) == 8)
            {
                // get tracked markers in global coordinates
                List<Vector> estimatedMarkers = cht.getEstimatedState();
                // transform to local coordinates
                estimatedMarkers = Geometry.transform(localCoordinates[0],
                    localCoordinates[1],
                    localCoordinates[2],
                    localCoordinates[3],
                    estimatedMarkers.AsReadOnly());
                List<Vector> labeledMarkers = grid.orderPositions(estimatedMarkers.AsReadOnly(),
                    MathUtility.reshape(priorGridScore.getState().ToColumnMatrix(), 11, 11));
                priorGridScore.step(new DenseVector(grid.MostRecentCostMatrix.ToColumnWiseArray()), dt);
                latestPositions.AddRange(Geometry.transformToGlobal(localCoordinates[0],
                    localCoordinates[1],
                    localCoordinates[2],
                    localCoordinates[3],
                    labeledMarkers.AsReadOnly()));
            }
            else
            {
                latestPositions.AddRange(Enumerable.Repeat<Vector>(estimatedPattern[0], 11));
            }
            if ((displayOptions & 4) == 4)
            {
                // toggle colors by assigning positions to correct group
                if ((displayOptions & 8) == 8)
                {
                    // both estimated and measured unlabeled markers should be shown
                    latestPositions.AddRange(Enumerable.Repeat<Vector>(estimatedPattern[0], 11));
                }
                int numUnlabeledToAdd = Math.Min(unlabeledList.Count, 11);
                for (int i = 0; i < numUnlabeledToAdd; ++i)
                {
                    latestPositions.Add(unlabeledList[i]);
                }
                for (int i = unlabeledList.Count; i < 11; ++i)
                {
                    latestPositions.Add(pattern[0]);
                }
                if ((displayOptions & 8) != 8)
                {
                    // only measured unlabeled markers are shown
                    latestPositions.AddRange(Enumerable.Repeat<Vector>(estimatedPattern[0], 11));
                }
            }
            else
            {
                latestPositions.AddRange(Enumerable.Repeat<Vector>(estimatedPattern[0], 22));
            }
            if ((displayOptions & 16) != 16)
            {
                latestPositions = Geometry.transform(localCoordinates[0],
                    localCoordinates[1],
                    localCoordinates[2],
                    localCoordinates[3],
                    latestPositions.AsReadOnly());
            }
            return latestPositions;
        }

        private Tuple<ReadOnlyCollection<Vector>, ReadOnlyCollection<Vector>> playbackOneFrame()
        {
            if (playback.EndOfStream)
            { //reinitialize
                playback.Close();
                playback = null;
                initialize(randSeed);
            }
            string[] tokens = playback.ReadLine().Split(new char[] { ',' });
            int labeledSize = 16, offset = 0;
            if (isDynamicRecording)
            {   // different file format
                if (tokens.Length == 1 && tokens[0].Equals("Start"))
                {
                    // start of a new performance, reinitialize and skip to next line of file
                    initialize(randSeed);
                    tokens = playback.ReadLine().Split(new char[] { ',' });
                }
                offset = 3;
                labeledSize = 27;
            }
            List<Vector> markerCap = new List<Vector>();
            List<Vector> unlabeledCap = new List<Vector>();
            for (int i = 0; i < 4; ++i)
            {
                if (isDynamicRecording)
                {
                    Vector next = new Vector(Double.Parse(tokens[3 * i + 3]),
                        Double.Parse(tokens[3 * i + 4]),
                        Double.Parse(tokens[3 * i + 5]));
                    markerCap.Add(next);
                    markerCap[i].set(next[0], next[1], next[2], next.getMagnitude() == 0);
                }
                else
                {
                    markerCap.Add(new Vector(Boolean.Parse(tokens[4 * i])));
                    markerCap[i].set(Double.Parse(tokens[4 * i + 1]),
                        Double.Parse(tokens[4 * i + 2]),
                        Double.Parse(tokens[4 * i + 3]),
                        markerCap[i].isInvalid());
                }
            }
            for (int j = 0; j < (tokens.Length - labeledSize) / 3; ++j)
            {
                Vector next = new Vector(Double.Parse(tokens[labeledSize + 3 * j]),
                    Double.Parse(tokens[labeledSize + 3 * j + 1]),
                    Double.Parse(tokens[labeledSize + 3 * j + 2]));
                if (isDynamicRecording && next.getMagnitude() == 0)
                {
                    break; // found the first filler invalid marker
                }
                else
                {
                    unlabeledCap.Add(next);
                }
            }
            return new Tuple<ReadOnlyCollection<Vector>, ReadOnlyCollection<Vector>>
                (markerCap.AsReadOnly(), unlabeledCap.AsReadOnly());

        }

        private bool isReady()
        {
            if (!ready)
            {
                ReadOnlyCollection<Vector> lhPat;
                if (isSimulation)
                {
                    lhPat = rrp.measure(true).AsReadOnly();
                }
                else if (isRecording)
                {
                    // note that this will indirectly cause this frame to be skipped.
                    //lhPat = playbackOneFrame().Item1;
                    ready = true;
                    return ready;
                }
                else
                {
                    lhPat = dataStream.LHPat;
                }
                ready = true;
                for (int i = 0; i < lhPat.Count; ++i)
                {
                    ready = !lhPat[i].isInvalid() & ready;
                }
            }
            return ready;
        }

        private Vector observeRotation(ReadOnlyCollection<Vector> measurements, double dt)
        {
            List<Vector> initial = new List<Vector> (previousMeasurement);
            List<Vector> final = new List<Vector>(new Vector[measurements.Count-1]);
            bool allVisible = true, someVisible = false;
            for (int i = 0; i < final.Count; ++i)
            {
                initial[i + 1] = initial[i + 1] - initial[0];
                final[i] = measurements[i + 1] - measurements[0];
                if (measurements[i + 1].isInvalid())
                {
                    allVisible = false;
                }
                else
                {
                    someVisible = true;
                }
            }
            if (!someVisible)
            {
                return new Vector(true);
            }
            // do we need to fill in missing information?
            if (!allVisible)
            {
                dpt.saveState();
                dpt.step(measurements, new Vector(true), dt);
                List<Vector> finalFiller = dpt.getPositions();
                for (int i = 0; i < final.Count; ++i)
                {
                    if (measurements[i + 1].isInvalid())
                    {
                        if (measurements[0].isInvalid())
                        {
                            final[i] = finalFiller[i + 1] - finalFiller[0];
                        }
                        else
                        {
                            final[i] = finalFiller[i + 1] - measurements[0];
                        }
                    }
                }
                dpt.revertState();
            }
            gammaHat = Geometry.calcAxisAngle(initial.GetRange(1, final.Count).AsReadOnly(), final.AsReadOnly());
            gammaHat.set(gammaHat[0], gammaHat[1], gammaHat[2], true); // force filters to ignore rotation measurement
            return gammaHat;
        }

        private void ilPanel3_Load(object sender, EventArgs e)
        {
            //ilPanel3.Driver = RendererTypes.GDI;
            //SimpleMarkerTrackerTest smt = new SimpleMarkerTrackerTest(1000, 1);
            //ilPanel1.Scene = smt.scene;
            ilPanel3_GenerateScene();
            //ILArray<float> A = ILMath.tosingle(ILMath.rand(3, 10000));

            //var scene = new ILScene {
            //    new ILPlotCube(twoDMode: false) {
            //        new ILPoints {
            //            Positions = A,
            //            Color = null,
            //            Colors = A,
            //            Size = 2,
            //        }
            //    }
            //};
            //var pcsm = ilPanel3.Scene.First<ILPlotCube>().ScaleModes;
            //pcsm.ZAxisScale = AxisScale.Logarithmic; // hack to try to make OpenGL driver work with Intel HD Graphics

            //ilPanel1.Scene = scene;
        }

        private void ilPanel4_Load(object sender, EventArgs e)
        {
            //ilPanel4.Driver = RendererTypes.GDI;
            //ilPanel4.Scene = ht.generateErrorPlot();
            //var pcsm = ilPanel3.Scene.First<ILPlotCube>().ScaleModes;
            //pcsm.ZAxisScale = AxisScale.Logarithmic; // hack to make OpenGL driver work with Intel HD Graphics
        }

        private void ilPanel3_GenerateScene()
        {
            ilPanel3.Scene = new ILScene();
            ILPlotCube cube = new ILPlotCube(twoDMode: false);
            var points = Shapes.Point;
            ILColormap colorMap = new ILColormap(ilmts.ColorScheme);
            var colors = colorMap.Map(groups);
            points.Positions.Update(ilmts.Positions);
            for (int i = 0; i < ilmts.NumMarkers; ++i)
            {
                points.Colors.Update(i * ilmts.BufferSize,
                    ilmts.BufferSize,
                    ILMath.repmat(
                        ILMath.array<float>(
                            colors.GetValue(i, 0)*2,
                            colors.GetValue(i, 1),
                            colors.GetValue(i, 2)*3/2,
                            colors.GetValue(i, 3)),
                    1,
                    ilmts.BufferSize));
                points.Color = null; // enable individual colors
                var tstrip = new ILLineStrip();
                tstrip.Positions = points.Positions;
                tstrip.Colors = points.Colors;
                tstrip.Indices.Update(ILMath.vec<int>(i * bufferSize, (i + 1) * bufferSize - 1));
                cube.Add(tstrip);
            }
            cube.Add(points);
            cube.Plots.Clipping = null;
            //cube.Limits.Set(new Vector3(-2000, -2000, 0),
            //    new Vector3(2000, 2000, 2000));
            ilPanel3.Scene.Add(cube);
        }

        private void ilPanel3_RefreshScene()
        {
            ilPanel3.Scene.First<ILPlotCube>().First<ILPoints>().Positions.Update(ilmts.Positions);
            ilPanel3.Scene.Configure();
            ilPanel3.Refresh();
        }

        private void ilPanel3_KeyDown(object sender, KeyEventArgs e)
        {
            if (isSimulation)
            {
                switch (e.KeyCode)
                {
                    case Keys.W:
                        rrp.exertForce(new Vector(0, 1, 0));
                        break;
                    case Keys.A:
                        rrp.exertForce(new Vector(-1, 0, 0));
                        break;
                    case Keys.S:
                        rrp.exertForce(new Vector(0, -1, 0));
                        break;
                    case Keys.D:
                        rrp.exertForce(new Vector(1, 0, 0));
                        break;
                    case Keys.Q:
                        rrp.exertForce(new Vector(0, 0, -1));
                        break;
                    case Keys.E:
                        rrp.exertForce(new Vector(0, 0, 1));
                        break;
                    case Keys.Up:
                        rrp.exertRotation(new Vector(-1, 0, 0));
                        break;
                    case Keys.Left:
                        rrp.exertRotation(new Vector(0, -1, 0));
                        break;
                    case Keys.Down:
                        rrp.exertRotation(new Vector(1, 0, 0));
                        break;
                    case Keys.Right:
                        rrp.exertRotation(new Vector(0, 1, 0));
                        break;
                }
            }
        }

        private void ilPanel3_DragEnter(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.FileDrop))
                e.Effect = DragDropEffects.Copy;
            else
            {
                String[] strGetFormats = e.Data.GetFormats();
                e.Effect = DragDropEffects.None;
            }
        }

        private void ilPanel3_DragDrop(object sender, DragEventArgs e)
        {
            if (isSimulation || isRecording)
            {
                string[] fileList = (string[])e.Data.GetData(DataFormats.FileDrop, false);
                if (System.IO.Path.GetExtension(fileList[0]).Equals(".csv"))
                {
                    recordingSource = fileList[0];
                    timer1.Stop();
                    isSimulation = false;
                    if (isRecording)
                    {
                        // if already recording, close other stream.
                        playback.Close();
                        playback = null;
                    }
                    isRecording = true;
                    if (recordingSource.ToLower().EndsWith(".dyn.csv"))
                    {
                        isDynamicRecording = true;
                    }
                    initialize(randSeed);
                    timer1.Start();
                    this.timer1.Enabled = true;
                }
            }
        }

        private void timer1_Tick(object sender, System.EventArgs e)
        {
            try
            {
                step();
                ilPanel3_RefreshScene();
                //ilPanel4.Scene.Configure();
                //ilPanel4.Refresh();
                //ilPanel3.Render(10);
                //ilPanel4.Render(10);
                textBox1_ChangeText();
            }
            catch (Exception ex)
            {
                if (!isRecording)
                {
                    // dump
                    timer1.Stop();
                    String errorMessage = ex.Message + " Dumping captured stream for debugging, then exiting.\nStack trace: " + ex.StackTrace;
                    System.Console.WriteLine(errorMessage);
                    MessageBox.Show(errorMessage);
                    dumpData();
                    Application.Exit();
                }
                else
                {
                    timer1.Stop();
                    dumpData();
                    String errorMessage = ex.Message + "\nRestarting stream." + "\nStack trace: " + ex.StackTrace;
                    System.Console.WriteLine(errorMessage);
                    MessageBox.Show(errorMessage);
                    initialize(randSeed);
                    timer1.Start();
                }
            }
        }

        private void dumpData()
        {
            if (!isRecording)
            {
                using (System.IO.StreamWriter writer
                            = new System.IO.StreamWriter("CaptureDump-(" + System.DateTime.Now.ToString().Replace(':', '-').Replace('/', '-') + ").csv"))
                {
                    writer.WriteLine(dt);
                    foreach (var measurement in history)
                    {
                        String nextLine = measurement[0].isInvalid().ToString() + "," + measurement[0].ToString()
                            + "," + measurement[1].isInvalid().ToString() + "," + measurement[1].ToString()
                            + "," + measurement[2].isInvalid().ToString() + "," + measurement[2].ToString()
                            + "," + measurement[3].isInvalid().ToString() + "," + measurement[3].ToString();
                        for (int i = 4; i < measurement.Count; ++i)
                        {
                            nextLine += "," + measurement[i].ToString();
                        }
                        writer.WriteLine(nextLine);
                    }
                }
            }
            using (System.IO.StreamWriter writer
                = new System.IO.StreamWriter("StatisticsDump-(" + System.DateTime.Now.ToString().Replace(':', '-').Replace('/', '-') + ").csv"))
            {
                writer.WriteLine(dt);
                foreach (var statistic in observedErrors)
                {
                    String nextLine = "" + statistic;
                    writer.WriteLine(nextLine);
                }
            }
        }

        private void textBox1_ChangeText()
        {
            List<Vector> positions = dpt.getPositions();
            string ntext = "Time: " + time.ToString("0.##");
            ntext += "   \tX\tY\tZ\r\n";
            ntext += "O = <\t" + positions[0].getX().ToString("0.##") + "\t"
                + positions[0].getY().ToString("0.##") + "\t"
                + positions[0].getZ().ToString("0.##") + ">\r\n";
            for (int i = 1; i < positions.Count; ++i) {
                ntext += "M_" + (i + 1) + " = <\t" + positions[i].getX().ToString("0.##") + "\t"
                    + positions[i].getY().ToString("0.##") + "\t"
                    + positions[i].getZ().ToString("0.##") + ">\r\n";
            }
            for (int i = 1; i < positions.Count; ++i)
            {
                ntext += "||e_" + i + "|| = \t" + (positions[i] - positions[0]).getMagnitude() + "\r\n";
            }
            ntext += "w = <\t" + dpt.getAngularVelocity().getX().ToString("0.##") + "\t"
                + dpt.getAngularVelocity().getY().ToString("0.##") + "\t"
                + dpt.getAngularVelocity().getZ().ToString("0.##") + ">\r\n";
            ntext += "a = <\t" + dpt.getAngularAcceleration().getX().ToString("0.##") + "\t"
                + dpt.getAngularAcceleration().getY().ToString("0.##") + "\t"
                + dpt.getAngularAcceleration().getZ().ToString("0.##") + ">\r\n";
            double error = 0;
            for (int i = 0; i < positions.Count; ++i)
            {
                if (!previousMeasurement[i].isInvalid())
                {
                    error += Math.Pow((positions[i] - previousMeasurement[i]).getMagnitude(), 2);
                }
            }
            observedErrors.Add(error);
            ntext += "Observed error = " + error + "\r\n";
            double dt = isSimulation ? 0.02 : dataStream.Dt;
            Vector gamma = dpt.getAngularVelocity() * dt + dpt.getAngularAcceleration() * (dt * dt / 2);
            error = (gamma - gammaHat).getMagnitude();
            ntext += "Observed rotation error = " + (error * error) + "\r\n";
            ntext += dpt.checkForNumericalIssues() + "\r\n";
            ntext += "Log-likelihood: " + dpt.LogLikelihood + "\r\n";
            ntext += "Mean Root Error: " + dpt.MeanError + "\r\n";
            textBox1.Text = ntext;
        }

        private void SigmaViconButton_Click(object sender, EventArgs e)
        {
            //MessageBox.Show(this.SigmaViconText.Text);
            double sigmaVicon = dpt.SigmaVicon;
            try
            {
                sigmaVicon = Double.Parse(SigmaViconText.Text);
            }
            catch (Exception) { }
            dpt.SigmaVicon = sigmaVicon;
        }

        private void SigmaAlphaButton_Click(object sender, EventArgs e)
        {
            //MessageBox.Show(this.SigmaAlphaText.Text);
            double sigmaAlpha = dpt.SigmaAlpha;
            try
            {
                sigmaAlpha = Double.Parse(SigmaAlphaText.Text);
            }
            catch (Exception) { }
            dpt.SigmaAlpha = sigmaAlpha;
        }

        private void SigmaAButton_Click(object sender, EventArgs e)
        {
            //MessageBox.Show(this.SigmaAText.Text);
            double sigmaA = dpt.SigmaA;
            try
            {
                sigmaA = Double.Parse(SigmaAText.Text);
            }
            catch (Exception) { }
            dpt.SigmaA = sigmaA;
        }
        protected override bool IsInputKey(Keys keyData)
        {
            switch (keyData)
            {
                case Keys.Right:
                case Keys.Left:
                case Keys.Up:
                case Keys.Down:
                    return true;
                case Keys.Shift | Keys.Right:
                case Keys.Shift | Keys.Left:
                case Keys.Shift | Keys.Up:
                case Keys.Shift | Keys.Down:
                    return true;
            }
            return base.IsInputKey(keyData);
        }
        protected override void OnKeyDown(KeyEventArgs e)
        {
            base.OnKeyDown(e);
            switch (e.KeyCode)
            {
                case Keys.Left:
                case Keys.Right:
                case Keys.Up:
                case Keys.Down:
                    if (e.Shift)
                    {

                    }
                    else
                    {
                    }
                    break;
            }
        }

        private void ViconPatternButton_Click(object sender, EventArgs e)
        {
            displayOptions = 1 ^ displayOptions;
        }

        private void KFPatternButton_Click(object sender, EventArgs e)
        {
            displayOptions = 2 ^ displayOptions;
        }

        private void ViconUnlabeledButton_Click(object sender, EventArgs e)
        {
            displayOptions = 4 ^ displayOptions;
        }

        private void KFUnlabeledButton_Click(object sender, EventArgs e)
        {
            displayOptions = 8 ^ displayOptions;
        }

        private void coordinateButton_Click(object sender, EventArgs e)
        {
            if ((displayOptions & 16) == 16)
            {
                coordinateButton.Text = "To Global Coordinates";
                ilPanel3.Scene.First<ILPlotCube>().Limits.Set(new Vector3(-30, -30, -200),
                    new Vector3(200, 200, 100));
            }
            else
            {
                coordinateButton.Text = "To Local Coordinates";
                ilPanel3.Scene.First<ILPlotCube>().Limits.Set(new Vector3(-2000, -2000, 0),
                    new Vector3(2000, 2000, 2000)); 
            }
            displayOptions = 16 ^ displayOptions;
        }


    }
}
