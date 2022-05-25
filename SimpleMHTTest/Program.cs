using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using javax.swing;
using java.awt.geom;
using java.awt;
using eu.anorien.mhl;
using java.util;
using java.lang;
using System.Reflection;

namespace SimpleMHTTest
{
    static class Program
    {
        class GroundTruthFrame : JFrame
        {
            object locker;
            List<Target> targets;

            public GroundTruthFrame(object locker, List<Target> targets)
                : base("Ground Truth")
            {
                this.locker = locker;
                this.targets = targets;
            }

            public override void paint(java.awt.Graphics g)
            {
                lock (locker)
                {
                    g.clearRect(0, 0, 400, 400);
                    foreach (Target target in targets)
                    {
                        g.fillOval((int)target.getX() - 2, (int)target.getY() - 2, 4, 4);
                    }
                }
            }
        }

        class MeasurementsFrame : JFrame
        {
            object locker;
            List<Point2D> noiseMeasurements, correctMeasurements;

            public MeasurementsFrame(object locker, List<Point2D> noiseMeasurements,
                List<Point2D> correctMeasurements)
                : base("Measurements")
            {
                this.locker = locker;
                this.noiseMeasurements = noiseMeasurements;
                this.correctMeasurements = correctMeasurements;
            }

            public override void paint(java.awt.Graphics g)
            {
                lock (locker)
                {
                    g.clearRect(0, 0, 400, 400);
                    g.setColor(Color.red);
                    foreach (Point2D point2D in noiseMeasurements)
                    {
                        g.fillOval((int)point2D.getX() - 2, (int)point2D.getY() - 2, 4, 4);
                    }
                    g.setColor(Color.green);
                    foreach (Point2D point2D in correctMeasurements)
                    {
                        g.fillOval((int)point2D.getX() - 2, (int)point2D.getY() - 2, 4, 4);
                    }
                }
            }

        }

        class TrackerFrame : JFrame
        {
            object locker;
            Tracker tracker;
            List<Target> targets;

            public TrackerFrame(object locker, Tracker tracker, List<Target> targets)
                : base("Tracker")
            {
                this.locker = locker;
                this.tracker = tracker;
                this.targets = targets;
            }

            public override void paint(Graphics g)
            {
                lock (locker)
                {
                    g.clearRect(0, 0, 400, 400);
                    g.setColor(Color.red);
                    for (java.util.Iterator it = tracker.getBestHypothesis().getFacts().keySet().iterator(); it.hasNext(); )
                    {
                        Fact fact = (Fact)it.next();
                        TargetFact target = (TargetFact)fact;
                        g.drawString("" + target.getId(), (int)target.getX() - 4, (int)target.getY() - 4);
                        //g.fillRect((int) target.getX() - 4, (int) target.getY() - 4, 4, 8);
                    }
                    g.setColor(Color.green);
                    foreach (Target target in targets)
                    {
                        g.fillOval((int)target.getX(), (int)target.getY() - 4, 4, 8);
                    }
                }
            }
        }

        class Looper : Runnable
        {
            JFrame groundTruthFrame;
            JFrame measurementsFrame;
            JFrame trackerFrame;

            public Looper(JFrame groundTruthFrame, JFrame measurementsFrame, JFrame trackerFrame)
            {
                this.groundTruthFrame = groundTruthFrame;
                this.measurementsFrame = measurementsFrame;
                this.trackerFrame = trackerFrame;
            }

            public void run()
            {
                groundTruthFrame.repaint();
                measurementsFrame.repaint();
                trackerFrame.repaint();
            }
        }

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            //Application.Run(new Form1());
            ikvm.runtime.Startup.addBootClassPathAssembly(Assembly.Load("C:\\Users\\Drew\\Documents\\Visual Studio 2013\\Projects\\GestureRecognition\\SimpleMHTTest\\bin\\Debug\\LisbonMHL-1.0.dll"));
            object locker = new object();
            Tracker tracker = new Tracker(6, 6, 5, 10, 0.01, 0.01, 0.1);
            List<Target> targets = new List<Target>();
            for (int i = 0; i < 2; i++)
            {
                targets.Add(new Target());
            }
            List<Point2D> noiseMeasurements = new List<Point2D>();
            List<Point2D> correctMeasurements = new List<Point2D>();
            System.Random rand = new System.Random(1);

            JFrame groundTruthFrame = new GroundTruthFrame(locker, targets);
            groundTruthFrame.setSize(400, 400);
            groundTruthFrame.setResizable(false);
            groundTruthFrame.setVisible(true);
            groundTruthFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            JFrame measurementsFrame = new MeasurementsFrame(locker, noiseMeasurements, correctMeasurements);
            measurementsFrame.setSize(400, 400);
            measurementsFrame.setResizable(false);
            measurementsFrame.setVisible(true);
            measurementsFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            JFrame trackerFrame = new TrackerFrame(locker, tracker, targets);
            trackerFrame.setSize(400, 400);
            trackerFrame.setResizable(false);
            trackerFrame.setVisible(true);
            trackerFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            while (true)
            {
                lock (locker)
                {
                    foreach (Target target in targets)
                    {
                        target.update();
                    }
                    noiseMeasurements.Clear();
                    correctMeasurements.Clear();
                    List<Point2D> finalMeasurements = new List<Point2D>();
                    for (int i = 0; i < 20; i++)
                    {
                        noiseMeasurements.Add(new Point2D.Double(rand.NextDouble() * 400, rand.NextDouble() * 400));
                    }
                    foreach (Target target in targets)
                    {
                        if (rand.NextDouble() < 0.7)
                        {
                            correctMeasurements.Add(new Point2D.Double(target.getX(), target.getY()));
                        }
                    }
                    finalMeasurements.AddRange(noiseMeasurements);
                    finalMeasurements.AddRange(correctMeasurements);
                    tracker.newScan(finalMeasurements);
                }

                EventQueue.invokeAndWait(new Looper(groundTruthFrame, measurementsFrame, trackerFrame));

                Thread.sleep(100);
            }
        }
    }
}
