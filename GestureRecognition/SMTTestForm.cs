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

namespace GestureRecognition
{
    public partial class SMTTestForm : Form
    {
        SimpleHandTrackerTest ht = new SimpleHandTrackerTest(20, 4, 1, 0.1, 0.0001);
        public SMTTestForm()
        {
            InitializeComponent();
        }

        private void ilPanel3_Load(object sender, EventArgs e)
        {
            //ilPanel3.Driver = RendererTypes.GDI;
            //SimpleMarkerTrackerTest smt = new SimpleMarkerTrackerTest(1000, 1);
            //ilPanel1.Scene = smt.scene;
            ilPanel3.Scene = ht.generateScene();
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
            ilPanel4.Scene = ht.generateErrorPlot();
            //var pcsm = ilPanel3.Scene.First<ILPlotCube>().ScaleModes;
            //pcsm.ZAxisScale = AxisScale.Logarithmic; // hack to make OpenGL driver work with Intel HD Graphics
        }

        private void timer1_Tick(object sender, System.EventArgs e)
        {
            ht.step();
            ilPanel3_Load(sender, e);
            ilPanel4_Load(sender, e);
            ilPanel3.Render(10);
            ilPanel4.Render(10);
        }
    }
}
