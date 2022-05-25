using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace GestureRecognition
{
    class Program
    {
        public static readonly string startupPath = System.IO.Directory.GetCurrentDirectory();

        [STAThread]
        static void Main()
        {
            //System.Console.WriteLine(startupPath);
            //DynamicLabeling dl = new DynamicLabeling(true, true);
            //dl.labelDynamicGestures();
            //return;
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new DPTTestForm(true));
        }
    }
}
