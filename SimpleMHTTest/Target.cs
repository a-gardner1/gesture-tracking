using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using org.apache.log4j;

namespace SimpleMHTTest
{
    class Target
    {
        private static Logger logger = Logger.getLogger("Target");
        private double x, y, heading, velocity;
        private Random rand = new Random();


        public Target()
        {
            x = rand.NextDouble() * 400;
            y = rand.NextDouble() * 400;
            heading = rand.NextDouble() * 2 * Math.PI;
            velocity = rand.NextDouble() * 5;
        }

        public void update()
        {
            heading = (rand.NextDouble() - 0.5) + heading;
            velocity = Math.Min(5, Math.Max(0, velocity + rand.NextDouble()));
            x = Math.Min(390, Math.Max(10, x + velocity * Math.Cos(heading)));
            y = Math.Min(390, Math.Max(10, y + velocity * Math.Sin(heading)));
        }

        public double getX()
        {
            return x;
        }

        public double getY()
        {
            return y;
        }
    }
}
