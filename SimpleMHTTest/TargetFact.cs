using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using eu.anorien.mhl;
using org.apache.log4j;
using java.awt.geom;

namespace SimpleMHTTest
{
    class TargetFact : Fact
    {

        private static Logger logger = Logger.getLogger("TargetFact");
        private readonly long id;
        private readonly long lastDetection;
        private readonly double x, y, velocityX, velocityY;

        public TargetFact(long id, long lastDetection, double x, double y, double velocityX, double velocityY)
        {
            this.id = id;
            this.lastDetection = lastDetection;
            this.x = x;
            this.y = y;
            this.velocityX = velocityX;
            this.velocityY = velocityY;
        }

        public bool measurementInGate(Point2D measurement)
        {
            return measurement.distance(x + velocityX, y + velocityY) < 7 ? true : false;
        }

        public double measurementProbability(Point2D measurement)
        {
            double dist = measurement.distance(x + velocityX, y + velocityY);
            return measurementInGate(measurement) ? (dist < 1 ? 1.0 : 1 / dist) : 0.0;
        }

        public long getId()
        {
            return id;
        }

        public long getLastDetection()
        {
            return lastDetection;
        }

        public double getVelocityX()
        {
            return velocityX;
        }

        public double getVelocityY()
        {
            return velocityY;
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
