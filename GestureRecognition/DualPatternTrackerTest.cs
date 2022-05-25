using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    class DualPatternTrackerTest
    {
        public class RandomRigidPattern
        {
            Random rand;
            Vector origin = new Vector(), velocity = new Vector(), acceleration = new Vector();
            List<Vector> edges = new List<Vector>();
            Vector omega = new Vector(), alpha = new Vector();
            double qAccel, qAng, R;

            public RandomRigidPattern(int numEdges, int seed, double QAccel, double QAng, double R)
            {
                rand = new Random(seed);
                origin = new Vector(0,0,0);
                for (int i = 0; i < numEdges; ++i)
                {
                    edges.Add(new Vector(rand.NextDouble() * 20, rand.NextDouble() * 20, rand.NextDouble() * 20));
                }
                this.R = R;
                qAccel = QAccel;
                qAng = QAng;
            }

            //Generate normal distribution using Box-Muller transform
            private static double normalDist(double stdDev, Random rand)
            {
                double u = rand.NextDouble();
                double v = rand.NextDouble();
                return stdDev * Math.Sqrt(-2 * Math.Log(u)) * Math.Cos(2 * Math.PI * v);
            }
            private Matrix<double> calcRho(double dt)
            {
                Vector gamma = omega * dt + alpha * (dt * dt / 2);
                Matrix<double> Gamma = Geometry.getCrossProductMatrix(gamma);
                Matrix<double> rho;
                double mag = gamma.getMagnitude();
                if (mag < 0.0001)
                {
                    rho = CreateMatrix.DenseIdentity<double>(3) + Gamma + Gamma * Gamma * 0.5;
                }
                else
                {
                    rho = CreateMatrix.DenseIdentity<double>(3) + Gamma * (Math.Sin(mag) / mag) + Gamma * Gamma * ((1 - Math.Cos(mag)) / (mag * mag));
                }
                return rho;
            }

            public void step(double dt)
            {
                //random translation
                //Vector jerk = new Vector(normalDist(qAccel, rand), normalDist(qAccel, rand), normalDist(qAccel, rand));
                //acceleration = acceleration + jerk * dt;
                //acceleration = new Vector(normalDist(qAccel, rand), normalDist(qAccel, rand), normalDist(qAccel, rand));
                acceleration *= 0.5;
                velocity = velocity + acceleration * dt;// +jerk * dt * dt / 2;
                origin = origin + velocity * dt + acceleration * dt * dt / 2;// +jerk * dt * dt * dt / 6;
                //random rotation
                //Vector xi = new Vector(normalDist(qAng, rand), normalDist(qAng, rand), normalDist(qAng, rand));
                //alpha = alpha + xi * dt;
                //alpha = new Vector(normalDist(qAng, rand), normalDist(qAng, rand), normalDist(qAng, rand));
                alpha *= 0.5;
                omega = omega + alpha * dt;// +xi * dt * dt / 2;
                Matrix<double> rho = calcRho(dt);
                for(int i = 0; i < edges.Count; ++i)
                {
                    edges[i] = new Vector(edges[i].dot(rho[0, 0], rho[0, 1], rho[0, 2]),
                        edges[i].dot(rho[1, 0], rho[1, 1], rho[1, 2]),
                        edges[i].dot(rho[2, 0], rho[2, 1], rho[2, 2]));
                }
            }

            /**
             * Exert a force slightly modified by the process noise.
             * */
            public void exertForce(Vector direction)
            {
                //normalize
                direction = direction / direction.getMagnitude();
                //velocity += direction;
                origin += direction*10;
            }


            /**
             * Exert a rotation slightly modified by the process noise.
             * */
            public void exertRotation(Vector direction)
            {
                //normalize
                direction = direction / direction.getMagnitude();
                omega += direction;
            }

            public List<Vector> measure(bool addNoise)
            {
                List<Vector> m = new List<Vector>();
                m.Add(origin);
                if (addNoise)
                {
                    m[0] += new Vector(normalDist(R, rand), normalDist(R, rand), normalDist(R, rand));
                }
                for (int i = 0; i < edges.Count; ++i)
                {
                    m.Add(edges[i] + origin);
                    if (addNoise)
                    {
                        m[i + 1] += new Vector(normalDist(R, rand), normalDist(R, rand), normalDist(R, rand));
                    }
                }
                return m;
            }
        }
    }
}
