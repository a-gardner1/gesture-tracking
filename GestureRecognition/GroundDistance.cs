using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GestureRecognition
{
    /**
     * Specialized notion of ground distance between Vectors
     * 
     * m_D in my paper
     * */
    abstract class GroundDistance
    {
        public abstract double getDistance(Vector a, Vector b);

        public double getThresholdedDistance(Vector a, Vector b, double nu)
        {
            nu = nu == 0 ? 1 : nu; // avoid division by zero in case they haven't considered it.
            return Math.Min(1, getDistance(a, b)/nu);
        }

    }

    class Manhattan : GroundDistance
    {
        public Manhattan() { }

        public override double getDistance(Vector a, Vector b)
        {
            return Math.Abs(a.getX() - b.getX()) + Math.Abs(a.getY() - b.getY()) + Math.Abs(a.getZ() - b.getZ());
        }
    }

    class Euclidean : GroundDistance
    {
        public Euclidean() { }

        public override double getDistance(Vector a, Vector b)
        {
            return (a - b).getMagnitude();
        }
    }

    class SquareEuclidean : GroundDistance
    {
        public SquareEuclidean() { }

        public override double getDistance(Vector a, Vector b)
        {
            return Math.Pow((a - b).getMagnitude(), 2);
        }
    }

    class Uniform : GroundDistance
    {
        public Uniform() { }

        public override double getDistance(Vector a, Vector b)
        {
            return Math.Max(Math.Abs(a.getX() - b.getX()), Math.Max(Math.Abs(a.getY() - b.getY()), Math.Abs(a.getZ() - b.getZ())));
        }
    }
}
