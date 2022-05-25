using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SimpleMHTTest
{
    class Vector 
    {
        double[] vector = new double[] { 0, 0, 0 }; // vector[0],[1],[2] = x, y, z
        double magnitude;
        bool invalid; // Is the vector valid information for computations?

        /*** See function prototype in '.h' file ***/
        public Vector()
        {
            invalid = false;
        }

        /*** See function prototype in '.h' file ***/
        public Vector(double x, double y, double z)
        {
            setX(x); setY(y); setZ(z);
            invalid = false;
            setMagnitude();
        }

        /*** See function prototype in '.h' file ***/
        public Vector(bool error)
        {
            invalid = error;
            if (invalid == true)
            {
                setX(0); setY(0); setZ(0);
            }
            setMagnitude();
        }

        public Vector(Vector other) 
        {
            if (other != null)
            {
                invalid = other.invalid;
                magnitude = other.magnitude;
                vector[0] = other.vector[0];
                vector[1] = other.vector[1];
                vector[2] = other.vector[2];
            }
            else
            {
                invalid = true;
                setX(0); setY(0); setZ(0);
                setMagnitude();
            }
        }

        public void set(double x, double y, double z, bool inv = false)
        {
            invalid = inv;
            vector[0] = x;
            vector[1] = y;
            vector[2] = z;
            setMagnitude();
        }

        /*** See function prototype in '.h' file ***/
        public double[] getVector()
        {
            return vector;
        }

        public double getMagnitude()
        {
            return magnitude;
        }

        /*** See function prototype in '.h' file ***/
        private void setMagnitude()
        {
            magnitude = Math.Sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());
        }

        /*** See function prototype in '.h' file ***/
        public static Vector operator *(Vector orig, double scalar)
        {
            return new Vector(orig.getX() * scalar, orig.getY() * scalar, orig.getZ() * scalar);
        }

        /*** See function prototype in '.h' file ***/
        public static Vector operator /(Vector orig, double scalar)
        {
            return new Vector(orig.getX() / scalar, orig.getY() / scalar, orig.getZ() / scalar);
        }

        /*** See function prototype in '.h' file ***/
        public static Vector operator +(Vector lhs, Vector rhs)
        {
            return new Vector(lhs.getX() + rhs.getX(), lhs.getY() + rhs.getY(), lhs.getZ() + rhs.getZ());
        }

        /*** See function prototype in '.h' file ***/
        public static Vector operator -(Vector lhs, Vector rhs)
        {
            return new Vector(lhs.getX() - rhs.getX(), lhs.getY() - rhs.getY(), lhs.getZ() - rhs.getZ());
        }

        /*** See function prototype in '.h' file ***/
        public Vector cross(Vector rhs)
        {
            Vector answer = new Vector();
            answer.setX(vector[1] * rhs.getZ() -
                vector[2] * rhs.getY());
            answer.setY(vector[2] * rhs.getX() -
                vector[0] * rhs.getZ());
            answer.setZ(vector[0] * rhs.getY() -
                vector[1] * rhs.getX());
            return answer;
        }

        /*** See function prototype in '.h' file ***/
        public Vector cross(double x, double y, double z)
        {
            Vector answer = new Vector();
            answer.setX(vector[1] * z -
                vector[2] * y);
            answer.setY(vector[2] * x -
                vector[0] * z);
            answer.setZ(vector[0] * y -
                vector[1] * x);
            return answer;
        }

        /*** See function prototype in '.h' file ***/
        public double dot(Vector rhs)
        {
            return getX() * rhs.getX() +
                getY() * rhs.getY() +
                getZ() * rhs.getZ();
        }

        /*** See function prototype in '.h' file ***/
        public double dot(double x, double y, double z)
        {
            return getX() * x +
                getY() * y +
                getZ() * z;
        }

        /*** See function prototype in '.h' file ***/
        public Vector project(Vector rhs)
        {
            Vector answer = rhs / rhs.getMagnitude();
            answer = answer * dot(answer);
            return answer;
        }

        /*** See function prototype in '.h' file ***/
        public Vector project(double x, double y, double z)
        {
            Vector answer = new Vector(x, y, z);
            return project(answer);
        }

        /*** See function prototype in '.h' file ***/
        public Vector projectToPlane(Vector normal)
        {
            Vector answer = project(normal);
            answer = getSelf() - answer;
            return answer;
        }

        /*** See function prototype in '.h' file ***/
        public Vector projectToPlane(double x, double y, double z)
        {
            Vector normal = new Vector(x, y, z);
            return projectToPlane(normal);
        }

        /*** See function prototype in '.h' file ***/
        public Vector transform(Vector xPrime, Vector yPrime, Vector zPrime)
        {
            return new Vector(dot(xPrime / xPrime.getMagnitude()),
                        dot(yPrime / yPrime.getMagnitude()),
                        dot(zPrime / zPrime.getMagnitude()));
        }

        /*** See function prototype in '.h' file ***/
        public Vector rotate(Vector axis, double rotation)
        {
            if (axis.getMagnitude() == 0 || rotation == 0)
            {
                return getSelf();
            }
            axis = axis / axis.getMagnitude();
            if (axis.getX() == getX() / magnitude && axis.getY() == getY() / magnitude && axis.getZ() == getZ() / magnitude)
            {
                return getSelf();
            }
            // a roundabout way of accomplishing the task
            // get a point in the plane that axis is normal to
            Vector xPrime = (axis + new Vector(axis.getX() * 2 + 1, axis.getY() * 3 + 1, axis.getZ() + 1)).projectToPlane(axis);
            // get a perpendicular vector to answer and axis
            Vector yPrime = axis.cross(xPrime);
            // transform this to rotation coordinate system
            Vector transformed = transform(xPrime, yPrime, axis);
            transformed = transformed.rotate2D(2, rotation);
            // transform back to old coordinate system
            return transformed = transformed.transform(new Vector(1, 0, 0).transform(xPrime, yPrime, axis),
                new Vector(0, 1, 0).transform(xPrime, yPrime, axis),
                new Vector(0, 0, 1).transform(xPrime, yPrime, axis));
        }

        /*** See function prototype in '.h' file ***/
        public Vector rotate2D(int normalAxis, double rotation)
        {
            int axis = normalAxis % 3;
            Vector answer = this.getSelf();
            double x = 0, y = 0;
            switch (axis)
            {
                case 0: // rotating about x axis
                    x = answer.getY(); y = answer.getZ();
                    answer.setY(x * Math.Cos(rotation) - y * Math.Sin(rotation));
                    answer.setZ(x * Math.Sin(rotation) + y * Math.Cos(rotation));
                    break;
                case 1: // rotating about y axis
                    x = answer.getZ(); y = answer.getX();
                    answer.setZ(x * Math.Cos(rotation) - y * Math.Sin(rotation));
                    answer.setX(x * Math.Sin(rotation) + y * Math.Cos(rotation));
                    break;
                case 2: // rotating about z axis
                    x = answer.getX(); y = answer.getY();
                    answer.setX(x * Math.Cos(rotation) - y * Math.Sin(rotation));
                    answer.setY(x * Math.Sin(rotation) + y * Math.Cos(rotation));
                    break;
            }
            return answer;
        }

        /*** See function prototype in '.h' file ***/
        public Vector rotateAboutPoint(Vector point, int normalAxis, double rotation)
        {
            return (getSelf() - point).rotate2D(normalAxis, rotation);
        }

        /*** See function prototype in '.h' file ***/
        public Vector getRotationAxis(Vector x, Vector y, Vector z, Vector xPrime, Vector yPrime, Vector zPrime)
        {
            Vector norm1 = new Vector(1, 1, 1).transform(x.transform(xPrime, yPrime, zPrime),
                y.transform(xPrime, yPrime, zPrime),
                z.transform(xPrime, yPrime, zPrime));
            Vector temp = new Vector(norm1);
            Vector norm2 = new Vector(-1, 1, -1).transform(x.transform(xPrime, yPrime, zPrime),
                y.transform(xPrime, yPrime, zPrime),
                z.transform(xPrime, yPrime, zPrime));
            norm1 = norm1 - new Vector(1, 1, 1);
            norm2 = norm2 - new Vector(-1, 1, -1);
            if (norm1.getMagnitude() != 0 || norm2.getMagnitude() != 0)
            {
                Vector axis = norm1.cross(norm2); // correct axis? maybe wrong direction tho
                norm1 = new Vector(1, 1, 1) - new Vector(1, 1, 1).project(axis);
                norm2 = temp - temp.project(axis);
                axis = norm1.cross(norm2);// corrected direction
                double amount = norm1.getAngleBetween(norm2);
                temp = x.rotate(axis, amount);
                if (temp.getAngleBetween(xPrime) > Math.PI - amount)
                {
                    amount = 2 * Math.PI - amount;
                }
                axis = axis / axis.getMagnitude() * amount; // store angle within axis
                return axis;
            }
            return new Vector(true);
        }

        /*** See function prototype in '.h' file ***/
        public double getAngleBetween(Vector rhs)
        {
            double dotted = dot(rhs);
            double cosTheta = dotted / (rhs.getMagnitude() * getMagnitude());
            cosTheta = (cosTheta > 1) ? 1 : cosTheta;
            return Math.Acos(cosTheta);
        }

        /*** See function prototype in '.h' file ***/
        public double getAngleBetween(double x, double y, double z)
        {
            double dotted = dot(x, y, z);
            double cosTheta = dotted / Math.Sqrt(x * x + y * y + z * z) / getMagnitude();
            cosTheta = (cosTheta > 1) ? 1 : cosTheta;
            return Math.Acos(cosTheta);
        }

        /*** See function prototype in '.h' file ***/
        public double getXYAngle()
        {
            double y = getY(), x = getX();
            double angle = Math.Atan(y / x);
            if (y > 0 && x > 0) //angle is in first quadrant
                return angle;
            else if (x < 0) // angle is in second or third quadrant
                return angle + Math.PI;
            else //angle is in fourth quadrant
                return 2 * Math.PI - angle;
        }

        public double this[int index]
        {
            get
            {
                try
                {
                    return vector[index];
                }
                catch (IndexOutOfRangeException)
                {
                    return System.Double.NaN;
                }
            }
        }


        /*** See function prototype in '.h' file ***/
        public double getX()
        {
            return vector[0];
        }

        /*** See function prototype in '.h' file ***/
        public double getY()
        {
            return vector[1];
        }

        /*** See function prototype in '.h' file ***/
        public double getZ()
        {
            return vector[2];
        }

        /*** See function prototype in '.h' file ***/
        public void setX(double x)
        {
            vector[0] = x;
            setMagnitude();
        }

        /*** See function prototype in '.h' file ***/
        public void setY(double y)
        {
            vector[1] = y;
            setMagnitude();
        }

        /*** See function prototype in '.h' file ***/
        public void setZ(double z)
        {
            vector[2] = z;
            setMagnitude();
        }

        /*** See function prototype in '.h' file ***/
        public void negate()
        {
            vector[0] *= -1;
            vector[1] *= -1;
            vector[2] *= -1;
        }

        /*** See function prototype in '.h' file ***/
        public bool isInvalid()
        {
            return invalid;
        }

        /*** See function prototype in '.h' file ***/
        /*
public void Vector::printVector()
{
    cout << "<" << getX() << ", " << getY() << ", " << getZ() << ">";
}*/

        /// <summary>
        /// Returns a comma-separated expansion of the Vector's components, i.e.
        /// "x,y,z"
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return "" + vector[0] + "," + vector[1] + "," + vector[2];
        }

        /*** See function prototype in '.h' file ***/
        private Vector getSelf()
        {
            Vector answer = new Vector(getX(), getY(), getZ());
            return answer;
        }
    }
}
