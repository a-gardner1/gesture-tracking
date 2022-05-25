using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GestureRecognition
{
    class Utility
    {
        public static void fill(List<int> list, int value)
        {
            for (int i = 0; i < list.Count; ++i)
            {
                list[i] = value;
            }
        }

        public static void fill(List<double> list, double value)
        {
            for (int i = 0; i < list.Count; ++i)
            {
                list[i] = value;
            }
        }

        public static List<List<T>> make2DList<T>(int s1, int s2)
        {
            List<List<T>> answer = new List<List<T>>(new List<T>[s1]);
            for (int i = 0; i < s1; ++i)
            {
                answer[i] = new List<T>(new T[s2]);
            }
            return answer;
        }

        public static List<List<List<T>>> make3DList<T>(int s1, int s2, int s3) 
        {
            List<List<List<T>>> answer = new List<List<List<T>>>(new List<List<T>>[s1]);
            for (int i = 0; i < s1; ++i)
            {
                answer[i] = make2DList<T>(s2, s3);
            }
            return answer;
        }


        public static List<List<List<List<T>>>> make4DList<T>(int s1, int s2, int s3, int s4)
        {
            List<List<List<List<T>>>> answer = new List<List<List<List<T>>>>(new List<List<List<T>>>[s1]);
            for (int i = 0; i < s1; ++i)
            {
                answer[i] = make3DList<T>(s2, s3, s4);
            }
            return answer;
        }

        public static List<List<List<List<List<T>>>>> make5DList<T>(int s1, int s2, int s3, int s4, int s5)
        {
            List<List<List<List<List<T>>>>> answer = new List<List<List<List<List<T>>>>>(new List<List<List<List<T>>>>[s1]);
            for (int i = 0; i < s1; ++i)
            {
                answer[i] = make4DList<T>(s2, s3, s4, s5);
            }
            return answer;
        }

        /**
         *  Returns a 2d list of uniform values normalized on the final index.
         * */
        public static List<List<double>> makeUniform2DList(int s1, int s2)
        {
            List<List<double>> answer = make2DList<double>(s1, s2);
            for (int i = 0; i < s1; ++i)
            {
                for (int j = 0; j < s2; ++j)
                {
                    answer[i][j] = 1.0/(double)s2;
                }
            }
            return answer;
        }

        /**
         *  Returns a 2d list of random value normalized on the final index.
         * */
        public static List<List<double>> makeRandom2DList(int s1, int s2, Random rand)
        {
            List<List<double>> answer = make2DList<double>(s1, s2);
            double sum = 0;
            for (int i = 0; i < s1; ++i)
            {
                sum = 0;
                for (int j = 0; j < s2; ++j)
                {
                    sum += (answer[i][j] = rand.NextDouble());
                }
                for (int j = 0; j < s2; ++j)
                {
                    answer[i][j] /= sum;
                }
            }
            return answer;
        }

        /**
         *  Returns a 3d list of random value normalized on the final index.
         * */
        public static List<List<List<double>>> makeRandom3DList(int s1, int s2, int s3, Random rand)
        {
            List<List<List<double>>> answer = make3DList<double>(s1, s2, s3);
            for (int i = 0; i < s1; ++i)
            {
                answer[i] = makeRandom2DList(s2, s3, rand);
            }
            return answer;
        }


        /**
         *  Returns a 3d list of uniform values normalized on the final index.
         * */
        public static List<List<List<double>>> makeUniform3DList(int s1, int s2, int s3)
        {
            List<List<List<double>>> answer = make3DList<double>(s1, s2, s3);
            for (int i = 0; i < s1; ++i)
            {
                answer[i] = makeUniform2DList(s2, s3);
            }
            return answer;
        }

        public static void permuteList<T>(List<T> list, Random seed)
        {
            Random permute = seed;
            for (int i = list.Count; i > 0; --i)
            {
                int other = permute.Next() % i;
                T temp = list[other];
                list[other] = list[i - 1];
                list[i - 1] = temp;
            }
        }

        public static void permuteParallel<T, U>(List<T> list, List<U> list2, Random seed)
        {
            Random permute = seed;
            for (int i = list.Count; i > 0; --i)
            {
                int other = permute.Next() % i;
                T temp = list[other];
                list[other] = list[i - 1];
                list[i - 1] = temp;
                U temp2 = list2[other];
                list2[other] = list2[i - 1];
                list2[i - 1] = temp2;
            }
        }

        public static void permuteParallel<T, U, V>(List<T> list, List<U> list2, List<V> list3, Random seed)
        {
            Random permute = seed;
            for (int i = list.Count; i > 0; --i)
            {
                int other = permute.Next() % i;
                T temp = list[other];
                list[other] = list[i - 1];
                list[i - 1] = temp;
                U temp2 = list2[other];
                list2[other] = list2[i - 1];
                list2[i - 1] = temp2;
                V temp3 = list3[other];
                list3[other] = list3[i - 1];
                list3[i - 1] = temp3;
            }
        }

        public static void permuteParallel<T, U, V, X>(List<T> list, List<U> list2, List<V> list3, List<X> list4, Random seed)
        {
            Random permute = seed;
            for (int i = list.Count; i > 0; --i)
            {
                int other = permute.Next() % i;
                T temp = list[other];
                list[other] = list[i - 1];
                list[i - 1] = temp;
                U temp2 = list2[other];
                list2[other] = list2[i - 1];
                list2[i - 1] = temp2;
                V temp3 = list3[other];
                list3[other] = list3[i - 1];
                list3[i - 1] = temp3;
                X temp4 = list4[other];
                list4[other] = list4[i - 1];
                list4[i - 1] = temp4;
            }
        }

        public static List<T> randomSublist<T>(List<T> list, int sampleCount, Random seed)
        {
            if (list == null || sampleCount >= list.Count)
            {
                return list == null ? new List<T>() : list;
            }
            else
            {
                List<T> answer = new List<T>(list);
                permuteList<T>(answer, seed);
                answer.RemoveRange(sampleCount, answer.Count - sampleCount);
                return answer;
            }
        }

        public static double getStandardDeviation(List<double> nums, double mean)
        {
            double dev = 0;
            foreach (int num in nums)
            {
                dev += (num - mean) * (num - mean);
            }
            dev /= ((double)nums.Count - 1.0);
            return Math.Sqrt(dev);
        }

        public static void writeVectorListToFile(List<Vector> vectors, System.IO.StreamWriter writer)
        {
            if (vectors.Count > 0)
            {
                writer.WriteLine(vectors[0].ToString());
                for (int i = 1; i < vectors.Count; ++i)
                {
                    writer.WriteLine(vectors[i].ToString());
                }
            }
        }

        public static Tuple<double, double> getMeanAndStdDev(List<double> values)
        {
            double mean = 0;
            double deviation = 0;
            if (values.Count > 0)
            {
                for (int i = 0; i < values.Count; ++i)
                {
                    mean += values[i];
                }
                mean /= values.Count;
                if (values.Count > 1)
                {
                    for (int i = 0; i < values.Count; ++i)
                    {
                        deviation += (values[i] - mean) * (values[i] - mean);
                    }
                    deviation /= values.Count - 1;
                }
                else
                {
                    deviation = Double.NaN;
                }
            }
            else
            {
                deviation = Double.NaN;
            }
            return new Tuple<double, double>(mean, Math.Sqrt(deviation));
        }

        
        public static void printMatrix(MathNet.Numerics.LinearAlgebra.Matrix<double> mat)
        {
            for (int r = 0; r < mat.RowCount; ++r)
            {
                for (int c = 0; c < mat.ColumnCount; ++c)
                {
                    System.Console.Write(mat[r, c].ToString("0.###") + "\t");
                }
                System.Console.WriteLine();
            }
        }
    }
}
