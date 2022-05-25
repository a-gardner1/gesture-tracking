using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GestureRecognition
{
    namespace Obsolete
    {
        /**
         * Doubles as both a continuous and discrete observation. If treated as a continuous observation, 
         * all 4 subobservation types should be considered (global, local, rotation, marker distribution).
         * If treated as a discrete observation, only the marker distribution is used to apply a label ID
         * for discretization, and thus only that will be modeled by a DiscreteDistribution.
         * See FrameDistribution, which models all 4 subobservation types either continuously or discretely.
         * */
        class Frame
        {
            private static int numTypes = 1; // should be set externally by frame clustering process with setNumTypes()

            public Vector global; // global velocity vector
            public Vector local; // local velocity vector (relative to previous frame(s))
            public Vector rotation; // local rotation vector (relative to previous frame(s))
            public List<Vector> markers; // local coordinates
            public Vector Origin; // origin in global coordinates
            public Vector XMarker;
            public Vector XAxis; // x-axis in global coordinates
            public Vector YMarker;
            public Vector YAxis; // y-axis in global coordinates
            public Vector Extra;
            public Vector ZAxis; // z-axis in global coordinates
            public int ID; // the class ID, 1-indexed
            public double time;
            public bool isLeft;

            public Frame()
            {
                time = 0;
                ID = 0;
                global = new Vector(true);
                local = new Vector(true);
                rotation = new Vector(true);
                markers = new List<Vector>();
                Origin = new Vector(true);
                XAxis = new Vector(true);
                YAxis = new Vector(true);
                ZAxis = new Vector(true);
            }


            /**
             * Given a csv-formatted line in the expected format, parses and returns both
             * left and right hand frames with local coordinates, classifications,
             * */
            public static Tuple<Frame, Frame> parseFrame(string line, bool transform, bool prune = false, double radius = 200)
            {
                Tuple<Frame, Frame> answer = new Tuple<Frame, Frame>(new Frame(), new Frame());
                List<string> tokens = new List<string>(line.Split(','));
                if (tokens.Count >= 27)
                {
                    // skip time field at tokens[0]
                    answer.Item1.time = Double.Parse(tokens[0]);
                    answer.Item2.time = answer.Item1.time;
                    // read in raw data
                    answer.Item1.ID = (int)Double.Parse(tokens[1]);
                    answer.Item2.ID = (int)Double.Parse(tokens[2]);
                    answer.Item1.Origin = parseVector(tokens, 3);
                    answer.Item1.XMarker = parseVector(tokens, 6);
                    answer.Item1.YMarker = parseVector(tokens, 9);
                    answer.Item1.Extra = parseVector(tokens, 12);
                    answer.Item2.Origin = parseVector(tokens, 15);
                    answer.Item2.XMarker = parseVector(tokens, 18);
                    answer.Item2.YMarker = parseVector(tokens, 21);
                    answer.Item2.Extra = parseVector(tokens, 24);
                    for (int i = 27; i < tokens.Count; i += 3)
                    {
                        answer.Item1.markers.Add(parseVector(tokens, i));
                        answer.Item2.markers.Add(parseVector(tokens, i));
                    }
                    // now transform to local coordinate system
                    if (prune)
                    {
                        pruneOutliers(true, answer.Item1, radius);
                        pruneOutliers(true, answer.Item2, radius);
                    }
                    if (transform)
                    {
                        transformToLocal(answer.Item1);
                        transformToLocal(answer.Item2);
                    }
                }
                return answer;
            }

            public static string ToString(Frame left, Frame right, int leftID, int rightID, double time, bool storeBoth)
            {
                StringBuilder answer = new StringBuilder();
                answer.Append(time + ",");
                answer.Append(leftID + ",");
                answer.Append(rightID + ",");
                answer.Append(left.Origin + ",");
                answer.Append(left.XMarker + ",");
                answer.Append(left.YMarker + ",");
                answer.Append(left.Extra + ",");
                answer.Append(right.Origin + ",");
                answer.Append(right.XMarker + ",");
                answer.Append(right.YMarker + ",");
                answer.Append(right.Extra);
                for (int i = 0; i < left.markers.Count; ++i)
                {
                    answer.Append("," + left.markers[i]);
                }
                if (storeBoth)
                {
                    for (int i = 0; i < right.markers.Count; ++i)
                    {
                        answer.Append("," + right.markers[i]);
                    }
                }
                return answer.ToString();
            }

            private static Vector parseVector(List<string> tokens, int index)
            {
                Vector answer = new Vector(Double.Parse(tokens[index]),
                    Double.Parse(tokens[index + 1]),
                    Double.Parse(tokens[index + 2]));
                if (answer.getMagnitude() == 0)
                {
                    return new Vector(true);
                }
                else
                {
                    return answer;
                }
            }

            private static void transformToLocal(Frame raw)
            {
                if (!raw.Origin.isInvalid())
                {
                    if (raw.XMarker.getMagnitude() != 0 && raw.YMarker.getMagnitude() != 0)
                    {
                        raw.XAxis = raw.XMarker - raw.Origin;
                        raw.YAxis = raw.YMarker - raw.Origin;
                        raw.ZAxis = raw.XAxis.cross(raw.YAxis);
                    }
                    else if (raw.YMarker.getMagnitude() != 0)
                    {
                        raw.YAxis = raw.YMarker - raw.Origin;
                        raw.ZAxis = (raw.Extra - raw.Origin).cross(raw.YAxis);
                        raw.XAxis = raw.YAxis.cross(raw.ZAxis);
                    }
                    else if (raw.XMarker.getMagnitude() != 0)
                    {
                        raw.XAxis = raw.XMarker - raw.Origin;
                        raw.YAxis = raw.Extra - raw.Origin;
                        raw.ZAxis = raw.XAxis.cross(raw.YAxis);
                        raw.YAxis = raw.ZAxis.cross(raw.XAxis);
                    }
                    raw.XAxis = raw.XAxis / raw.XAxis.getMagnitude();
                    for (int i = 0; i < raw.markers.Count; ++i)
                    {
                        raw.markers[i] = (raw.markers[i] - raw.Origin).transform(raw.XAxis, raw.YAxis, raw.ZAxis);
                    }
                }
            }

            private static void transformToGlobal(Frame processed)
            {
                if (!processed.Origin.isInvalid() && !processed.YAxis.isInvalid())
                {
                    Vector inverseX = new Vector(1, 0, 0);
                    Vector inverseY = new Vector(0, 1, 0);
                    Vector inverseZ = new Vector(0, 0, 1);
                    inverseX = inverseX.transform(processed.XAxis, processed.YAxis, processed.ZAxis);
                    inverseY = inverseY.transform(processed.XAxis, processed.YAxis, processed.ZAxis);
                    inverseZ = inverseZ.transform(processed.XAxis, processed.YAxis, processed.ZAxis);
                    for (int i = 0; i < processed.markers.Count; ++i)
                    {
                        processed.markers[i] = processed.markers[i].transform(inverseX, inverseY, inverseZ);
                    }
                }
            }

            /* *
             * Delete markers farther than the radius away from the origin and invalid markers (magnitude 0)
             * 
             * @param isRaw Is this in global or local coordinates?
             * */
            private static void pruneOutliers(bool isRaw, Frame frame, double radius)
            {
                for (int i = frame.markers.Count - 1; i >= 0; --i)
                {
                    if (frame.markers[i].isInvalid() ||
                        (isRaw && (frame.markers[i] - frame.Origin).getMagnitude() > radius) ||
                        (!isRaw && frame.markers[i].getMagnitude() > radius))
                    {
                        frame.markers.RemoveAt(i);
                    }
                }
            }
        }
    }
}