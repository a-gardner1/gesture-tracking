using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ViconDataStreamSDK.DotNET;
using System.Collections.ObjectModel;

namespace GestureRecognition
{
    
    class FilteredDataStream
    {
        public class ConnectionException : Exception
        {
            public ConnectionException()
            {
            }

            public ConnectionException(string message)
                : base(message)
            {

            }

            public ConnectionException(string message, Exception inner)
                : base(message, inner)
            {

            }

        }

        private ViconDataStreamSDK.DotNET.Client vicon;
        private SimpleHandTracker filter;
        private List<Vector> globalPositions = new List<Vector>();

        internal ReadOnlyCollection<Vector> GlobalPositions
        {
            get { return globalPositions.AsReadOnly(); }
        }

        private List<Vector> lhPat;

        internal ReadOnlyCollection<Vector> LHPat
        {
            get { return lhPat.AsReadOnly(); }
        }

        private double dt = 0.01;

        public double Dt
        {
            get { return dt; }
        }

        private uint frameNumber;

        private string[] lhNames = { "Origin", "X-Axis", "Y-Axis", "Extra" };

        

        public FilteredDataStream(double Q, double R)
        {
            vicon = new Client();
            filter = new SimpleHandTracker(11, Q, R);
            lhPat = new List<Vector>(new Vector[4]);
        }

        public void initiate()
        {
            vicon.Connect("localhost");
            if (vicon.IsConnected().Connected)
            {
                vicon.SetStreamMode(ViconDataStreamSDK.DotNET.StreamMode.ClientPull);
                vicon.EnableMarkerData();
                vicon.EnableUnlabeledMarkerData();
            }
            else
            {
                throw new ConnectionException("Failed to initialize connection to Vicon DataStream server. Perhaps a cable is unplugged or the server is offline?");
            }
        }

        /**
         * Pull and process (filter) the latest frame available from Vicon. 
         * */
        public void getFrame()
        {
            if(vicon.IsConnected().Connected)
            {
                var frameResult = vicon.GetFrame();
                if(frameResult.Result.Equals(Result.NoFrame))
                {
                    throw new ConnectionException("Error: Cannot retrieve frame. A resource used by the Vicon DataStream SDK likely was not released from a previous execution. Try restarting the computer.");
                }
                dt = 1.0 / vicon.GetFrameRate().FrameRateHz;
                frameNumber = vicon.GetFrameNumber().FrameNumber;
                globalPositions.Clear();
                uint numUnlabeled = vicon.GetUnlabeledMarkerCount().MarkerCount;
                double[] u;
                for (uint i = 0; i < numUnlabeled; ++i)
                {
                    u = vicon.GetUnlabeledMarkerGlobalTranslation(i).Translation;
                    globalPositions.Add(new Vector(u[0], u[1], u[2]));
                }
                for(int i = 0; i < 4; ++i)
                {
                    var v = vicon.GetMarkerGlobalTranslation("LHPat", lhNames[i]);
                    if(v.Occluded) {
                        lhPat[i] = new Vector(true);
                    }
                    else
                    {
                        lhPat[i] = new Vector(v.Translation[0], v.Translation[1], v.Translation[2]);
                    }
                }

            }
            else
            {
                throw new ConnectionException("No connection to Vicon DataStream server established. Either the connection was lost or it was never initiated.");
            }
        }

    }
}
