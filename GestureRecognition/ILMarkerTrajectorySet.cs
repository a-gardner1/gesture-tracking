using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ILNumerics;
using ILNumerics.Drawing;
using ILNumerics.Drawing.Plotting;

namespace GestureRecognition
{
    class ILMarkerTrajectorySet
    {
        ILArray<float> positions;

        public ILArray<float> Positions
        {
            get { return positions; }
        }

        ILArray<float> colors;
        ILArray<float> groups;
        /// <summary>
        /// Determines how different groups are colored based upon their relative group IDs
        /// </summary>
        Colormaps colorScheme;

        public Colormaps ColorScheme
        {
            get { return colorScheme; }
            set { colorScheme = value; }
        }
        /// <summary>
        /// Size of each marker's history.
        /// </summary>
        int bufferSize;

        public int BufferSize
        {
            get { return bufferSize; }
        }
        int numMarkers;

        public int NumMarkers
        {
            get { return numMarkers; }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="_bufferSize"></param>
        /// <param name="initialPositions"></param>
        /// <param name="groups"></param>
        /// <param name="_colorScheme"></param>
        public ILMarkerTrajectorySet(uint _bufferSize, List<Vector> initialPositions, ILArray<float> groups, Colormaps _colorScheme)
        {
            numMarkers = initialPositions.Count;
            this.bufferSize = (int)_bufferSize;
            positions = ILMath.zeros<float>(3, numMarkers * bufferSize);
            this.groups = groups;
            // fill up the buffer with copies of the initial position
            for (int i = 0; i < numMarkers; ++i)
            {
                for (int j = 0; j < bufferSize; ++j)
                {
                    insertVector(i, initialPositions[i]);
                }
            }
            colorScheme = _colorScheme;
        }

        private void insertVector(int markerID, Vector next)
        {
            //rotate queue for this marker
            positions[":", "" + (markerID * bufferSize) + ":" + ((markerID + 1) * bufferSize - 2)] 
                = positions[":", "" + (markerID * bufferSize + 1) + ":" + ((markerID + 1) * bufferSize - 1)];
            //insert marker at end of queue
            positions[0, (markerID + 1) * bufferSize - 1] = (float)next[0];
            positions[1, (markerID + 1) * bufferSize - 1] = (float)next[1];
            positions[2, (markerID + 1) * bufferSize - 1] = (float)next[2];
        }

        public void step(List<Vector> nextPositions)
        {
            if(nextPositions.Count != numMarkers)
            {
                throw new Exception("Wrong number of positions!");
            }
            else
            {
                for(int i = 0; i < numMarkers; ++i)
                {
                    insertVector(i, nextPositions[i]);
                }
            }
        }

    }
}
