using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GestureRecognition
{
    namespace Obsolete
    {
        /* *
         * Responsible for reading and storing data from multiple users organized by class.
         * May also be used for partial preprocessing of frames as they are read from file 
         * by transforming to local coordinates.
         * */
        class DataReader
        {
            string root;
            Dictionary<string, List<List<Frame>>> staticL = new Dictionary<string, List<List<Frame>>>();
            Dictionary<string, List<List<Frame>>> staticR = new Dictionary<string, List<List<Frame>>>();
            Dictionary<string, List<List<List<Frame>>>> dynamicL = new Dictionary<string, List<List<List<Frame>>>>();
            Dictionary<string, List<List<List<Frame>>>> dynamicR = new Dictionary<string, List<List<List<Frame>>>>();
            int maxStaticID = 0, maxDynamicID = 0;

            public DataReader()
            {
                root = "";
            }

            /* *
             * Set the path to the root "Users" directory. 
             * Clears previously stored frames if the new path is valid.
             * @param   dataPath  The path of the "Users" root directory.
             * */
            public bool setRootDir(string dataPath)
            {
                if (System.IO.Path.GetFileName(dataPath) == "Users")
                {
                    root = dataPath;
                    // clear previous data
                    staticL = new Dictionary<string, List<List<Frame>>>();
                    staticR = new Dictionary<string, List<List<Frame>>>();
                    dynamicL = new Dictionary<string, List<List<List<Frame>>>>();
                    dynamicR = new Dictionary<string, List<List<List<Frame>>>>();
                    return true;
                }
                else
                {
                    return false;
                }
            }

            /* *
             * Load static gestures for all users into memory.
             * 
             * @param   left  Load left (true) or right (false) gesture data?
             * */
            public bool loadStaticGestures(bool left)
            {
                List<string> users = new List<string>(System.IO.Directory.EnumerateDirectories(root));
                foreach (string user in users)
                {
                    loadStaticGestures(System.IO.Path.GetFileName(user), left);
                }
                return true;
            }

            /* *
             * Returns true on success, false on failure.
             * Overwrites previously stored information if called on same user twice.
             * Transforms all data to local coordinates and performs basic pruning of all markers
             * more than 200 mm from hand origin.
             * 
             * @param   left  Load left (true) or right (false) gesture data?
             * */
            public bool loadStaticGestures(string userName, bool left)
            {
                string path = root + "\\" + userName + "\\" + (left ? "Left" : "Right");
                try
                {
                    List<string> files = new List<string>(System.IO.Directory.EnumerateFiles(path, "*.stc.csv"));
                    if (left)
                    {
                        staticL[userName] = new List<List<Frame>>();
                    }
                    else
                    {
                        staticR[userName] = new List<List<Frame>>();
                    }
                    foreach (string file in files)
                    {
                        // do not assume that each file is a distinct class
                        FrameFileReader reader = new FrameFileReader();
                        reader.setFile(file);
                        List<Tuple<Frame, Frame>> frames = reader.readAllFrames(true, true, 200);
                        List<List<Frame>> storage = left ? staticL[userName] : staticR[userName];
                        for (int f = 0; f < frames.Count; ++f)
                        {
                            Frame current = left ? frames[f].Item1 : frames[f].Item2;
                            if (current.ID >= storage.Count)
                            {
                                // create storage space for this class
                                for (int i = storage.Count; i <= current.ID; ++i)
                                {
                                    storage.Add(new List<Frame>());
                                }
                                maxStaticID = current.ID > maxStaticID ? current.ID : maxStaticID;
                            }
                            storage[current.ID].Add(current);
                        }
                    }
                    return true;
                }
                catch (System.IO.DirectoryNotFoundException e)
                {
                    System.Console.Write("Directory not found");
                    return false;
                }
            }

            /* *
             * Load dynamic gestures for all users into memory.
             * 
             * @param   left  Load left (true) or right (false) gesture data?
             * */
            public bool loadDynamicGestures(bool left)
            {
                List<string> users = new List<string>(System.IO.Directory.EnumerateDirectories(root));
                foreach (string user in users)
                {
                    loadDynamicGestures(System.IO.Path.GetFileName(user), left);
                }
                return true;
            }

            /* *
             * Returns true on success, false on failure.
             * Overwrites previously stored information if called on same user twice.
             * Transforms all data to local coordinates and performs basic pruning of all markers
             * more than 200 mm from hand origin.
             * 
             * @param   left  Load left (true) or right (false) gesture data?
             * */
            public bool loadDynamicGestures(string userName, bool left)
            {
                string path = root + "\\" + userName + "\\" + (left ? "Left" : "Right");
                try
                {
                    List<string> files = new List<string>(System.IO.Directory.EnumerateFiles(path, "*.dyn.csv"));
                    if (left)
                    {
                        dynamicL[userName] = new List<List<List<Frame>>>();
                    }
                    else
                    {
                        dynamicR[userName] = new List<List<List<Frame>>>();
                    }
                    foreach (string file in files)
                    {
                        // do not assume that each file is a distinct class
                        FrameFileReader reader = new FrameFileReader();
                        reader.setFile(file);
                        List<Tuple<Frame, Frame>> frames = reader.readAllFrames(true, true, 200);
                        List<List<List<Frame>>> storage = left ? dynamicL[userName] : dynamicR[userName];
                        bool first = true; // whether we are on the first frame of a new dynamic gesture
                        int currentID = 0; // first frame sets ID for succeeding frames of dynamic gesture
                        for (int f = 0; f < frames.Count; ++f)
                        {
                            if (frames[f] != null)
                            {
                                Frame current = left ? frames[f].Item1 : frames[f].Item2;
                                if (first && current.ID >= storage.Count) // make space for this class
                                {
                                    // create storage space for this class
                                    for (int i = storage.Count; i <= current.ID; ++i)
                                    {
                                        storage.Add(new List<List<Frame>>());
                                    }
                                    maxDynamicID = current.ID > maxDynamicID ? current.ID : maxDynamicID;
                                }
                                if (first) // make space for this gesture
                                {
                                    currentID = current.ID;
                                    storage[currentID].Add(new List<Frame>());
                                }
                                storage[currentID].Last().Add(current);
                                first = false;
                            }
                            else
                            {
                                first = true;
                            }
                        }
                    }
                    return true;
                }
                catch (System.IO.DirectoryNotFoundException e)
                {
                    System.Console.Write("Directory not found");
                    return false;
                }
            }

            /* *
             * Returns all left or right static gestures currently stored in the reader,
             * organized by user.
             * */
            public List<List<List<Frame>>> getAllStaticGesturesByUser(bool left)
            {
                List<string> users = new List<string>(System.IO.Directory.EnumerateDirectories(root));
                List<List<List<Frame>>> answer = new List<List<List<Frame>>>();
                foreach (string user in users)
                {
                    answer.Add(getStaticGestures(System.IO.Path.GetFileName(user), left));
                }
                return answer;
            }

            public List<List<List<List<Frame>>>> getAllDynamicGesturesByUser(bool left)
            {
                List<string> users = new List<string>(System.IO.Directory.EnumerateDirectories(root));
                List<List<List<List<Frame>>>> answer = new List<List<List<List<Frame>>>>();
                foreach (string user in users)
                {
                    answer.Add(getDynamicGestures(System.IO.Path.GetFileName(user), left));
                }
                return answer;
            }

            public List<List<Frame>> getAllStaticGestures(bool left)
            {
                List<List<Frame>> answer = new List<List<Frame>>();
                for (int i = 0; i <= maxStaticID; ++i)
                {
                    answer.Add(getStaticGesture(i, left));
                }
                return answer;
            }

            public List<List<List<Frame>>> getAllDynamicGestures(bool left)
            {
                List<List<List<Frame>>> answer = new List<List<List<Frame>>>();
                for (int i = 0; i < maxStaticID; ++i)
                {
                    answer.Add(getDynamicGesture(i, left));
                }
                return answer;
            }

            // get gestures user by user
            public List<List<Frame>> getStaticGestures(string userName, bool left)
            {
                return left ? staticL[userName] : staticR[userName];
            }

            public List<List<List<Frame>>> getDynamicGestures(string userName, bool left)
            {
                return left ? dynamicL[userName] : dynamicR[userName];
            }

            // get gestures by class
            public List<Frame> getStaticGesture(int id, bool left)
            {
                List<Frame> gestures = new List<Frame>();
                foreach (KeyValuePair<string, List<List<Frame>>> data in (left ? staticL : staticR))
                {
                    if (data.Value.Count > id)
                    {
                        gestures.AddRange(data.Value[id]);
                    }
                }
                return gestures;
            }

            public List<List<Frame>> getDynamicGesture(int id, bool left)
            {
                List<List<Frame>> gestures = new List<List<Frame>>();
                foreach (KeyValuePair<string, List<List<List<Frame>>>> data in (left ? dynamicL : dynamicR))
                {
                    if (data.Value.Count > id)
                    {
                        gestures.AddRange(data.Value[id]);
                    }
                }
                return gestures;
            }
        }
    }
}
