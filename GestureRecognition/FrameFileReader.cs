using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GestureRecognition
{
    namespace Obsolete
    {
        class FrameFileReader
        {
            public enum FileType
            {
                STATIC, DYNAMIC, UNKNOWN
            }
            private string fileName;
            private System.IO.StreamReader reader;
            private FileType fileType;

            public FrameFileReader()
            {
                fileName = "";
            }

            public FileType getFileType()
            {
                return fileType;
            }

            public void setFile(string name)
            {
                fileName = name;
                string extension = name.Substring(fileName.Length - 8);
                if (extension.Equals(".stc.csv"))
                {
                    fileType = FileType.STATIC;
                }
                else if (extension.Equals(".dyn.csv"))
                {
                    fileType = FileType.DYNAMIC;
                }
                else
                {
                    fileType = FileType.UNKNOWN;
                }
                if (fileType == FileType.STATIC || fileType == FileType.DYNAMIC)
                    reader = new System.IO.StreamReader(fileName);
                else
                    reader = null;
            }

            /**
             * Reads a frame from the current file. Returns null at end of file or start/end of dynamic gesture.
             * */
            public Tuple<Frame, Frame> nextFrame(bool transformToLocal, bool prune, double radius)
            {
                if (reader != null && fileType != FileType.UNKNOWN)
                {
                    string line = reader.ReadLine();
                    if (line != null && line.Length > 0)
                    {
                        if (fileType == FileType.STATIC || (fileType == FileType.DYNAMIC && !line.Equals("Start")))
                        {
                            return Frame.parseFrame(line, transformToLocal, prune, radius);
                        }
                    }
                }
                return null;
            }

            /**
             * Reads all frames from the current file and returns a list.
             * The start of each dynamic gesture read this way is a null Tuple.
             * */
            public List<Tuple<Frame, Frame>> readAllFrames(bool transformToLocal, bool prune, double radius)
            {
                List<Tuple<Frame, Frame>> frames = new List<Tuple<Frame, Frame>>();
                Tuple<Frame, Frame> next = nextFrame(transformToLocal, prune, radius);
                if (fileType == FileType.STATIC || fileType == FileType.UNKNOWN)
                {
                    while (next != null)
                    {
                        frames.Add(next);
                        next = nextFrame(transformToLocal, prune, radius);
                    }
                }
                else
                {
                    // go until we get two nulls in a row
                    Tuple<Frame, Frame> prev = next;
                    next = nextFrame(transformToLocal, prune, radius);
                    while (!(prev == null && next == null))
                    {
                        frames.Add(prev);
                        prev = next;
                        next = nextFrame(transformToLocal, prune, radius);
                    }
                }
                return frames;
            }
        }
    }
}