using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics;

namespace GestureRecognition
{
    abstract class Assignment
    {
        static bool initialized = false;
        static int reserveSize = 50;
        static List<double> pi = new List<double> (new double[reserveSize]); // labeling function, ordering: source, test elements, ref elements, sink
        static List<int> S = new List<int>(new int [reserveSize]); //s, test elements, ref elements, t
        static List<double> sigma  = new List<double>(new double [reserveSize]); // cumulative weights
        static List<int> predecessor  = new List<int>(new int [reserveSize]); // node that minimized sigma for given element
        static List<int> contained  = new List<int>(new int [reserveSize]); // whether an element is in S; 1 yes, -2 no
        // -2 indicates infinity or absence; special boundary conditions
        static double infinity = Double.MaxValue; // used as the distance to fictitious nodes

        private static void initialize(int size)
        {
            int osize = reserveSize;
            while (size > reserveSize)
            {
                pi.Add(0);
                S.Add(-2);
                sigma.Add(-2);
                predecessor.Add(-2);
                contained.Add(-2);
                ++reserveSize;
            }
            if (osize == reserveSize) osize = size;
            for(int i = 0; i < osize; ++i) {
                pi[i] = 0;
                S[i] = -2;
                sigma[i] = -2.0;
                predecessor[i] = -2;
                contained[i] = -2;
            }
            initialized = true;
        }

        /*
        * Subordinate function of augmentFlow. Rotates all indices (> index && < ceiling) of S one index to the left.
        */
        private static void rotateS(int index, int ceiling)
        {
	        for(int i = index+1; i < ceiling; ++i) {
		        S[i-1] = S[i];
	        }
        }

        /*
        * Subordinate function of getAssignment(). Augments the flow described by the given and global variables.
        * Assumes that S, sigma, predecessor and contained have been properly set beforehand.
        */
        private static double augmentFlow(double flowK, List<int> assignment, List<int> revAssign, MathNet.Numerics.LinearAlgebra.Matrix<double> matrix, int nodeCount)
        {
            int refSize = matrix.RowCount;
            int testSize = matrix.ColumnCount;
            int size = 2 * nodeCount + 2;
            S[0] = -1; // the source; can only access test elements
            contained[0] = 1;
            sigma[0] = 0; // no cost to reach the source
            int count = 1; // count of current elements in S
            int u;
            int uStar;
            int pointer;
            // shortest path algorithm (Dijkstra's)
            while (S[0] != -2)
            {
                uStar = S[0] + 1;
                pointer = 0;
                for (u = 0; u < count; u++)
                {
                    if (sigma[S[u] + 1] < sigma[uStar])
                    {
                        uStar = S[u] + 1;
                        pointer = u;
                    }
                }
                uStar = uStar - 1; // now uStar refers to actual element
                //std::cout << uStar << std::endl;
                contained[uStar + 1] = -2; // remove from S
                S[pointer] = -2;
                rotateS(pointer, count);
                //std::rotate(S.begin()+pointer, S.begin()+pointer+1, S.begin()+count); // bottleneck
                --count;
                if (uStar == -1)
                { // at the source
                    // source can reach all unmatched test elements, add all to S
                    for (u = 0; u < nodeCount; u++)
                    {
                        if (assignment[u] == -1)
                        {
                            S[count++] = u;
                            contained[u + 1] = 1;
                            sigma[u + 1] = pi[uStar + 1] - pi[u + 1]; // can reach at no cost
                            predecessor[u + 1] = uStar + 1;
                        }
                    }
                }
                else if (uStar < nodeCount)
                { // at a test element
                    // can reach all ref elements, except currently matched
                    int matched = assignment[uStar];
                    for (u = 0; u < nodeCount; ++u)
                    {
                        int index = nodeCount + u + 1;
                        if (matched != u)
                        {
                            double temp = sigma[uStar + 1] + pi[uStar + 1] - pi[index];
                            temp = (u >= refSize || uStar >= testSize) ? (temp + infinity) : (temp + matrix[u, uStar]);
                            if (sigma[index] == -2 || temp + 0.00000000001 < sigma[index])
                            {//
                                if (contained[index] == -2)
                                {
                                    S[count++] = index - 1;
                                    contained[index] = 1;
                                }
                                sigma[index] = temp;
                                predecessor[index] = uStar + 1;
                            }
                        }
                    }
                }
                else if (uStar < (2 * nodeCount) && uStar >= nodeCount)
                {
                    // at a ref element
                    // can reach matched test element or sink
                    u = revAssign[uStar - nodeCount];
                    if (u != -1)
                    { // if matched
                        double temp = sigma[uStar + 1] + pi[uStar + 1] - pi[u + 1];//
                        temp = (uStar - nodeCount >= refSize || u >= testSize) ? (temp + infinity * -1) : (temp + matrix[uStar - nodeCount, u] * -1);
                        if (sigma[u + 1] == -2 || temp + 0.00000000001 < sigma[u + 1])
                        { //
                            if (contained[u + 1] == -2)
                            {
                                S[count++] = u;
                                contained[u + 1] = 1;
                            }
                            sigma[u + 1] = temp;
                            predecessor[u + 1] = uStar + 1;
                        }
                    }
                    // handle sink, only if not matched
                    else
                    {
                        int index = size - 1;
                        double temp = sigma[uStar + 1] + pi[uStar + 1] - pi[index];
                        if (sigma[index] == -2 || temp < sigma[index])
                        {
                            if (contained[index] == -2)
                            {
                                S[count++] = index - 1;
                                contained[index] = 1;
                            }
                            sigma[index] = temp;
                            predecessor[index] = uStar + 1;
                        }
                    }
                }
                else
                { // at the sink, nowhere to go
                    //break;
                }
            }
            /*std::copy(predecessor,
                      predecessor+size,
                      std::ostream_iterator<int>(std::cout,",")
                     );
            std::cout << std::endl;
            std::copy(sigma,
                      sigma+size,
                      std::ostream_iterator<double>(std::cout,",")
                     );
            std::cout << std::endl;*/
            int refer = predecessor[size - 1];
            int pred;
            // reconstruct augmenting path, if it exists
            if (refer != -2)
            {
                //std::cout << ref << ">";
                bool testFlag = false; //if backtracking on test element
                while (predecessor[refer] != 0)
                { // until we reach the source
                    pred = predecessor[refer];
                    //std::cout << pred << ">";
                    // destroy matching
                    if (testFlag == true)
                    {
                        //(*assignment)[ref-1] = -1; <== This took many hours to track down and comment out. This line remains as a testament to the sweat, tears (figurative), and cursewords that were profusely shed in the previously feared futile search.
                        revAssign[pred - nodeCount - 1] = -1;
                    }
                    // make new matching
                    else
                    {
                        assignment[pred - 1] = refer - nodeCount - 1;
                        revAssign[refer - nodeCount - 1] = pred - 1;
                    }
                    refer = pred;
                    testFlag = !testFlag;

                }
                //std::cout << std::endl;
                // update labeling function pi
                for (u = 0; u < size; ++u)
                {
                    if (sigma[u] != -2)
                    {
                        pi[u] = pi[u] + sigma[u];
                    }
                    else
                    {
                        pi[u] = -2;
                    }
                }
                ++flowK;
            }
            return flowK;
        }
        
        /*
        * Assigns elements from one test set to elements of another (reference) based upon 
        * the given Distance matrix describing the elements' separation such that the 
        * summation of distances defined by pairwise assignments is minimized.
        * Returns a new, allocated int array whose reference will be stolen.
        * matrix[i][j] = distance from reference i to test j
        * Let int[i] = index of reference set to which element i of the test set is assigned.
        *			 = -1 if this element was not assigned.
        * For example, int[5] = 4 means that the 5th test element was assigned to the 4th
        * reference element.
        * The int array is the same size as the test set.
        */
        public static List<int> getAssignment(MathNet.Numerics.LinearAlgebra.Matrix<double> matrix)
        {
            if(matrix == null || matrix.ColumnCount == 0 || matrix.RowCount == 0) return new List<int> ();
	        int nodeCount = (matrix.ColumnCount >= matrix.RowCount) ? matrix.ColumnCount : matrix.RowCount;
	        int size = 2*nodeCount+2;
	        List<int> assignment = new List<int> (new int [nodeCount]); // will store the final assignment
            Utility.fill(assignment, -1);
	        List<int> revAssign = new List<int> (new int [nodeCount]);
            Utility.fill(revAssign, -1);
	        double flow; // current flow
	        double flowK = 0; // next flow
            initialize(size);
	        do { // while the flow may still be augmentable
		        // reset for next flow
		        flow = flowK;
		        for(int i = 0; i < size; i++) {
			        S[i] = -2;
			        sigma[i] = -2;
			        predecessor[i] = -2;
			        contained[i] = -2;
		        }
		        flowK = augmentFlow(flowK, assignment, revAssign, matrix, nodeCount);
		        /*std::cout << std::endl;
		        std::copy(assignment,
                      assignment+matrix->getTestSize(),
                      std::ostream_iterator<int>(std::cout,",")
                     );
		        std::cout << std::endl;*/
	        } while(flow < flowK);
	        List<int> answer = new List<int>(new int [matrix.ColumnCount]);
	        for(int i = 0; i < matrix.ColumnCount; i++) {
		        answer[i] = (assignment[i] < matrix.RowCount) ? assignment[i] : -1;
	        }
	        return answer;
        }

        public static double getAssignmentCost(MathNet.Numerics.LinearAlgebra.Double.DenseMatrix matrix)
        {
            return getAssignmentCost(matrix, getAssignment(matrix));
        }

        public static double getAssignmentCost(MathNet.Numerics.LinearAlgebra.Double.DenseMatrix matrix, List<int> assignment)
        {
            double answer = 0;
            for (int i = 0; i < assignment.Count; ++i)
            {
                if (assignment[i] != -1)
                {
                    answer += matrix[assignment[i], i];
                }
            }
            return answer;
        }
    }
}
