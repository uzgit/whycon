/*
 * Filename:    CNecklace.cs
 * Author:      Qinbing Fu
 * Date:        Jan 2017
 * Description: This code is rewritted through WhyCon code. According to circular pattern detection in File "CCircleDetect.cs", this coding adds necklaces to inner circles.
 * Licence:     If you use this class, please cite related works [1], [2].
 * References:  [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 * References:  [2] Peter Lightbody, et al.: A Versatile High-Performance Visual Fiducial Marker Detection System with Scalable Identity Encoding. The 32nd ACM Symposium on Applied Computing, SAC 2017.
 */

using System;

namespace WhyConID
{
    public struct SNecklace
    {
        public int id;
        public int rotation;
    }

    public class CNecklace
    {
        #region FIELDS
        private int length;
        private int idLength;
        private SNecklace[] idArray;
        private SNecklace unknown;
        #endregion

        #region METHODS
        /// <summary>
        /// Default Constructor
        /// </summary>
        public CNecklace()
        { }

        /// <summary>
        /// Parameterized Constructor
        /// </summary>
        /// <param name="bits"></param>
        public CNecklace(int bits)
        {
            length = bits;
            idLength = (int)Math.Pow(2, length);
            idArray = new SNecklace[idLength];

            int currentID = 1; //ID starts from 1
            int tempID, bit, rotations;

            //for every possible id
            for (int id = 0; id < idLength; id++)
            {
                //check if there is a lower number that could be created by bitshifting it
                tempID = id;
                rotations = 0;
                int[] cached = new int[length];
                bool isSymmetrical = false;

                do{
                    bit = tempID % 2;
                    tempID = tempID / 2 + bit * (int)Math.Pow(2, length - 1);

                    if ((bit != 0) || (id == 0))
                    {
                        for (int i = 0; i < rotations && !isSymmetrical; i++)
                        {
                            //check for symmetry
                            if (cached[i] == tempID)
                                isSymmetrical = true;
                        }
                    }
                    cached[rotations] = tempID;
                }while (rotations++ < length - 1 && id <= tempID && !isSymmetrical);

                if (isSymmetrical)
                {
                    idArray[id].id = -1;
                    idArray[id].rotation = -1;
                }
                else if (id > tempID)
                {
                    if (idArray[tempID].id != -1)
                    {
                        idArray[id] = idArray[tempID];
                        idArray[id].rotation += rotations;
                    }
                    else
                        idArray[id].rotation += rotations;
                }
                else
                {
                    idArray[id].id = currentID++;
                    idArray[id].rotation = 0;
                }
                //Console.WriteLine("ID: {0} {1} {2}", id, idArray[id].id, idArray[id].rotation);
            }

            idArray[idLength - 1].id = 0;
            idArray[idLength - 1].rotation = 0;
            unknown.id = -1;
            unknown.rotation = -1;
        }

        public SNecklace Get(int sequence)
        {
            if (sequence > 0 && sequence < idLength)
                return idArray[sequence];
            return unknown;
        }
        #endregion
    }
}
