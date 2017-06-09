/*
 * Filename:    OffCircleDetect.cs
 * Author:      Qinbing Fu
 * Date:        Jan 2017
 * Description: This code is an off-line edition of WhyConID code, to process input RGB-only videos.
 * Licence:     This code is reorganized referring to WhyConID code, if you use this class, please cite [1], [2].
 * References:  [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 * References:  [2] Peter Lightbody, et al.: A Versatile High-Performance Visual Fiducial Marker Detection System with Scalable Identity Encoding. The 32nd ACM Symposium on Applied Computing, SAC 2017.
 */

using System;
using System.IO;

namespace WhyConID
{
    public class OffCircleDetect
    {
        #region FIELDS
        //flags to draw results - used for debugging
        public bool draw, drawAll, lastTrackOK, drawInner, drawInnerCircle;
        //debug level
        public int debug;
        //used when selecting the circle by mouse click
        public bool localSearch;
        //attempt to identify segments
        public bool identify;
        public int idIdentificationCount;

        //private fields
        private CNecklace decoder;
        private bool track;
        private int maxFailed;
        private int numFailed;
        private int threshold;
        private int minSize;
        private int lastThreshold;
        private int maxThreshold;
        private int IDLocker;

        private float circularTolerance;
        private float circularityTolerance;
        private float ratioTolerance;
        private float centerDistanceToleranceRatio;
        private int centerDistanceToleranceAbs;
        private bool enableCorrections;

        private int ID;
        private SSegment inner;
        private SSegment outer;
        private float outerAreaRatio, innerAreaRatio, areasRatio;
        private int queueStart, queueEnd, queueOldStart, numSegments;
        private int width, height, len, siz;

        private int tima, timb, timc, timd, sizer, sizerAll;
        private float diameterRatio;
        #endregion

        #region METHODS
        /// <summary>
        /// Default Constructor
        /// </summary>
        public OffCircleDetect()
        { }

        /// <summary>
        /// Parameterized Constructor
        /// </summary>
        /// <param name="wi"></param>
        /// <param name="he"></param>
        /// <param name="numBots"></param>
        /// <param name="idi"></param>
        public OffCircleDetect(int wi, int he, int idbits, int idi = -1)
        {
            this.decoder = new CNecklace(idbits);                //Necklace/bracelet code for tag identification
            this.identify = true;                                //should we attempt to identify codes?
            this.localSearch = false;                            //by default, search for the pattern everywhere, true is used when position of the pattern is indicated by a click
            this.ID = idi;                                       //pattern ID - not used in this case
            this.enableCorrections = false;                      //enables brightness compensation (Eq. 3.5 of [1])
            this.lastTrackOK = false;                            //was the pattern detected in the previous step? used to initialize the search position
            this.debug = 0;                                      //debug level
            this.draw = true;                                    //draw the detected segments in bright colors to indicate segmentation results
            this.drawInner = false;                              //draw the inner segments which identify the ID
            this.drawInnerCircle = false;                        //draw the inner circle
            this.drawAll = true;                                 //draw all segmentation results - used for debugging
            this.maxFailed = 0;                                  //used to decide when to start changing the threshold
            this.minSize = 15;                                   //minimal pattern size in pixels
            this.maxThreshold = 1 * 256;                         //related to thresholding in case of unsuccessful dtections (3.2 of [1])
            this.centerDistanceToleranceRatio = 0.1f;		     //max allowd distance of the inner and outer circle centers (relative to pattern dimensions)
            this.centerDistanceToleranceAbs = 15;		         //max allowed distance of the inner and outer circle centers (in pixels)
            this.circularTolerance = 2.5f;			             //maximal tolerance of bounding box dimensions vs expected pixel area - see Eq. 2 of the paper [1] 
            this.ratioTolerance = 2.4f;				             //maximal tolerance of black to white pixel ratios - see Algorithm 2 of [1]
            this.threshold = maxThreshold / 2;			         //default threshold
            this.numFailed = maxFailed;				             //used to decide when to start changing the threshold 
            this.track = true;					                 //initiate the search from the last position ?
            this.circularityTolerance = 0.02f;			         //final circularity test, see Eq. 5 of [1]

            /*initialization of supporting structures according to the image size provided*/
            this.width = wi;
            this.height = he;
            this.len = width * height;
            this.siz = len * 3;

            /*inner vs. outer circle diameter, used to calculate expected pixel ratios, see Alg 2 and Eq. 2 of [1]   */
            this.diameterRatio = 50.0f / 122.0f;
            float areaRatioInner_Outer = diameterRatio * diameterRatio;
            this.outerAreaRatio = (float)(Math.PI * (1 - areaRatioInner_Outer) / 4);
            this.innerAreaRatio = (float)(Math.PI / 4);
            this.areasRatio = (float)((1 - areaRatioInner_Outer) / areaRatioInner_Outer);

            //timers for benchmarking
            this.tima = this.timb = this.timc = this.timd = this.sizer = this.sizerAll = 0;

            this.idIdentificationCount = 0; //used to lock ID in a few times identifications
            this.IDLocker = 6;       //if ID is unchanged in 'IDLocker' times identifications, it is locked
        }

        /// <summary>
        /// main detection method, implements Algorithm 2 of [1]
        /// </summary>
        /// <param name="init"></param>
        /// <param name="noc"></param>
        /// <returns></returns>
        public SSegment findSegment(SSegment init, int noc)
        {
            //initializations
            byte[] data = transform3Dto1D(OfflineGlobalData.globalImageData, width, height, noc);
            int resol = width * height;
            numSegments = 0;
            int pos = 0;
            int ii = 0;
            int start = 0;
            bool run = true;

            //start to track
            if (init.valid && track)
            {
                ii = (int)init.y * width + (int)init.x;
                start = ii;
            }
            while (run)
            {
                if (OfflineGlobalData.buffer[ii] == 0)
                {
                    if ((data[ii * noc + 0] + data[ii * noc + 1] + data[ii * noc + 2]) < threshold) OfflineGlobalData.buffer[ii] = -2;
                }
                if (OfflineGlobalData.buffer[ii] == -2)
                {
                    //new segment found
                    queueEnd = 0;
                    queueStart = 0;
                    //if the segment looks like a ring, we check its inside area
                    if (examineSegment(data, ref outer, ii, outerAreaRatio, noc))
                    {
                        pos = (int)outer.y * width + (int)outer.x;
                        if (OfflineGlobalData.buffer[pos] == 0)
                        {
                            OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * noc] + data[pos * noc + 1] + data[pos * noc + 2]) >= threshold) - 2;
                        }
                        if (OfflineGlobalData.buffer[pos] == -1)
                        {
                            if (examineSegment(data, ref inner, pos, innerAreaRatio, noc))
                            {
                                //the inside area is a circle. now what is the area ratio of the black and white ? also, are the circles concentric ?
                                if (debug > 5) Console.WriteLine("Area ratio should be {0:F}, but is {1:F}, that is {2:F} off. ", areasRatio, outer.size / inner.size, (1 - outer.size / areasRatio / inner.size) * 100);
                                if (outer.size / areasRatio / inner.size - ratioTolerance < 1.0 && outer.size / areasRatio / inner.size + ratioTolerance > 1.0)
                                {
                                    if (debug > 5) Console.WriteLine("Segment BW ratio OK.");
                                    if (debug > 5) Console.WriteLine("Concentricity {0:F} {1:F} {2:F} {3:F}.", inner.x, inner.y, outer.x, outer.y);
                                    if ((Math.Abs(inner.x - outer.x) <= centerDistanceToleranceAbs + centerDistanceToleranceRatio * ((float)(outer.maxx - outer.minx))) &&
                                    (Math.Abs(inner.y - outer.y) <= centerDistanceToleranceAbs + centerDistanceToleranceRatio * ((float)(outer.maxy - outer.miny))))
                                    {
                                        if (debug > 5) Console.WriteLine("Concentricity OK.\n");
                                        long six, siy, tx, ty, cm0, cm1, cm2;
                                        six = siy = cm0 = cm1 = cm2 = 0;

                                        for (int p = queueOldStart; p < queueEnd; p++)
                                        {
                                            pos = OfflineGlobalData.queue[p];
                                            tx = pos % width;
                                            ty = pos / width;
                                            six += tx;
                                            siy += ty;
                                            cm0 += tx * tx;
                                            cm1 += tx * ty;
                                            cm2 += ty * ty;
                                        }
                                        inner = calcSegment(inner, queueEnd - queueOldStart, six, siy, cm0, cm1, cm2);

                                        for (int p = 0; p < queueOldStart; p++)
                                        {
                                            pos = OfflineGlobalData.queue[p];
                                            tx = pos % width;
                                            ty = pos / width;
                                            six += tx;
                                            siy += ty;
                                            cm0 += tx * tx;
                                            cm1 += tx * ty;
                                            cm2 += ty * ty;
                                        }
                                        outer = calcSegment(outer, queueEnd, six, siy, cm0, cm1, cm2);
                                        outer.bwRatio = (float)inner.size / outer.size;

                                        sizer += outer.size + inner.size; //for debugging
                                        sizerAll += len; 								    //for debugging
                                        float circularity = (float)Math.PI * 4 * (outer.m0) * (outer.m1) / queueEnd;
                                        if (debug > 5) Console.WriteLine("Segment circularity: {0} {1:F} {2:F} \n", queueEnd, Math.PI * 4 * (outer.m0) * (outer.m1) / queueEnd, Math.PI * 4 * (outer.m0) * (outer.m1));
                                        if (circularity - 1 < circularityTolerance && circularity - 1 > -circularityTolerance)
                                        {
                                            //chromatic aberation correction
                                            if (enableCorrections)
                                            {
                                                float r = diameterRatio * diameterRatio;
                                                float m0o = outer.m0;
                                                float m1o = outer.m1;
                                                float ratio = (float)inner.size / (outer.size + inner.size);
                                                float m0i = (float)Math.Sqrt(ratio) * m0o;
                                                float m1i = (float)Math.Sqrt(ratio) * m1o;
                                                float a = 1 - r;
                                                float b = -(m0i + m1i) - (m0o + m1o) * r;
                                                float c = (m0i * m1i) - (m0o * m1o) * r;
                                                float t = (-b - (float)Math.Sqrt(b * b - 4 * a * c)) / (2 * a);
                                                m0i -= t; m1i -= t; m0o += t; m1o += t;
                                                inner.m0 = m0o;
                                                inner.m1 = m1o;
                                            }
                                            outer.size = outer.size + inner.size;
                                            outer.horizontal = outer.x - inner.x;
                                            if (Math.Abs(inner.v0 * outer.v0 + inner.v1 * outer.v1) > 0.5)
                                            {
                                                outer.r0 = inner.m0 / outer.m0;
                                                outer.r1 = inner.m1 / outer.m1;
                                            }
                                            else
                                            {
                                                outer.r0 = inner.m1 / outer.m0;
                                                outer.r1 = inner.m0 / outer.m1;
                                            }

                                            float orient = (float)Math.Atan2(outer.y - inner.y, outer.x - inner.x);
                                            outer.angle = (float)Math.Atan2(outer.v1, outer.v0);
                                            if (debug > 5) Console.WriteLine("Angle: {0:F} {1:F} \n", outer.angle, orient);
                                            if (Math.Abs(normalizeAngle(outer.angle - orient)) > Math.PI / 2) outer.angle = normalizeAngle(outer.angle + (float)Math.PI);

                                            outer.valid = inner.valid = true;
                                            threshold = (outer.mean + inner.mean) / 2;
                                            if (track) ii = start - 1;
                                        }
                                        else
                                        {
                                            if (track && init.valid)
                                            {
                                                ii = start - 1;
                                                if (debug > 0) Console.WriteLine("Segment failed circularity test.\n");
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (track && init.valid)
                                        {
                                            ii = start - 1;
                                            if (debug > 0) Console.WriteLine("Segment failed concentricity test.\n");
                                        }
                                    }
                                }
                                else
                                {
                                    //tracking failed
                                    if (track && init.valid)
                                    {
                                        ii = start - 1;
                                        if (debug > 0) Console.WriteLine("Segment failed BW test.\n");
                                    }
                                }
                            }
                            else
                            {
                                //tracking failed
                                if (track && init.valid)
                                {
                                    ii = start - 1;
                                    if (debug > 0) Console.WriteLine("Inner segment not a circle\n");
                                }
                            }
                        }
                        else
                        {
                            if (track && init.valid)
                            {
                                ii = start - 1;
                                if (debug > 0) Console.WriteLine("Inner segment not white {0} {1} {2}\n", threshold, data[pos * 3] + data[pos * 3 + 1] + data[pos * 3 + 2], outer.size);
                            }
                        }
                    }
                    else
                    {
                        //tracking failed
                        if (track && init.valid)
                        {
                            ii = start - 1;
                            if (debug > 0) Console.WriteLine("Outer segment {0:F0} {1:F0} {2} not a circle\n", outer.x, outer.y, outer.size);
                        }
                    }
                }
                ii++;
                if (ii >= len) ii = 0;
                run = (ii != start);
            }
            if (debug > 5) Console.WriteLine("II: {0} {1}\n", ii, start);
            if (debug > 1) Console.WriteLine("Inner {0:F} {1:F} Area: {2} Vx: {3} Vy: {4} Mean: {5} Thr: {6} Eigen: {7:F} {8:F} {9:F} {10:F} Axes: {11:F} \n", inner.x, inner.y, inner.size, inner.maxx - inner.minx, inner.maxy - inner.miny, inner.mean, threshold, inner.m0, inner.m1, inner.v0, inner.v1, inner.v0 * outer.v0 + inner.v1 * outer.v1);
            if (debug > 1) Console.WriteLine("Outer {0:F} {1:F} Area: {2} Vx: {3} Vy: {4} Mean: {5} Thr: {6} Eigen: {7:F} {8:F} {9:F} {10:F} Ratios: {11:F3} {12:F3} {13}\n", outer.x, outer.y, outer.size, outer.maxx - outer.minx, outer.maxy - outer.miny, outer.mean, threshold, outer.m0, outer.m1, outer.v0, outer.v1, outer.r0 * 150, outer.r1 * 150, outer.ID);
            if (outer.valid)
            {
                if (numSegments == 2)
                {
                    lastTrackOK = true;
                    localSearch = false;
                }
                else
                {
                    lastTrackOK = false;
                    if (localSearch) outer.valid = false;
                }
            }
            //threshold management
            if (outer.valid)
            {
                lastThreshold = threshold;
                drawAll = false;
                numFailed = 0;
            }
            else if (numFailed < maxFailed)
            {
                if (numFailed++ % 2 == 0) changeThreshold(); else threshold = lastThreshold;
                if (debug > 5) drawAll = true;
            }
            else
            {
                numFailed++;
                if (changeThreshold() == false) numFailed = 0;
                if (debug > 5) drawAll = true;
            }

            if (outer.valid)
            {
                //outer.angle = init.angle;
                outer.angle = inner.angle;
                outer.ID = init.ID;
                if (identify)
                {
                    //inner.m0 = outer.m0 * 0.6f;
                    //inner.m1 = outer.m1 * 0.6f;
                    int segment = identifySegment(ref inner, ref data);
                    if (segment > -1) //known ID
                    {
                        //outer.angle = inner.angle;
                        outer.ID = segment;
                        if (outer.lastID == outer.ID)
                            idIdentificationCount++;
                        else
                            idIdentificationCount--;
                        if (idIdentificationCount < 0)
                            idIdentificationCount = 0;
                        outer.lastID = outer.ID;
                    }
                    if (idIdentificationCount == IDLocker)
                        this.identify = false;
                }
            }
            //Drawing results 
            if (outer.valid && drawInner)
            {
                //inner
                for (int p = queueOldStart; p < queueEnd; p++)
                {
                    pos = OfflineGlobalData.queue[p];
                    if (pos > 0 && pos < resol)
                        //B=G=R=0
                        data[noc * pos + 0] = data[noc * pos + 1] = data[noc * pos + 2] = 0;
                }
            }
            if (draw)
            {
                if (init.valid || track || lastTrackOK)
                {
                    //inner
                    for (int p = queueOldStart; p < queueEnd; p++)
                    {
                        pos = OfflineGlobalData.queue[p];
                        if (pos > 0 && pos < resol)
                        {
                            data[noc * pos + 0] = 0;  //B
                            data[noc * pos + 1] = 255;    //G
                            data[noc * pos + 2] = 255;    //R
                        }
                    }
                    //outer
                    for (int p = 0; p < queueOldStart; p++)
                    {
                        pos = OfflineGlobalData.queue[p];
                        if (pos > 0 && pos < resol)
                        {
                            data[noc * pos + 0] = 255;    //B
                            data[noc * pos + 1] = 0;    //G
                            data[noc * pos + 2] = 0;  //R
                        }
                    }
                }
            }
            bufferCleanup(outer);

            //transform to global image array
            OfflineGlobalData.globalImageData = transform1Dto3D(data, width, height, noc);

            //return
            return outer;
        }

        /// <summary>
        /// transform 3D matrix to 1D matrix
        /// </summary>
        /// <param name="inputMat"></param>
        /// <param name="wid"></param>
        /// <param name="hei"></param>
        /// <param name="channel"></param>
        /// <returns></returns>
        private byte[] transform3Dto1D(byte[,,] inputMat, int wid, int hei, int channel)
        {
            byte[] outputMat = new byte[hei * wid * channel];
            Buffer.BlockCopy(inputMat, 0, outputMat, 0, wid * hei * channel);
            return outputMat;
        }

        /// <summary>
        /// transform 1D matrix to 3D matrix
        /// </summary>
        /// <param name="inputMat"></param>
        /// <param name="wid"></param>
        /// <param name="hei"></param>
        /// <param name="channel"></param>
        /// <returns></returns>
        private byte[,,] transform1Dto3D(byte[] inputMat, int wid, int hei, int channel)
        {
            byte[,,] outputMat = new byte[hei, wid, channel];
            Buffer.BlockCopy(inputMat, 0, outputMat, 0, wid * hei * channel);
            return outputMat;
        }

        /// <summary>
        /// local pattern search - implements Algorithm 1 of [1]
        /// </summary>
        /// <param name="data"></param>
        /// <param name="segmen"></param>
        /// <param name="ii"></param>
        /// <param name="areaRatio"></param>
        /// <param name="c"></param>
        /// <returns></returns>
        public bool examineSegment(byte[] data, ref SSegment segmen, int ii, float areaRatio, int c)
        {
            int vx, vy;
            queueOldStart = queueStart;
            int position = 0;
            int pos;
            bool result = false;
            int type = OfflineGlobalData.buffer[ii];
            int maxx, maxy, minx, miny;

            OfflineGlobalData.buffer[ii] = ++numSegments;
            segmen.x = ii % width;
            segmen.y = ii / width;
            minx = maxx = (int)segmen.x;
            miny = maxy = (int)segmen.y;
            segmen.valid = false;
            segmen.round = false;
            //push segment coords to the queue
            OfflineGlobalData.queue[queueEnd++] = ii;
            int dataLength = width * height;
            //and until queue is empty
            while (queueEnd > queueStart)
            {
                //pull the coord from the queue
                position = OfflineGlobalData.queue[queueStart++];
                //search neighbours
                pos = position + 1;
                while (pos >= dataLength) pos = dataLength - 1;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    maxx = Max(maxx, pos % width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
                pos = position - 1;
                while (pos < 0) pos = 0;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    minx = Min(minx, pos % width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
                pos = position - width;
                while (pos < 0) pos = 0;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c + 0] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    miny = Min(miny, pos / width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
                pos = position + width;
                while (pos >= dataLength) pos = dataLength - 1;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c + 0] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    maxy = Max(maxy, pos / width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
            }

            //once the queue is empty, i.e. segment is complete, we compute its size
            segmen.size = queueEnd - queueOldStart;
            if (segmen.size > minSize)
            {
                //and if its large enough, we compute its other properties
                segmen.maxx = maxx;
                segmen.maxy = maxy;
                segmen.minx = minx;
                segmen.miny = miny;
                segmen.type = -type;
                vx = (segmen.maxx - segmen.minx + 1);
                vy = (segmen.maxy - segmen.miny + 1);
                segmen.x = (segmen.maxx + segmen.minx) / 2;
                segmen.y = (segmen.maxy + segmen.miny) / 2;
                segmen.roundness = vx * vy * areaRatio / segmen.size;
                //we check if the segment is likely to be a ring 
                if (segmen.roundness - circularTolerance < 1.0 && segmen.roundness + circularTolerance > 1.0)
                {
                    //if its round, we compute yet another properties 
                    segmen.round = true;
                    segmen.mean = 0;
                    for (int p = queueOldStart; p < queueEnd; p++)
                    {
                        pos = OfflineGlobalData.queue[p];
                        segmen.mean += data[pos * c] + data[pos * c + 1] + data[pos * c + 2];
                    }
                    segmen.mean = segmen.mean / segmen.size;
                    result = true;
                }
            }

            return result;
        }

        /// <summary>
        /// calculate the pattern dimensions by means of eigenvalue decomposition, see 3.3 of [1]
        /// </summary>
        /// <param name="segment"></param>
        /// <param name="size"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="cm0"></param>
        /// <param name="cm1"></param>
        /// <param name="cm2"></param>
        /// <returns></returns>
        public SSegment calcSegment(SSegment segment, int size, long x, long y, long cm0, long cm1, long cm2)
        {
            float cm0f, cm1f, cm2f, fm0, fm1, fm2, f0, f1;
            SSegment result = segment;
            float sx = (float)x / size;
            float sy = (float)y / size;
            cm0f = (cm0 - sx * sx * size);
            cm1f = (cm1 - sx * sy * size);
            cm2f = (cm2 - sy * sy * size);
            fm0 = cm0f / size;
            fm1 = cm1f / size;
            fm2 = cm2f / size;
            float det = (fm0 + fm2) * (fm0 + fm2) - 4 * (fm0 * fm2 - fm1 * fm1);
            if (det > 0) det = (float)Math.Sqrt(det); else det = 0;
            f0 = ((fm0 + fm2) + det) / 2;
            f1 = ((fm0 + fm2) - det) / 2;
            result.x = sx;
            result.y = sy;
            result.m0 = (float)Math.Sqrt(f0);
            result.m1 = (float)Math.Sqrt(f1);
            if (fm1 != 0)
            {
                result.v0 = -fm1 / (float)Math.Sqrt(fm1 * fm1 + (fm0 - f0) * (fm0 - f0));
                result.v1 = (fm0 - f0) / (float)Math.Sqrt(fm1 * fm1 + (fm0 - f0) * (fm0 - f0));
                //result.v1 = (float)((fm0 - f0) / Math.Sqrt(fm1 * fm1 + (fm0 - f0) * (fm0 - f0)));
            }
            else
            {
                result.v0 = result.v1 = 0;
                if (fm0 > fm2) result.v0 = 1.0f; else result.v1 = 1.0f;
            }
            //Console.WriteLine("An: {0:F} {1:F} {2:F} {3:F} {4:F} {5:F} {6:F} {7:F}", det,result.v0,result.v1,fm0,fm1,fm2,f0,f1);
            return result;
        }

        /// <summary>
        /// establish circle ID
        /// </summary>
        /// <param name="inner"></param>
        /// <param name="data"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <returns></returns>
        public int identifySegment(ref SSegment inner, ref byte[] data)
        {
            int pos;
            float PI = (float)Math.PI;
            int ID_SAMPLES = Form1.ID_SAMPLES;
            int ID_BITS = Form1.ID_BITS;
            float[] x = new float[ID_SAMPLES];
            float[] y = new float[ID_SAMPLES];
            float[] signal = new float[ID_SAMPLES];
            float[] differ = new float[ID_SAMPLES];
            float[] smooth = new float[ID_SAMPLES];
            int segmentWidth = ID_SAMPLES / ID_BITS / 2; //pixels in each tooth
            int reso = width * height;

            //calculate appropriate positions
            for (int a = 0; a < ID_SAMPLES; a++)
            {
                float tmp = (float)a / ID_SAMPLES * 2 * PI;
                x[a] = inner.x + (float)((inner.m0 * Math.Cos(tmp) * inner.v0 + inner.m1 * Math.Sin(tmp) * inner.v1) * 2.0);
                y[a] = inner.y + (float)((inner.m0 * Math.Cos(tmp) * inner.v1 - inner.m1 * Math.Sin(tmp) * inner.v0) * 2.0);
            }

            //retrieve the image brightness on these using bilinear transformation
            float gx, gy;
            int px, py;
            for (int a = 0; a < ID_SAMPLES; a++)
            {
                px = (int)x[a];
                py = (int)y[a];
                gx = x[a] - px;
                gy = y[a] - py;
                pos = px + py * width;
                //detection from the image
                signal[a] = data[pos * 3 + 0] * (1 - gx) * (1 - gy) + data[(pos + 1) * 3 + 0] * gx * (1 - gy) + data[(pos + width) * 3 + 0] * (1 - gx) * gy + data[3 * (pos + (width + 1)) + 0] * gx * gy;
                signal[a] += data[pos * 3 + 1] * (1 - gx) * (1 - gy) + data[(pos + 1) * 3 + 1] * gx * (1 - gy) + data[(pos + width) * 3 + 1] * (1 - gx) * gy + data[3 * (pos + (width + 1)) + 1] * gx * gy;
                signal[a] += data[pos * 3 + 2] * (1 - gx) * (1 - gy) + data[(pos + 1) * 3 + 2] * gx * (1 - gy) + data[(pos + width) * 3 + 2] * (1 - gx) * gy + data[3 * (pos + (width + 1)) + 2] * gx * gy;
            }
            //calculate signal gradient 
            for (int a = 1; a < ID_SAMPLES; a++) differ[a] = signal[a] - signal[a - 1];
            differ[0] = signal[0] - signal[ID_SAMPLES - 1];
            //and smooth the gradient out
            smooth[0] = 0;
            for (int a = ID_SAMPLES - segmentWidth; a < ID_SAMPLES; a++) smooth[0] += differ[a];
            for (int a = 1; a < ID_SAMPLES; a++) smooth[a] = smooth[a - 1] - differ[(a + ID_SAMPLES - segmentWidth) % ID_SAMPLES] + differ[a - 1];
            //find the strongest edge response
            int maxIndex = -1;
            float strength = -1000;
            for (int a = 0; a < ID_SAMPLES; a++)
            {
                if (smooth[a] > strength)
                {
                    strength = smooth[a];
                    maxIndex = a;
                }
            }
            //and determine the following edges
            int aa = 1;
            int state = 0;
            int position0 = (maxIndex + segmentWidth) % ID_SAMPLES;       //position0 < ID_SAMPLES
            int position1 = (maxIndex + 2 * segmentWidth) % ID_SAMPLES;     //position1 < ID_SAMPLES
            char[] code = new char[ID_BITS * 4];
            code[0] = '0';
            while (aa < ID_BITS * 2)
            {
                //is the following edge a local minimum?
                if (state == 0)
                {
                    if (smooth[position0] > smooth[position1])
                    {
                        code[aa++] = 'X';
                        position0 += segmentWidth;
                        position0 %= ID_SAMPLES;
                    }
                    state = 1;
                    code[aa] = '1';
                }
                else
                {
                    if (smooth[position0] < smooth[position1])
                    {
                        code[aa++] = 'X';
                        position0 += segmentWidth;
                        position0 %= ID_SAMPLES;
                    }
                    state = 0;
                    code[aa] = '0';
                }
                if (code[aa] == '0')
                {
                    if (smooth[position0] < smooth[(position0 + ID_SAMPLES - 1) % ID_SAMPLES]) position0 = (position0 + ID_SAMPLES - 1) % ID_SAMPLES;
                    if (smooth[position0] < smooth[(position0 + ID_SAMPLES + 1) % ID_SAMPLES]) position0 = (position0 + ID_SAMPLES + 1) % ID_SAMPLES;
                }
                if (code[aa] == '1')
                {
                    if (smooth[position0] > smooth[(position0 + ID_SAMPLES - 1) % ID_SAMPLES]) position0 = (position0 + ID_SAMPLES - 1) % ID_SAMPLES;
                    if (smooth[position0] > smooth[(position0 + ID_SAMPLES + 1) % ID_SAMPLES]) position0 = (position0 + ID_SAMPLES + 1) % ID_SAMPLES;
                }
                position0 += segmentWidth;
                position0 = position0 % ID_SAMPLES;
                position1 = (position0 + segmentWidth) % ID_SAMPLES;
                aa++;
            }
            code[ID_BITS * 2] = (char)0;

            //determine the control edges' positions
            int edgeIndex = 0;
            for (int a = 0; a < code.Length; a++)
            {
                if (code[a] == 'X') edgeIndex = a;
            }
            char[] realCode = new char[ID_BITS * 4];
            edgeIndex = 1 - (edgeIndex % 2);
            int ID = 0;
            for (int a = 0; a < ID_BITS; a++)
            {
                realCode[a] = code[edgeIndex + 2 * a];
                if (realCode[a] == 'X') ID = -1;
                if (ID > -1)
                {
                    ID = ID * 2;
                    if (realCode[a] == '1') ID++;
                }
            }
            realCode[ID_BITS] = (char)0;

            SNecklace result = decoder.Get(ID);
            inner.angle = 2 * PI * (-(float)maxIndex / ID_SAMPLES + (float)result.rotation / ID_BITS) + (float)Math.Atan2(inner.v1, inner.v0) + 1.5f * PI / ID_BITS;
            while (inner.angle > PI) inner.angle -= 2 * PI;
            while (inner.angle < -PI) inner.angle += 2 * PI;
            if (drawInnerCircle)
            {
                for (int a = 0; a < ID_SAMPLES; a++)
                {
                    pos = (int)x[a] + (int)y[a] * width;
                    if (pos > 0 && pos < reso)
                    {
                        data[3 * pos + 0] = 0;                              //B
                        //data[3*pos+1] = (byte)(255.0*a/ID_SAMPLES);         //G
                        data[3 * pos + 1] = 255;                            //G
                        data[3 * pos + 2] = 0;                              //R
                    }
                }
            }

            return result.id;
        }

        /// <summary>
        /// cleanup the shared buffers - see 3.6 of [1]
        /// </summary>
        /// <param name="init"></param>
        public void bufferCleanup(SSegment init)
        {
            int pos = (height - 1) * width;
            if (init.valid == false || track == false || lastTrackOK == false)
            {
                Array.Clear(OfflineGlobalData.buffer, 0, len);
                for (int i = 0; i < width; i++)
                {
                    OfflineGlobalData.buffer[i] = -1000;
                    OfflineGlobalData.buffer[pos + i] = -1000;
                }
                for (int i = 0; i < height; i++)
                {
                    OfflineGlobalData.buffer[width * i] = -1000;
                    OfflineGlobalData.buffer[width * i + width - 1] = -1000;
                }
            }
            else
            {
                int poss, ix, ax, iy, ay;
                ix = Max(init.minx - 2, 1);
                ax = Min(init.maxx + 2, width - 2);
                iy = Max(init.miny - 2, 1);
                ay = Min(init.maxy + 2, height - 2);
                for (int y = iy; y < ay; y++)
                {
                    poss = y * width;
                    for (int x = ix; x < ax; x++) OfflineGlobalData.buffer[poss + x] = 0;
                }
            }
        }

        /// <summary>
        /// load descriptions for circle ID's (not for use in this version)
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public int loadCircleID(char[] id)
        { return 0; }

        /// <summary>
        /// change threshold if circle not detected, see 3.2 of [1]
        /// </summary>
        /// <returns></returns>
        public bool changeThreshold()
        {
            int div = 1;
            int dum = numFailed;
            while (dum > 1)
            {
                dum = dum / 2;
                div *= 2;
            }
            int step = 256 / div;
            threshold = 3 * (step * (numFailed - div) + step / 2);
            if (debug > 5)
                Console.WriteLine("Threshold: {0} {1} {2}\n", div, numFailed, threshold / 3);
            return step > 8;
        }
        #endregion

        #region ADDITIONAL MATH
        /// <summary>
        /// Get min from two integers
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private int Min(int a, int b)
        {
            return ((a) < (b) ? (a) : (b));
        }
        /// <summary>
        /// Get max from two integers
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private int Max(int a, int b)
        {
            return ((a) > (b) ? (a) : (b));
        }
        /// <summary>
        /// Get min from two doubles
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private double Min(double a, double b)
        {
            return ((a) < (b) ? (a) : (b));
        }
        /// <summary>
        /// Get max from two doubles
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private double Max(double a, double b)
        {
            return ((a) > (b) ? (a) : (b));
        }
        /// <summary>
        /// logarithm (2 as base)
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        private int log2(int value)
        {
            int r = 0;
            while (value > 0)
            {
                value = value / 2;
                r++;
            }
            return r - 1;
        }
        /// <summary>
        /// exponential (2 as base)
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        private int exp2(int value)
        {
            int r = 1;
            for (int i = 0; i < value; i++)
            {
                r = r * 2;
            }
            return r;
        }
        /// <summary>
        /// normalize angle between -pi and pi
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        private float normalizeAngle(float a)
        {
            while (a > +Math.PI) a += (float)(-2 * Math.PI);
            while (a < -Math.PI) a += (float)(2 * Math.PI);
            return a;
        }
        #endregion
    }

    public class OffCosPhiDetect
    {
        #region FIELDS

        public bool draw, drawAll, lastTrackOK;
        public int debug;
        public bool localSearch;
        public int idIdentificationCount;
        public bool identify;

        private static int maskNum = 0;
        private bool track;
        private int maxFailed;
        private int numFailed;
        private int threshold;

        private int minSize;
        private int lastThreshold;
        private int thresholdBias;
        private int maxThreshold;
        private int IDLocker;
        private bool ifLockID;

        private int thresholdStep;
        private float circularTolerance;
        private float circularityTolerance;
        private float ratioTolerance;
        private float centerDistanceToleranceRatio;
        private int centerDistanceToleranceAbs;
        private bool enableCorrections;

        private int ID;
        private CosSegment inner;
        private CosSegment outer;
        private float outerAreaRatio, innerAreaRatio, areasRatio;
        private int queueStart, queueEnd, queueOldStart, numSegments;
        private int width, height, len, siz;
        private int sizer, sizerAll;
        private float diameterRatio;
        private bool ownBuffer;
        private int numberIDs;

        private int[] expand;
        private float[] idx;
        private float[] idy;

        #endregion

        #region CONSTRUCTORS

        /// <summary>
        /// Default Constructor
        /// </summary>
        public OffCosPhiDetect() { }

        /// <summary>
        /// Paremeterized Constructor
        /// </summary>
        /// <param name="wi"></param>
        /// <param name="he"></param>
        /// <param name="numbots"></param>
        /// <param name="ID"></param>
        public OffCosPhiDetect(int wi, int he, int numbots, int idi = -1)
        {
            expand = new int[4];
            idx = new float[numbots];
            idy = new float[numbots];

            localSearch = false;
            ID = idi;
            numberIDs = 0;
            enableCorrections = false;
            lastTrackOK = false;
            debug = 0;
            draw = false;
            drawAll = false;
            maxFailed = 0;
            minSize = 15;
            thresholdStep = 256;
            maxThreshold = 3 * 256;
            centerDistanceToleranceRatio = 0.1f;
            centerDistanceToleranceAbs = 15;
            circularityTolerance = 0.02f;
            circularTolerance = 1.5f;
            ratioTolerance = 2.4f;
            numFailed = maxFailed;

            threshold = maxThreshold / 2;
            track = true;
            //fixed params
            width = wi;
            height = he;
            len = width * height;
            siz = len * 3;
            ownBuffer = false;

            clearCalibMask();
            diameterRatio = 5.0f / 14.0f;
            float areaRatioInner_Outer = diameterRatio * diameterRatio;
            outerAreaRatio = (float)Math.PI * (1.0f - areaRatioInner_Outer) / 4;
            innerAreaRatio = (float)Math.PI / 4;
            areasRatio = (1 - areaRatioInner_Outer) / areaRatioInner_Outer;
            sizer = sizerAll = 0;
            loadCircleID();

            idIdentificationCount = 0;
            IDLocker = 3;
            ifLockID = true;
            identify = true;
        }

        #endregion

        #region METHODS

        /// <summary>
        ///  establish circle ID
        /// </summary>
        /// <param name="segment"></param>
        public void identifySegment(ref CosSegment segment)
        {
            float maxDistance = 1000;
            int index = -1;
            float dx, dy;
            for (int i = 0; i < numberIDs; i++)
            {
                dx = segment.r0 - idx[i];
                dy = segment.r1 - idy[i];
                if (dx * dx + dy * dy < maxDistance)
                {
                    maxDistance = dx * dx + dy * dy;
                    index = i;
                }
            }
            segment.ID = index;
            //color defined by ID
            //segment.color = Color.FromArgb(255/segment.ID,255/segment.ID,0);
        }

        /// <summary>
        /// cleanup the shared buffers - see 3.6 of [1]
        /// </summary>
        /// <param name="init"></param>
        public void bufferCleanup(CosSegment init)
        {
            int pos = (height - 1) * width;
            if (init.valid == false || track == false || lastTrackOK == false)
            {
                Array.Clear(OfflineGlobalData.buffer, 0, len);
                for (int i = 0; i < width; i++)
                {
                    OfflineGlobalData.buffer[i] = -1000;
                    OfflineGlobalData.buffer[pos + i] = -1000;
                }
                for (int i = 0; i < height; i++)
                {
                    OfflineGlobalData.buffer[width * i] = -1000;
                    OfflineGlobalData.buffer[width * i + width - 1] = -1000;
                }
            }
            else
            {
                int poss, ix, ax, iy, ay;
                ix = Max(init.minx - 2, 1);
                ax = Min(init.maxx + 2, width - 2);
                iy = Max(init.miny - 2, 1);
                ay = Min(init.maxy + 2, height - 2);
                for (int y = iy; y < ay; y++)
                {
                    poss = y * width;
                    for (int x = ix; x < ax; x++) OfflineGlobalData.buffer[poss + x] = 0;
                }
            }
        }

        /// <summary>
        /// main detection method, implements Algorithm 2 of [1]
        /// </summary>
        /// <param name="init"></param>
        /// <param name="noc"></param>
        /// <returns></returns>
        public CosSegment findSegment(CosSegment init, int noc)
        {
            //initializations
            byte[] data = transform3Dto1D(OfflineGlobalData.globalImageData, width, height, noc);
            int resol = width * height;
            numSegments = 0;
            int pos = 0;
            int ii = 0;
            int start = 0;
            bool cont = true;
            //start to track
            if (init.valid && track)
            {
                ii = (int)init.y * width + (int)init.x;
                start = ii;
            }
            while (cont)
            {
                if (OfflineGlobalData.buffer[ii] == 0)
                {
                    if ((data[ii * noc + 0] + data[ii * noc + 1] + data[ii * noc + 2]) < threshold) OfflineGlobalData.buffer[ii] = -2;
                }
                if (OfflineGlobalData.buffer[ii] == -2)
                {
                    //new segment found
                    queueEnd = 0;
                    queueStart = 0;
                    //if the segment looks like a ring, we check its inside area
                    if (examineSegment(data, ref outer, ii, outerAreaRatio, noc))
                    {
                        pos = (int)outer.y * width + (int)outer.x;
                        if (OfflineGlobalData.buffer[pos] == 0)
                        {
                            OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * noc] + data[pos * noc + 1] + data[pos * noc + 2]) >= threshold) - 2;
                        }
                        if (OfflineGlobalData.buffer[pos] == -1)
                        {
                            if (examineSegment(data, ref inner, pos, innerAreaRatio, noc))
                            {
                                //the inside area is a circle. now what is the area ratio of the black and white ? also, are the circles concentric ?
                                if (outer.size / areasRatio / inner.size - ratioTolerance < 1.0 && outer.size / areasRatio / inner.size + ratioTolerance > 1.0)
                                {
                                    if ((Math.Abs(inner.x - outer.x) <= centerDistanceToleranceAbs + centerDistanceToleranceRatio * ((float)(outer.maxx - outer.minx))) &&
                                    (Math.Abs(inner.y - outer.y) <= centerDistanceToleranceAbs + centerDistanceToleranceRatio * ((float)(outer.maxy - outer.miny))))
                                    {
                                        long six, siy, tx, ty, cm0, cm1, cm2;
                                        six = siy = cm0 = cm1 = cm2 = 0;

                                        for (int p = queueOldStart; p < queueEnd; p++)
                                        {
                                            pos = OfflineGlobalData.queue[p];
                                            tx = pos % width;
                                            ty = pos / width;
                                            six += tx;
                                            siy += ty;
                                            cm0 += tx * tx;
                                            cm1 += tx * ty;
                                            cm2 += ty * ty;
                                        }
                                        inner = calcSegment(inner, queueEnd - queueOldStart, six, siy, cm0, cm1, cm2);
                                        for (int p = 0; p < queueOldStart; p++)
                                        {
                                            pos = OfflineGlobalData.queue[p];
                                            tx = pos % width;
                                            ty = pos / width;
                                            six += tx;
                                            siy += ty;
                                            cm0 += tx * tx;
                                            cm1 += tx * ty;
                                            cm2 += ty * ty;
                                        }
                                        outer = calcSegment(outer, queueEnd, six, siy, cm0, cm1, cm2);
                                        outer.bwRatio = (float)inner.size / outer.size;

                                        float circularity = (float)Math.PI * 4 * (outer.m0) * (outer.m1) / queueEnd;
                                        if (circularity - 1.0 < circularityTolerance && circularity - 1.0 > -circularityTolerance)
                                        {

                                            //chromatic aberation correction
                                            if (enableCorrections)
                                            {
                                                float r = diameterRatio * diameterRatio;
                                                float m0o = outer.m0;
                                                float m1o = outer.m1;
                                                float ratio = (float)inner.size / (outer.size + inner.size);
                                                float m0i = (float)Math.Sqrt(ratio) * m0o;
                                                float m1i = (float)Math.Sqrt(ratio) * m1o;
                                                float a = 1 - r;
                                                float b = -(m0i + m1i) - (m0o + m1o) * r;
                                                float c = (m0i * m1i) - (m0o * m1o) * r;
                                                float t = (-b - (float)Math.Sqrt(b * b - 4 * a * c)) / (2 * a);
                                                m0i -= t; m1i -= t; m0o += t; m1o += t;
                                                inner.m0 = m0o;
                                                inner.m1 = m1o;
                                            }
                                            outer.size = outer.size + inner.size;
                                            outer.horizontal = outer.x - inner.x;
                                            if (Math.Abs(inner.v0 * outer.v0 + inner.v1 * outer.v1) > 0.5)
                                            {
                                                outer.r0 = inner.m0 / outer.m0;
                                                outer.r1 = inner.m1 / outer.m1;
                                            }
                                            else
                                            {
                                                outer.r0 = inner.m1 / outer.m0;
                                                outer.r1 = inner.m0 / outer.m1;
                                            }
                                            float orient = (float)Math.Atan2(outer.y - inner.y, outer.x - inner.x);
                                            outer.angle = (float)Math.Atan2(outer.v1, outer.v0);
                                            if (Math.Abs(normalizeAngle(outer.angle - orient)) > Math.PI / 2) outer.angle = normalizeAngle(outer.angle + (float)Math.PI);
                                            //fiducial identification - experimental only
                                            //identify ID
                                            if (identify)
                                            {
                                                identifySegment(ref outer);
                                                if (ifLockID)
                                                {
                                                    if (ID == outer.ID)
                                                        idIdentificationCount++;
                                                    else
                                                        idIdentificationCount--;
                                                    if (idIdentificationCount < 0)
                                                        idIdentificationCount = 0;
                                                    ID = outer.ID;
                                                    if (idIdentificationCount == IDLocker)
                                                        identify = false;
                                                }
                                            }
                                            //threshold
                                            outer.valid = inner.valid = true;
                                            threshold = (outer.mean + inner.mean) / 2;
                                            if (track) ii = start - 1;
                                        }
                                        else
                                        {
                                            if (track && init.valid)
                                            {
                                                ii = start - 1;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (track && init.valid)
                                        {
                                            ii = start - 1;
                                        }
                                    }
                                }
                                else
                                {
                                    //tracking failed
                                    if (track && init.valid)
                                    {
                                        ii = start - 1;
                                    }
                                }
                            }
                            else
                            {
                                //tracking failed
                                if (track && init.valid)
                                {
                                    ii = start - 1;
                                }
                            }
                        }
                        else
                        {
                            if (track && init.valid)
                            {
                                ii = start - 1;
                            }
                        }
                    }
                    else
                    {
                        //tracking failed
                        if (track && init.valid)
                        {
                            ii = start - 1;
                        }
                    }
                }
                ii++;
                if (ii >= len) ii = 0;
                cont = (ii != start);
            }
            if (outer.valid)
            {
                if (numSegments == 2)
                {
                    lastTrackOK = true;
                    localSearch = false;
                }
                else
                {
                    lastTrackOK = false;
                    if (localSearch) outer.valid = false;
                }
            }
            //threshold management
            if (outer.valid)
            {
                lastThreshold = threshold;
                drawAll = false;
                numFailed = 0;
            }
            else if (numFailed < maxFailed)
            {
                if (numFailed++ % 2 == 0) changeThreshold(); else threshold = lastThreshold;
                if (debug > 5) drawAll = true;
            }
            else
            {
                numFailed++;
                if (changeThreshold() == false) numFailed = 0;
                if (debug > 5) drawAll = true;
            }

            //Drawing results 
            if (outer.valid && drawAll)
            {
                for (int p = queueOldStart; p < queueEnd; p++)
                {
                    pos = OfflineGlobalData.queue[p];
                    if (pos > 0 && pos < resol)
                        data[noc * pos + 0] = data[noc * pos + 1] = data[noc * pos + 2] = (byte)(outer.mean / 3);
                }
            }
            if (draw)
            {
                if (init.valid || track || lastTrackOK)
                {
                    //inner
                    for (int p = queueOldStart; p < queueEnd; p++)
                    {
                        pos = OfflineGlobalData.queue[p];
                        if (pos > 0 && pos < resol)
                        {
                            data[noc * pos + 0] = 0;  //B
                            data[noc * pos + 1] = 255;  //G
                            data[noc * pos + 2] = 255;    //R
                        }
                    }
                    //outer
                    for (int p = 0; p < queueOldStart; p++)
                    {
                        pos = OfflineGlobalData.queue[p];
                        if (pos > 0 && pos < resol)
                        {
                            data[noc * pos + 0] = 255;    //B
                            data[noc * pos + 1] = 0;  //G
                            data[noc * pos + 2] = 0;  //R
                        }
                    }
                }
            }
            bufferCleanup(outer);
            //transform to global image array
            OfflineGlobalData.globalImageData = transform1Dto3D(data, width, height, noc);
            return outer;
        }

        /// <summary>
        /// local pattern search - implements Algorithm 1 of [1]
        /// </summary>
        /// <param name="data"></param>
        /// <param name="segmen"></param>
        /// <param name="ii"></param>
        /// <param name="areaRatio"></param>
        /// <param name="c"></param>
        /// <returns></returns>
        public bool examineSegment(byte[] data, ref CosSegment segmen, int ii, float areaRatio, int c)
        {
            int vx, vy;
            queueOldStart = queueStart;
            int position = 0;
            int pos;
            bool result = false;
            int type = OfflineGlobalData.buffer[ii];
            int maxx, maxy, minx, miny;

            OfflineGlobalData.buffer[ii] = ++numSegments;
            segmen.x = ii % width;
            segmen.y = ii / width;
            minx = maxx = (int)segmen.x;
            miny = maxy = (int)segmen.y;
            segmen.valid = false;
            segmen.round = false;
            //push segment coords to the queue
            OfflineGlobalData.queue[queueEnd++] = ii;
            int dataLength = width * height;
            //and until queue is empty
            while (queueEnd > queueStart)
            {
                //pull the coord from the queue
                position = OfflineGlobalData.queue[queueStart++];
                //search neighbours
                pos = position + 1;
                while (pos >= dataLength) pos = dataLength - 1;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    maxx = Max(maxx, pos % width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
                pos = position - 1;
                while (pos < 0) pos = 0;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    minx = Min(minx, pos % width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
                pos = position - width;
                while (pos < 0) pos = 0;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c + 0] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    miny = Min(miny, pos / width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
                pos = position + width;
                while (pos >= dataLength) pos = dataLength - 1;
                if (OfflineGlobalData.buffer[pos] == 0)
                {
                    OfflineGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c + 0] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (OfflineGlobalData.buffer[pos] == type)
                {
                    OfflineGlobalData.queue[queueEnd++] = pos;
                    maxy = Max(maxy, pos / width);
                    OfflineGlobalData.buffer[pos] = numSegments;
                }
            }

            //once the queue is empty, i.e. segment is complete, we compute its size
            segmen.size = queueEnd - queueOldStart;
            if (segmen.size > minSize)
            {
                //and if its large enough, we compute its other properties
                segmen.maxx = maxx;
                segmen.maxy = maxy;
                segmen.minx = minx;
                segmen.miny = miny;
                segmen.type = -type;
                vx = (segmen.maxx - segmen.minx + 1);
                vy = (segmen.maxy - segmen.miny + 1);
                segmen.x = (segmen.maxx + segmen.minx) / 2;
                segmen.y = (segmen.maxy + segmen.miny) / 2;
                segmen.roundness = vx * vy * areaRatio / segmen.size;
                //we check if the segment is likely to be a ring 
                if (segmen.roundness - circularTolerance < 1.0 && segmen.roundness + circularTolerance > 1.0)
                {
                    //if its round, we compute yet another properties 
                    segmen.round = true;
                    segmen.mean = 0;
                    for (int p = queueOldStart; p < queueEnd; p++)
                    {
                        pos = OfflineGlobalData.queue[p];
                        segmen.mean += data[pos * c] + data[pos * c + 1] + data[pos * c + 2];
                    }
                    segmen.mean = segmen.mean / segmen.size;
                    result = true;
                }
            }
            return result;
        }

        /// <summary>
        /// calculate the pattern dimensions by means of eigenvalue decomposition, see 3.3 of [1]
        /// </summary>
        /// <param name="segment"></param>
        /// <param name="size"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="cm0"></param>
        /// <param name="cm1"></param>
        /// <param name="cm2"></param>
        /// <returns></returns>
        public CosSegment calcSegment(CosSegment segment, int size, long x, long y, long cm0, long cm1, long cm2)
        {
            float cm0f, cm1f, cm2f, fm0, fm1, fm2, f0, f1;
            CosSegment result = segment;
            float sx = (float)x / size;
            float sy = (float)y / size;
            cm0f = (cm0 - sx * sx * size);
            cm1f = (cm1 - sx * sy * size);
            cm2f = (cm2 - sy * sy * size);
            fm0 = cm0f / size;
            fm1 = cm1f / size;
            fm2 = cm2f / size;
            float det = (fm0 + fm2) * (fm0 + fm2) - 4 * (fm0 * fm2 - fm1 * fm1);
            if (det > 0) det = (float)Math.Sqrt(det); else det = 0;
            f0 = ((fm0 + fm2) + det) / 2;
            f1 = ((fm0 + fm2) - det) / 2;
            result.x = sx;
            result.y = sy;
            result.m0 = (float)Math.Sqrt(f0);
            result.m1 = (float)Math.Sqrt(f1);
            if (fm1 != 0)
            {
                result.v0 = -fm1 / (float)Math.Sqrt(fm1 * fm1 + (fm0 - f0) * (fm0 - f0));
                result.v1 = (fm0 - f0) / (float)Math.Sqrt(fm1 * fm1 + (fm0 - f0) * (fm0 - f0));
            }
            else
            {
                result.v0 = result.v1 = 0;
                if (fm0 > fm2) result.v0 = 1.0f; else result.v1 = 1.0f;
            }
            return result;
        }

        public void clearCalibMask()
        {
            maskNum = 0;
        }

        public void applyCalibMask(byte[] data, int c)
        {
            int pos;
            for (int p = 0; p < maskNum; p++)
            {
                pos = OfflineGlobalData.mask[p];
                data[c * pos + 0] = data[c * pos + 1] = data[c * pos + 2] = 0;
            }
        }

        public void addCalibMask()
        {
            for (int p = queueOldStart; p < queueEnd; p++) OfflineGlobalData.mask[maskNum++] = OfflineGlobalData.queue[p];
        }

        /// <summary>
        /// read IDs from local txt file
        /// 'ID.txt' by default
        /// </summary>
        /// <returns></returns>
        public int loadCircleID()
        {
            string filename = "ID.txt";
            int dummy = 0;
            numberIDs = 0;
            using (StreamReader rf = new StreamReader(@filename))
            {
                if (rf == null)
                {
                    Console.WriteLine("ID file not found...\n");
                    return -2;
                }
                string s;
                while ((s = rf.ReadLine()) != null)
                {
                    Console.WriteLine(s);
                    //Split each line by 'space'
                    string[] split = s.Split(' ');
                    dummy = int.Parse(split[0]);
                    if (dummy != numberIDs)
                    {
                        Console.WriteLine("ID file corrupted...\n");
                        return -1;
                    }
                    idx[numberIDs] = float.Parse(split[1]);
                    idy[numberIDs] = float.Parse(split[2]);
                    numberIDs++;
                }
            }

            return 0;
        }

        /// <summary>
        /// change threshold if circle not detected, see 3.2 of [1]
        /// </summary>
        /// <returns></returns>
        public bool changeThreshold()
        {
            int div = 1;
            int dum = numFailed;
            while (dum > 1)
            {
                dum = dum / 2;
                div *= 2;
            }
            int step = 256 / div;
            threshold = 3 * (step * (numFailed - div) + step / 2);
            if (debug > 5) Console.WriteLine("Threshold: {0} {1} {2}\n", div, numFailed, threshold / 3);
            return step > 8;
        }

        /// <summary>
        /// transform 3D matrix to 1D matrix
        /// </summary>
        /// <param name="inputMat"></param>
        /// <param name="wid"></param>
        /// <param name="hei"></param>
        /// <param name="channel"></param>
        /// <returns></returns>
        private byte[] transform3Dto1D(byte[,,] inputMat, int wid, int hei, int channel)
        {
            byte[] outputMat = new byte[hei * wid * channel];
            Buffer.BlockCopy(inputMat, 0, outputMat, 0, wid * hei * channel);
            return outputMat;
        }

        /// <summary>
        /// transform 1D matrix to 3D matrix
        /// </summary>
        /// <param name="inputMat"></param>
        /// <param name="wid"></param>
        /// <param name="hei"></param>
        /// <param name="channel"></param>
        /// <returns></returns>
        private byte[,,] transform1Dto3D(byte[] inputMat, int wid, int hei, int channel)
        {
            byte[,,] outputMat = new byte[hei, wid, channel];
            Buffer.BlockCopy(inputMat, 0, outputMat, 0, wid * hei * channel);
            return outputMat;
        }

        #endregion

        #region ADDITIONAL MATH
        /// <summary>
        /// Get min from two integers
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private int Min(int a, int b)
        {
            return ((a) < (b) ? (a) : (b));
        }
        /// <summary>
        /// Get max from two integers
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private int Max(int a, int b)
        {
            return ((a) > (b) ? (a) : (b));
        }
        /// <summary>
        /// Get min from two doubles
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private double Min(double a, double b)
        {
            return ((a) < (b) ? (a) : (b));
        }
        /// <summary>
        /// Get max from two doubles
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private double Max(double a, double b)
        {
            return ((a) > (b) ? (a) : (b));
        }
        /// <summary>
        /// logarithm (2 as base)
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        private int log2(int value)
        {
            int r = 0;
            while (value > 0)
            {
                value = value / 2;
                r++;
            }
            return r - 1;
        }
        /// <summary>
        /// exponential (2 as base)
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        private int exp2(int value)
        {
            int r = 1;
            for (int i = 0; i < value; i++)
            {
                r = r * 2;
            }
            return r;
        }
        /// <summary>
        /// normalize angle between -pi and pi
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        private float normalizeAngle(float a)
        {
            while (a > +Math.PI) a += (float)(-2 * Math.PI);
            while (a < -Math.PI) a += (float)(2 * Math.PI);
            return a;
        }
        #endregion
    }

    public static class OfflineGlobalData
    {
        public static int[] buffer;
        public static int[] queue;
        public static int[] mask;
        public static byte[,,] globalImageData;
    }
}
