/*
 * Filename:    CosPhi.cs
 * Author:      Qinbing Fu
 * Date:        Jan 2017
 * Description: This algorithm computationally implements an efficient and precist circular pattern detection with elipses rather than necklace ID.
 * Licence:     This code is reorganized referring to CosPhi code, if you use this class, please cite [1].
 * Comments:    All similar code comments please refer to file 'CCircleDetect.cs'.
 * Reference:  [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */


using System;
using System.Drawing;
using System.IO;

namespace WhyConID
{
    public struct CosSegment
    {
        public float x;
        public float y;
        public float angle, horizontal;
        public int size;
        public int maxy, maxx, miny, minx;
        public int mean;
        public int type;
        public float roundness;
        public float bwRatio;
        public bool round;
        public bool valid;
        public float m0, m1;
        public float v0, v1;
        public float r0, r1;
        public int ID;
        //public Color color;
    };

    public class CosPhi
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
        public CosPhi() { }

        /// <summary>
        /// Paremeterized Constructor
        /// </summary>
        /// <param name="wi"></param>
        /// <param name="he"></param>
        /// <param name="numbots"></param>
        /// <param name="ID"></param>
        public CosPhi(int wi, int he, int numbots, int idi = -1)
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
            draw = true;
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
                Array.Clear(CosPhiGlobalData.buffer, 0, len);
                for (int i = 0; i < width; i++)
                {
                    CosPhiGlobalData.buffer[i] = -1000;
                    CosPhiGlobalData.buffer[pos + i] = -1000;
                }
                for (int i = 0; i < height; i++)
                {
                    CosPhiGlobalData.buffer[width * i] = -1000;
                    CosPhiGlobalData.buffer[width * i + width - 1] = -1000;
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
                    for (int x = ix; x < ax; x++) CosPhiGlobalData.buffer[poss + x] = 0;
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
            byte[] data = transform3Dto1D(CosPhiGlobalData.globalImageData, width, height, noc);
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
                if (CosPhiGlobalData.buffer[ii] == 0)
                {
                    if ((data[ii * noc + 0] + data[ii * noc + 1] + data[ii * noc + 2]) < threshold) CosPhiGlobalData.buffer[ii] = -2;
                }
                if (CosPhiGlobalData.buffer[ii] == -2)
                {
                    //new segment found
                    queueEnd = 0;
                    queueStart = 0;
                    //if the segment looks like a ring, we check its inside area
                    if (examineSegment(data, ref outer, ii, outerAreaRatio, noc))
                    {
                        pos = (int)outer.y * width + (int)outer.x;
                        if (CosPhiGlobalData.buffer[pos] == 0)
                        {
                            CosPhiGlobalData.buffer[pos] = Convert.ToInt32((data[pos * noc] + data[pos * noc + 1] + data[pos * noc + 2]) >= threshold) - 2;
                        }
                        if (CosPhiGlobalData.buffer[pos] == -1)
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
                                            pos = CosPhiGlobalData.queue[p];
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
                                            pos = CosPhiGlobalData.queue[p];
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
                    pos = CosPhiGlobalData.queue[p];
                    if (pos > 0 && pos < resol)
                        data[noc * pos + 0] = data[noc * pos + 1] = data[noc * pos + 2] = (byte)(outer.mean/3);
                }
            }
            if (draw)
            {
                if (init.valid || track || lastTrackOK)
                {
                    //inner
                    for (int p = queueOldStart; p < queueEnd; p++)
                    {
                        pos = CosPhiGlobalData.queue[p];
                        if (pos > 0 && pos < resol)
                        {
                            data[noc * pos + 0] = 0;  //B
                            data[noc * pos + 1] = 0;  //G
                            data[noc * pos + 2] = 255;    //R
                        }
                    }
                    //outer
                    for (int p = 0; p < queueOldStart; p++)
                    {
                        pos = CosPhiGlobalData.queue[p];
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
            CosPhiGlobalData.globalImageData = transform1Dto3D(data, width, height, noc);
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
            int type = CosPhiGlobalData.buffer[ii];
            int maxx, maxy, minx, miny;

            CosPhiGlobalData.buffer[ii] = ++numSegments;
            segmen.x = ii % width;
            segmen.y = ii / width;
            minx = maxx = (int)segmen.x;
            miny = maxy = (int)segmen.y;
            segmen.valid = false;
            segmen.round = false;
            //push segment coords to the queue
            CosPhiGlobalData.queue[queueEnd++] = ii;
            int dataLength = width * height;
            //and until queue is empty
            while (queueEnd > queueStart)
            {
                //pull the coord from the queue
                position = CosPhiGlobalData.queue[queueStart++];
                //search neighbours
                pos = position + 1;
                while (pos >= dataLength) pos = dataLength - 1;
                if (CosPhiGlobalData.buffer[pos] == 0)
                {
                    CosPhiGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (CosPhiGlobalData.buffer[pos] == type)
                {
                    CosPhiGlobalData.queue[queueEnd++] = pos;
                    maxx = Max(maxx, pos % width);
                    CosPhiGlobalData.buffer[pos] = numSegments;
                }
                pos = position - 1;
                while (pos < 0) pos = 0;
                if (CosPhiGlobalData.buffer[pos] == 0)
                {
                    CosPhiGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (CosPhiGlobalData.buffer[pos] == type)
                {
                    CosPhiGlobalData.queue[queueEnd++] = pos;
                    minx = Min(minx, pos % width);
                    CosPhiGlobalData.buffer[pos] = numSegments;
                }
                pos = position - width;
                while (pos < 0) pos = 0;
                if (CosPhiGlobalData.buffer[pos] == 0)
                {
                    CosPhiGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c + 0] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (CosPhiGlobalData.buffer[pos] == type)
                {
                    CosPhiGlobalData.queue[queueEnd++] = pos;
                    miny = Min(miny, pos / width);
                    CosPhiGlobalData.buffer[pos] = numSegments;
                }
                pos = position + width;
                while (pos >= dataLength) pos = dataLength - 1;
                if (CosPhiGlobalData.buffer[pos] == 0)
                {
                    CosPhiGlobalData.buffer[pos] = Convert.ToInt32((data[pos * c + 0] + data[pos * c + 1] + data[pos * c + 2]) > threshold) - 2;
                }
                if (CosPhiGlobalData.buffer[pos] == type)
                {
                    CosPhiGlobalData.queue[queueEnd++] = pos;
                    maxy = Max(maxy, pos / width);
                    CosPhiGlobalData.buffer[pos] = numSegments;
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
                        pos = CosPhiGlobalData.queue[p];
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
                pos = CosPhiGlobalData.mask[p];
                data[c * pos + 0] = data[c * pos + 1] = data[c * pos + 2] = 0;
            }
        }

        public void addCalibMask()
        {
            for(int p = queueOldStart; p < queueEnd; p++) CosPhiGlobalData.mask[maskNum++] = CosPhiGlobalData.queue[p];
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
            if (debug > 5) Console.WriteLine("Threshold: {0} {1} {2}\n", div, numFailed, threshold/3);
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

    public static class CosPhiGlobalData
    {
        public static int[] buffer;
        public static int[] queue;
        public static int[] mask;
        public static byte[,,] globalImageData;
    }
}
