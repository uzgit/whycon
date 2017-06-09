/*
 * Filename: Form1.cs
 * Author: Qinbing Fu
 * Date: Jan 2017
 * Description: This is the main form.
 * Advice:      1.Generally, this UI contains multi-threading which could handle two tracking system concurrently, but they share some global parameters (can make them separately, yet in simplify in this version).
 *              2.So, it is better to run only one tracking system each time. However, the other operations, such as recording, show raw capture, etc. are fine to run in parallel.
 *              3.User needs to change some file read/write directory paths and names in FIELDS rather than on UI.
 */

using System;
using System.Threading.Tasks;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.CvEnum;
using System.Diagnostics;
using System.Threading;
using System.Collections.Generic;
using System.Drawing;
using System.IO;

namespace WhyConID
{
    public partial class Form1 : Form
    {
        #region FIELDS
        //subform object
        private Form2 form2;
        private Form3 form3;
        private Form4 form4;
        private Form5 form5;
        /********************************************************************************/
        //File path and name for user to change here.
        private string fileDirectoryPath = @"D:\PhD\EXPERIMENTS\2-Jan-Feb-Mar-Apr Lincoln\Camera Recordings\";  //save and load files directory path
        private string saveVideoName = "fqb.avi";           //recording file name
        private string whyconTxtFileName = "whycon.txt";    //WhyConID tracking results in text file
        private string cosphiTxtFileName = "cosphi.txt";    //CosPhi tracking results in text file
        private string offTxtFileName = "fqb.txt";      //offline tracking results
        /********************************************************************************/
        //frame list for recording
        private List<Image<Bgr, byte>> recordingList;
        //timers
        //private System.Timers.Timer cap_tick;
        //capture object
        public static Capture cap;
        private UInt64 globalCounter;
        public static int? fps;
        private int channel;                         //number of image channels
        
        //User can change following boolean controls.
        /*666*****************************************************666*/
        private int debug = 0;                       //1 to debug track frame-by-frame, 0 to run real time
        /*666*****************************************************666*/
        private int elapsed_time = 0;                //elapsed time in ms
        private bool elapsed = true;                 //monitor elapsed time or not
        /*666*****************************************************666*/
        private bool drawPoints = true;             //if drawing the center of inner pattern
        /*666*****************************************************666*/
        private bool saveTrack = false;
        /*666*****************************************************666*/
        private bool saveImg = false;
        private bool separatedWindow = true;        //if show CosPhi tracking in another new window form
        private List<Point> whyconPoints = new List<Point>();                    //list of points to draw whycon track
        private List<Point>[] cosphiPoints;                    //list of points to draw cosphi track
        /*666*****************************************************666*/

        //-----These parameters need to be adjusted by the user -----------------------
        //black circle diameter
        private float circleDiameter;
        /* X and Y dimensions of the coordinate system */
        private float fieldLength = 1.0f;
        private float fieldWidth = 1.0f;
        /* robot detection variables */
        public static int numBots;                          //num of robots to track
        private int numFound;                               //num of robots detected in the last step
        private int phiFound;                               //num of cosphi marker detected
        private int found;                                  //num of robots found to shown in status
        private int numStatic;                              //num of non-moving robots
        private int phiStatic;                              //num of non-moving cosphi agents
        private float static_thre;                          //the threshold to decide whether static
        private CCircleDetect[] detectorArray;              //detector array (each pattern has its own detector)
        private CosPhi[] cosphiArray;                       //cosphi pattern detector array (each pattern has its own detector)
        private CosSegment[] currentCosphiArray;            //cosphi segment array (detected objects in image view)
        private CosSegment[] lastCosphiArray;               //cosphi segment position in the last step (allows for tracking)
        private SSegment[] currentSegmentArray;             //segment array (detected objects in image space)
        private SSegment[] lastSegmentArray;                //segment position in the last step (allows for tracking)
        private STrackedObject[] objectArray;               //object array (detected objects in metric space)
        private CTransformation trans;				        //allows to transform from image to metric coordinates

        /*variables related to (auto) calibration*/
        private const int calibrationSteps = 20;			    //how many measurements to average to estimate calibration pattern position (manual calib)
        private const int autoCalibrationSteps = 30; 			//how many measurements to average to estimate calibration pattern position (automatic calib)  
        private const int autoCalibrationPreSteps = 10;		    //how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
        private int calibNum;				                //number of objects acquired for calibration (5 means calibration winished inactive)
        private STrackedObject[] calib; //array to store calibration patterns positions
        //private STrackedObject[] calibTmp;	//array to store several measurements of a given calibration pattern
        private int calibStep;		//actual calibration step (num of measurements of the actual pattern)
        private bool autocalibrate;			//is the autocalibration in progress ?
        private ETransformType lastTransformType; //pre-calibration transform (used to preserve pre-calibation transform type)
        private int wasBots;				//pre-calibration number of robots to track (used to preserve pre-calibation number of robots to track)

        /*program flow control*/
        //bool saveVideo;		//save video to output folder?
        //bool saveLog;		//save log to output folder?
        //private bool stop;		    //stop and exit ?
        //private int moveOne, moveVal;		    //how many frames to process ? how many frames to process now (setting moveOne to 0 or lower freezes the video stream)

        public const byte COLOR_PRECISION = 32;
        public const byte COLOR_STEP = 8;
        public const byte INNER = 0;
        public const byte OUTER = 1;
        /*********************************/
        public const byte MAX_PATTERNS =16;        //maximum patterns to track
        /*********************************/

        //used for circle identification
        public const int ID_SAMPLES = 320;
        public const int ID_BITS = 8;
        //-----------------------------------------------------------------------------
        #endregion

        #region CAMERA PROPERTY
        // image resolutiuon
        public static int frame_width;
        public static int frame_height;
        public static int max_frame_width;
        public static int min_frame_width;
        public static int max_frame_height;
        public static int min_frame_height;
        public static int exposure;
        public static int brightness;
        public static int contrast;
        public static int gain;
        public static int gamma;
        public static int sharpness;
        public static int temperature;
        public string frameRate
        {
            get;
            set;
        }
        private long lastTime;
        private long delta;
        private int frameCount;
        #endregion

        #region FORM INIT
        /// <summary>
        /// Constructing
        /// </summary>
        public Form1()
        {
            InitializeComponent();
            //Control.CheckForIllegalCrossThreadCalls = false;                          //not recommended
            //Capture
            cap = new Capture();                                                        //capture object
            form2 = new Form2();                                                        //Sub form object (camera properties)
            form3 = new Form3();                                                        //Sub form object (raw frame capture show)
            form4 = new Form4();                                                        //Sub form object (CosPhi show)
            form5 = new Form5();                                                        //Sub form object (Offline show)
            globalCounter = 0;
            /******************/
            InitializeCameraProperty();
            InitializeFrameRate();
            /************************************************/

            Console.WriteLine("WINDOWS INIT COMPLETED.......");

            //buttons
            buttonDetect.Enabled = false;
            buttonAddRobot.Enabled = false;
            saveRecordButton.Enabled = false;
            buttonRecord.Enabled = false;
            buttonSubRobot.Enabled = false;
            buttonCosPhiTrackInit.Enabled = false;
            buttonDetectCosPhi.Enabled = false;
            buttonCosPhiTrackInit.Enabled = false;
            //buttonRunOfflineCosPhi.Enabled = false;
        }

        /// <summary>
        /// Form1 Loading
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Load(object sender, EventArgs e)
        {
            recordingList = new List<Image<Bgr, byte>>();
            this.timer1.Interval = 1000;
            this.timer1.Start();
        }
        #endregion

        #region MAIN FUNCTIONS

        /// <summary>
        /// Init and Get Camera Setting
        /// </summary>
        private void InitializeCameraProperty()
        {
            frame_width = int.Parse(form2.textBoxWidthText.Text);                       //get frame width from properties setting
            frame_height = int.Parse(form2.textBoxHeightText.Text);                     //get frame height from properties setting
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_WIDTH, frame_width);      //set frame width
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_HEIGHT, frame_height);    //set frame height
            mainImageBox.SizeMode = PictureBoxSizeMode.StretchImage;                    //stretch image
            max_frame_width = 1280;                                                     //max frame width
            max_frame_height = 1024;                                                    //max frame height
            min_frame_width = 640;                                                      //min frame width
            min_frame_height = 480;                                                     //min frame height
            exposure = (int)cap.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_EXPOSURE);      //get exposure value
            brightness = (int)cap.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_BRIGHTNESS);  //get brightness value
            contrast = (int)cap.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_CONTRAST);      //get contrast value
            gain = (int)cap.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_GAIN);              //get gain value
            gamma = (int)cap.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_GAMMA);            //get gamma value
            sharpness = (int)cap.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_SHARPNESS);    //get sharpness value
            temperature = (int)cap.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_TEMPERATURE);//get temperature value
        }

        /// <summary>
        /// Reset Camera Property
        /// </summary>
        private void SetCameraProperty()
        {
            if (frame_width > max_frame_width)
                frame_width = max_frame_width;
            if (frame_width < min_frame_width)
                frame_width = min_frame_width;
            if (frame_height > max_frame_height)
                frame_height = max_frame_height;
            if (frame_height < min_frame_height)
                frame_height = min_frame_height;
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_WIDTH, frame_width);      //set frame width
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_HEIGHT, frame_height);    //set frame height
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_EXPOSURE, exposure);            //set exposure
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_BRIGHTNESS, brightness);        //set brightness
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_CONTRAST, contrast);            //set contrast
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_GAIN, gain);                    //set gain
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_GAMMA, gamma);                  //set gamma
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_TEMPERATURE, temperature);      //set temperature
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_SHARPNESS, sharpness);          //set sharpness
        }

        /// <summary>
        /// Initialize Frame Rate
        /// </summary>
        private void InitializeFrameRate()
        {
            lastTime = DateTime.Now.ToFileTimeUtc();
            frameRate = "FPS 0";
        }

        /// <summary>
        /// A way of calculating frame rate
        /// </summary>
        private void CalculateFrameRate()
        {
            long current = DateTime.Now.ToFileTimeUtc();
            delta += (current - lastTime) / 10000;
            lastTime = current;
            frameCount++;
            if (delta > 1000)
            {
                delta -= 1000;
                frameRate = "FPS " + frameCount.ToString();
                frameCount = 0;
            }
        }

        /// <summary>
        /// Processing Raw Capturing
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ProcessRawFrame(object sender, EventArgs e)
        {
            if (form2.updateSetting == true)
            {
                SetCameraProperty();
                form2.updateSetting = false;
            }
            using (Image<Bgr, Byte> rawFrame = cap.QueryFrame())
            {
                //Image<Bgr, Byte> gauss = rawFrame.Sobel(1,0,5);
                //mainImageBox.Image = gauss;
                mainImageBox.Image = rawFrame;
            }
        }

        /// <summary>
        /// Processing Raw Image Capturing
        /// </summary>
        private void ProcessRawFrame()
        {
            long iter = 100000000;
            for (int t = 0; t < iter; t++)
            {
                if (form2.updateSetting == true)
                {
                    SetCameraProperty();
                    form2.updateSetting = false;
                }
                using (Image<Bgr, Byte> rawFrame = cap.QueryFrame())
                {
                    //Image<Bgr, Byte> gauss = rawFrame.Sobel(1,0,5);
                    //mainImageBox.Image = gauss;
                    //mainImageBox.Image = rawFrame;
                    form3.rawImageBox.Image = rawFrame;
                    Thread.Sleep(10);
                }
            }
        }

        private Task ProcessRawAsync()
        {
            return Task.Factory.StartNew(() => ProcessRawFrame());
        }

        private async void CallProcessRawFrame()
        {
            await ProcessRawAsync();
        }

        /// <summary>
        /// Set FPS
        /// </summary>
        public static void SetFps()
        {
            cap.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_FPS, Convert.ToDouble(fps));
        }

        /// <summary>
        /// Process Online Tracking
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ProcessOnlineTrack(object sender, EventArgs e)
        {
            //font to show id
            MCvFont font = new MCvFont();
            font.hscale = 0.6f; font.vscale = 0.6f; font.thickness = 2;
            //if updating camera setting
            if (form2.updateSetting == true)
            {
                SetCameraProperty();
                form2.updateSetting = false;
            }
            //if monitoring elaped time
            var stopwatch = new Stopwatch();
            if (elapsed)
            {
                stopwatch.Start();
            }

            //main localization system
            using (Image<Bgr, Byte> rawFrame = cap.QueryFrame())
            {
                if (rawFrame != null)
                {
                    //Image<Bgr, Byte> laplace = rawFrame.Laplace(1).Convert<Bgr, Byte>();
                    using (Image<Bgr, Byte> cloneFrame = rawFrame.Clone())
                    {
                        channel = cloneFrame.NumberOfChannels;
                        numFound = numStatic = 0;
                        //numFound = 0;

                        //update global image data
                        GlobalData.globalImageData = cloneFrame.Data;

                        //start tracking
                        //track the robots found in the last attemp
                        for (int i = 0; i < numBots; i++)
                        {
                            if (currentSegmentArray[i].valid)
                            {
                                lastSegmentArray[i] = currentSegmentArray[i];
                                currentSegmentArray[i] = detectorArray[i].findSegment(lastSegmentArray[i], channel);
                                Console.WriteLine("Object -> {0}, ID -> {1}", i, currentSegmentArray[i].ID);
                            }
                        }

                        //search for untracked robots (not detected in the last frame)
                        for (int i = 0; i < numBots; i++)
                        {
                            if (currentSegmentArray[i].valid == false)
                            {
                                lastSegmentArray[i].valid = false;
                                currentSegmentArray[i] = detectorArray[i].findSegment(lastSegmentArray[i], channel);
                            }
                            //do not make sense to search for more patterns if the last one was not found
                            if (currentSegmentArray[i].valid == false) break;
                        }

                        //perform transformations from camera to world coordinates
                        for (int i = 0; i < numBots; i++)
                        {
                            if (currentSegmentArray[i].valid)
                            {
                                //objectArray[i] = trans.transform(currentSegmentArray[i], false);
                                numFound++;
                                Console.WriteLine("Pattern {0} {1:F} {2:F} {3:F}", currentSegmentArray[i].ID, currentSegmentArray[i].x, currentSegmentArray[i].y, currentSegmentArray[i].angle);
                                if (Math.Abs(currentSegmentArray[i].x - lastSegmentArray[i].x) < static_thre) numStatic++;
                            }
                        }
                        Console.WriteLine("Pattern detection: Found -> {0} Static -> {1}", numFound, numStatic);
                        //Console.WriteLine("Pattern detection: Found -> {0}\n", numFound);

                        //establishing the coordinate system by manual or autocalibration
                        //if (autocalibrate && numFound == numBots) autocalibration();
                        //if (calibNum < 4) manualcalibration();

                        //print out results
                        /*
                        for (int i = 0; i < numBots; i++)
                        {
                            if (currentSegmentArray[i].valid) Console.WriteLine("Object {0} {1:F} {2:F} {3:F} {4:F}", objectArray[i].ID, objectArray[i].x, objectArray[i].y, objectArray[i].z, objectArray[i].yaw);
                        }
                        */
                        //Console.WriteLine();

                        //display
                        found = numFound;
                        cloneFrame.Data = GlobalData.globalImageData;
                        //draw ID number with marker on the image
                        for (int i = 0; i < numBots; i++)
                        {
                            if (currentSegmentArray[i].valid)
                                cloneFrame.Draw(currentSegmentArray[i].ID.ToString(), ref font, new Point((int)currentSegmentArray[i].x, (int)currentSegmentArray[i].y), new Bgr(Color.Red));
                        }
                        mainImageBox.Image = cloneFrame;
                        //try manual dispose
                        //cloneFrame.Dispose();

                        //if monitoring elpased time in tracking
                        if (elapsed)
                        {
                            stopwatch.Stop();
                            elapsed_time = (int)stopwatch.ElapsedMilliseconds;
                            Console.WriteLine("Elapsed time: {0}\n", elapsed_time);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Process Online Tracking (can define recording tracking or not)
        /// </summary>
        private void ProcessOnlineTrack()
        {
            //font to show id
            MCvFont font = new MCvFont();
            font.hscale = 0.6f; font.vscale = 0.6f; font.thickness = 2; //User can change font properties here.
            //text file name
            string textFileName = fileDirectoryPath + whyconTxtFileName;
            //track video name
            //string video = fileDirectoryPath + saveVideoName;

            using (TextWriter fqb = File.CreateText(textFileName))
            {
                //iterations of track, set to large number
                long iter = 100000000;
                for (int t = 0; t < iter; t++)
                {
                    //if updating camera setting
                    if (form2.updateSetting == true)
                    {
                        SetCameraProperty();
                        form2.updateSetting = false;
                    }
                    //if monitoring elaped time
                    var stopwatch = new Stopwatch();
                    if (elapsed)
                    {
                        stopwatch.Start();
                    }

                    //main localization system
                    //output IDs, Xs, Ys to console window
                    //output IDs, Xs, Ys, angles to text file
                    using (Image<Bgr, Byte> rawFrame = cap.QueryFrame())
                    {
                        if (rawFrame != null)
                        {
                            using (Image<Bgr, Byte> cloneFrame = rawFrame.Clone())
                            {
                                channel = cloneFrame.NumberOfChannels;
                                numFound = numStatic = 0;
                                //numFound = 0;

                                //update global image data
                                GlobalData.globalImageData = cloneFrame.Data;

                                //start tracking
                                //track the robots found in the last attemp
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentSegmentArray[i].valid)
                                    {
                                        lastSegmentArray[i] = currentSegmentArray[i];
                                        currentSegmentArray[i] = detectorArray[i].findSegment(lastSegmentArray[i], channel);
                                        Console.WriteLine("Object -> {0}, ID -> {1}", i, currentSegmentArray[i].ID);
                                    }
                                }

                                //search for untracked robots (not detected in the last frame)
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentSegmentArray[i].valid == false)
                                    {
                                        lastSegmentArray[i].valid = false;
                                        currentSegmentArray[i] = detectorArray[i].findSegment(lastSegmentArray[i], channel);
                                    }
                                    //do not make sense to search for more patterns if the last one was not found
                                    if (currentSegmentArray[i].valid == false)
                                        break;
                                    //check if detect tracked patterns, valid set to false if true
                                    else
                                    {
                                        for (int j = 0; j < numBots; j++)
                                        {
                                            if (i == j) continue;
                                            if (Math.Abs(currentSegmentArray[i].x - currentSegmentArray[j].x) < 1 && Math.Abs(currentSegmentArray[i].y - currentSegmentArray[j].y) < 1)
                                                currentSegmentArray[i].valid = false;
                                        }
                                    }
                                }

                                //perform transformations from camera to world coordinates
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentSegmentArray[i].valid)
                                    {
                                        //objectArray[i] = trans.transform(currentSegmentArray[i], false);
                                        numFound++;

                                        //if draw points
                                        if (drawPoints)
                                            whyconPoints.Add(new Point((int)currentSegmentArray[i].x, (int)currentSegmentArray[i].y));

                                        //Console.WriteLine("Pattern {0} {1:F} {2:F} {3:F}", currentSegmentArray[i].ID, currentSegmentArray[i].x, currentSegmentArray[i].y, currentSegmentArray[i].angle);
                                        if (Math.Abs(currentSegmentArray[i].x - lastSegmentArray[i].x) < static_thre) numStatic++;
                                    }
                                }
                                Console.WriteLine("Pattern detection: Found -> {0} Static -> {1}", numFound, numStatic);

                                //display
                                found = numFound;
                                cloneFrame.Data = GlobalData.globalImageData;

                                //draw ID number with marker on the image, and write results to text
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentSegmentArray[i].valid)
                                    {
                                        cloneFrame.Draw(currentSegmentArray[i].ID.ToString(), ref font, new Point((int)currentSegmentArray[i].x, (int)currentSegmentArray[i].y), new Bgr(Color.Red));
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", t, currentSegmentArray[i].ID, (int)currentSegmentArray[i].x, (int)currentSegmentArray[i].y, currentSegmentArray[i].angle);
                                    }
                                    else
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", t, lastSegmentArray[i].ID, (int)lastSegmentArray[i].x, (int)lastSegmentArray[i].y, lastSegmentArray[i].angle);
                                }
                                //if drawing points
                                if (drawPoints)
                                {
                                    cloneFrame.DrawPolyline(whyconPoints.ToArray(),false,new Bgr(202,196,250),2);
                                }
                                //if recording track
                                if (saveTrack)
                                    recordingList.Add(cloneFrame);

                                //display in image view
                                mainImageBox.Image = cloneFrame;

                                //if save image
                                if (saveImg)
                                {
                                    if (textBoxSaveImage.Text == "")
                                        mainImageBox.Image.Save(fileDirectoryPath + globalCounter.ToString() + ".png");
                                    else
                                        mainImageBox.Image.Save(fileDirectoryPath + textBoxSaveImage.Text + ".png");
                                    saveImg = false;
                                }

                                //if monitoring elpased time in tracking
                                if (elapsed)
                                {
                                    stopwatch.Stop();
                                    elapsed_time = (int)stopwatch.ElapsedMilliseconds;
                                    Console.WriteLine("Elapsed time: {0}\n", elapsed_time);
                                }

                                Thread.Sleep(30);
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Asynchronous task of processing online tracking
        /// </summary>
        /// <returns></returns>
        private Task ProcessOnlineAsync()
        {
            return Task.Factory.StartNew(() => ProcessOnlineTrack());
        }

        /// <summary>
        /// Call on-line tracking task
        /// </summary>
        private async void CallProcessOnlineTrack()
        {
            await ProcessOnlineAsync();
        }

        /// <summary>
        /// (WhyCon Edition) Process off-line input RGB-only video (can run concurrently with on-line tracking), output results to text file
        /// </summary>
        private void ProcessOfflineTrack()
        {
            //elapsed time start to count
            var stopwatch = new Stopwatch();
            if (elapsed)
            {
                stopwatch.Start();
            }

            //output text file init
            string outputTextFileName = fileDirectoryPath + offTxtFileName;

            //input video init
            string inputFileName = fileDirectoryPath + textBoxOfflineVideo.Text;
            Capture fc = new Capture(inputFileName);
            int frames = (int)fc.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_COUNT);

            //local members init: this process can run concurrently with other processes, so take care of global variables.
            int numOfBots = int.Parse(textBoxOffNO.Text);
            int start = 1;
            OffCircleDetect[] detector = new OffCircleDetect[numOfBots];
            SSegment[] currentSegment = new SSegment[numOfBots];
            SSegment[] lastSegment = new SSegment[numOfBots];
            for (int i = 0; i < numOfBots; i++)
            {
                detector[i] = new OffCircleDetect(frame_width, frame_height, ID_BITS, i);
            }

            //global variables
            OfflineGlobalData.globalImageData = new byte[frame_width, frame_height, 3];
            OfflineGlobalData.buffer = new int[frame_height * frame_width];
            OfflineGlobalData.queue = new int[frame_height * frame_width];
            //font to show id
            //MCvFont font = new MCvFont();
            //font.hscale = 0.6f; font.vscale = 0.6f; font.thickness = 2; //User can change font properties here.
            //localization system
            //to output results in local text file
            using (TextWriter fqb = File.CreateText(outputTextFileName))
            {
                while (start <= frames)
                {
                    fc.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_POS_FRAMES,start);
                    //start to find segment and calculate posture
                    using (Image<Bgr, Byte> rawFrame = fc.QueryFrame())
                    {
                        if (rawFrame == null)
                        {
                            start++;
                            continue;
                        }
                        else
                        {
                            using (Image<Bgr, Byte> cloneFrame = rawFrame.Clone())
                            {
                                int channel = cloneFrame.NumberOfChannels;
                                //update global image data
                                OfflineGlobalData.globalImageData = cloneFrame.Data;
                                //start tracking
                                //track the robots found in the last attemp
                                for (int i = 0; i < numOfBots; i++)
                                {
                                    if (currentSegment[i].valid)
                                    {
                                        lastSegment[i] = currentSegment[i];
                                        currentSegment[i] = detector[i].findSegment(lastSegment[i], channel);
                                    }
                                }
                                //search for untracked robots (not detected in the last frame)
                                for (int i = 0; i < numOfBots; i++)
                                {
                                    if (currentSegment[i].valid == false)
                                    {
                                        lastSegment[i].valid = false;
                                        currentSegment[i] = detector[i].findSegment(lastSegment[i], channel);
                                    }
                                    //do not make sense to search for more patterns if the last one was not found
                                    if (currentSegment[i].valid == false)
                                        break;
                                    //check if detect tracked patterns, valid set to false if true
                                    /*
                                    else
                                    {
                                        for (int j = 0; j < numOfBots; j++)
                                        {
                                            if (i == j) continue;
                                            if (Math.Abs(currentSegment[i].x - currentSegment[j].x) < 1 && Math.Abs(currentSegment[i].y - currentSegment[j].y) < 1)
                                                currentSegment[i].valid = false;
                                        }
                                    }
                                    */
                                }
                                //cloneFrame.Data = OfflineGlobalData.globalImageData;

                                //draw ID number with marker on the image, and write results to text
                                for (int i = 0; i < numOfBots; i++)
                                {
                                    if (currentSegment[i].valid)
                                    {
                                        //cloneFrame.Draw(currentSegment[i].ID.ToString(), ref font, new Point((int)currentSegment[i].x, (int)currentSegment[i].y), new Bgr(Color.Red));
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", start, currentSegment[i].ID, (int)currentSegment[i].x, (int)currentSegment[i].y, currentSegment[i].angle);
                                    }
                                    else
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", start, lastSegment[i].ID, (int)lastSegment[i].x, (int)lastSegment[i].y, lastSegment[i].angle);
                                }
                                //display in image view of Form5
                                //form5.offlineImageBox.Image = cloneFrame;
                                //Thread.Sleep(50);
                            }

                            start++;
                        }
                    }
                }
            }

            //elapsed time
            if (elapsed)
            {
                stopwatch.Stop();
                elapsed_time = (int)stopwatch.ElapsedMilliseconds;
                MessageBox.Show("Offline file processed in " + elapsed_time + " ms");
            }
        }

        /// <summary>
        /// Asynchronous task to process off-line tracking algorithm
        /// </summary>
        /// <returns></returns>
        private Task ProcessOfflineTrackAsync()
        {
            return Task.Factory.StartNew(() => ProcessOfflineTrack());
        }

        /// <summary>
        /// Call off-line tracking task
        /// </summary>
        private async void CallProcessOfflineTrack()
        {
            await ProcessOfflineTrackAsync();
        }

        /// <summary>
        /// (CosPhi Edition) Process off-line input RGB-only video (can run concurrently with on-line tracking), save each frame with drawing to video, output results to text file
        /// </summary>
        private void ProcessCosPhiOff()
        {   
            //elapsed time start to count
            var stopwatch = new Stopwatch();
            if (elapsed)
            {
                stopwatch.Start();
            }

            //output text file init
            string outputTextFileName = fileDirectoryPath + offTxtFileName;

            //input video init
            string inputFileName = fileDirectoryPath + textBoxOfflineVideo.Text;
            Capture fc = new Capture(inputFileName);
            int fps = (int)fc.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_FPS);
            int frames = (int)fc.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_COUNT);
            int frame_height = (int)fc.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_HEIGHT);
            int frame_width = (int)fc.GetCaptureProperty(CAP_PROP.CV_CAP_PROP_FRAME_WIDTH);
            //local members init: this process can run concurrently with other processes, so take care of global variables.
            int numOfBots = int.Parse(textBoxOffNO.Text);
            int start = 1;
            OffCosPhiDetect[] cosphiArray = new OffCosPhiDetect[numOfBots];
            CosSegment[] currentCosphiArray = new CosSegment[numOfBots];
            CosSegment[] lastCosphiArray = new CosSegment[numOfBots];
            for (int i = 0; i < numOfBots; i++)
            {
                cosphiArray[i] = new OffCosPhiDetect(frame_width, frame_height, MAX_PATTERNS, i);
            }
            cosphiPoints = new List<Point>[numOfBots];
            for (int i = 0; i < cosphiPoints.Length; i++)
            {
                cosphiPoints[i] = new List<Point>();
            }
            //global variables
            OfflineGlobalData.globalImageData = new byte[frame_width, frame_height, 3];
            OfflineGlobalData.buffer = new int[frame_height * frame_width];
            OfflineGlobalData.queue = new int[frame_height * frame_width];
            //font to show id
            MCvFont font = new MCvFont();
            font.hscale = 0.6f; font.vscale = 0.6f; font.thickness = 2; //User can change font properties here.
            //localization system
            //to output results in local text file
            using (TextWriter fqb = File.CreateText(outputTextFileName))
            {
                while (start <= frames)
                {
                    fc.SetCaptureProperty(CAP_PROP.CV_CAP_PROP_POS_FRAMES, start);
                    //start to find segment and calculate posture
                    using (Image<Bgr, Byte> rawFrame = fc.QueryFrame())
                    {
                        if (rawFrame == null)
                        {
                            start++;
                            continue;
                        }
                        else
                        {
                            using (Image<Bgr, Byte> cloneFrame = rawFrame.Clone())
                            {
                                //update global image data
                                OfflineGlobalData.globalImageData = cloneFrame.Data;

                                //start tracking
                                //track the robots found in the last attemp
                                for (int i = 0; i < numOfBots; i++)
                                {
                                    if (currentCosphiArray[i].valid)
                                    {
                                        lastCosphiArray[i] = currentCosphiArray[i];
                                        currentCosphiArray[i] = cosphiArray[i].findSegment(lastCosphiArray[i], 3);
                                    }
                                }

                                //search for untracked robots (not detected in the last frame)
                                for (int i = 0; i < numOfBots; i++)
                                {
                                    if (currentCosphiArray[i].valid == false)
                                    {
                                        lastCosphiArray[i].valid = false;
                                        currentCosphiArray[i] = cosphiArray[i].findSegment(lastCosphiArray[i], 3);
                                    }
                                    //do not make sense to search for more patterns if the last one was not found
                                    if (currentCosphiArray[i].valid == false)
                                        break;
                                }

                                //perform transformations from camera to world coordinates
                                for (int i = 0; i < numOfBots; i++)
                                {
                                    if (currentCosphiArray[i].valid)
                                    {
                                        //if draw points
                                        if (drawPoints)
                                            cosphiPoints[i].Add(new Point((int)currentCosphiArray[i].x, (int)currentCosphiArray[i].y));
                                    }
                                }

                                cloneFrame.Data = OfflineGlobalData.globalImageData;

                                //draw ID number with marker on the image, and write results to text
                                for (int i = 0; i < numOfBots; i++)
                                {
                                    if (currentCosphiArray[i].valid)
                                    {
                                        cloneFrame.Draw(currentCosphiArray[i].ID.ToString(), ref font, new Point((int)currentCosphiArray[i].x, (int)currentCosphiArray[i].y), new Bgr(Color.Red));
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", start, currentCosphiArray[i].ID, (int)currentCosphiArray[i].x, (int)currentCosphiArray[i].y, currentCosphiArray[i].angle);
                                    }
                                    else
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", start, lastCosphiArray[i].ID, (int)lastCosphiArray[i].x, (int)lastCosphiArray[i].y, lastCosphiArray[i].angle);
                                }

                                //if drawing points
                                if (drawPoints)
                                {
                                    for (int i = 0; i < numOfBots; i++)
                                    {
                                        if (cosphiPoints[i] != null)
                                            cloneFrame.DrawPolyline(cosphiPoints[i].ToArray(), false, new Bgr(0, 255 * 8 / MAX_PATTERNS, 0), 2);
                                    }
                                }

                                //display
                                if (separatedWindow)
                                {
                                    form5.offlineImageBox.Image = cloneFrame;
                                    //save img
                                    if (saveImg)
                                    {
                                        saveImg = false;
                                        form5.offlineImageBox.Image.Save(fileDirectoryPath + globalCounter + ".png");
                                    }
                                }
                                else
                                    mainImageBox.Image = cloneFrame;
                                Thread.Sleep(10);
                            }
                            start++;
                        }
                    }
                }
                //save last frame to file
                //form5.offlineImageBox.Image.Save(fileDirectoryPath + "final.png");
            }
            
            //elapsed time
            if (elapsed)
            {
                stopwatch.Stop();
                elapsed_time = (int)stopwatch.ElapsedMilliseconds;
                MessageBox.Show("Offline file processed in " + elapsed_time + " ms");
            }
        }

        /// <summary>
        /// Asynchronous task to process offline Cosphi tracking algorithm
        /// </summary>
        /// <returns></returns>
        private Task ProcessCosPhiOffAsync()
        {
            return Task.Factory.StartNew(()=>ProcessCosPhiOff());
        }

        /// <summary>
        /// Call process offline Cosphi detection
        /// </summary>
        private async void CallProcessCosPhiOff()
        {
            await ProcessCosPhiOffAsync();
        }

        /// <summary>
        /// find four outermost circles and use them to set-up the coordinate system - [0,0] is the left-top, [0, fieldLength] next in clockwise direction, and so on.
        /// </summary>
        private void AutoCalibration()
        {
            bool saveVals = true;
            for (int i = 0; i < numBots; i++)
            {
                if (detectorArray[i].lastTrackOK == false) saveVals = false;
            }
            if (saveVals)
            {
                int[] index = {0, 0, 0, 0};
                int maxEval = 0;
                int eval = 0;
                int[] sX = {-1, 1, -1, 1};
                int[] sY = {1, 1, -1, -1};
                for (int i = 0; i < 4; i++)
                {
                    maxEval = -10000000;
                    for (int j = 0; j < numBots; j++)
                    {
                        eval = (int)(sX[i] * currentSegmentArray[j].x + sY[i] * currentSegmentArray[j].y);
                        if (eval > maxEval)
                        {
                            maxEval = eval;
                            index[i] = j;
                        }
                    }
                }
                Console.WriteLine("\nINDEX: {0} {1} {2} {3}\n", index[0], index[1], index[2], index[3]);
                for (int i = 0; i < 4; i++)
                {
                    if (calibStep <= autoCalibrationPreSteps) calib[i].x = calib[i].y = calib[i].z = 0;
                    calib[i].x += objectArray[index[i]].x;
                    calib[i].y += objectArray[index[i]].y;
                    calib[i].z += objectArray[index[i]].z;
                }
                calibStep++;
                if (calibStep == autoCalibrationSteps)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        calib[i].x = calib[i].x / (autoCalibrationSteps - autoCalibrationPreSteps);
                        calib[i].y = calib[i].y / (autoCalibrationSteps - autoCalibrationPreSteps);
                        calib[i].z = calib[i].z / (autoCalibrationSteps - autoCalibrationPreSteps);
                    }
                    trans.calibrate2D(calib, fieldLength, fieldWidth);
                    trans.calibrate3D(calib, fieldLength, fieldWidth);
                    trans.calibrate4D(calib, fieldLength, fieldWidth);
                    calibNum++;
                    numBots = wasBots;
                    trans.transformType = lastTransformType;
                    autocalibrate = false;
                }
            }
        }

        /// <summary>
        /// Record raw frame
        /// </summary>
        private void RecordRawFrame()
        {
            int iter = 300;
            for (int i = 0; i < iter; i++)
            {
                recordingList.Add(cap.QueryFrame());
            }
            MessageBox.Show("Recorded");
        }

        /// <summary>
        /// Asynchronous task to record raw frame
        /// </summary>
        /// <returns></returns>
        private Task RecordRawFrameAsync()
        {
            return Task.Factory.StartNew(() => RecordRawFrame());
        }

        /// <summary>
        /// Call record task
        /// </summary>
        private async void CallRecordRawFrame()
        {
            await RecordRawFrameAsync();
        }

        /// <summary>
        /// Save recording video to file
        /// </summary>
        private void SaveRecordingToFile()
        {
            int fps = 90; //manually control fps here, not in textbox of UI
            string outputFileName = fileDirectoryPath + saveVideoName;
            using (VideoWriter fqb = new VideoWriter(outputFileName, CvInvoke.CV_FOURCC('M', 'J', 'P', 'G'), fps, frame_width, frame_height, true))
            {
                for (int i = 1; i < recordingList.Count - 1; i++)
                {
                    if (recordingList[i] != null)
                        fqb.WriteFrame<Bgr, Byte>(recordingList[i]);
                    else
                        continue;
                }
            }
            recordingList.Clear();
            MessageBox.Show("Recording Saved");
        }

        /// <summary>
        /// Asynchronous task to save recording to file
        /// </summary>
        /// <returns></returns>
        private Task SaveToFileAsync()
        {
            return Task.Factory.StartNew(() => SaveRecordingToFile());
        }

        /// <summary>
        /// Call save task
        /// </summary>
        private async void CallSaveRecordingToFile()
        {
            await SaveToFileAsync();
        }

        /// <summary>
        /// Track clicked agent in the image view
        /// </summary>
        /// <param name="e"></param>
        private void TrackClickedBot(MouseEventArgs e)
        {
            if (numBots > 0)
            {
                //cosphi
                if (buttonDetectCosPhi.Enabled == true || separatedWindow)
                {
                    currentCosphiArray[numBots - 1].x = e.X;
                    currentCosphiArray[numBots - 1].y = e.Y;
                    currentCosphiArray[numBots - 1].valid = true;
                    //cosphiArray[numBots - 1].localSearch = true;
                    cosphiArray[numBots - 1].identify = true;
                    cosphiArray[numBots - 1].idIdentificationCount = 0;
                }
                //whyconID
                else
                {
                    currentSegmentArray[numBots - 1].x = e.X;
                    currentSegmentArray[numBots - 1].y = e.Y;
                    currentSegmentArray[numBots - 1].valid = true;
                    //detectorArray[numBots - 1].localSearch = true;
                    detectorArray[numBots - 1].identify = true;
                    detectorArray[numBots - 1].idIdentificationCount = 0;
                }
            }
        }

        /// <summary>
        /// Asynchronous task to track clicked agent
        /// </summary>
        /// <param name="e"></param>
        /// <returns></returns>
        private Task TrackClickedBotAsync(MouseEventArgs e)
        {
            return Task.Factory.StartNew(() => TrackClickedBot(e));
        }

        /// <summary>
        /// Call method of tracking clicked agent
        /// </summary>
        /// <param name="e"></param>
        private async void CallTrackClickedBot(MouseEventArgs e)
        {
            await TrackClickedBotAsync(e);
        }

        /// <summary>
        /// Process on-line CosPhi Pattern Tracking
        /// </summary>
        private void ProcessOnlineCosPhi()
        {
            //font to show id
            MCvFont font = new MCvFont();
            font.hscale = 0.6f; font.vscale = 0.6f; font.thickness = 2; //User can change font properties here.
            //text file name
            string textFileName = fileDirectoryPath + cosphiTxtFileName;

            using (TextWriter fqb = File.CreateText(textFileName))
            {
                long iter = 100000000;  //iterations of track, set to large number
                for (int t = 0; t < iter; t++)
                {
                    //if updating camera setting
                    if (form2.updateSetting == true)
                    {
                        SetCameraProperty();
                        form2.updateSetting = false;
                    }
                    //if monitoring elaped time
                    var stopwatch = new Stopwatch();
                    if (elapsed)
                    {
                        stopwatch.Start();
                    }

                    //main localization system
                    //output IDs, Xs, Ys to console window
                    //output IDs, Xs, Ys, angles to text file
                    using (Image<Bgr, Byte> rawFrame = cap.QueryFrame())
                    {
                        if (rawFrame != null)
                        {
                            using (Image<Bgr, Byte> cloneFrame = rawFrame.Clone())
                            {
                                phiFound = phiStatic = 0;
                                //update global image data
                                CosPhiGlobalData.globalImageData = cloneFrame.Data;

                                //start tracking
                                //track the robots found in the last attemp
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentCosphiArray[i].valid)
                                    {
                                        lastCosphiArray[i] = currentCosphiArray[i];
                                        currentCosphiArray[i] = cosphiArray[i].findSegment(lastCosphiArray[i],3);
                                        Console.WriteLine("CosPhi -> {0}, ID -> {1}", i, currentCosphiArray[i].ID);
                                    }
                                }

                                //search for untracked robots (not detected in the last frame)
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentCosphiArray[i].valid == false)
                                    {
                                        lastCosphiArray[i].valid = false;
                                        currentCosphiArray[i] = cosphiArray[i].findSegment(lastCosphiArray[i],3);
                                    }
                                    //do not make sense to search for more patterns if the last one was not found
                                    if (currentCosphiArray[i].valid == false)
                                        break;
                                }

                                //perform transformations from camera to world coordinates
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentCosphiArray[i].valid)
                                    {
                                        //objectArray[i] = trans.transform(currentSegmentArray[i], false);
                                        phiFound++;

                                        //if draw points
                                        if (drawPoints)
                                            cosphiPoints[i].Add(new Point((int)currentCosphiArray[i].x,(int)currentCosphiArray[i].y));

                                        //Console.WriteLine("Agent {0} {1:F} {2:F} {3:F}", currentCosphiArray[i].ID, currentCosphiArray[i].x, currentCosphiArray[i].y, currentCosphiArray[i].angle);
                                        if (Math.Abs(currentCosphiArray[i].x - lastCosphiArray[i].x) < static_thre) phiStatic++;
                                    }
                                }
                                Console.WriteLine("CosPhi detection: Found -> {0} Static -> {1}", phiFound, phiStatic);

                                cloneFrame.Data = CosPhiGlobalData.globalImageData;

                                //draw ID number with marker on the image, and write results to text
                                for (int i = 0; i < numBots; i++)
                                {
                                    if (currentCosphiArray[i].valid)
                                    {
                                        cloneFrame.Draw(currentCosphiArray[i].ID.ToString(), ref font, new Point((int)currentCosphiArray[i].x, (int)currentCosphiArray[i].y), new Bgr(Color.Red));
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", t, currentCosphiArray[i].ID, (int)currentCosphiArray[i].x, (int)currentCosphiArray[i].y, currentCosphiArray[i].angle);
                                    }
                                    else
                                        fqb.WriteLine("{0} {1} {2} {3} {4:F}", t, lastCosphiArray[i].ID, (int)lastCosphiArray[i].x, (int)lastCosphiArray[i].y, lastCosphiArray[i].angle);
                                }

                                //if drawing points
                                if (drawPoints)
                                {
                                    for (int i = 0; i < numBots; i++)
                                        cloneFrame.DrawPolyline(cosphiPoints[i].ToArray(), false, new Bgr(0, 255 / currentCosphiArray[i].ID, 255 / currentCosphiArray[i].ID), 1);
                                }

                                //display
                                if (separatedWindow)
                                    form4.cosPhiImageBox.Image = cloneFrame;
                                else
                                    mainImageBox.Image = cloneFrame;

                                //if save image
                                if (saveImg)
                                {
                                    if (textBoxSaveImage.Text == "")
                                        mainImageBox.Image.Save(fileDirectoryPath + globalCounter.ToString() + ".png");
                                    else
                                        mainImageBox.Image.Save(fileDirectoryPath + textBoxSaveImage.Text + ".png");
                                    saveImg = false;
                                }

                                //if monitoring elpased time in tracking
                                if (elapsed)
                                {
                                    stopwatch.Stop();
                                    elapsed_time = (int)stopwatch.ElapsedMilliseconds;
                                    Console.WriteLine("Elapsed time: {0}\n", elapsed_time);
                                }

                                Thread.Sleep(10);
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Asynchronous task to process CosPhi in real time
        /// </summary>
        /// <returns></returns>
        private Task ProcessOnlineCosPhiAsync()
        {
            return Task.Factory.StartNew(() => ProcessOnlineCosPhi());
        }

        /// <summary>
        /// Call method of processing on-line CosPhi tracking
        /// </summary>
        private async void CallProcessOnlineCosPhi()
        {
            await ProcessOnlineCosPhiAsync();
        }

        private void SaveImage()
        {
            saveImg = true;
        }

        private Task SaveImageAsync()
        {
            return Task.Factory.StartNew(() => SaveImage());
        }

        private async void CallSaveImage()
        {
            await SaveImageAsync();
        }

        #endregion

        #region MAIN WINFORM COMPONENTS

        /// <summary>
        /// Camera Raw Image Capture Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonCpt_Click(object sender, EventArgs e)
        {
            try
            {
                //cap.ImageGrabbed += ProcessRawFrame;
                //Application.Idle += new EventHandler(ProcessRawFrame);
                //cap_tick.Start();
                //cap_tick.Elapsed += new ElapsedEventHandler(ProcessRawFrame);
                form3.Show();
                CallProcessRawFrame();
            }
            catch (Exception exception)
            {
                MessageBox.Show(exception.ToString());
            }
        }

        /// <summary>
        /// Save Capture Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonSaveImage_Click(object sender, EventArgs e)
        {
            try
            {
                CallSaveImage();
                MessageBox.Show("Image View Saved");
            }
            catch (Exception exception)
            {
                MessageBox.Show(exception.ToString());
            }
        }

        /// <summary>
        /// Timer Function
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void timer1_Tick(object sender, EventArgs e)
        {
            globalCounter++;                                                                        //global counter
            //CalculateFrameRate();
            toolStripStatusLabelTime.Text = "Date time: " + DateTime.Now.ToShortTimeString();                  //update time display
            toolStripStatusLabelFrameWidth.Text = "Width: " + frame_width.ToString();                      //update frame width display
            toolStripStatusLabelFrameHeight.Text = "Height: " + frame_height.ToString();                    //update frame height display
            toolStripStatusLabelFound.Text = "number found: " + found.ToString();
            //this.toolStripStatusLabelFps.Text = fps.ToString();                                   
            //this.toolStripStatusLabelFps.Text = frameRate;                                          //update fps display
            //Thread.Sleep(50);
            form2.UpdateScrollBar();
        }

        /// <summary>
        /// Exit Window Menu Item
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                this.Close();
            }
            catch (Exception exception)
            {
                MessageBox.Show(exception.ToString());
            }
        }

        /// <summary>
        /// Show Device Property Window
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void propertyToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                form2.Show();
            }
            catch (Exception exc)
            {
                MessageBox.Show(exc.ToString());
            }
        }

        /// <summary>
        /// Application Restart
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void restartToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Application.Restart();
        }

        /// <summary>
        /// Real-time Track Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonDetect_Click(object sender, EventArgs e)
        {
            //MessageBox.Show("fun");
            try
            {
                if (debug == 1)
                    ProcessOnlineTrack(sender, e);
                else
                {
                    //Parallel.For(0,1000000,x=>ProcessOnlineTrackParallel());
                    //Application.Idle += new EventHandler(ProcessOnlineTrack);
                    CallProcessOnlineTrack();
                }
            }
            catch (Exception exception)
            {
                MessageBox.Show(exception.ToString());
                if (exception is AccessViolationException)
                    return;
                throw;
            }
        }

        /// <summary>
        /// Video Track Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonOffline_Click(object sender, EventArgs e)
        {
            try
            {
                //form5.Show();
                CallProcessOfflineTrack();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Init Tracking System Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonTrackSystemInit_Click(object sender, EventArgs e)
        {
            try
            {
                var stopwatch = new Stopwatch();
                if (elapsed)
                {
                    stopwatch.Start();
                }
                //Tracking system
                this.circleDiameter = 0.122f;
                /*----------------------------------------------*/
                numBots = int.Parse(textBoxRobotNO.Text);
                if (numBots > MAX_PATTERNS)
                {
                    numBots = MAX_PATTERNS;
                    MessageBox.Show("number of tracking bots should not exceed pre-defined max......");
                }
                this.numFound = 0;
                this.numStatic = 0;
                this.found = 0;
                this.wasBots = 0;
                this.static_thre = 1.0f;
                /*----------------------------------------------*/
                this.detectorArray = new CCircleDetect[MAX_PATTERNS];
                this.currentSegmentArray = new SSegment[MAX_PATTERNS];
                this.lastSegmentArray = new SSegment[MAX_PATTERNS];
                this.objectArray = new STrackedObject[MAX_PATTERNS];

                //this.calibNum = 5;
                //this.calib = new STrackedObject[5];
                //calibTmp = new STrackedObject[calibrationSteps];
                //this.calibStep = calibrationSteps + 2;
                //this.autocalibrate = true;
                //this.lastTransformType = ETransformType.TRANSFORM_2D;
                //this.wasBots = 1;

                //saveVideo = true;
                //saveLog = true;
                //stop = false;
                //this.moveOne = 1;
                //this.moveVal = 1;

                //this.trans = new CTransformation(frame_width, frame_height, circleDiameter, true);
                //trans.transformType = ETransformType.TRANSFORM_2D;
                //this.trans.transformType = ETransformType.TRANSFORM_NONE;
                for (int i = 0; i < MAX_PATTERNS; i++)
                {
                    detectorArray[i] = new CCircleDetect(frame_width,frame_height,ID_BITS,i);
                }

                //buttons
                buttonDetect.Enabled = true;
                buttonAddRobot.Enabled = true;
                buttonTrackSystemInit.Enabled = false;
                buttonRecord.Enabled = true;
                saveRecordButton.Enabled = true;
                buttonSubRobot.Enabled = true;
                buttonCosPhiTrackInit.Enabled = true;

                //textbox
                textBoxRobotNO.ReadOnly = true;

                //global variables
                if (GlobalData.isgray)
                    GlobalData.globalImageData = new byte[frame_height, frame_width, 1];
                else
                    GlobalData.globalImageData = new byte[frame_width, frame_height, 3];
                GlobalData.buffer = new int[frame_height * frame_width];
                GlobalData.queue = new int[frame_height * frame_width];
                //GlobalData.mask = new int[frame_height*frame_width];

                //elapsed time
                if (elapsed)
                {
                    stopwatch.Stop();
                    elapsed_time = (int)stopwatch.ElapsedMilliseconds;
                    Console.WriteLine("System Ready in " + elapsed_time + " ms");
                }
            }
            catch (Exception exception)
            {
                MessageBox.Show(exception.ToString());
            }
        }

        /// <summary>
        /// Save Recording Video Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void saveRecordButton_Click(object sender, EventArgs e)
        {
            try
            {
                CallSaveRecordingToFile();
                Console.WriteLine("#Recording saved......#");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Button of recording raw frame
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonRecord_Click(object sender, EventArgs e)
        {
            try
            {
                CallRecordRawFrame();
                Console.WriteLine("#Start to record......#");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Add bot Buttion
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonAddRobot_Click(object sender, EventArgs e)
        {
            if (numBots == MAX_PATTERNS)
            {
                numBots = MAX_PATTERNS;
                //buttonAddRobot.Enabled = false;
                MessageBox.Show("agents number should not exceed pre-defined max...");
            }
            else
            {
                numBots = numBots + 1; //can change added number here.
            }
        }

        /// <summary>
        /// Subtract tracking agent Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonSubRobot_Click(object sender, EventArgs e)
        {
            if (numBots < 0)
            {
                numBots = 0;
                //buttonSubRobot.Enabled = false;
                MessageBox.Show("agents number reaches minimum...");
            }
            else
            {
                numBots = numBots - 1; //can change subtracted number here.
            }
        }

        /// <summary>
        /// Mouse Click Event: track the clicked bot separately in parallel
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void mainImageBox_MouseClick(object sender, MouseEventArgs e)
        {
            try
            {
                if (debug != 1)
                    CallTrackClickedBot(e);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Init CosPhi Marker on-line tracking Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonCosPhiTrackInit_Click(object sender, EventArgs e)
        {
            try
            {
                //Tracking system
                this.phiFound = 0;
                this.phiStatic = 0;
                /*----------------------------------------------*/
                this.cosphiArray = new CosPhi[MAX_PATTERNS];
                this.currentCosphiArray = new CosSegment[MAX_PATTERNS];
                this.lastCosphiArray = new CosSegment[MAX_PATTERNS];
                this.cosphiPoints = new List<Point>[MAX_PATTERNS];

                for (int i = 0; i < MAX_PATTERNS; i++)
                {
                    cosphiArray[i] = new CosPhi(frame_width, frame_height, MAX_PATTERNS, i);
                }

                //botton enable
                buttonDetectCosPhi.Enabled = true;
                //buttonRunOfflineCosPhi.Enabled = true;

                //global variables
                CosPhiGlobalData.globalImageData = new byte[frame_width, frame_height, 3];
                CosPhiGlobalData.buffer = new int[frame_height * frame_width];
                CosPhiGlobalData.queue = new int[frame_height * frame_width];
                //CosPhiGlobalData.mask = new int[frame_height*frame_width];
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Detect CosPhi Marker Button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonDetectCosPhi_Click(object sender, EventArgs e)
        {
            try
            {
                if (separatedWindow)
                    form4.Show();
                CallProcessOnlineCosPhi();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Nothing to do currently
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonSaveTrackingResults_Click(object sender, EventArgs e)
        {
            try
            {
                //nothing
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Run Offline Cosphi Detection
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonRunOfflineCosPhi_Click(object sender, EventArgs e)
        {
            try
            {
                if (separatedWindow)
                    form5.Show();
                CallProcessCosPhiOff();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }    
        }

        /// <summary>
        /// Key press functions
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_KeyPress(object sender, KeyPressEventArgs e)
        { }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        { }

        #endregion

    }
}
