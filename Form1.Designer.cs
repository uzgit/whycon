namespace WhyConID
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.mainImageBox = new Emgu.CV.UI.ImageBox();
            this.buttonCpt = new System.Windows.Forms.Button();
            this.buttonDetect = new System.Windows.Forms.Button();
            this.buttonSaveImage = new System.Windows.Forms.Button();
            this.textBoxSaveImage = new System.Windows.Forms.TextBox();
            this.labelImageFileName = new System.Windows.Forms.Label();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.menuStripDevice = new System.Windows.Forms.MenuStrip();
            this.deviceToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.propertyToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.exitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.restartToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.statusStrip1 = new System.Windows.Forms.StatusStrip();
            this.toolStripStatusLabelFrameWidth = new System.Windows.Forms.ToolStripStatusLabel();
            this.toolStripStatusLabelFrameHeight = new System.Windows.Forms.ToolStripStatusLabel();
            this.toolStripStatusLabelFps = new System.Windows.Forms.ToolStripStatusLabel();
            this.toolStripStatusLabelTime = new System.Windows.Forms.ToolStripStatusLabel();
            this.toolStripStatusLabelFound = new System.Windows.Forms.ToolStripStatusLabel();
            this.textBoxRobotNO = new System.Windows.Forms.TextBox();
            this.labelRobotNO = new System.Windows.Forms.Label();
            this.buttonAddRobot = new System.Windows.Forms.Button();
            this.buttonOffline = new System.Windows.Forms.Button();
            this.labelVideoName = new System.Windows.Forms.Label();
            this.buttonTrackSystemInit = new System.Windows.Forms.Button();
            this.saveRecordButton = new System.Windows.Forms.Button();
            this.buttonRecord = new System.Windows.Forms.Button();
            this.buttonSubRobot = new System.Windows.Forms.Button();
            this.textBoxOfflineVideo = new System.Windows.Forms.TextBox();
            this.textBoxOffNO = new System.Windows.Forms.TextBox();
            this.labelOffNO = new System.Windows.Forms.Label();
            this.buttonCosPhiTrackInit = new System.Windows.Forms.Button();
            this.buttonDetectCosPhi = new System.Windows.Forms.Button();
            this.buttonSaveTrackingResults = new System.Windows.Forms.Button();
            this.buttonRunOfflineCosPhi = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.mainImageBox)).BeginInit();
            this.menuStripDevice.SuspendLayout();
            this.statusStrip1.SuspendLayout();
            this.SuspendLayout();
            // 
            // mainImageBox
            // 
            this.mainImageBox.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(192)))), ((int)(((byte)(255)))), ((int)(((byte)(192)))));
            this.mainImageBox.FunctionalMode = Emgu.CV.UI.ImageBox.FunctionalModeOption.Minimum;
            this.mainImageBox.Location = new System.Drawing.Point(0, 27);
            this.mainImageBox.Name = "mainImageBox";
            this.mainImageBox.Size = new System.Drawing.Size(1280, 1024);
            this.mainImageBox.TabIndex = 2;
            this.mainImageBox.TabStop = false;
            this.mainImageBox.MouseClick += new System.Windows.Forms.MouseEventHandler(this.mainImageBox_MouseClick);
            // 
            // buttonCpt
            // 
            this.buttonCpt.BackColor = System.Drawing.Color.Red;
            this.buttonCpt.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonCpt.Location = new System.Drawing.Point(1290, 27);
            this.buttonCpt.Name = "buttonCpt";
            this.buttonCpt.Size = new System.Drawing.Size(83, 59);
            this.buttonCpt.TabIndex = 3;
            this.buttonCpt.Text = "SHOW RAW FRAME";
            this.buttonCpt.UseVisualStyleBackColor = false;
            this.buttonCpt.Click += new System.EventHandler(this.buttonCpt_Click);
            // 
            // buttonDetect
            // 
            this.buttonDetect.BackColor = System.Drawing.Color.Blue;
            this.buttonDetect.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonDetect.ForeColor = System.Drawing.Color.Red;
            this.buttonDetect.Location = new System.Drawing.Point(1291, 222);
            this.buttonDetect.Name = "buttonDetect";
            this.buttonDetect.Size = new System.Drawing.Size(83, 78);
            this.buttonDetect.TabIndex = 9;
            this.buttonDetect.Text = "RUN REALTIME";
            this.buttonDetect.UseVisualStyleBackColor = false;
            this.buttonDetect.Click += new System.EventHandler(this.buttonDetect_Click);
            // 
            // buttonSaveImage
            // 
            this.buttonSaveImage.BackColor = System.Drawing.Color.Snow;
            this.buttonSaveImage.Font = new System.Drawing.Font("Algerian", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonSaveImage.Location = new System.Drawing.Point(1290, 347);
            this.buttonSaveImage.Name = "buttonSaveImage";
            this.buttonSaveImage.Size = new System.Drawing.Size(83, 45);
            this.buttonSaveImage.TabIndex = 12;
            this.buttonSaveImage.Text = "SAVE IMG";
            this.buttonSaveImage.UseVisualStyleBackColor = false;
            this.buttonSaveImage.Click += new System.EventHandler(this.buttonSaveImage_Click);
            // 
            // textBoxSaveImage
            // 
            this.textBoxSaveImage.Font = new System.Drawing.Font("Arial Narrow", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxSaveImage.Location = new System.Drawing.Point(1291, 416);
            this.textBoxSaveImage.Name = "textBoxSaveImage";
            this.textBoxSaveImage.Size = new System.Drawing.Size(83, 26);
            this.textBoxSaveImage.TabIndex = 13;
            // 
            // labelImageFileName
            // 
            this.labelImageFileName.AutoSize = true;
            this.labelImageFileName.Font = new System.Drawing.Font("Algerian", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelImageFileName.Location = new System.Drawing.Point(1288, 395);
            this.labelImageFileName.Name = "labelImageFileName";
            this.labelImageFileName.Size = new System.Drawing.Size(36, 18);
            this.labelImageFileName.TabIndex = 14;
            this.labelImageFileName.Text = "Img";
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // menuStripDevice
            // 
            this.menuStripDevice.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.deviceToolStripMenuItem,
            this.exitToolStripMenuItem,
            this.restartToolStripMenuItem});
            this.menuStripDevice.Location = new System.Drawing.Point(0, 0);
            this.menuStripDevice.Name = "menuStripDevice";
            this.menuStripDevice.Size = new System.Drawing.Size(1580, 26);
            this.menuStripDevice.TabIndex = 15;
            // 
            // deviceToolStripMenuItem
            // 
            this.deviceToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.propertyToolStripMenuItem});
            this.deviceToolStripMenuItem.Font = new System.Drawing.Font("Algerian", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.deviceToolStripMenuItem.Name = "deviceToolStripMenuItem";
            this.deviceToolStripMenuItem.Size = new System.Drawing.Size(74, 22);
            this.deviceToolStripMenuItem.Text = "Device";
            // 
            // propertyToolStripMenuItem
            // 
            this.propertyToolStripMenuItem.Name = "propertyToolStripMenuItem";
            this.propertyToolStripMenuItem.Size = new System.Drawing.Size(153, 22);
            this.propertyToolStripMenuItem.Text = "Property";
            this.propertyToolStripMenuItem.Click += new System.EventHandler(this.propertyToolStripMenuItem_Click);
            // 
            // exitToolStripMenuItem
            // 
            this.exitToolStripMenuItem.Font = new System.Drawing.Font("Algerian", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.exitToolStripMenuItem.Name = "exitToolStripMenuItem";
            this.exitToolStripMenuItem.Size = new System.Drawing.Size(55, 22);
            this.exitToolStripMenuItem.Text = "Exit";
            this.exitToolStripMenuItem.Click += new System.EventHandler(this.exitToolStripMenuItem_Click);
            // 
            // restartToolStripMenuItem
            // 
            this.restartToolStripMenuItem.Font = new System.Drawing.Font("Algerian", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.restartToolStripMenuItem.Name = "restartToolStripMenuItem";
            this.restartToolStripMenuItem.Size = new System.Drawing.Size(89, 22);
            this.restartToolStripMenuItem.Text = "RESTART";
            this.restartToolStripMenuItem.Click += new System.EventHandler(this.restartToolStripMenuItem_Click);
            // 
            // statusStrip1
            // 
            this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripStatusLabelFrameWidth,
            this.toolStripStatusLabelFrameHeight,
            this.toolStripStatusLabelFps,
            this.toolStripStatusLabelTime,
            this.toolStripStatusLabelFound});
            this.statusStrip1.Location = new System.Drawing.Point(0, 851);
            this.statusStrip1.Name = "statusStrip1";
            this.statusStrip1.Size = new System.Drawing.Size(1580, 22);
            this.statusStrip1.TabIndex = 16;
            this.statusStrip1.Text = "statusStrip1";
            // 
            // toolStripStatusLabelFrameWidth
            // 
            this.toolStripStatusLabelFrameWidth.Name = "toolStripStatusLabelFrameWidth";
            this.toolStripStatusLabelFrameWidth.Size = new System.Drawing.Size(15, 17);
            this.toolStripStatusLabelFrameWidth.Text = "0";
            // 
            // toolStripStatusLabelFrameHeight
            // 
            this.toolStripStatusLabelFrameHeight.Name = "toolStripStatusLabelFrameHeight";
            this.toolStripStatusLabelFrameHeight.Size = new System.Drawing.Size(15, 17);
            this.toolStripStatusLabelFrameHeight.Text = "0";
            // 
            // toolStripStatusLabelFps
            // 
            this.toolStripStatusLabelFps.Name = "toolStripStatusLabelFps";
            this.toolStripStatusLabelFps.Size = new System.Drawing.Size(39, 17);
            this.toolStripStatusLabelFps.Text = "FPS 0";
            // 
            // toolStripStatusLabelTime
            // 
            this.toolStripStatusLabelTime.Name = "toolStripStatusLabelTime";
            this.toolStripStatusLabelTime.Size = new System.Drawing.Size(39, 17);
            this.toolStripStatusLabelTime.Text = "00:00";
            // 
            // toolStripStatusLabelFound
            // 
            this.toolStripStatusLabelFound.Name = "toolStripStatusLabelFound";
            this.toolStripStatusLabelFound.Size = new System.Drawing.Size(105, 17);
            this.toolStripStatusLabelFound.Text = "number found: 0";
            // 
            // textBoxRobotNO
            // 
            this.textBoxRobotNO.Font = new System.Drawing.Font("Algerian", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxRobotNO.Location = new System.Drawing.Point(1290, 122);
            this.textBoxRobotNO.Name = "textBoxRobotNO";
            this.textBoxRobotNO.Size = new System.Drawing.Size(83, 29);
            this.textBoxRobotNO.TabIndex = 17;
            this.textBoxRobotNO.Text = "1";
            this.textBoxRobotNO.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // labelRobotNO
            // 
            this.labelRobotNO.AutoSize = true;
            this.labelRobotNO.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelRobotNO.Location = new System.Drawing.Point(1286, 104);
            this.labelRobotNO.Name = "labelRobotNO";
            this.labelRobotNO.Size = new System.Drawing.Size(76, 15);
            this.labelRobotNO.TabIndex = 18;
            this.labelRobotNO.Text = "Robot NO.";
            // 
            // buttonAddRobot
            // 
            this.buttonAddRobot.BackColor = System.Drawing.Color.Lime;
            this.buttonAddRobot.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonAddRobot.Location = new System.Drawing.Point(1398, 395);
            this.buttonAddRobot.Name = "buttonAddRobot";
            this.buttonAddRobot.Size = new System.Drawing.Size(83, 59);
            this.buttonAddRobot.TabIndex = 19;
            this.buttonAddRobot.Text = "Add Bots";
            this.buttonAddRobot.UseVisualStyleBackColor = false;
            this.buttonAddRobot.Click += new System.EventHandler(this.buttonAddRobot_Click);
            // 
            // buttonOffline
            // 
            this.buttonOffline.BackColor = System.Drawing.Color.Snow;
            this.buttonOffline.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonOffline.Location = new System.Drawing.Point(1398, 545);
            this.buttonOffline.Name = "buttonOffline";
            this.buttonOffline.Size = new System.Drawing.Size(83, 71);
            this.buttonOffline.TabIndex = 22;
            this.buttonOffline.Text = "RUN whycon OFFLINE";
            this.buttonOffline.UseVisualStyleBackColor = false;
            this.buttonOffline.Click += new System.EventHandler(this.buttonOffline_Click);
            // 
            // labelVideoName
            // 
            this.labelVideoName.AutoSize = true;
            this.labelVideoName.Font = new System.Drawing.Font("Algerian", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelVideoName.Location = new System.Drawing.Point(1288, 519);
            this.labelVideoName.Name = "labelVideoName";
            this.labelVideoName.Size = new System.Drawing.Size(53, 18);
            this.labelVideoName.TabIndex = 24;
            this.labelVideoName.Text = "Video";
            // 
            // buttonTrackSystemInit
            // 
            this.buttonTrackSystemInit.BackColor = System.Drawing.Color.Aqua;
            this.buttonTrackSystemInit.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonTrackSystemInit.Location = new System.Drawing.Point(1291, 157);
            this.buttonTrackSystemInit.Name = "buttonTrackSystemInit";
            this.buttonTrackSystemInit.Size = new System.Drawing.Size(83, 59);
            this.buttonTrackSystemInit.TabIndex = 25;
            this.buttonTrackSystemInit.Text = "1.INIT ONLINE";
            this.buttonTrackSystemInit.UseVisualStyleBackColor = false;
            this.buttonTrackSystemInit.Click += new System.EventHandler(this.buttonTrackSystemInit_Click);
            // 
            // saveRecordButton
            // 
            this.saveRecordButton.BackColor = System.Drawing.Color.Yellow;
            this.saveRecordButton.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.saveRecordButton.Location = new System.Drawing.Point(1399, 691);
            this.saveRecordButton.Name = "saveRecordButton";
            this.saveRecordButton.Size = new System.Drawing.Size(83, 57);
            this.saveRecordButton.TabIndex = 26;
            this.saveRecordButton.Text = "2. Save record";
            this.saveRecordButton.UseVisualStyleBackColor = false;
            this.saveRecordButton.Click += new System.EventHandler(this.saveRecordButton_Click);
            // 
            // buttonRecord
            // 
            this.buttonRecord.BackColor = System.Drawing.Color.Yellow;
            this.buttonRecord.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonRecord.Location = new System.Drawing.Point(1292, 691);
            this.buttonRecord.Name = "buttonRecord";
            this.buttonRecord.Size = new System.Drawing.Size(83, 59);
            this.buttonRecord.TabIndex = 27;
            this.buttonRecord.Text = "1. Record Raw";
            this.buttonRecord.UseVisualStyleBackColor = false;
            this.buttonRecord.Click += new System.EventHandler(this.buttonRecord_Click);
            // 
            // buttonSubRobot
            // 
            this.buttonSubRobot.BackColor = System.Drawing.Color.Lime;
            this.buttonSubRobot.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonSubRobot.Location = new System.Drawing.Point(1399, 460);
            this.buttonSubRobot.Name = "buttonSubRobot";
            this.buttonSubRobot.Size = new System.Drawing.Size(83, 59);
            this.buttonSubRobot.TabIndex = 28;
            this.buttonSubRobot.Text = "Sub Bots";
            this.buttonSubRobot.UseVisualStyleBackColor = false;
            this.buttonSubRobot.Click += new System.EventHandler(this.buttonSubRobot_Click);
            // 
            // textBoxOfflineVideo
            // 
            this.textBoxOfflineVideo.Font = new System.Drawing.Font("Arial Narrow", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxOfflineVideo.Location = new System.Drawing.Point(1292, 540);
            this.textBoxOfflineVideo.Name = "textBoxOfflineVideo";
            this.textBoxOfflineVideo.Size = new System.Drawing.Size(83, 26);
            this.textBoxOfflineVideo.TabIndex = 23;
            this.textBoxOfflineVideo.Text = "fqb.avi";
            // 
            // textBoxOffNO
            // 
            this.textBoxOffNO.Font = new System.Drawing.Font("Algerian", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxOffNO.Location = new System.Drawing.Point(1291, 587);
            this.textBoxOffNO.Name = "textBoxOffNO";
            this.textBoxOffNO.Size = new System.Drawing.Size(83, 29);
            this.textBoxOffNO.TabIndex = 29;
            this.textBoxOffNO.Text = "1";
            this.textBoxOffNO.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // labelOffNO
            // 
            this.labelOffNO.AutoSize = true;
            this.labelOffNO.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelOffNO.Location = new System.Drawing.Point(1288, 569);
            this.labelOffNO.Name = "labelOffNO";
            this.labelOffNO.Size = new System.Drawing.Size(86, 15);
            this.labelOffNO.TabIndex = 30;
            this.labelOffNO.Text = "Offline NO.";
            // 
            // buttonCosPhiTrackInit
            // 
            this.buttonCosPhiTrackInit.BackColor = System.Drawing.Color.Fuchsia;
            this.buttonCosPhiTrackInit.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonCosPhiTrackInit.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(0)))), ((int)(((byte)(0)))), ((int)(((byte)(64)))));
            this.buttonCosPhiTrackInit.Location = new System.Drawing.Point(1398, 157);
            this.buttonCosPhiTrackInit.Name = "buttonCosPhiTrackInit";
            this.buttonCosPhiTrackInit.Size = new System.Drawing.Size(83, 59);
            this.buttonCosPhiTrackInit.TabIndex = 32;
            this.buttonCosPhiTrackInit.Text = "2.INIT CosPhi";
            this.buttonCosPhiTrackInit.UseVisualStyleBackColor = false;
            this.buttonCosPhiTrackInit.Click += new System.EventHandler(this.buttonCosPhiTrackInit_Click);
            // 
            // buttonDetectCosPhi
            // 
            this.buttonDetectCosPhi.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(0)))), ((int)(((byte)(64)))));
            this.buttonDetectCosPhi.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonDetectCosPhi.ForeColor = System.Drawing.Color.Lime;
            this.buttonDetectCosPhi.Location = new System.Drawing.Point(1398, 222);
            this.buttonDetectCosPhi.Name = "buttonDetectCosPhi";
            this.buttonDetectCosPhi.Size = new System.Drawing.Size(83, 78);
            this.buttonDetectCosPhi.TabIndex = 31;
            this.buttonDetectCosPhi.Text = "RUN CosPhi On-line";
            this.buttonDetectCosPhi.UseVisualStyleBackColor = false;
            this.buttonDetectCosPhi.Click += new System.EventHandler(this.buttonDetectCosPhi_Click);
            // 
            // buttonSaveTrackingResults
            // 
            this.buttonSaveTrackingResults.BackColor = System.Drawing.Color.Blue;
            this.buttonSaveTrackingResults.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonSaveTrackingResults.ForeColor = System.Drawing.Color.Red;
            this.buttonSaveTrackingResults.Location = new System.Drawing.Point(1292, 880);
            this.buttonSaveTrackingResults.Name = "buttonSaveTrackingResults";
            this.buttonSaveTrackingResults.Size = new System.Drawing.Size(83, 57);
            this.buttonSaveTrackingResults.TabIndex = 33;
            this.buttonSaveTrackingResults.Text = "save tracking";
            this.buttonSaveTrackingResults.UseVisualStyleBackColor = false;
            this.buttonSaveTrackingResults.Click += new System.EventHandler(this.buttonSaveTrackingResults_Click);
            // 
            // buttonRunOfflineCosPhi
            // 
            this.buttonRunOfflineCosPhi.BackColor = System.Drawing.Color.Black;
            this.buttonRunOfflineCosPhi.Font = new System.Drawing.Font("Algerian", 10.5F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonRunOfflineCosPhi.ForeColor = System.Drawing.Color.Lime;
            this.buttonRunOfflineCosPhi.Location = new System.Drawing.Point(1487, 545);
            this.buttonRunOfflineCosPhi.Name = "buttonRunOfflineCosPhi";
            this.buttonRunOfflineCosPhi.Size = new System.Drawing.Size(83, 71);
            this.buttonRunOfflineCosPhi.TabIndex = 34;
            this.buttonRunOfflineCosPhi.Text = "RUN cosphi OFFLINE";
            this.buttonRunOfflineCosPhi.UseVisualStyleBackColor = false;
            this.buttonRunOfflineCosPhi.Click += new System.EventHandler(this.buttonRunOfflineCosPhi_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1580, 873);
            this.Controls.Add(this.buttonRunOfflineCosPhi);
            this.Controls.Add(this.buttonSaveTrackingResults);
            this.Controls.Add(this.buttonCosPhiTrackInit);
            this.Controls.Add(this.buttonDetectCosPhi);
            this.Controls.Add(this.labelOffNO);
            this.Controls.Add(this.textBoxOffNO);
            this.Controls.Add(this.buttonSubRobot);
            this.Controls.Add(this.buttonRecord);
            this.Controls.Add(this.saveRecordButton);
            this.Controls.Add(this.buttonTrackSystemInit);
            this.Controls.Add(this.labelVideoName);
            this.Controls.Add(this.textBoxOfflineVideo);
            this.Controls.Add(this.buttonOffline);
            this.Controls.Add(this.buttonAddRobot);
            this.Controls.Add(this.labelRobotNO);
            this.Controls.Add(this.textBoxRobotNO);
            this.Controls.Add(this.statusStrip1);
            this.Controls.Add(this.labelImageFileName);
            this.Controls.Add(this.textBoxSaveImage);
            this.Controls.Add(this.buttonSaveImage);
            this.Controls.Add(this.buttonDetect);
            this.Controls.Add(this.buttonCpt);
            this.Controls.Add(this.mainImageBox);
            this.Controls.Add(this.menuStripDevice);
            this.MainMenuStrip = this.menuStripDevice;
            this.Name = "Form1";
            this.Text = "LIVE";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.KeyDown += new System.Windows.Forms.KeyEventHandler(this.Form1_KeyDown);
            this.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.Form1_KeyPress);
            ((System.ComponentModel.ISupportInitialize)(this.mainImageBox)).EndInit();
            this.menuStripDevice.ResumeLayout(false);
            this.menuStripDevice.PerformLayout();
            this.statusStrip1.ResumeLayout(false);
            this.statusStrip1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        public Emgu.CV.UI.ImageBox mainImageBox;
        private System.Windows.Forms.Button buttonCpt;
        private System.Windows.Forms.Button buttonDetect;
        private System.Windows.Forms.Button buttonSaveImage;
        private System.Windows.Forms.TextBox textBoxSaveImage;
        private System.Windows.Forms.Label labelImageFileName;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.MenuStrip menuStripDevice;
        private System.Windows.Forms.ToolStripMenuItem exitToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem deviceToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem propertyToolStripMenuItem;
        private System.Windows.Forms.StatusStrip statusStrip1;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabelFrameWidth;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabelFrameHeight;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabelFps;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabelTime;
        private System.Windows.Forms.ToolStripMenuItem restartToolStripMenuItem;
        private System.Windows.Forms.TextBox textBoxRobotNO;
        private System.Windows.Forms.Label labelRobotNO;
        private System.Windows.Forms.Button buttonAddRobot;
        private System.Windows.Forms.Button buttonOffline;
        private System.Windows.Forms.Label labelVideoName;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabelFound;
        private System.Windows.Forms.Button buttonTrackSystemInit;
        private System.Windows.Forms.Button saveRecordButton;
        private System.Windows.Forms.Button buttonRecord;
        private System.Windows.Forms.Button buttonSubRobot;
        private System.Windows.Forms.TextBox textBoxOfflineVideo;
        private System.Windows.Forms.TextBox textBoxOffNO;
        private System.Windows.Forms.Label labelOffNO;
        private System.Windows.Forms.Button buttonCosPhiTrackInit;
        private System.Windows.Forms.Button buttonDetectCosPhi;
        private System.Windows.Forms.Button buttonSaveTrackingResults;
        private System.Windows.Forms.Button buttonRunOfflineCosPhi;
    }
}

