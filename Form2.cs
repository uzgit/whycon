/*
 * Filename: Form2.cs
 * Author: Qinbing Fu
 * Date: Jan 2017
 * Description: This is the sub form.
 */

using System;
using System.Windows.Forms;

namespace WhyConID
{
    public partial class Form2 : Form
    {
        public bool updateSetting = false;

        /// <summary>
        /// Constructing
        /// </summary>
        public Form2()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Form2 Loading
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form2_Load(object sender, EventArgs e)
        {
            this.textBoxExposureText.Text = Form1.exposure.ToString();
            this.textBoxBrightnessText.Text = Form1.brightness.ToString();
            this.textBoxContrastText.Text = Form1.contrast.ToString();
            this.textBoxGainText.Text = Form1.gain.ToString();
            this.textBoxGammaText.Text = Form1.gamma.ToString();
            this.textBoxSharpnessText.Text = Form1.sharpness.ToString();
            this.textBoxTempText.Text = Form1.temperature.ToString();
        }

        #region FORM COMPONENTS
        /// <summary>
        /// Button Close Property Window
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonClose_Click(object sender, EventArgs e)
        {
            try
            {
                this.Hide();
            }
            catch (Exception exception)
            {
                MessageBox.Show(exception.ToString());
            }
        }
        /// <summary>
        /// Update and Save Camera Setting
        /// Settings in textboxes have priorities to be used
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void buttonSave_Click(object sender, EventArgs e)
        {
            try
            {
                updateSetting = true;
                Form1.frame_width = int.Parse(textBoxWidthText.Text);
                Form1.frame_height = int.Parse(textBoxHeightText.Text);
                if (Form1.frame_width > Form1.max_frame_width || Form1.frame_width < Form1.min_frame_width)
                    MessageBox.Show("Frame Width should be between 640 and 1280");
                if (Form1.frame_height > Form1.max_frame_height || Form1.frame_height < Form1.min_frame_height)
                    MessageBox.Show("Frame Height should be between 480 and 1024");
                Form1.exposure = int.Parse(textBoxExposureText.Text);
                Form1.brightness = int.Parse(textBoxBrightnessText.Text);
                Form1.contrast = int.Parse(textBoxContrastText.Text);
                Form1.gain = int.Parse(textBoxGainText.Text);
                Form1.gamma = int.Parse(textBoxGammaText.Text);
                Form1.sharpness = int.Parse(textBoxSharpnessText.Text);
                Form1.temperature = int.Parse(textBoxTempText.Text);
            }
            catch (Exception exc)
            {
                MessageBox.Show(exc.ToString());
            }
        }

        /// <summary>
        /// FPS ComboBox Selection
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void comboBoxFps_SelectedIndexChanged(object sender, EventArgs e)
        {
            try
            {
                Form1.fps = this.comboBoxFps.SelectedValue as int?;
                Form1.SetFps();
                MessageBox.Show("FPS Updated");
            }
            catch (Exception exc)
            {
                MessageBox.Show(exc.ToString());
            }
        }

        /// <summary>
        /// Initialize Scroll Bars
        /// </summary>
        public void UpdateScrollBar()
        {
            //Exposure Scroll Bar
            this.hScrollBarExposure.Maximum = 0;
            this.hScrollBarExposure.Minimum = -10;
            this.hScrollBarExposure.SmallChange = 1;
            this.hScrollBarExposure.LargeChange = 2;
            this.hScrollBarExposure.Scroll += new ScrollEventHandler(hScrollBarExposure_Scroll);

            //Brightness Scroll Bar
            this.hScrollBarBrightness.Maximum = 300;
            this.hScrollBarBrightness.Minimum = 0;
            this.hScrollBarBrightness.SmallChange = 10;
            this.hScrollBarBrightness.LargeChange = 50;
            this.hScrollBarBrightness.Scroll += new ScrollEventHandler(hScrollBarBrightness_Scroll);

            //Contrast Scroll Bar
            this.hScrollBarContrast.Maximum = 300;
            this.hScrollBarContrast.Minimum = 0;
            this.hScrollBarContrast.SmallChange = 10;
            this.hScrollBarContrast.LargeChange = 50;
            this.hScrollBarContrast.Scroll += new ScrollEventHandler(hScrollBarContrast_Scroll);

            //Gain Scroll Bar
            this.hScrollBarGain.Maximum = 1000;
            this.hScrollBarGain.Minimum = 0;
            this.hScrollBarGain.SmallChange = 10;
            this.hScrollBarGain.LargeChange = 100;
            this.hScrollBarGain.Scroll += new ScrollEventHandler(hScrollBarGain_Scroll);

            //Gamma Scroll Bar
            this.hScrollBarGamma.Maximum = 500;
            this.hScrollBarGamma.Minimum = 1;
            this.hScrollBarGamma.SmallChange = 10;
            this.hScrollBarGamma.LargeChange = 50;
            this.hScrollBarGamma.Scroll += new ScrollEventHandler(hScrollBarGamma_Scroll);

            //Sharpness Scroll Bar
            this.hScrollBarSharpness.Maximum = 14;
            this.hScrollBarSharpness.Minimum = 0;
            this.hScrollBarSharpness.SmallChange = 1;
            this.hScrollBarSharpness.LargeChange = 2;
            this.hScrollBarSharpness.Scroll += new ScrollEventHandler(hScrollBarSharpness_Scroll);

            //Temperature Scroll Bar
            this.hScrollBarTemp.Maximum = 100;
            this.hScrollBarTemp.Minimum = -100;
            this.hScrollBarTemp.SmallChange = 10;
            this.hScrollBarTemp.LargeChange = 30;
            this.hScrollBarTemp.Scroll += new ScrollEventHandler(hScrollBarTemp_Scroll);
        }

        private void hScrollBarExposure_Scroll(object sender, ScrollEventArgs e)
        {
            this.textBoxExposureText.Text = hScrollBarExposure.Value.ToString();
        }

        private void hScrollBarBrightness_Scroll(object sender, ScrollEventArgs e)
        {
            this.textBoxBrightnessText.Text = hScrollBarBrightness.Value.ToString();
        }

        private void hScrollBarContrast_Scroll(object sender, ScrollEventArgs e)
        {
            this.textBoxContrastText.Text = hScrollBarContrast.Value.ToString();
        }

        private void hScrollBarGain_Scroll(object sender, ScrollEventArgs e)
        {
            this.textBoxGainText.Text = hScrollBarGain.Value.ToString();
        }

        private void hScrollBarGamma_Scroll(object sender, ScrollEventArgs e)
        {
            this.textBoxGammaText.Text = hScrollBarGamma.Value.ToString();
        }

        private void hScrollBarSharpness_Scroll(object sender, ScrollEventArgs e)
        {
            this.textBoxSharpnessText.Text = hScrollBarSharpness.Value.ToString();
        }

        private void hScrollBarTemp_Scroll(object sender, ScrollEventArgs e)
        {
            this.textBoxTempText.Text = hScrollBarTemp.Value.ToString();
        }
        #endregion
        
    }
}
