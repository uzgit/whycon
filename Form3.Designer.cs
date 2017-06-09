namespace WhyConID
{
    partial class Form3
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
            this.rawImageBox = new Emgu.CV.UI.ImageBox();
            this.buttonClose = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.rawImageBox)).BeginInit();
            this.SuspendLayout();
            // 
            // rawImageBox
            // 
            this.rawImageBox.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(192)))), ((int)(((byte)(192)))), ((int)(((byte)(255)))));
            this.rawImageBox.Location = new System.Drawing.Point(2, 2);
            this.rawImageBox.Name = "rawImageBox";
            this.rawImageBox.Size = new System.Drawing.Size(1280, 1024);
            this.rawImageBox.TabIndex = 3;
            this.rawImageBox.TabStop = false;
            // 
            // buttonClose
            // 
            this.buttonClose.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(192)))), ((int)(((byte)(255)))), ((int)(((byte)(255)))));
            this.buttonClose.Font = new System.Drawing.Font("Algerian", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonClose.Location = new System.Drawing.Point(1289, 12);
            this.buttonClose.Name = "buttonClose";
            this.buttonClose.Size = new System.Drawing.Size(83, 59);
            this.buttonClose.TabIndex = 4;
            this.buttonClose.Text = "Close";
            this.buttonClose.UseVisualStyleBackColor = false;
            this.buttonClose.Click += new System.EventHandler(this.buttonClose_Click);
            // 
            // Form3
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1384, 1053);
            this.Controls.Add(this.buttonClose);
            this.Controls.Add(this.rawImageBox);
            this.Name = "Form3";
            this.Text = "Raw Frame";
            ((System.ComponentModel.ISupportInitialize)(this.rawImageBox)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        public Emgu.CV.UI.ImageBox rawImageBox;
        private System.Windows.Forms.Button buttonClose;
    }
}