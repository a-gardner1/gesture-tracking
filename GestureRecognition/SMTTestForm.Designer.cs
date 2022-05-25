namespace GestureRecognition
{
    partial class SMTTestForm
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(SMTTestForm));
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.ilPanel3 = new ILNumerics.Drawing.ILPanel();
            this.ilPanel4 = new ILNumerics.Drawing.ILPanel();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).BeginInit();
            this.timer1.Enabled = true;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            this.timer1.Interval = 10;
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.SuspendLayout();
            // 
            // splitContainer1
            // 
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(0, 0);
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.Controls.Add(this.ilPanel3);
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.ilPanel4);
            this.splitContainer1.Size = new System.Drawing.Size(682, 354);
            this.splitContainer1.SplitterDistance = 323;
            this.splitContainer1.TabIndex = 2;
            // 
            // ilPanel3
            // 
            this.ilPanel3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.ilPanel3.Driver = ILNumerics.Drawing.RendererTypes.OpenGL;
            this.ilPanel3.Editor = null;
            this.ilPanel3.Location = new System.Drawing.Point(0, 0);
            this.ilPanel3.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.ilPanel3.Name = "ilPanel3";
            this.ilPanel3.Rectangle = ((System.Drawing.RectangleF)(resources.GetObject("ilPanel3.Rectangle")));
            this.ilPanel3.ShowUIControls = false;
            this.ilPanel3.Size = new System.Drawing.Size(323, 354);
            this.ilPanel3.TabIndex = 0;
            this.ilPanel3.Load += new System.EventHandler(this.ilPanel3_Load);
            // 
            // ilPanel4
            // 
            this.ilPanel4.Dock = System.Windows.Forms.DockStyle.Fill;
            this.ilPanel4.Driver = ILNumerics.Drawing.RendererTypes.OpenGL;
            this.ilPanel4.Editor = null;
            this.ilPanel4.Location = new System.Drawing.Point(0, 0);
            this.ilPanel4.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.ilPanel4.Name = "ilPanel4";
            this.ilPanel4.Rectangle = ((System.Drawing.RectangleF)(resources.GetObject("ilPanel4.Rectangle")));
            this.ilPanel4.ShowUIControls = false;
            this.ilPanel4.Size = new System.Drawing.Size(355, 354);
            this.ilPanel4.TabIndex = 0;
            this.ilPanel4.Load += new System.EventHandler(this.ilPanel4_Load);
            // 
            // SMTTestForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(682, 354);
            this.Controls.Add(this.splitContainer1);
            this.Name = "SMTTestForm";
            this.Text = "SMTTestForm";
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).EndInit();
            this.splitContainer1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.SplitContainer splitContainer1;
        private ILNumerics.Drawing.ILPanel ilPanel3;
        private ILNumerics.Drawing.ILPanel ilPanel4;
        private System.Windows.Forms.Timer timer1;
    }
}