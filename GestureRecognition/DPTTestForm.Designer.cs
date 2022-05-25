using System.Windows.Forms;
namespace GestureRecognition
{
    partial class DPTTestForm
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(DPTTestForm));
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.ilPanel3 = new ILNumerics.Drawing.ILPanel();
            this.coordinateButton = new System.Windows.Forms.Button();
            this.KFUnlabeledButton = new System.Windows.Forms.Button();
            this.ViconUnlabeledButton = new System.Windows.Forms.Button();
            this.KFPatternButton = new System.Windows.Forms.Button();
            this.ViconPatternButton = new System.Windows.Forms.Button();
            this.SigmaViconButton = new System.Windows.Forms.Button();
            this.SigmaAlphaButton = new System.Windows.Forms.Button();
            this.SigmaAButton = new System.Windows.Forms.Button();
            this.SigmaViconText = new System.Windows.Forms.TextBox();
            this.SigmaAlphaText = new System.Windows.Forms.TextBox();
            this.SigmaAText = new System.Windows.Forms.TextBox();
            this.textBox1 = new System.Windows.Forms.RichTextBox();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).BeginInit();
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.SuspendLayout();
            // 
            // splitContainer1
            // 
            this.splitContainer1.AllowDrop = true;
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(0, 0);
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.AllowDrop = true;
            this.splitContainer1.Panel1.Controls.Add(this.ilPanel3);
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.coordinateButton);
            this.splitContainer1.Panel2.Controls.Add(this.KFUnlabeledButton);
            this.splitContainer1.Panel2.Controls.Add(this.ViconUnlabeledButton);
            this.splitContainer1.Panel2.Controls.Add(this.KFPatternButton);
            this.splitContainer1.Panel2.Controls.Add(this.ViconPatternButton);
            this.splitContainer1.Panel2.Controls.Add(this.SigmaViconButton);
            this.splitContainer1.Panel2.Controls.Add(this.SigmaAlphaButton);
            this.splitContainer1.Panel2.Controls.Add(this.SigmaAButton);
            this.splitContainer1.Panel2.Controls.Add(this.SigmaViconText);
            this.splitContainer1.Panel2.Controls.Add(this.SigmaAlphaText);
            this.splitContainer1.Panel2.Controls.Add(this.SigmaAText);
            this.splitContainer1.Panel2.Controls.Add(this.textBox1);
            this.splitContainer1.Size = new System.Drawing.Size(826, 409);
            this.splitContainer1.SplitterDistance = 390;
            this.splitContainer1.TabIndex = 2;
            // 
            // ilPanel3
            // 
            this.ilPanel3.AllowDrop = true;
            this.ilPanel3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.ilPanel3.Driver = ILNumerics.Drawing.RendererTypes.OpenGL;
            this.ilPanel3.Editor = null;
            this.ilPanel3.Location = new System.Drawing.Point(0, 0);
            this.ilPanel3.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.ilPanel3.Name = "ilPanel3";
            this.ilPanel3.Rectangle = ((System.Drawing.RectangleF)(resources.GetObject("ilPanel3.Rectangle")));
            this.ilPanel3.ShowUIControls = false;
            this.ilPanel3.Size = new System.Drawing.Size(390, 409);
            this.ilPanel3.TabIndex = 0;
            this.ilPanel3.Load += new System.EventHandler(this.ilPanel3_Load);
            this.ilPanel3.DragDrop += new System.Windows.Forms.DragEventHandler(this.ilPanel3_DragDrop);
            this.ilPanel3.DragEnter += new System.Windows.Forms.DragEventHandler(this.ilPanel3_DragEnter);
            // 
            // coordinateButton
            // 
            this.coordinateButton.Location = new System.Drawing.Point(354, 237);
            this.coordinateButton.Name = "coordinateButton";
            this.coordinateButton.Size = new System.Drawing.Size(75, 40);
            this.coordinateButton.TabIndex = 11;
            this.coordinateButton.Text = "To Local Coordinates";
            this.coordinateButton.UseVisualStyleBackColor = true;
            this.coordinateButton.Click += new System.EventHandler(this.coordinateButton_Click);
            // 
            // KFUnlabeledButton
            // 
            this.KFUnlabeledButton.Location = new System.Drawing.Point(354, 181);
            this.KFUnlabeledButton.Name = "KFUnlabeledButton";
            this.KFUnlabeledButton.Size = new System.Drawing.Size(75, 50);
            this.KFUnlabeledButton.TabIndex = 10;
            this.KFUnlabeledButton.Text = "Show Labeled Markers";
            this.KFUnlabeledButton.UseVisualStyleBackColor = true;
            this.KFUnlabeledButton.Click += new System.EventHandler(this.KFUnlabeledButton_Click);
            // 
            // ViconUnlabeledButton
            // 
            this.ViconUnlabeledButton.Location = new System.Drawing.Point(354, 125);
            this.ViconUnlabeledButton.Name = "ViconUnlabeledButton";
            this.ViconUnlabeledButton.Size = new System.Drawing.Size(75, 50);
            this.ViconUnlabeledButton.TabIndex = 9;
            this.ViconUnlabeledButton.Text = "Show Unlabeled Markers";
            this.ViconUnlabeledButton.UseVisualStyleBackColor = true;
            this.ViconUnlabeledButton.Click += new System.EventHandler(this.ViconUnlabeledButton_Click);
            // 
            // KFPatternButton
            // 
            this.KFPatternButton.Location = new System.Drawing.Point(354, 67);
            this.KFPatternButton.Name = "KFPatternButton";
            this.KFPatternButton.Size = new System.Drawing.Size(75, 52);
            this.KFPatternButton.TabIndex = 8;
            this.KFPatternButton.Text = "Show Estimated Pattern";
            this.KFPatternButton.UseVisualStyleBackColor = true;
            this.KFPatternButton.Click += new System.EventHandler(this.KFPatternButton_Click);
            // 
            // ViconPatternButton
            // 
            this.ViconPatternButton.Location = new System.Drawing.Point(354, 12);
            this.ViconPatternButton.Name = "ViconPatternButton";
            this.ViconPatternButton.Size = new System.Drawing.Size(75, 49);
            this.ViconPatternButton.TabIndex = 7;
            this.ViconPatternButton.Text = "Show Measured Pattern";
            this.ViconPatternButton.UseVisualStyleBackColor = true;
            this.ViconPatternButton.Click += new System.EventHandler(this.ViconPatternButton_Click);
            // 
            // SigmaViconButton
            // 
            this.SigmaViconButton.Location = new System.Drawing.Point(231, 365);
            this.SigmaViconButton.Name = "SigmaViconButton";
            this.SigmaViconButton.Size = new System.Drawing.Size(75, 23);
            this.SigmaViconButton.TabIndex = 6;
            this.SigmaViconButton.Text = "Set R";
            this.SigmaViconButton.UseVisualStyleBackColor = true;
            this.SigmaViconButton.Click += new System.EventHandler(this.SigmaViconButton_Click);
            // 
            // SigmaAlphaButton
            // 
            this.SigmaAlphaButton.Location = new System.Drawing.Point(125, 365);
            this.SigmaAlphaButton.Name = "SigmaAlphaButton";
            this.SigmaAlphaButton.Size = new System.Drawing.Size(75, 23);
            this.SigmaAlphaButton.TabIndex = 5;
            this.SigmaAlphaButton.Text = "Set Q_2";
            this.SigmaAlphaButton.UseVisualStyleBackColor = true;
            this.SigmaAlphaButton.Click += new System.EventHandler(this.SigmaAlphaButton_Click);
            // 
            // SigmaAButton
            // 
            this.SigmaAButton.Location = new System.Drawing.Point(18, 365);
            this.SigmaAButton.Name = "SigmaAButton";
            this.SigmaAButton.Size = new System.Drawing.Size(75, 23);
            this.SigmaAButton.TabIndex = 4;
            this.SigmaAButton.Text = "Set Q_1";
            this.SigmaAButton.UseVisualStyleBackColor = true;
            this.SigmaAButton.Click += new System.EventHandler(this.SigmaAButton_Click);
            // 
            // SigmaViconText
            // 
            this.SigmaViconText.Location = new System.Drawing.Point(219, 345);
            this.SigmaViconText.Name = "SigmaViconText";
            this.SigmaViconText.Size = new System.Drawing.Size(100, 20);
            this.SigmaViconText.TabIndex = 3;
            // 
            // SigmaAlphaText
            // 
            this.SigmaAlphaText.Location = new System.Drawing.Point(113, 344);
            this.SigmaAlphaText.Name = "SigmaAlphaText";
            this.SigmaAlphaText.Size = new System.Drawing.Size(100, 20);
            this.SigmaAlphaText.TabIndex = 2;
            // 
            // SigmaAText
            // 
            this.SigmaAText.Location = new System.Drawing.Point(7, 345);
            this.SigmaAText.Name = "SigmaAText";
            this.SigmaAText.Size = new System.Drawing.Size(100, 20);
            this.SigmaAText.TabIndex = 1;
            // 
            // textBox1
            // 
            this.textBox1.DetectUrls = false;
            this.textBox1.Location = new System.Drawing.Point(3, 12);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.ShortcutsEnabled = false;
            this.textBox1.Size = new System.Drawing.Size(349, 330);
            this.textBox1.TabIndex = 0;
            this.textBox1.Text = "";
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 20;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // DPTTestForm
            // 
            this.AllowDrop = true;
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(826, 409);
            this.Controls.Add(this.splitContainer1);
            this.KeyPreview = true;
            this.Name = "DPTTestForm";
            this.Text = "SMTTestForm";
            this.KeyDown += new System.Windows.Forms.KeyEventHandler(this.ilPanel3_KeyDown);
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            this.splitContainer1.Panel2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).EndInit();
            this.splitContainer1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        protected override CreateParams CreateParams
        {
            get
            {
                CreateParams cp = base.CreateParams;
                cp.ExStyle |= 0x02000000;  // Turn on WS_EX_COMPOSITED
                return cp;
            }
        }

        #endregion

        private System.Windows.Forms.SplitContainer splitContainer1;
        private ILNumerics.Drawing.ILPanel ilPanel3;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.RichTextBox textBox1;
        private System.Windows.Forms.TextBox SigmaViconText;
        private System.Windows.Forms.TextBox SigmaAlphaText;
        private System.Windows.Forms.TextBox SigmaAText;
        private System.Windows.Forms.Button SigmaViconButton;
        private System.Windows.Forms.Button SigmaAlphaButton;
        private System.Windows.Forms.Button SigmaAButton;
        private Button KFUnlabeledButton;
        private Button ViconUnlabeledButton;
        private Button KFPatternButton;
        private Button ViconPatternButton;
        private Button coordinateButton;
    }
}