using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Media.Imaging;
using System.IO.Ports;

namespace PcInterpreter
{
    public partial class MainWindow : Window
    {
        Radar radar;
        SerialPort monitor;
        string savePath;

        public MainWindow()
        {
            InitializeComponent();

            string[] ports = SerialPort.GetPortNames();
            foreach (var port in ports)
            {
                comboComPorts.Items.Add(port);
            }
            if (comboComPorts.Items.Count > 0) {
                comboComPorts.SelectedIndex = comboComPorts.Items.Count - 1;
            }
            savePath = Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location), "outputImages");
            if (!Directory.Exists(savePath))
                Directory.CreateDirectory(savePath);
        }

        /// <summary>
        /// Start monitoring serial port for polling data
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ButtonMonitor_Click(object sender, RoutedEventArgs e)
        {
            radar = new Radar();
            radar.ImageUpdatedHandler += handleImageUpdated;
            buttonMonitor.IsEnabled = false;
            comboComPorts.IsEnabled = false;
            monitor = new SerialPort(comboComPorts.SelectedValue.ToString(), 115200, Parity.None, 8, StopBits.One);
            monitor.Handshake = Handshake.None;
            monitor.DataReceived += SerialPortDataReceived;
            monitor.Open();
            
            
        }

        /// <summary>
        /// Handler for receiving serial port data
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void SerialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            var dataRecv = monitor.ReadLine();
            Regex rx = new Regex(@"\[POLL:([0-9]+),([0-9\.]+)\]", RegexOptions.Compiled);
            var matches = rx.Matches(dataRecv);
            var points = new Dictionary<int, double>();
            foreach (Match match in matches)
            {
                var groups = match.Groups;
                int step = int.Parse(groups[1].Value);
                int distance = int.Parse(groups[2].Value);
                radar.AddPoint(step, distance);
            }
        }


        /// <summary>
        /// Event handler for radar class image updating
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void handleImageUpdated(object sender, EventArgs e)
        {
            if (sender is Bitmap)
            {
                // Forgive me, gods of programming, for the time I do not have.
                var currentDateString = DateTime.Now.ToString("yyyyMMdd-HHmmss-fffffff");
                var saveFile = "";
                Bitmap image = (Bitmap)sender;
                imageRadar.Dispatcher.Invoke(() =>
                {
                    // There is no cleanup for this. There WILL be a heap of files leftover.
                    saveFile = Path.Combine(savePath, $"{currentDateString}.bmp");
                    image.Save(saveFile);
                    var bitmapImage = new BitmapImage(new Uri(saveFile));
                    bitmapImage.CacheOption = BitmapCacheOption.None;
                    imageRadar.Source = bitmapImage;
                });

                if (savePath.Length < 1)
                    return;
            }
        }
    }
}
