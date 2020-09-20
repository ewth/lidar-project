using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PcInterpreter
{
    class RadarPoint
    {
        public int AngleDegrees { get; private set; }
        public double AngleRadians { get; private set; }
        public int Length { get; private set; }
        public int X { get; private set; }
        public int Y { get; private set; }
        public int X2 { get; private set; } = -100;
        public int Y2 { get; private set; } = -100;

        public RadarPoint(int angleDegrees, int length, int originX, int originY)
        {
            AngleDegrees = angleDegrees;
            Length = length;
            int hypotenuse = length;
            AngleRadians = angleDegrees * Math.PI / 180.0;
            double opposite = hypotenuse * Math.Sin(AngleRadians);
            double adjacent = Math.Sqrt(Math.Pow(hypotenuse, 2) - Math.Pow(opposite, 2));
            
            Y = originY - (int)opposite;
            if (angleDegrees <= 90)
                X = originX - (int)adjacent;
            else
                X = originX + (int)adjacent;
        }

        public void SetX2(int X2)
        {
            this.X2 = X2;
        }

        public void SetY2(int Y2)
        {
            this.Y2 = Y2;
        }

    }
    class Radar
    {

        public event EventHandler ImageUpdatedHandler;

        public Bitmap image { get; private set; }

        private const int maxPoints = 360;
        private Queue<RadarPoint> points = new Queue<RadarPoint>();
        private int maxProjectionLength;
        private int prevX = -100;
        private int prevY = -100;

        public Radar(int maxProjectionLength = 2500)
        {
            if (maxProjectionLength < 1)
            {
                throw new Exception("Projection length must be at least 1.");
            }
            this.maxProjectionLength = maxProjectionLength;
            
            // It's a 180 degree arc; x can be +/-, y can only be +
            // Adding 100 for good measure
            image = new Bitmap(maxProjectionLength * 2 + 100, maxProjectionLength + 100);

            // Paint it black
            using (Graphics graph = Graphics.FromImage(image))
            {
                var imageRectangle = new Rectangle(0, 0, image.Width, image.Height);
                graph.FillRectangle(Brushes.Black, imageRectangle);
            }
        }

        /// <summary>
        /// Add a new point to our radar
        /// </summary>
        /// <param name="angleDegrees">The angle (in degrees)</param>
        /// <param name="projectionLength">The length of the projection</param>
        public void AddPoint(int angleDegrees, int projectionLength)
        {
            // Cancel out noise/things too far away
            if (projectionLength >= maxProjectionLength)
                projectionLength = 0;

            var newPoint = new RadarPoint(angleDegrees, projectionLength, image.Width/2, image.Height-200);
            if (newPoint.X >= 1 && newPoint.Y >= 1 && newPoint.X < image.Width && newPoint.Y < image.Height)
                image.SetPixel(newPoint.X, newPoint.Y, Color.Green);

            if (prevX != -100 && prevY != -100)
            {
                newPoint.SetX2(prevX);
                newPoint.SetY2(prevY);
                using (var graphics = Graphics.FromImage(image))
                {
                    graphics.DrawLine(new Pen(Color.LimeGreen, 2), prevX, prevY, newPoint.X, newPoint.Y);
                }
            }

            prevX = newPoint.X;
            prevY = newPoint.Y;
            points.Enqueue(newPoint);
            while (points.Count >= maxPoints)
            {
                var point = points.Dequeue();
                if (point.X2 != -100 && point.Y2 != -100)
                {
                    using (var graphics = Graphics.FromImage(image))
                    {
                        // graphics.DrawLine(new Pen(Color.Red, 2), newPoint.X2, newPoint.Y2, newPoint.X, newPoint.Y);
                        graphics.DrawLine(new Pen(Color.Black, 2), point.X, point.Y, point.X2, point.Y2);
                    }
                }
            }

            ImageUpdated(EventArgs.Empty);

        }

        /// <summary>
        /// Pass on updated image to event handler, if specified
        /// </summary>
        /// <param name="e"></param>
        protected virtual void ImageUpdated(EventArgs e)
        {
            ImageUpdatedHandler?.Invoke(image, e);
        }
    }
}
