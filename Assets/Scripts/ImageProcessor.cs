using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenCVForUnity;

namespace Assets.Scripts.Detector
{
    internal sealed class ImageProcessor : IDisposable
    {

        private readonly Size _prefilterSize = new Size(320, 240);
        private double _minimalMarkerBoxSide;

        private Action<string> _stopWatch;

        private Action<string> StopWatch
        {
            get { return _stopWatch ?? (x => { }); }
        }

        private bool _isDisposed = false;
        private Mat _graySmall;
        private Mat _grayFull;
        private Mat _drawing;

        internal const int GridSize = 6; // 4 meaningful + 2 borders

        private Action<Mat> _setDebugMat;
        private double _resizeRatio;

        private Action<Mat> SetDebugMat
        {
            get { return _setDebugMat ?? (x => { }); }
        }

        internal bool DebugMode
        {
            get { return _stopWatch != null || _setDebugMat != null; }
        }

        public void Init()
        {
            ThrowIfDisposed();
            _graySmall = new Mat(_prefilterSize, CvType.CV_8UC1);
        }

        public Point[] ProcessFrame(Mat image)
        {
            ThrowIfDisposed();
            _grayFull = new Mat(image.size(), CvType.CV_8UC1);

            _resizeRatio = Math.Max((double)_grayFull.width() / _prefilterSize.width, (double)_grayFull.height() / _prefilterSize.height);
            _minimalMarkerBoxSide = 2 * GridSize;
            Imgproc.cvtColor(image, _grayFull, Imgproc.COLOR_RGB2GRAY);
            Imgproc.resize(_grayFull, _graySmall, _prefilterSize, _resizeRatio, _resizeRatio, Imgproc.INTER_NEAREST);

            var approxContours = FindContours() ?? new List<MatOfPoint>();
            var contoursCnt = approxContours.Count;

            var rectangleCandidates = FilterRectangleCandidates(_prefilterSize, approxContours) ?? new List<MatOfPoint2f>(); // null happens in Unity/mono

            var rectsCnt = rectangleCandidates.Count;
            StopWatch("rects");
            var markerExtractor = new MarkerExtractor(this, _grayFull, _resizeRatio, _drawing);

            var results = rectangleCandidates.Select<MatOfPoint2f, Point[]>(markerExtractor.ExtractMessageBitsIfCorrect);

            Point[] res = results.FirstOrDefault(x => x != null);

            return res;
        }




        private List<MatOfPoint> FindContours()
        {
            using (var blurred = new Mat())
            using (var edges = new Mat())
            using (var clahe = Imgproc.createCLAHE(5, new Size(_minimalMarkerBoxSide, _minimalMarkerBoxSide)))

            using (var hierarchy = new Mat())
            //using (var bw = new Mat())
            {

                //var threshToZero = 50;
                //var hugeKernel = 2 * (int)(_minimalMarkerBoxSide) + 1;
                var tinyKernel = 1 + 2 * (int)(Math.Min(_prefilterSize.width, _prefilterSize.height) / 100);
                //Imgproc.GaussianBlur(_graySmall, blurred, new Size(tinyKernel, tinyKernel), 0);

                //Imgproc.adaptiveThreshold(blurred, bw, threshToZero, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, hugeKernel, 20);
                ////const double scale = 2.0;
                //Core.addWeighted(blurred, 1, bw, 2, -threshToZero, blurred);
                ////Imgproc.GaussianBlur(blurred, blurred, new Size(tinyKernel, tinyKernel), 0);


                //StopWatch("ggm9");

                //Imgproc.Canny(blurred, edges, 250, 350, 3, true); //TODO auto values                
                //StopWatch("canny2");
                //Imgproc.dilate(edges, dilated, dilationKernel);
                //StopWatch("dilate");
                clahe.apply(_graySmall, blurred);
                Imgproc.GaussianBlur(blurred, blurred, new Size(tinyKernel, tinyKernel), tinyKernel / 4.0);
                StopWatch("blur4");

                //clahe.apply(blurred, blurred);
                //Imgproc.equalizeHist(blurred, blurred);
                //StopWatch("clahe2");

                Imgproc.Canny(blurred, edges, 2000, 3000, 5, true); //TODO auto values         
                var contours = new List<MatOfPoint>(200);
                Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
                StopWatch("cntrs");


                if (_drawing != null)
                {
                    Imgproc.cvtColor(blurred, _drawing, Imgproc.COLOR_GRAY2RGB, 3);
                }


                var result = new List<MatOfPoint>(contours.Count);
                for (var i = 0; i < contours.Count; i++)
                {
                    var h = hierarchy.get(0, i);
                    var hasChild = h[2] > -1;
                    //Only interested in contours which have inner contours
                    if (hasChild)
                    {
                        result.Add(contours[i]);
                    }
                    else
                    {
                        contours[i].Dispose();

                        if (DebugMode)
                            result.Add(null);

                    }
                }

                if (_drawing != null)
                {
                    var cntrs = result.Where(x => x != null).ToList();
                    var color = new Scalar(0, 255, 0);
                    for (var i = 0; i < cntrs.Count; ++i)
                    {
                        Imgproc.drawContours(_drawing, cntrs, i, color, 1, Imgproc.LINE_8, hierarchy, 0, new Point(0, 0));

                    }
                    StopWatch("draw");
                }
                return result;
            }
        }

        private static float Dist2(float[] x, int i, int j)
        {
            var d0 = x[i] - x[j];
            var d1 = x[i + 1] - x[j + 1];
            return d0 * d0 + d1 * d1;
        }

        private List<MatOfPoint2f> FilterRectangleCandidates(Size origSize, List<MatOfPoint> contours)
        {
            var result = new List<MatOfPoint2f>(contours.Count);
            Console.WriteLine("{0} contours to check", contours.Count);
            using (var convexedContourPoints = new MatOfInt())
            using (var convexedContour = new MatOfPoint2f())
            using (var processedContour = new MatOfPoint2f())
            {

                foreach (var contour in contours)
                {
                    if (contour == null)
                        continue;
                    //TODO: Check other approximations
                    Imgproc.convexHull(contour, convexedContourPoints, true);
                    var pointsCnt = convexedContourPoints.rows();
                    if (pointsCnt < 4) return null;

                    var points = new Point[pointsCnt];

                    for (var p = 0; p < pointsCnt; p++)
                    {
                        var index = (int)convexedContourPoints.get(p, 0)[0];
                        var c = contour.get(index, 0);
                        points[p] = new Point(c[0], c[1]);
                    }

                    convexedContour.fromArray(points);

                    var arcLength = Imgproc.arcLength(convexedContour, true);
                    var arcInfo = new StringBuilder();

                    if (DebugMode)
                        arcInfo.Append("ARC (").Append(arcLength).Append(") ");

                    var sideLimitLow = _minimalMarkerBoxSide;
                    var sideLimitHigh = origSize.height / 2.0;

                    if (arcLength > 4 * sideLimitLow && arcLength < 4 * sideLimitHigh) //not too short or too long arc
                    {
                        Imgproc.approxPolyDP(convexedContour, processedContour, Math.Max(arcLength * 0.05, 2), true);

                        if (processedContour.rows() == 4)
                        {
                            var p = new float[8];
                            processedContour.get(0, 0, p);
                            var d1 = Dist2(p, 0, 2);
                            var d2 = Dist2(p, 2, 4);
                            var d3 = Dist2(p, 4, 6);
                            var d4 = Dist2(p, 0, 6);
                            var limitHigh2 = sideLimitHigh * sideLimitHigh;
                            var limitLow2 = sideLimitLow * sideLimitLow;
                            if (d1 < limitLow2 || d2 < limitLow2 || d3 < limitLow2 || d4 < limitLow2)
                            {
                                if (DebugMode)
                                {
                                    arcInfo.Append("has too short edge");
                                }

                            }
                            else if (d1 > limitHigh2 || d2 > limitHigh2 || d3 > limitHigh2 || d4 > limitHigh2)
                            {
                                if (DebugMode)
                                {
                                    arcInfo.Append("has too long edge");
                                }
                            }
                            else
                            {
                                if (DebugMode)
                                {
                                    arcInfo.Append("accepted");
                                }
                                result.Add(new MatOfPoint2f(processedContour.toArray()));
                            }

                        }
                        else if (DebugMode)
                        {
                            arcInfo.Append("has ").Append(processedContour.rows()).Append(" corners");
                        }
                    }
                    else
                    {
                        if (DebugMode)
                            arcInfo.Append("too ").Append(arcLength < 4 * sideLimitLow ? "short" : "long");
                    }
                    if (DebugMode)
                    {
                        result.Add(null);
                        Console.WriteLine(arcInfo.ToString());
                    }
                }
            }
            return result;
        }


        public void Destroy()
        {
            ThrowIfDisposed(); // just for fun, no reason
            if (_graySmall != null) _graySmall.Dispose();
            _graySmall = null;

            if (_grayFull != null) _grayFull.Dispose();
            _grayFull = null;

            if (_drawing != null) _drawing.Dispose();
            _drawing = null;
        }

        public void EnableDebug(Action<Mat> setDebugMat, Action<string> stopwatch)
        {
            ThrowIfDisposed(); // just for fun, no reason
            _setDebugMat = setDebugMat;
            _stopWatch = stopwatch;
            if (_setDebugMat != null)
                _drawing = new Mat(_prefilterSize, CvType.CV_8UC3);
        }

        private void ThrowIfDisposed()
        {
            if (_isDisposed) throw new ObjectDisposedException("ImageProcessor was used after disposal");
        }

        public void Dispose()
        {
            Dispose(true);
        }

        private void Dispose(bool disposing)
        {
            if (_isDisposed) return;

            if (disposing) Destroy();

            _isDisposed = true;
        }
    }
}
