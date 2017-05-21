using System;
using System.Collections.Generic;
using System.Threading;
using JetBrains.Annotations;
using OpenCVForUnity;
using UnityEngine;

namespace Assets.Scripts.Detector
{
    internal sealed class MarkerExtractor
    {
        private const int BitStep = 10;
        private const int MappedSize = BitStep * ImageProcessor.GridSize;

        [NotNull]
        private readonly ImageProcessor _imageProcessor;

        private static readonly MatOfPoint2f MappedSpace = new MatOfPoint2f(new Point(0, 0), new Point(MappedSize, 0),
            new Point(MappedSize, MappedSize), new Point(0, MappedSize));

        private readonly double _resizeRatio;
        private readonly Mat _drawing;
        [NotNull]
        private readonly Mat _grayFull;


        private bool DebugMode { get { return _imageProcessor.DebugMode; } }

        public MarkerExtractor(ImageProcessor imageProcessor, Mat gray, double resizeRatio, Mat drawing)
        {
            _imageProcessor = imageProcessor;
            _resizeRatio = resizeRatio;
            _drawing = drawing;
            _grayFull = gray;
        }


        internal class MessageContext : IDisposable
        {
            private MessageContext()
            {
            }

            private Mat _src;
            public Point[] OrigCandidate { get; private set; }
            private Mat FlatMarker { get; set; }
            private Mat BlurredMarker { get; set; }
            private Mat HomographyMatrix { get; set; }
            internal Mat FilteredMarker { get; set; }
            private MatOfPoint2f Candidate { get; set; }
            public bool HasProcessingErrors { get; set; }

            public const int ThreshMaxVal = 255;
            private void Init()
            {
                try
                {
                    HomographyMatrix = Imgproc.getPerspectiveTransform(Candidate, MappedSpace);
                    Imgproc.warpPerspective(_src, FlatMarker, HomographyMatrix, new Size(MappedSize, MappedSize),

                        0, 0 /*Imgproc.BORDER_CONSTANT*/, Scalar.all(0) /*black*/);
                    //TODO check if I need it: resize to 6x6, then threshold and that's all (????)
                    //Imgproc.bilateralFilter(FlatMarker, BlurredMarker, 5, 10, 5, 0);
                    // blur, but preserving borders                
                    Imgproc.threshold(FlatMarker, FilteredMarker, 0, ThreshMaxVal,
                        Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

                }
                finally
                {
                    _src = null;
                }
            }

            public void Dispose()
            {
                if (FlatMarker != null)
                {
                    FlatMarker.Dispose();
                    FlatMarker = null;
                }
                if (BlurredMarker != null)
                {
                    BlurredMarker.Dispose();
                    BlurredMarker = null;
                }
                if (HomographyMatrix != null)
                {
                    HomographyMatrix.Dispose();
                    HomographyMatrix = null;
                }
                if (FilteredMarker != null)
                {
                    FilteredMarker.Dispose();
                    FilteredMarker = null;
                }
                if (Candidate != null)
                {
                    Candidate.Dispose();
                    Candidate = null;
                }
            }

            internal static MessageContext Create(MatOfPoint2f candidate, Mat src, double resizeRatio)
            {

                var context = new MessageContext()
                {
                    FlatMarker = new Mat(MappedSize, MappedSize, CvType.CV_8UC1),
                    BlurredMarker = new Mat(MappedSize, MappedSize, CvType.CV_8UC1),
                    HomographyMatrix = null,
                    FilteredMarker = new Mat(MappedSize, MappedSize, CvType.CV_8UC1),
                    Candidate = new MatOfPoint2f(),
                    OrigCandidate = candidate.toArray(),
                    _src = src
                };
                Core.multiply(candidate, Scalar.all(resizeRatio), context.Candidate);
                return context;

            }

            public bool? GetBitBox(int i, int j)
            {
                if (_src != null)
                    Init();
                const int ratio = 3;
                // TODO remove submat to optimize, use adjustROI                        

                var centerY = i * BitStep + BitStep / 2;
                var centerX = (ImageProcessor.GridSize - 1 - j) * BitStep + BitStep / 2;
                var block = FilteredMarker.submat(centerX - BitStep / ratio, centerX + BitStep / ratio,
                    centerY - BitStep / ratio, centerY + BitStep / ratio);
                var blockMean = Core.sumElems(block).val[0] / block.rows() / block.cols() / MessageContext.ThreshMaxVal;
                return blockMean < 0.4 ? true : blockMean > 0.6 ? false : new bool?();
            }
        }

        internal Point[] ExtractMessageBitsIfCorrect(MatOfPoint2f candidate)
        {
            if (candidate == null)
                return null;

            //var c1x = (p[0] + p[4]) / 2;
            //var c1y = (p[1] + p[5]) / 2;
            //var c2x = (p[2] + p[6]) / 2;
            //var c2y = (p[3] + p[7]) / 2;
            //var d1x = resize * (c1x - p[0]);
            //var d1y = resize * (c1y - p[1]);
            //var d2x = resize * (c2x - p[2]);
            //var d2y = resize * (c2y - p[3]);
            //var r = _resizeRatio;
            //var pts = new[] { new Point(r*(c1x - d1x), r*(c1y - d1y)),
            //                                      new Point(r*(c2x - d2x), r*(c2y - d2y)),
            //                                      new Point(r*(c1x + d1x), r*(c1y + d1y)),
            //                                      new Point(r*(c2x + d2x), r*(c2y + d2y))};
            //result.Add(new MatOfPoint2f(pts));


            using (var marker = Marker.CreateFromImage(candidate, _grayFull, _resizeRatio))
            {

                var red = new Scalar(255, 0, 0); // when fails to detect some bit
                var blue = new Scalar(0, 0, 255); // when OK, but before parity checks
                var yellow = new Scalar(255, 255, 0); // when border is incorrect
                var purple = new Scalar(150, 0, 150);
                var color = red;

                try
                {

                    if (!marker.CheckBorder())
                    {
                        color = yellow;
                        return null;
                    }

                    /* marker message is like
                     * 1 . . 0
                     * . . P .
                     * 0 . . .
                     * 1 . . 0
                     * P is for parity.
                     * One reserved 0 bit on the left, one of 0 corners and
                     * parity bit P (the least one, please) can be reused for data bits later
                     * true - 1 from dark/black
                     * false -- 0 from light/white
                     */

                    color = purple;
                    var c1 = marker[4, 1];
                    var c2 = marker[4, 4];
                    var c3 = marker[1, 4];
                    var c4 = marker[1, 1];


                    if (!c1.HasValue || !c2.HasValue || !c3.HasValue || !c4.HasValue)
                    {
                        return null;
                    }
                    // our markers have two white and two black corners
                    // fast check before rotation
                    var wasError = c1.Value == c3.Value || c2.Value == c4.Value;
                    if (wasError && !DebugMode)
                    {
                        return null;
                    }

                    using (var message = c1.Value
                        ? (c4.Value ? marker : marker.Rotate90Clockwise())
                        : (c2.Value ? marker.Rotate180() : marker.Rotate90CounterClockwise()))  //improve coordinates
                    {

                        var cnt = 0;
                        Func<int, int, int, int> getBit = (row, col, bitPos) =>
                        {
                            var bit = message[row, col];
                            if (!bit.HasValue)
                            {
                                wasError = true;
                                return 0;
                            }
                            if (!bit.Value)
                                return 0;
                            Interlocked.Increment(ref cnt);
                            return 1 << bitPos;
                        };



                        var corners = getBit(1, 1, 0) | getBit(1, 4, 1) | getBit(4, 4, 2) | getBit(4, 1, 3);
                        // corners must be (1,0,0,1) clockwise frop top-left
                        wasError |= corners != 9;

                        if (wasError)
                        {
                            if (!DebugMode)
                                return null;
                            //return -corners * 10000;
                            return null;
                        }

                        color = blue;
                        // reserved to be zeroes
                        wasError |= 0 != getBit(1, 4, 0) + getBit(3, 1, 1);

                        if (message.Context.HasProcessingErrors && !DebugMode)
                        {
                            // return for speed-up
                            // but this is correct marker in general 
                            return null;
                        }


                        var result = 0;
                        cnt = 0;

                        result |= getBit(4, 2, 12);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(4, 3, 11);
                        if (wasError && !DebugMode) return null;
                        //reserved as white/0 
                        result |= getBit(4, 4, 10);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(3, 1, 9);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(3, 2, 8);
                        if (wasError && !DebugMode) return null;
                        //reserved for parity
                        result |= getBit(3, 3, 7);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(3, 4, 6);
                        if (wasError && !DebugMode) return null;
                        //reserved as white/0 
                        result |= getBit(2, 1, 5);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(2, 2, 4);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(2, 3, 3);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(2, 4, 2);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(1, 2, 1);
                        if (wasError && !DebugMode) return null;
                        result |= getBit(1, 3, 0);
                        wasError |= cnt % 2 != 0;
                        wasError |= marker.Context.HasProcessingErrors;
                        if (wasError)
                            result = -result;

                        //marker.Context.OrigCandidate
                        //cube.transform.Translate(1.0f, 1.0f, 0.0f);
                        var coordinates = marker.Context.OrigCandidate;

                        if (wasError && !DebugMode) return null;
                        return coordinates;
                        //return result;
                    }

                }
                finally
                {
                    if (DebugMode)
                    {

                        Console.WriteLine("Evaluated {0} bits ({1} errors)", marker.CountEvaluatedBits(), marker.Context.HasProcessingErrors ? "WITH" : "without");
                        if (_drawing != null)
                        {
                            marker.DrawAt(_drawing, marker.Context.HasProcessingErrors ? red : color);
                        }
                    }
                }

            }

        }

        internal struct Marker : IDisposable
        {
            [NotNull]
            private readonly MessageContext _context;
            [NotNull]
            private readonly MessageBit[,] _bits;

            public MessageContext Context { get { return _context; } }

            private Marker(MessageContext context, MessageBit[,] bits)
            {
                _context = context;
                _bits = bits;
            }

            public static Marker CreateFromImage(MatOfPoint2f candidate, Mat src, double resizeRatio)
            {
                var context = MessageContext.Create(candidate, src, resizeRatio);
                var marker = new Marker(context, new MessageBit[ImageProcessor.GridSize, ImageProcessor.GridSize]);

                for (var i = 0; i < ImageProcessor.GridSize; i++)
                {
                    for (var j = 0; j < ImageProcessor.GridSize; j++)
                    {
                        marker._bits[i, j] = new MessageBit(i, j, context);
                    }
                }

                return marker;
            }

            public void DrawAt(Mat drawing, Scalar color)
            {
                if (drawing == null || color == null)
                    return;
                using (var candidate = new MatOfPoint(Context.OrigCandidate))
                {
                    var pts = new List<MatOfPoint>() { candidate };
                    Imgproc.polylines(drawing, pts, true, color, 1);
                }
            }


            public bool? this[int i, int j]
            {
                get
                {
                    {
                        var bit = _bits[i, j].Value;
                        Context.HasProcessingErrors |= !bit.HasValue;
                        return bit;
                    }
                }
            }

            public Marker Rotate90Clockwise()
            {
                var n = _bits.GetLength(0);
                var ret = new MessageBit[n, n];

                for (var i = 0; i < n; ++i)
                {
                    for (var j = 0; j < n; ++j)
                    {
                        ret[i, j] = _bits[n - j - 1, i];
                    }
                }
                Point temp = Context.OrigCandidate[0];
                Context.OrigCandidate[0] = Context.OrigCandidate[3];
                Context.OrigCandidate[3] = Context.OrigCandidate[2];
                Context.OrigCandidate[2] = Context.OrigCandidate[1];
                Context.OrigCandidate[1] = temp;
                return new Marker(Context, ret);
            }

            public Marker Rotate90CounterClockwise()
            {
                var n = _bits.GetLength(0);
                var ret = new MessageBit[n, n];

                for (var i = 0; i < n; ++i)
                {
                    for (var j = 0; j < n; ++j)
                    {
                        ret[i, j] = _bits[j, n - i - 1];
                    }
                }

                Point temp = Context.OrigCandidate[0];
                Context.OrigCandidate[0] = Context.OrigCandidate[1];
                Context.OrigCandidate[1] = Context.OrigCandidate[2];
                Context.OrigCandidate[2] = Context.OrigCandidate[3];
                Context.OrigCandidate[3] = temp;
                return new Marker(Context, ret);
            }

            public Marker Rotate180()
            {
                var n = _bits.GetLength(0);
                var ret = new MessageBit[n, n];

                for (var i = 0; i < n; ++i)
                {
                    for (var j = 0; j < n; ++j)
                    {
                        ret[i, j] = _bits[n - i - 1, n - j - 1];
                    }
                }
                Point temp = Context.OrigCandidate[1];
                Context.OrigCandidate[1] = Context.OrigCandidate[3];
                Context.OrigCandidate[3] = temp;
                temp = Context.OrigCandidate[0];
                Context.OrigCandidate[0] = Context.OrigCandidate[2];
                Context.OrigCandidate[2] = temp;
                return new Marker(Context, ret);
            }

            public int CountEvaluatedBits()
            {
                var cnt = 0;
                foreach (var bit in _bits)
                {
                    if (bit.HasValue)
                        cnt++;
                }
                return cnt;
            }

            public void Dispose()
            {
                Context.Dispose();
            }

            public bool CheckBorder()
            {
                var size = _bits.GetLength(0);
                for (var i = 0; i < size; i++)
                {
                    var left = this[i, 0];
                    if (!left.HasValue || !left.Value)
                        return false;

                    var top = this[0, i];
                    if (!top.HasValue || !top.Value)
                        return false;

                    var right = this[i, size - 1];
                    if (!right.HasValue || !right.Value)
                        return false;

                    var bottom = this[size - 1, i];
                    if (!bottom.HasValue || !bottom.Value)
                        return false;
                }

                return true;
            }
        }
    }

    //Lazy<bool?> implementation    
    internal struct MessageBit : IDisposable
    {
        private bool? _value;
        [NotNull]
        private readonly WeakReference _context;
        private readonly int _col;
        private readonly int _row;

        public MessageBit(int row, int col, [NotNull] MarkerExtractor.MessageContext context)
        {
            _row = row;
            _col = col;
            _context = new WeakReference(context);
            _value = null;
        }

        public bool HasValue { get { return _context.Target == null; } }
        public bool? Value
        {
            get
            {
                if (_context.Target == null)
                    return _value;
                _value = ((MarkerExtractor.MessageContext)_context.Target).GetBitBox(_row, _col);
                _context.Target = null;
                return _value;
            }
        }

        public void Dispose()
        {
            _context.Target = null;
            _value = null;
        }
    }
}