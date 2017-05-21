using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity;
using Assets.Scripts.Detector;
using System;


public class Detection : MonoBehaviour {

    public GUITexture BTexture;

    public Renderer previewTexture = null;
    public Renderer edgesTexture = null;
    public UnityEngine.UI.Text text = null;

    private Point maxCoord = new Point();

    private Point[] coord;
    Point[] newCoord = new Point[8];

    public bool debugEnabled = true;
    public bool isProfiling = false;


    private Mat _cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);

    //PC's webcam
    //private MatOfDouble _distCoeffs = new MatOfDouble(1.7538937146703923e-01, -1.4791482438857808e+00, 0, 0, 2.1035973798172236e+00);
    //Phone backcam
    //private MatOfDouble _distCoeffs = new MatOfDouble(7.0053178005603539e-02, 3.8229851408219778e-01, 0, 0, -1.5576837627232218e+00);
    private MatOfDouble _distCoeffs = new MatOfDouble(7.9853651456921873e-02, 1.6961659493831194e-01, -4.0078838690470820e-03, -3.9888642294897325e-04, -9.2707257089689465e-01);

    private double[] position = new double[3];
    private double[] orientation = new double[4];

    private Mat rvecs = new Mat();
    private Mat tvecs = new Mat();

    private volatile bool processingFinished = true;
    private object processingLock = new object();

    private static Mat processingBuffer = new Mat();
    private Mat debugMat = new Mat();

    private int count = 0;

    private Mat essenRvec;
    private Mat essenTvec;
    private double temp1, temp2, temp3;
    WebcamImageProvider imageProvider;

    public struct PosAng {
        public Vector3 pos;
        public Quaternion Ang;
        public bool flag;
    }
    public static PosAng currentPosAng = new PosAng();


    private void ProcessingThread()
    {
        using (ImageProcessor imageProcessor = new ImageProcessor())
        {
            imageProcessor.Init();

            while (true)
            {
                lock (processingLock)
                {
                    //timer.Start();
                    var res = imageProcessor.ProcessFrame(processingBuffer);
                    coord = res;
                    count++;
                    //timer.Finish();
                    processingFinished = true;
                }

                // Wait for next frame
                while (processingFinished) System.Threading.Thread.Sleep(30);
            }
        }
    }

    // Use this for initialization
    void Start () {
        BTexture = gameObject.AddComponent<GUITexture>();
        BTexture.pixelInset = new UnityEngine.Rect(0, 0, 0, 0);

        //PC's webcam
        _cameraMatrix.put(0, 0, 6.6121360541705678e+02);
        _cameraMatrix.put(0, 1, 0d);
        _cameraMatrix.put(0, 2, 320f);
        _cameraMatrix.put(1, 0, 0d);
        _cameraMatrix.put(1, 1, 6.6121360541705678e+02);
        _cameraMatrix.put(1, 2, 240f);
        _cameraMatrix.put(2, 0, 0d);
        _cameraMatrix.put(2, 1, 0d);
        _cameraMatrix.put(2, 2, 1d);
        //Phone backcam
        //_cameraMatrix.put(0, 0, 1.4890091900442617e+03);
        //_cameraMatrix.put(0, 1, 0d);
        //_cameraMatrix.put(0, 2, 9.5950000000000000e+02);
        //_cameraMatrix.put(1, 0, 0d);
        //_cameraMatrix.put(1, 1, 1.4890091900442617e+03);
        //_cameraMatrix.put(1, 2, 5.3950000000000000e+02);
        //_cameraMatrix.put(2, 0, 0d);
        //_cameraMatrix.put(2, 1, 0d);
        //_cameraMatrix.put(2, 2, 1d);

        StartCoroutine(MainCoroutine());
    }

    private void OnDestroy()
    {
        imageProvider.webcamTexture.Stop();
    }

    private IEnumerator MainCoroutine()
    {
        imageProvider = new WebcamImageProvider();
        imageProvider.Init();
        yield return StartCoroutine(imageProvider.GetImage(processingBuffer));

        BTexture.texture = imageProvider.webcamTexture;

        (new System.Threading.Thread(this.ProcessingThread)).Start();

        while(true){

            while (!processingFinished) yield return new WaitForEndOfFrame();

            lock (processingLock)
            {
                yield return StartCoroutine(imageProvider.GetImage(processingBuffer));
                processingFinished = false;
            }

            while (!processingFinished) yield return new WaitForEndOfFrame();

            lock (processingLock)
            {
           
                if (coord != null)
                {
                    estimatePoseSingleMarkers(coord, 0.05f, _cameraMatrix, _distCoeffs, rvecs, tvecs);

                    if (essenRvec == null || essenTvec == null) {
                        essenRvec = rvecs;
                        essenTvec = tvecs;
                        temp1 = rvecs.get(0, 0)[0];
                        temp2 = rvecs.get(1, 0)[0];
                        temp3 = rvecs.get(2, 0)[0];
                    }

                    double[] newPos = new double[3];
                    newCoord = drawAxis(_cameraMatrix, _distCoeffs, rvecs, tvecs, 0.05f);
                    GetParams(rvecs, tvecs, newPos, orientation);

                    currentPosAng.pos = new Vector3((float)newCoord[0].x, (float)newCoord[0].y, (float)tvecs.get(2, 0)[0]);
                   
                    if (Math.Abs(temp1 - rvecs.get(0, 0)[0]) > 0.005 && Math.Abs(temp1 - rvecs.get(0, 0)[0]) < 0.1 * count)
                        if (Math.Abs(temp2 - rvecs.get(1, 0)[0]) > 0.005 && Math.Abs(temp2 - rvecs.get(1, 0)[0]) < 0.1 * count)
                            if (Math.Abs(temp3 - rvecs.get(2, 0)[0]) > 0.005 && Math.Abs(temp3 - rvecs.get(2, 0)[0]) < 0.1 * count)
                            {
                                Quaternion temp = new Quaternion((float)orientation[1], -(float)orientation[2], -(float)orientation[3], (float)orientation[0]);
                                currentPosAng.Ang = temp;
                                temp1 = rvecs.get(0, 0)[0];
                                temp2 = rvecs.get(1, 0)[0];
                                temp3 = rvecs.get(2, 0)[0];
                                currentPosAng.flag = true;
                            }
                }
            }

            yield return new WaitForEndOfFrame();
        }
    }


    // 0.8 x + 0 y + 0.6 z = 9.7
    // 0   x + 1 y + 0.3 z = 3.75
    //                   z = 0
    public static Point getMaxRenderingPoint(float z) {
        Point result = new Point();
        Plane[] frustumPlanes = GeometryUtility.CalculateFrustumPlanes(Camera.main);
        float a1 = frustumPlanes[1].normal.x;
        float b1 = frustumPlanes[1].normal.y;
        float c1 = frustumPlanes[1].normal.z;
        float d1 = frustumPlanes[1].distance;

        float a2 = frustumPlanes[2].normal.x;
        float b2 = frustumPlanes[2].normal.y;
        float c2 = frustumPlanes[2].normal.z;
        float d2 = frustumPlanes[2].distance;

        result.x = (-d1 - c1 * z) / a1;
        result.y = (-d2 - c2 * z) / b2;

        return result;
    }

    MatOfPoint3f _getSingleMarkerObjectPoints(float markerLength)
    {
        Point3[] _result = {new Point3(-markerLength / 2, markerLength / 2, 0f),
                            new Point3(markerLength / 2,  markerLength / 2, 0f),
                            new Point3(markerLength / 2,  -markerLength / 2, 0f),
                            new Point3(-markerLength / 2, -markerLength / 2, 0f)};

        MatOfPoint3f result = new MatOfPoint3f(_result);
        return result;
    }
    void estimatePoseSingleMarkers(Point[] _corners, float markerLength, Mat _cameraMatrix, MatOfDouble _distCoeffs,
                                   Mat _rvecs, Mat _tvecs)
    {
        MatOfPoint3f _marker = _getSingleMarkerObjectPoints(markerLength);
        MatOfPoint2f temp = new MatOfPoint2f(_corners);
        Calib3d.solvePnP(_marker, temp, _cameraMatrix, _distCoeffs, _rvecs, _tvecs);
    }


    void GetParams(Mat rvecs, Mat tvecs, double[] position, double[] orientation)
    {
        position[0] = -tvecs.get(0, 0)[0];
        position[1] = -tvecs.get(1, 0)[0];
        position[2] = tvecs.get(2, 0)[0];

        Mat Rot = new Mat(3, 3, CvType.CV_32FC1);
        Calib3d.Rodrigues(rvecs, Rot);

        double[,] stAxes = new double[3, 3];

        stAxes[0, 0] = -Rot.get(0, 0)[0];
        stAxes[0, 1] = -Rot.get(1, 0)[0];
        stAxes[0, 2] =  Rot.get(2, 0)[0];

        stAxes[1, 0] = -Rot.get(0, 1)[0];
        stAxes[1, 1] = -Rot.get(1, 1)[0];
        stAxes[1, 2] =  Rot.get(2, 1)[0];

        stAxes[2, 0] = stAxes[0, 1] * stAxes[1, 2] - stAxes[0, 2] * stAxes[1, 1];
        stAxes[2, 1] = -stAxes[0, 0] * stAxes[1, 2] + stAxes[0, 2] * stAxes[1, 0];
        stAxes[2, 2] = stAxes[0, 0] * stAxes[1, 1] - stAxes[0, 1] * stAxes[1, 0];

        double[,] axes = new double[3, 3];

        axes[0, 0] = stAxes[0, 0];
        axes[1, 0] = stAxes[0, 1];
        axes[2, 0] = stAxes[0, 2];

        axes[0, 1] = stAxes[1, 0];
        axes[1, 1] = stAxes[1, 1];
        axes[2, 1] = stAxes[1, 2];

        axes[0, 2] = stAxes[2, 0];
        axes[1, 2] = stAxes[2, 1];
        axes[2, 2] = stAxes[2, 2];

        double fTrace = axes[0, 0] + axes[1, 1] + axes[2, 2];
        double fRoot;

        if (fTrace > 0.0)
        {
            // |w| > 1/2, may as well choose w > 1/2
            fRoot = Math.Sqrt(fTrace + 1.0); // 2w
            orientation[0] = 0.5 * fRoot;
            fRoot = 0.5 / fRoot; // 1/(4w)
            orientation[1] = (axes[2, 1] - axes[1, 2]) * fRoot;
            orientation[2] = (axes[0, 2] - axes[2, 0]) * fRoot;
            orientation[3] = (axes[1, 0] - axes[0, 1]) * fRoot;
        }
        else
        {
            // |w| <= 1/2
            uint[] s_iNext = { 1, 2, 0 };
            uint i = 0;
            if (axes[1, 1] > axes[0, 0])
                i = 1;
            if (axes[2, 2] > axes[i, i])
                i = 2;
            uint j = s_iNext[i];
            uint k = s_iNext[j];

            fRoot = Math.Sqrt(axes[i, i] - axes[j, j] - axes[k, k] + 1.0);
            orientation[i + 1] = 0.5 * fRoot;
            fRoot = 0.5 / fRoot;
            orientation[0] = (axes[k, j] - axes[j, k]) * fRoot;
            orientation[j + 1] = (axes[j, i] + axes[i, j]) * fRoot;
            orientation[k + 1] = (axes[k, i] + axes[i, k]) * fRoot;
        }

    }

    Point[] drawAxis(Mat _cameraMatrix, MatOfDouble _distCoeffs, Mat rvec, Mat tvec, float length)
    {
        MatOfPoint3f axisPoints = new MatOfPoint3f(new Point3(0, 0, 0),
            new Point3(length, length, length), new Point3(0, length, 0), new Point3(0, 0, length));
        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(axisPoints, rvec, tvec, _cameraMatrix, _distCoeffs, imagePoints);
        return imagePoints.toArray();
    }
}
