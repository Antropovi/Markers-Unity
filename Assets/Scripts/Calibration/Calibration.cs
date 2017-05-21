using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity;
using System;


public class Calibration : MonoBehaviour
{

    public enum Mode { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
    public GUITexture BTexture;
    public Texture2D tempTexture;
    WebcamImageProvider imageProvider = new WebcamImageProvider();
    public static Mode mode = Mode.DETECTION;
    Mat cameraMatrix = new Mat();
    Mat distCoeffs = new Mat();
    Size imageSize = new Size();
    List<Mat> imagePoints = new List<Mat>();

    void Start()
    {
        BTexture = gameObject.AddComponent<GUITexture>();
        tempTexture = new Texture2D(640, 480);
        //BTexture.texture = new Texture2D(;
        BTexture.pixelInset = new UnityEngine.Rect(0, 0, 0, 0);
        BTexture.texture = tempTexture;
        imageProvider.Init();
        Mat tem = new Mat();
        imageProvider.GetImage(tem);
        StartCoroutine(calib());
    }


    private IEnumerator calib()
    {


        Scalar RED = new Scalar(255, 0, 0);
        Scalar GREEN = new Scalar(0, 255, 0);

        for (int i = 0; ; ++i)
        {
            Mat view = new Mat();

            yield return StartCoroutine(imageProvider.GetImage(view));

            //-----  If no more image, or got enough, then stop calibration and show result -------------
            if (mode == Mode.CAPTURING && imagePoints.Count >= PlayerPrefs.GetInt("NumFrames"))
            {
                if (runCalibrationAndSave())
                    mode = Mode.CALIBRATED;
                else
                    mode = Mode.DETECTION;
            }

            imageSize = view.size();  // Format input image.

            MatOfPoint2f pointBuf = new MatOfPoint2f();

            bool found = Calib3d.findChessboardCorners(view,
                new Size(PlayerPrefs.GetInt("WidthNum"), PlayerPrefs.GetInt("HeightNum")), pointBuf, 11);


            if (found)                // If done with success,
            {
                i = 0;
                Mat viewGray = new Mat();
                Imgproc.cvtColor(view, viewGray, 6);
                Imgproc.cornerSubPix(viewGray, pointBuf, new Size(11, 11),
                   new Size(-1, -1), new TermCriteria(3, 30, 0.1));

                if (mode == Mode.CAPTURING)
                {
                    imagePoints.Add(pointBuf);
                    System.Threading.Thread.Sleep(50);
                }

                Calib3d.drawChessboardCorners(view, new Size(PlayerPrefs.GetInt("WidthNum"), PlayerPrefs.GetInt("HeightNum")),
                    pointBuf, found);

            }
            Utils.matToTexture(view, BTexture.texture);

        }
    }

    private void calcBoardCornerPositions(Size boardSize, float squareSize, List<Point3> corners)
    {
        corners.Clear();
        for (int i = 0; i < boardSize.height; ++i)
            for (int j = 0; j < boardSize.width; ++j)
                corners.Add(new Point3((float)(j * squareSize), (float)(i * squareSize), 0f));
    }

    private bool runCalibration(List<Mat> rvecs, List<Mat> tvecs,
                               List<float> reprojErrs, double totalAvgErr)
    {

        cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, 1.0);

        distCoeffs = Mat.zeros(8, 1, CvType.CV_64F);
        List<Mat> objectPoints = new List<Mat>();

        List<Point3> temp = new List<Point3>();
        //Converters.Mat_to_vector_Point3(objectPoints[0], temp);
        calcBoardCornerPositions(new Size(PlayerPrefs.GetInt("WidthNum"), PlayerPrefs.GetInt("HeightNum")),
            (float)PlayerPrefs.GetInt("SizeField"), temp);

        for (int j = 0; j < imagePoints.Count; ++j)
            objectPoints.Add(Converters.vector_Point3_to_Mat(temp, CvType.CV_64F));

        //objectPoints.resize(imagePoints.size(), objectPoints[0]);

        //Find intrinsic and extrinsic camera parameters
        double rms = _calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                    distCoeffs, rvecs, tvecs, 6158);

        //s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5

        bool ok = Core.checkRange(cameraMatrix) && Core.checkRange(distCoeffs);

        return ok;
    }

    //Print camera parameters to the output file
    private void saveCameraParams(List<Mat> rvecs, List<Mat> tvecs,
                                  List<float> reprojErrs, double totalAvgErr)
    {
        PlayerPrefs.SetFloat("M00", (float)cameraMatrix.get(0, 0)[0]);
        PlayerPrefs.SetFloat("M01", (float)cameraMatrix.get(0, 1)[0]);
        PlayerPrefs.SetFloat("M02", (float)cameraMatrix.get(0, 2)[0]);
        PlayerPrefs.SetFloat("M10", (float)cameraMatrix.get(1, 0)[0]);
        PlayerPrefs.SetFloat("M11", (float)cameraMatrix.get(1, 1)[0]);
        PlayerPrefs.SetFloat("M12", (float)cameraMatrix.get(1, 2)[0]);
        PlayerPrefs.SetFloat("M20", (float)cameraMatrix.get(2, 0)[0]);
        PlayerPrefs.SetFloat("M21", (float)cameraMatrix.get(2, 1)[0]);
        PlayerPrefs.SetFloat("M22", (float)cameraMatrix.get(2, 2)[0]);
        PlayerPrefs.SetFloat("d1", (float)distCoeffs.get(0, 0)[0]);
        PlayerPrefs.SetFloat("d2", (float)distCoeffs.get(1, 0)[0]);
        PlayerPrefs.SetFloat("d3", (float)distCoeffs.get(2, 0)[0]);
        PlayerPrefs.SetFloat("d4", (float)distCoeffs.get(3, 0)[0]);
        PlayerPrefs.SetFloat("d5", (float)distCoeffs.get(4, 0)[0]);

    }

    bool runCalibrationAndSave()
    {
        List<Mat> rvecs = new List<Mat>();
        List<Mat> tvecs = new List<Mat>();
        List<float> reprojErrs = new List<float>();
        double totalAvgErr = 0;

        bool ok = runCalibration(rvecs, tvecs, reprojErrs, totalAvgErr);

        if (ok)
            saveCameraParams(rvecs, tvecs, reprojErrs, totalAvgErr);
        return ok;
    }


    private Mat _prepareCameraMatrix(Mat cameraMatrix0, int rtype)
    {
        Mat cameraMatrix = Mat.eye(3, 3, rtype);
        if (cameraMatrix0.size() == cameraMatrix.size())
            cameraMatrix0.convertTo(cameraMatrix, rtype);
        return cameraMatrix;
    }

    static Mat _prepareDistCoeffs(Mat distCoeffs0, int rtype)
    {
        Mat distCoeffs = Mat.zeros(distCoeffs0.cols() == 1 ? new Size(1, 5) : new Size(5, 1), rtype);
        if (distCoeffs0.size() == new Size(1, 4) ||
           distCoeffs0.size() == new Size(1, 5) ||
           distCoeffs0.size() == new Size(4, 1) ||
           distCoeffs0.size() == new Size(5, 1))
        {
            Mat dstCoeffs = new Mat(distCoeffs, new OpenCVForUnity.Rect(0, 0, distCoeffs0.cols(), distCoeffs0.rows()));
            distCoeffs0.convertTo(dstCoeffs, rtype);
        }
        return distCoeffs;
    }


    //List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, 
    //Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria)

    double _calibrateCamera(List<Mat> objectPoints, List<Mat> imagePoints,
                              Size imageSize, Mat cameraMatrix, Mat distCoeffs,
                              List<Mat> rvecs, List<Mat> tvecs, int flags)
    {
        int rtype = CvType.CV_64F;
        cameraMatrix = _prepareCameraMatrix(cameraMatrix, rtype);
        distCoeffs = _prepareDistCoeffs(distCoeffs, rtype);

        int i, nimages = objectPoints.Count;
        //CV_Assert(nimages > 0 );
        Mat objPt = new Mat();
        Mat imgPt = new Mat();
        Mat npoints = new Mat();
        Mat rvecM = new Mat((int)nimages, 3, CvType.CV_64FC1);
        Mat tvecM = new Mat((int)nimages, 3, CvType.CV_64FC1);

        _collectCalibrationData(objectPoints, imagePoints, new List<Mat>(),
                                ref objPt, ref imgPt, null, ref npoints);

        Mat _objPt = objPt, _imgPt = imgPt, _npoints = npoints;
        Mat _cameraMatrix = cameraMatrix, _distCoeffs = distCoeffs;
        Mat _rvecM = rvecM, _tvecM = tvecM;

        double reprojErr = CalibrateCamera2(_objPt, _imgPt, _npoints, imageSize, _cameraMatrix, _distCoeffs, _rvecM, _tvecM, flags);
        //rvecs.resize(nimages);
        //tvecs.resize(nimages);
        //for( i = 0; i<nimages; i++ )
        //{
        //    rvecM.row((int)i).copyTo(rvecs[i]);
        //tvecM.row((int)i).copyTo(tvecs[i]);
        //        }
        return reprojErr;
    }

    void _collectCalibrationData(List<Mat> objectPoints, List<Mat> imagePoints,
                                 List<Mat> imagePoints2, ref Mat objPtMat, ref Mat imgPtMat,
                                 Mat imgPtMat2, ref Mat npoints)
    {
        int i, j = 0, nimages = objectPoints.Count;
        //CV_Assert(nimages > 0 && nimages == imagePoints.size() &&
        //    (!imgPtMat2 || nimages == imagePoints2.size()));
        int temp1 = PlayerPrefs.GetInt("HeightNum");
        int temp2 = PlayerPrefs.GetInt("WidthNum");
        int ni = temp1 * temp2;
        int total = ni * nimages;

        npoints = new Mat(1, (int)nimages, CvType.CV_32S);
        objPtMat = new Mat(1, (int)total, CvType.CV_32FC3);
        imgPtMat = new Mat(1, (int)total, CvType.CV_32FC2);
        Point imgPtData2 = new Point();

        //if (imgPtMat2 == null)
        //{
        //    imgPtMat2 = new Mat(1, (int)total, CvType.CV_32FC2);
        //    //     imgPtData2 = imgPtMat2->ptr<Point2f>();
        //}

        //Point3f* objPtData = objPtMat.ptr<Point3f>();
        //Point2f* imgPtData = imgPtMat.ptr<Point2f>();

        for (i = 0; i < nimages; i++, j += ni)
        {
            for (int k1 = 0; k1 < temp1; ++k1)
                for (int k2 = 0; k2 < temp2; ++k2)
                {
                    objPtMat.put(0, j + k1 * temp2 + k2, objectPoints[i].get(k1 * temp2 + k2, 0));
                    imgPtMat.put(0, j + k1 * temp2 + k2, imagePoints[i].get(k1 * temp2 + k2, 0));
                    npoints.put(0, i, (temp1 * temp2));
                    //    }
                    //        ni = temp;// objectPoints[i].size();
                    //((int*)npoints.data)[i] = (int)ni;
                    //std::copy(objectPoints[i].begin(), objectPoints[i].end(), objPtData + j);
                    //std::copy(imagePoints[i].begin(), imagePoints[i].end(), imgPtData + j);

                    //if (imgPtMat2)
                    //    std::copy(imagePoints2[i].begin(), imagePoints2[i].end(), imgPtData2 + j);
                }
        }
    }



    double CalibrateCamera2(Mat objectPoints, Mat imagePoints, Mat npoints,
                            Size imageSize, Mat cameraMatrix, Mat distCoeffs,
                            Mat rvecs, Mat tvecs, int flags)
    {
        const int NINTRINSIC = 9;
        Mat matM, _m, _Ji, _Je, _err;
        LevMarq solver = new LevMarq();
        double reprojErr = 0;

        //double[] A = new double[9];
        //double[] k = { 0, 0, 0, 0, 0 };
        Mat matA = new Mat(3, 3, CvType.CV_64F);
        Mat _k = new Mat();

        int i, nimages, maxPoints = 0, ni = 0, pos, total = 0, nparams, npstep, cn;
        double aspectRatio = 0.0;

        //// 0. check the parameters & allocate buffers
        //if (!CV_IS_MAT(objectPoints) || !CV_IS_MAT(imagePoints) ||
        //    !CV_IS_MAT(npoints) || !CV_IS_MAT(cameraMatrix) || !CV_IS_MAT(distCoeffs))
        //    CV_Error(CV_StsBadArg, "One of required vector arguments is not a valid matrix");

        if (imageSize.width <= 0 || imageSize.height <= 0)
            throw new CvException("image width and height must be positive");

        if (npoints.type() != CvType.CV_32SC1 ||
            (npoints.rows() != 1 && npoints.cols() != 1))
            throw new CvException("the array of point counters must be 1-dimensional integer vector");

        nimages = npoints.rows() * npoints.cols();
        npstep = 1;

        //if (rvecs)
        //{
        //    cn = CV_MAT_CN(rvecs->type);
        //    if (!CV_IS_MAT(rvecs) ||
        //        (CV_MAT_DEPTH(rvecs->type) != CV_32F && CV_MAT_DEPTH(rvecs->type) != CV_64F) ||
        //        ((rvecs->rows != nimages || (rvecs->cols * cn != 3 && rvecs->cols * cn != 9)) &&
        //        (rvecs->rows != 1 || rvecs->cols != nimages || cn != 3)))
        //        CV_Error(CV_StsBadArg, "the output array of rotation vectors must be 3-channel "
        //            "1xn or nx1 array or 1-channel nx3 or nx9 array, where n is the number of views");
        //}

        //if (tvecs)
        //{
        //    cn = CV_MAT_CN(tvecs->type);
        //    if (!CV_IS_MAT(tvecs) ||
        //        (CV_MAT_DEPTH(tvecs->type) != CV_32F && CV_MAT_DEPTH(tvecs->type) != CV_64F) ||
        //        ((tvecs->rows != nimages || tvecs->cols * cn != 3) &&
        //        (tvecs->rows != 1 || tvecs->cols != nimages || cn != 3)))
        //        CV_Error(CV_StsBadArg, "the output array of translation vectors must be 3-channel "
        //            "1xn or nx1 array or 1-channel nx3 array, where n is the number of views");
        //}

        //if ((CV_MAT_TYPE(cameraMatrix->type) != CV_32FC1 &&
        //    CV_MAT_TYPE(cameraMatrix->type) != CV_64FC1) ||
        //    cameraMatrix->rows != 3 || cameraMatrix->cols != 3)
        //    CV_Error(CV_StsBadArg,
        //        "Intrinsic parameters must be 3x3 floating-point matrix");

        //if ((CV_MAT_TYPE(distCoeffs->type) != CV_32FC1 &&
        //    CV_MAT_TYPE(distCoeffs->type) != CV_64FC1) ||
        //    (distCoeffs->cols != 1 && distCoeffs->rows != 1) ||
        //    (distCoeffs->cols * distCoeffs->rows != 4 &&
        //    distCoeffs->cols * distCoeffs->rows != 5))
        //    CV_Error(CV_StsBadArg,
        //        "Distortion coefficients must be 4x1, 1x4, 5x1 or 1x5 floating-point matrix");

        //for (i = 0; i < nimages; i++)
        //{
        //    ni = npoints.get(0, i)
        //    ni = npoints->data.i[i * npstep];
        //    if (ni < 4)
        //    {
        //        char buf[100];
        //        sprintf(buf, "The number of points in the view #%d is < 4", i);
        //        CV_Error(CV_StsOutOfRange, buf);
        //    }
        //}

        int temp1 = PlayerPrefs.GetInt("HeightNum");
        int temp2 = PlayerPrefs.GetInt("WidthNum");
        maxPoints = temp1 * temp2;
        total = maxPoints * nimages;

        matM = new Mat(1, total, CvType.CV_64FC3);
        _m = new Mat(1, total, CvType.CV_64FC2);

        //cvConvertPointsHomogeneous(objectPoints, matM);
        //cvConvertPointsHomogeneous(imagePoints, _m);

        Calib3d.convertPointsToHomogeneous(objectPoints, matM);      //TODO: CHECK
        Calib3d.convertPointsToHomogeneous(imagePoints, _m);

        matM = _from4ChTo3Ch(matM);
        _m = _from3ChTo2Ch(_m);

        nparams = NINTRINSIC + nimages * 6;
        _Ji = Mat.zeros(maxPoints * 2, NINTRINSIC, CvType.CV_64FC1);
        _Je = new Mat(maxPoints * 2, 6, CvType.CV_64FC1);
        _err = new Mat(maxPoints * 2, 1, CvType.CV_64FC1);

        //cvZero(_Ji);

        _k = new Mat(distCoeffs.rows(), distCoeffs.cols(), CvType.makeType(CvType.CV_64F, CvType.channels(distCoeffs.type())));
        //_k = cvMat( distCoeffs->rows, distCoeffs->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(distCoeffs->type)), k);

        if (distCoeffs.rows() * distCoeffs.cols() * CvType.channels(distCoeffs.type()) == 4)
            flags |= 128;  // #define CV_CALIB_FIX_K3  128

        // 1. initialize intrinsic parameters & LM solver
        //#define CV_CALIB_USE_INTRINSIC_GUESS  1
        if ((flags & 1) != 0)
        {
            Core.convertScaleAbs(cameraMatrix, matA, 1, 0); //cvConvert(cameraMatrix, &matA);

            if (matA.get(0,0)[0] <= 0 || matA.get(1, 1)[0] <= 0)
                throw new CvException("Focal length (fx and fy) must be positive");
            if (matA.get(0, 2)[0] < 0 || matA.get(0, 2)[0] >= imageSize.width ||
                matA.get(1, 2)[0] < 0 || matA.get(1, 2)[0] >= imageSize.height)
                throw new CvException("Principal point must be within the image");

            if (Math.Abs(matA.get(0,1)[0]) > 1e-5)
                throw new CvException("Non-zero skew is not supported by the function");

            if (Math.Abs(matA.get(1, 0)[0]) > 1e-5 || Math.Abs(matA.get(2, 0)[0]) > 1e-5 ||
                Math.Abs(matA.get(2, 1)[0]) > 1e-5 || Math.Abs(matA.get(2, 2)[0] - 1) > 1e-5)
                throw new CvException("The intrinsic matrix must have [fx 0 cx; 0 fy cy; 0 0 1] shape");
            matA.put(0, 1, 0.0);
            matA.put(1, 1, 0.0);
            matA.put(2, 1, 0.0);
            matA.put(2, 2, 0.0);
            matA.put(2, 3, 0.0);
            //A[1] = A[3] = A[6] = A[7] = 0.0;
            //A[8] = 1.0;

            if ((flags & 2) != 0) // #define CV_CALIB_FIX_ASPECT_RATIO     2
                aspectRatio = matA.get(0,0)[0] / matA.get(1,1)[0];
            Core.convertScaleAbs(distCoeffs, _k, 1, 0);  //cvConvert(distCoeffs, &_k);
        }
        else
        {
            MatOfDouble mean = new MatOfDouble();
            MatOfDouble sdv = new MatOfDouble();
            Core.meanStdDev(matM, mean, sdv); //cvAvgSdv(matM, &mean, &sdv);

            if (Math.Abs(mean.get(2, 0)[0]) > 1e-5 || Math.Abs(sdv.get(2, 0)[0]) > 1e-5)
                throw new CvException(
                "For non-planar calibration rigs the initial intrinsic matrix must be specified");

            for (i = 0; i < total; i++)
            {
                //((CvPoint3D64f*)matM->data.db)[i].z = 0.0;
                double[] t = matM.get(0, i);
                t[2] = 0.0;
                matM.put(0, i, t);
            }

            if ((flags & 2) != 0)  //#define CV_CALIB_FIX_ASPECT_RATIO     2
            {
                aspectRatio = cameraMatrix.get(0, 0)[0]; //aspectRatio = cvmGet(cameraMatrix, 0, 0);
                aspectRatio /= cameraMatrix.get(1, 1)[0];//aspectRatio /= cvmGet(cameraMatrix, 1, 1);

                if (aspectRatio < 0.01 || aspectRatio > 100)
                    throw new CvException(
                        "The specified aspect ratio (=A[0][0]/A[1][1]) is incorrect");
            }


            //List<MatOfPoint2f> temp2f = new List<MatOfPoint2f>();
            //Converters.Mat_to_vector_vector_Point2f(_m, temp2f);

            //Mat temp12 = _from3ChTo2Ch(matM);
            //List<MatOfPoint3f> temp3f = new List<MatOfPoint3f>();
            //Converters.Mat_to_vector_vector_Point3f(temp12, temp3f);

            _InitIntrinsicParams2D(matM, _m, npoints, imageSize, ref matA, aspectRatio);
            //matA = Calib3d.initCameraMatrix2D(matM, _m, imageSize, aspectRatio);      //FIX

            //cvInitIntrinsicParams2D(matM, _m, npoints, imageSize, &matA, aspectRatio);

        }

        solver.init(nparams, 0, new TermCriteria(3, 30, double.Epsilon), false); //CV_TERMCRIT_ITER + CV_TERMCRIT_EPS 1 + 2

        {

            //double* param = solver.param->data.db;
            //uchar* mask = solver.mask->data.ptr;

            solver.param.put(0, 0, matA.get(0, 0)[0]);
            solver.param.put(0, 1, matA.get(1, 1)[0]);
            solver.param.put(0, 2, matA.get(0, 2)[0]);
            solver.param.put(0, 3, matA.get(1, 2)[0]);
            solver.param.put(0, 4, _k.get(0, 0)[0]);
            solver.param.put(0, 5, _k.get(1, 0)[0]);
            solver.param.put(0, 6, _k.get(2, 0)[0]);
            solver.param.put(0, 7, _k.get(3, 0)[0]);
            solver.param.put(0, 8, _k.get(4, 0)[0]);

            if ((flags & 16) != 0)  //#define CV_CALIB_FIX_FOCAL_LENGTH 16
            {
                solver.mask.put(0, 0, 0);
                solver.mask.put(0, 1, 0);
            }

            if ((flags & 4) != 0) //#define CV_CALIB_FIX_PRINCIPAL_POINT  4
            {
                solver.mask.put(0, 2, 0);
                solver.mask.put(0, 3, 0);
            }

            if ((flags & 8) != 0) //#define CV_CALIB_ZERO_TANGENT_DIST    8
            {
                solver.param.put(0, 6, 0);
                solver.param.put(0, 7, 0);
                solver.mask.put(0, 6, 0);
                solver.mask.put(0, 7, 0);
            }

            //#define CV_CALIB_FIX_K1  32
            //#define CV_CALIB_FIX_K2  64
            //#define CV_CALIB_FIX_K3  128
            if ((flags & 32) != 0)
                solver.mask.put(0, 4, 0);
            if ((flags & 64) != 0)
                solver.mask.put(0, 5, 0);
            if ((flags & 128) != 0)
                solver.mask.put(0, 8, 0);
        }

        // 2. initialize extrinsic parameters
        for (i = 0, pos = 0; i < nimages; i++, pos += ni)
        {
            Mat _Mi, _mi, _ri, _ti;
            ni = temp1 * temp2;

            _ri = solver.param.rowRange(NINTRINSIC + i * 6, NINTRINSIC + i * 6 + 3);
            _ti = solver.param.rowRange(NINTRINSIC + i * 6 + 3, NINTRINSIC + i * 6 + 6);

            //cvGetRows(solver.param, &_ri, NINTRINSIC + i * 6, NINTRINSIC + i * 6 + 3);
            //cvGetRows(solver.param, &_ti, NINTRINSIC + i * 6 + 3, NINTRINSIC + i * 6 + 6);


            _Mi = matM.colRange(pos, pos + ni);
            _mi = _m.colRange(pos, pos + ni);

            //cvGetCols(matM, &_Mi, pos, pos + ni);
            //cvGetCols(_m, &_mi, pos, pos + ni);

            MatOfPoint3f _Mitemp1 = new MatOfPoint3f(_Mi);
            MatOfPoint2f _mitemp1 = new MatOfPoint2f(_mi);
            MatOfDouble _ktemp1 = new MatOfDouble(_k);
            Calib3d.solvePnP(_Mitemp1, _mitemp1, matA, _ktemp1, _ri, _ti);
            _mi = _mitemp1;
            _k = _ktemp1;
            //cvFindExtrinsicCameraParams2(&_Mi, &_mi, &matA, &_k, &_ri, &_ti);

        }

        // 3. run the optimization
        for (;;)
        {
            Mat _param = new Mat();
            //const CvMat* _param = 0;
            Mat _JtJ = new Mat();
            Mat _JtErr = new Mat();
            //CvMat* _JtJ = 0, *_JtErr = 0;
            double _errNorm = 0;
            //double* _errNorm = 0;
            bool proceed = solver.updateAlt(_param, _JtJ, _JtErr, _errNorm);

            //double* param = solver.param->data.db, *pparam = solver.prevParam->data.db;


            if ((flags & 2) != 0) // #define CV_CALIB_FIX_ASPECT_RATIO     2
            {
                solver.param.put(0, 0, solver.param.get(1, 0)[0] * aspectRatio);
                //param[0] = param[1] * aspectRatio;
                solver.prevParam.put(0, 0, solver.prevParam.get(1, 0)[0] * aspectRatio);
                //pparam[0] = pparam[1] * aspectRatio;
            }
            matA.put(0, 0, solver.param.get(0, 0)[0]);
            matA.put(0, 2, solver.param.get(2, 0)[0]);
            matA.put(1, 1, solver.param.get(1, 0)[0]);
            matA.put(1, 2, solver.param.get(3, 0)[0]);

            _k.put(0, 0, solver.param.get(4, 0)[0]);
            _k.put(1, 0, solver.param.get(5, 0)[0]);
            _k.put(2, 0, solver.param.get(6, 0)[0]);
            _k.put(3, 0, solver.param.get(7, 0)[0]);
            _k.put(4, 0, solver.param.get(8, 0)[0]);

            //A[0] = solver.param.get(0, 0)[0]; A[4] = solver.param.get(0, 1)[0];
            //A[2] = solver.param.get(0, 2)[0]; A[5] = solver.param.get(0, 3)[0];
            //k[0] = solver.param.get(0, 4)[0]; k[1] = solver.param.get(0, 5)[0];
            //k[2] = solver.param.get(0, 6)[0];
            //k[3] = solver.param.get(0, 7)[0];
            //k[4] = solver.param.get(0, 8)[0];

            if (!proceed)
                break;

            reprojErr = 0;

            for (i = 0, pos = 0; i < nimages; i++, pos += ni)
            {   //CvMat
                Mat _Mi, _mi, _ri, _ti, _dpdr, _dpdt, _dpdf, _dpdc, _dpdk, _mp, _part;

                ni = temp1 * temp2;


                _ri = solver.param.rowRange(NINTRINSIC + i * 6, NINTRINSIC + i * 6 + 3);

                _ti = solver.param.rowRange(NINTRINSIC + i * 6 + 3, NINTRINSIC + i * 6 + 6);

                //cvGetRows(solver.param, &_ri, NINTRINSIC + i * 6, NINTRINSIC + i * 6 + 3);
                //cvGetRows(solver.param, &_ti, NINTRINSIC + i * 6 + 3, NINTRINSIC + i * 6 + 6);


                _Mi = matM.colRange(pos, pos + ni);
                _mi = _m.colRange(pos, pos + ni);
                //cvGetCols(matM, &_Mi, pos, pos + ni);
                //cvGetCols(_m, &_mi, pos, pos + ni);


                Mat tempMat = new Mat(ni * 2, _Je.rows(), _Je.type(), new Scalar(0));
                _substr(_Je, ref tempMat, 0, 0, _Je.cols(), _Je.rows());
                tempMat.copyTo(_Je);

                tempMat = new Mat(ni * 2, _Ji.rows(), _Ji.type(), new Scalar(0));
                _substr(_Ji, ref tempMat, 0, 0, _Ji.cols(), _Ji.rows());
                tempMat.copyTo(_Ji);

                tempMat = new Mat(ni * 2, _err.rows(), _err.type(), new Scalar(0));
                _substr(_err, ref tempMat, 0, 0, _err.cols(), _err.rows());
                tempMat.copyTo(_err);

                //_Je->rows = _Ji->rows = _err->rows = ni * 2;


                _dpdr = _Je.colRange(0, 3);
                _dpdt = _Je.colRange(3, 6);
                _dpdf = _Ji.colRange(0, 2);
                _dpdc = _Ji.colRange(2, 4);
                _dpdk = _Ji.colRange(4, NINTRINSIC);
                //cvGetCols(_Je, &_dpdr, 0, 3);
                //cvGetCols(_Je, &_dpdt, 3, 6);
                //cvGetCols(_Ji, &_dpdf, 0, 2);
                //cvGetCols(_Ji, &_dpdc, 2, 4);
                //cvGetCols(_Ji, &_dpdk, 4, NINTRINSIC);

                _mp = _err.reshape(2, 1);
                //cvReshape(_err, &_mp, 2, 1);


                MatOfPoint3f _Mitemp = new MatOfPoint3f(_Mi);
                MatOfDouble _ktemp = new MatOfDouble(_k);

                List<Point> _mptemp1 = new List<Point>();

                for (int q = 0; q < _mp.cols(); ++q)
                {
                    _mptemp1.Add(new Point(_mp.get(0, q)[0], _mp.get(0, q)[1]));
                }

                MatOfPoint2f temp123 = new MatOfPoint2f();
                temp123.fromList(_mptemp1);


                Mat jac = new Mat();
                if (_JtJ != null || _JtErr != null)
                {
                    Calib3d.projectPoints(_Mitemp, _ri, _ti, matA, _ktemp, temp123, jac,
                                      ((flags & 2) != 0) ? aspectRatio : 0);
                    _k = _ktemp;
                    _mp = temp123;

                    //#define CV_CALIB_FIX_FOCAL_LENGTH 16
                    //#define CV_CALIB_FIX_PRINCIPAL_POINT  4
                    // #define CV_CALIB_FIX_ASPECT_RATIO     2

                    //cvProjectPoints2(&_Mi, &_ri, &_ti, &matA, &_k, &_mp, &_dpdr, &_dpdt,
                    //                  (flags & CV_CALIB_FIX_FOCAL_LENGTH) ? 0 : &_dpdf,
                    //                  (flags & CV_CALIB_FIX_PRINCIPAL_POINT) ? 0 : &_dpdc, &_dpdk,
                    //                  (flags & CV_CALIB_FIX_ASPECT_RATIO) ? aspectRatio : 0);
                }
                else
                {
                    Calib3d.projectPoints(_Mitemp, _ri, _ti, matA, _ktemp, temp123);
                    _k = _ktemp;
                    _mp = temp123;
                }

                //cvProjectPoints2(&_Mi, &_ri, &_ti, &matA, &_k, &_mp);

                Core.subtract(_mp, _mi, _mp);

                //cvSub(&_mp, &_mi, &_mp);

                if (_JtJ != null || _JtErr != null)
                {

                    int q = 0;
                    q = 4 + 5;
                    //_substr(_JtJ, ref _part, 0, 0, NINTRINSIC, NINTRINSIC);
                    _part = new Mat(_JtJ, (new OpenCVForUnity.Rect(0, 0, NINTRINSIC, NINTRINSIC)));
                    
                    q = 15 + 5;
                    Core.gemm(_Ji, _Ji, 1, _part, 1, _part, 1); //#define CV_GEMM_A_T 1
                    //cvGetSubRect(_JtJ, &_part, cvRect(0, 0, NINTRINSIC, NINTRINSIC));
                    //cvGEMM(_Ji, _Ji, 1, &_part, 1, &_part, CV_GEMM_A_T);


                    //_substr(_JtJ, ref _part, NINTRINSIC + i * 6, NINTRINSIC + i * 6, 6, 6);
                    _part = new Mat(_JtJ, (new OpenCVForUnity.Rect(NINTRINSIC + i * 6, NINTRINSIC + i * 6, 6, 6)));
                    Core.gemm(_Je, _Je, 1, new Mat(), 0, _part, 1);

                    //cvGetSubRect(_JtJ, &_part, cvRect(NINTRINSIC + i * 6, NINTRINSIC + i * 6, 6, 6));
                    //cvGEMM(_Je, _Je, 1, 0, 0, &_part, CV_GEMM_A_T);


                    //_substr(_JtJ, ref _part, NINTRINSIC + i * 6, 0, 6, NINTRINSIC);
                    _part = new Mat(_JtJ, (new OpenCVForUnity.Rect(NINTRINSIC + i * 6, 0, 6, NINTRINSIC)));
                    Core.gemm(_Ji, _Je, 1, new Mat(), 0, _part, 1);

                    //cvGetSubRect(_JtJ, &_part, cvRect(NINTRINSIC + i * 6, 0, 6, NINTRINSIC));
                    //cvGEMM(_Ji, _Je, 1, 0, 0, &_part, CV_GEMM_A_T);

                    _rowRange(_JtErr, ref _part, 0, NINTRINSIC);
                    //_part = _JtErr.rowRange(0, NINTRINSIC);
                    Core.gemm(_Ji, _err, 1, _part, 1, _part, 1);

                    //cvGetRows(_JtErr, &_part, 0, NINTRINSIC);
                    //cvGEMM(_Ji, _err, 1, &_part, 1, &_part, CV_GEMM_A_T);

                    _rowRange(_JtErr, ref _part, NINTRINSIC + i * 6, NINTRINSIC + (i + 1) * 6);
                    // _part = _JtErr.rowRange(NINTRINSIC + i * 6, NINTRINSIC + (i + 1) * 6);
                    Core.gemm(_Je, _err, 1, new Mat(), 0, _part, 1);

                    //cvGetRows(_JtErr, &_part, NINTRINSIC + i * 6, NINTRINSIC + (i + 1) * 6);
                    //cvGEMM(_Je, _err, 1, 0, 0, &_part, CV_GEMM_A_T);
                }


                double errNorm = Core.norm(_mp, 4);
                //double errNorm = cvNorm(&_mp, 0, CV_L2);
                reprojErr += errNorm * errNorm;
            }

            //if (_errNorm != 0)
            _errNorm = reprojErr;
        }



        // 4. store the results
        matA.convertTo(cameraMatrix, matA.type());
        //cvConvert(&matA, cameraMatrix);
        _k.convertTo(distCoeffs, _k.type());
        //cvConvert(&_k, distCoeffs);

        //for (i = 0; i < nimages; i++)
        //{
        //    CvMat src, dst;
        //    if (rvecs)
        //    {
        //        src = cvMat(3, 1, CV_64F, solver.param->data.db + NINTRINSIC + i * 6);
        //        if (rvecs->rows == nimages && rvecs->cols * CV_MAT_CN(rvecs->type) == 9)
        //        {
        //            dst = cvMat(3, 3, CV_MAT_DEPTH(rvecs->type),
        //                rvecs->data.ptr + rvecs->step * i);
        //            cvRodrigues2(&src, &matA);
        //            cvConvert(&matA, &dst);
        //        }
        //        else
        //        {
        //            dst = cvMat(3, 1, CV_MAT_DEPTH(rvecs->type), rvecs->rows == 1 ?
        //                rvecs->data.ptr + i * CV_ELEM_SIZE(rvecs->type) :
        //                rvecs->data.ptr + rvecs->step * i);
        //            cvConvert(&src, &dst);
        //        }
        //    }
        //    if (tvecs)
        //    {
        //        src = cvMat(3, 1, CV_64F, solver.param->data.db + NINTRINSIC + i * 6 + 3);
        //        dst = cvMat(3, 1, CV_MAT_TYPE(tvecs->type), tvecs->rows == 1 ?
        //                            tvecs->data.ptr + i * CV_ELEM_SIZE(tvecs->type) :
        //                            tvecs->data.ptr + tvecs->step * i);
        //        cvConvert(&src, &dst);
        //    }
        //}

        return reprojErr;
    }


    private void _InitIntrinsicParams2D(Mat objectPoints, Mat imagePoints, Mat npoints,
                          Size imageSize, ref Mat cameraMatrix,
                          double aspectRatio)
    {
        Mat matA, _b, _allH, _allK;

        int i, j, pos, nimages, total, ni = 0;
        //double[] a = { 0, 0, 0, 0, 0, 0, 0, 0, 1 };
        //double[] H;
        //double[] f;

        Mat _a = Mat.zeros(3, 3, CvType.CV_64F); // a
        _a.put(2, 2, 1);
        Mat matH = new Mat(3, 3, CvType.CV_64F); // H
        Mat _f = new Mat(2, 1, CvType.CV_64F); // f


        //assert(CV_MAT_TYPE(npoints->type) == CV_32SC1 &&
        //        CV_IS_MAT_CONT(npoints->type));

        nimages = npoints.rows() + npoints.cols() - 1;

        //if ((CV_MAT_TYPE(objectPoints->type) != CV_32FC3 &&
        //    CV_MAT_TYPE(objectPoints->type) != CV_64FC3) ||
        //    (CV_MAT_TYPE(imagePoints->type) != CV_32FC2 &&
        //    CV_MAT_TYPE(imagePoints->type) != CV_64FC2))
        //    CV_Error(CV_StsUnsupportedFormat, "Both object points and image points must be 2D");

        if (objectPoints.rows() != 1 || imagePoints.rows() != 1)
            throw new CvException("object points and image points must be a single-row matrices");

        matA = new Mat(2 * nimages, 2, CvType.CV_64F);
        _b = new Mat(2 * nimages, 1, CvType.CV_64F);
        //a[2] = (imageSize.width - 1) * 0.5;
        _a.put(0, 2, (imageSize.width - 1) * 0.5);

        //a[5] = (imageSize.height - 1) * 0.5;
        _a.put(1, 2, (imageSize.height - 1) * 0.5);

        _allH = new Mat(nimages, 9, CvType.CV_64F);

        total = (int)Math.Round(Core.sumElems(npoints).val[0]);


        // extract vanishing points in order to obtain initial value for the focal length
        for (i = 0, pos = 0; i < nimages; i++, pos += ni)
        {

            //double* Ap = matA->data.db + i * 4;
            matA.get(i * 2, 0);
            //double* bp = _b->data.db + i * 2;
            _b.get(i * 2, 0);
            ni = npoints.cols() * npoints.rows();

            double[] h = new double[3];
            double[] v = new double[3];
            double[] d1 = new double[3];
            double[] d2 = new double[3];
            double[] n = { 0, 0, 0, 0 };
            Mat _m, matM;

            matM = objectPoints.colRange(pos, pos + ni);
            _m = imagePoints.colRange(pos, pos + ni);
            //cvGetCols(objectPoints, &matM, pos, pos + ni);
            //cvGetCols(imagePoints, &_m, pos, pos + ni);
            List<Point> tempAr1 = new List<Point>();
            List<Point> tempAr2 = new List<Point>();

            for (int q = 0; q < 25; ++q) {
                tempAr1.Add(new Point(matM.get(0, q)[0], matM.get(0, q)[1]));
                tempAr2.Add(new Point(_m.get(0, q)[0], _m.get(0, q)[1]));
                //tempAr1[q].x = matM.get(0, q)[0];
                //tempAr1[q].y = matM.get(0, q)[1];
                //tempAr2[q].x = _m.get(0, q)[0];
                //tempAr2[q].y = _m.get(0, q)[1];
            }
            MatOfPoint2f temp123 = new MatOfPoint2f();
            temp123.fromList(tempAr1);

            MatOfPoint2f temp1234 = new MatOfPoint2f();
            temp1234.fromList(tempAr2);

            matH = Calib3d.findHomography(temp123, temp1234);


            //cvFindHomography(&matM, &_m, &matH);
            for (int q1 = 0; q1 < 2; ++q1)
                for (int q2 = 0; q2 < 2; ++q2)
                _allH.put(i, q1*3 + q2, matH.get(q1, q2)[0]);

            //memcpy(_allH->data.db + i * 9, H, sizeof(H));


            matH.put(0, 0, matH.get(0, 0)[0] - (matH.get(2, 0)[0] * _a.get(0, 2)[0]));
            matH.put(0, 1, matH.get(0, 1)[0] - (matH.get(2, 1)[0] * _a.get(0, 2)[0]));
            matH.put(0, 2, matH.get(0, 2)[0] - (matH.get(2, 2)[0] * _a.get(0, 2)[0]));
            matH.put(1, 0, matH.get(1, 0)[0] - (matH.get(2, 0)[0] * _a.get(1, 2)[0]));
            matH.put(1, 1, matH.get(1, 1)[0] - (matH.get(2, 1)[0] * _a.get(1, 2)[0]));
            matH.put(1, 2, matH.get(1, 2)[0] - (matH.get(2, 2)[0] * _a.get(1, 2)[0]));       
                 
            //H[0] -= H[6] * _a.get(0, 2)[0];
            //H[1] -= H[7] * _a.get(0, 2)[0];
            //H[2] -= H[8] * _a.get(0, 2)[0];
            //H[3] -= H[6] * _a.get(1, 2)[0];
            //H[4] -= H[7] * _a.get(1, 2)[0];
            //H[5] -= H[8] * _a.get(1, 2)[0];



            for (j = 0; j < 3; j++)
            {
                double t0 = matH.get(j, 0)[0];  // H[j * 3];
                double t1 = matH.get(j, 1)[0];  // H[j * 3 + 1];
                h[j] = t0; v[j] = t1;
                d1[j] = (t0 + t1) * 0.5;
                d2[j] = (t0 - t1) * 0.5;
                n[0] += t0 * t0; n[1] += t1 * t1;
                n[2] += d1[j] * d1[j];
                n[3] += d2[j] * d2[j];
            }



            for (j = 0; j < 4; j++)
                n[j] = 1.0/ Math.Sqrt(n[j]);

            for (j = 0; j < 3; j++)
            {
                h[j] *= n[0]; v[j] *= n[1];
                d1[j] *= n[2]; d2[j] *= n[3];
            }

            matA.get(i * 2, 0)[0] = h[0] * v[0];
            matA.get(i * 2, 1)[0] = h[1] * v[1];
            matA.get(i * 2 + 1, 0)[0] = d1[0] * d2[0];
            matA.get(i * 2 + 1, 1)[0] = d1[1] * d2[1];
            _b.get(i * 2, 0)[0] = -h[2] * v[2];
            _b.get(i * 2 + 1, 0)[0] = -d1[2] * d2[2];
        }


        Core.solve(matA, _b, _f, 17);
        //cvSolve(matA, _b, &_f, CV_NORMAL + CV_SVD); //CV_NORMAL + CV_SVD 16 + 1

        _a.put(0, 0, Math.Sqrt(Math.Abs(1.0 / _f.get(0, 0)[0])));
        //a[0] = Math.Sqrt(Math.Abs(1.0/ f[0]));
        _a.put(1, 1, Math.Sqrt(Math.Abs(1.0 / _f.get(1, 0)[0])));
        //a[4] = Math.Sqrt(Math.Abs(1.0/ f[1]));

        if (aspectRatio != 0)
        {
            double tf = (_a.get(0,0)[0] + _a.get(1,1)[0]) / (aspectRatio + 1.0);
            _a.put(0,0, aspectRatio * tf);
            _a.put(1,1, tf);
        }
        _a.convertTo(cameraMatrix, _a.type());
        //cvConvert(&_a, cameraMatrix);
    }




    private void _colRange(Mat dst, ref Mat src, int from, int to)
    {
        for (int i = from; i <= to; ++i)
        {
            for (int j = 0; j < src.rows(); ++j)
            {
                src.put(j, i, dst.get(j, i)[0]);
            }
        }
    }
    private void _rowRange(Mat dst, ref Mat src, int from, int to)
    {
        for (int i = from; i <= to; ++i)
        {
            for (int j = 0; j < src.rows(); ++j)
            {
                src.put(i, j, dst.get(i, j)[0]);
            }
        }
    }
    private void _substr(Mat dst, ref Mat src, int x, int y, int width, int height)
    {
        for (int i = x; i < x * height; ++i)
        {
            for (int j = y; j < y * width; ++j)
            {
                src.put(y, x, dst.get(y, x)[0]);
            }
        }
    }
    private Mat _from4ChTo3Ch(Mat dst)
    {
        Mat temp = new Mat(dst.cols(), dst.rows(), CvType.CV_32FC3);
        for (int i = 0; i < dst.rows(); ++i)
            for (int j = 0; j < dst.cols(); ++j)
            {
                double[] t = { dst.get(i, j)[0], dst.get(i, j)[1], dst.get(i, j)[2] };
                temp.put(j, i, t);
            }
        return temp;
    }
    private Mat _from3ChTo2Ch(Mat dst)
    {
        Mat temp = new Mat(dst.cols(), dst.rows(), CvType.CV_32FC2);
        for (int i = 0; i < dst.rows(); ++i)
            for (int j = 0; j < dst.cols(); ++j)
            {
                double[] t = { dst.get(i, j)[0], dst.get(i, j)[1] };
                temp.put(j, i, t);
            }
        return temp;
    }

}
