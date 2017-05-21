using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity;


public class LevMarq {

    enum State { DONE = 0, STARTED = 1, CALC_J = 2, CHECK_ERR = 3 };

    public  Mat mask;
    public  Mat prevParam;
    public  Mat param;
    private Mat J;
    private Mat err;
    private Mat JtJ;
    private Mat JtJN;
    private Mat JtErr;
    private Mat JtJV;
    private Mat JtJW;
    private double prevErrNorm, errNorm;
    private int lambdaLg10;
    private TermCriteria criteria;
    private State state;
    private int iters;
    private bool completeSymmFlag;
    private int solveMethod;

    public LevMarq()
    {
        mask = prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = new Mat();
        lambdaLg10 = 0; state = State.DONE;
        criteria = new TermCriteria(0,0,0);
        iters = 0;
        completeSymmFlag = false;
    }

    public LevMarq(int nparams, int nerrs, TermCriteria criteria0, bool _completeSymmFlag )
    {
        mask = prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = new Mat();
        init(nparams, nerrs, criteria0, _completeSymmFlag);
    }

    private void clear()
    {
        mask.release();
        prevParam.release();
        param.release();
        J.release();
        err.release();
        JtJ.release();
        JtJN.release();
        JtErr.release();
        JtJV.release();
        JtJW.release();
    }

    ~LevMarq()
    {
        clear();
    }

    public void init(int nparams, int nerrs, TermCriteria criteria0, bool _completeSymmFlag)
    {
        //if (!param || param->rows != nparams || nerrs != (err ? err->rows : 0))
        //    clear();
        mask = new Mat(nparams, 1, CvType.CV_8U);

        mask.setTo(new Scalar(1,1,1,1));
        //Set(mask, cvScalarAll(1));

        prevParam = new Mat(nparams, 1, CvType.CV_64F);
        param = new Mat(nparams, 1, CvType.CV_64F);
        JtJ = new Mat(nparams, nparams, CvType.CV_64F);
        JtJN = new Mat(nparams, nparams, CvType.CV_64F);
        JtJV = new Mat(nparams, nparams, CvType.CV_64F);
        JtJW = new Mat(nparams, 1, CvType.CV_64F);
        JtErr = new Mat(nparams, 1, CvType.CV_64F);

        if (nerrs > 0)
        {
            J = new Mat(nerrs, nparams, CvType.CV_64F);
            err = new Mat(nerrs, 1, CvType.CV_64F);
        }

        prevErrNorm = double.MaxValue; // DBL_MAX;
        lambdaLg10 = -3;
        criteria = criteria0;


        if ((criteria.type & 1) != 0) //#define CV_TERMCRIT_ITER    1
            criteria.maxCount = System.Math.Min(System.Math.Max(criteria.maxCount, 1), 1000);
        else
            criteria.maxCount = 30;

        if ((criteria.type & 2) != 0)  //#define CV_TERMCRIT_EPS     2 
            criteria.epsilon = System.Math.Max(criteria.epsilon, 0);
        else
            criteria.epsilon = double.Epsilon; //DBL_EPSILON;

        state = State.STARTED;
        iters = 0;
        completeSymmFlag = _completeSymmFlag;
    }

    public bool update(ref Mat _param, ref Mat matJ, ref Mat _err)
    {
        double change;

        //matJ = _err = new Mat();

    //assert( !err.empty() );
        if (state == State.DONE)
        {
            _param = param;
            return false;
        }

        if (state == State.STARTED)
        {
            _param = param;
            J = Mat.zeros(J.size(), J.type());       // cvZero(J );
            err =  Mat.zeros(err.size(), err.type()); //cvZero(err);
            matJ = J;
            _err = err;
            state = State.CALC_J;
            return true;
        }

        if( state == State.CALC_J )
        {
            Core.mulTransposed(J, JtJ, true);    //cvMulTransposed(J, JtJ, 1 );
            Core.gemm(J, err, 1, Mat.zeros(J.size(), J.type()), 0, JtErr, 1); // cvGEMM(J, err, 1, 0, 0, JtErr, CV_GEMM_A_T ); #define CV_GEMM_A_T 1

            prevParam.copyTo(param);   //cvCopy(param, prevParam );  ??????
            step();
            if (iters == 0)
                Core.norm(err, 4);//prevErrNorm = cvNorm(err, 0, CV_L2); #define CV_L2    4  ????
            _param = param;
            err = Mat.zeros(err.size(), err.type());   //cvZero(err );
            _err = err;
            state = State.CHECK_ERR;
            return true;
        }

       // assert(state == CHECK_ERR );
        errNorm = Core.norm(err, 4); //errNorm = cvNorm(err, 0, CV_L2); ??????????????
        if ( errNorm > prevErrNorm )
        {
            lambdaLg10++;
            step();
            _param = param;
            err = Mat.zeros(err.size(), err.type());//cvZero(err );
            _err = err;
            state = State.CHECK_ERR;
            return true;
        }

        lambdaLg10 = System.Math.Max(lambdaLg10-1, -16);
            //#define CV_RELATIVE_L2  (CV_RELATIVE | CV_L2)   8 | 4 = 12
        if ( ++iters >= criteria.maxCount ||
            (change = Core.norm(param, prevParam, 12)) < criteria.epsilon )
        {
            _param = param;
            state = State.DONE;
            return true;
        }

        prevErrNorm = errNorm;
        _param = param;
        J = Mat.zeros(J.size(), J.type());   // cvZero(J);
        matJ = J;
        _err = err;
        state = State.CALC_J;
        return true;
    }


    public void step()
    {
        double LOG10 = System.Math.Log(10.0);
        double lambda = System.Math.Exp(lambdaLg10 * LOG10);

        int i, j, nparams = param.rows();

        for (i = 0; i < nparams; i++)
            if (mask.get(i, 0)[0] == 0)  //
            {
                //double* row = JtJ->data.db + i * nparams;
                //double* col = JtJ->data.db + i;


                //for (j = 0; j < nparams; j++)
                //    row[j] = col[j * nparams] = 0;

                for (j = 0; j < nparams; ++j) {
                    JtJ.put(i, j, (0.0));
                    JtJ.put(j, i, (0.0));
                }
                JtErr.put(i, 0, (0));
                //JtErr->data.db[i] = 0;
            }

        if (err != null)
            Core.completeSymm(JtJ, completeSymmFlag);
        //cvCompleteSymm(JtJ, completeSymmFlag);

        JtJ = JtJN; //cvCopy( JtJ, JtJN );
        for( i = 0; i < nparams; i++ )
            //JtJN->data.db[(nparams+1)*i] *= 1.0 + lambda;
            JtJN.put(i, i, (JtJN.get(i,i)[0]) * (1.0 + lambda));

        Core.SVDecomp(JtJN, JtJW, Mat.zeros(JtJW.size(), JtJW.type()), JtJV, 7);

        //#define CV_SVD_MODIFY_A   1
        //#define CV_SVD_U_T        2
        //#define CV_SVD_V_T        4
        //cvSVD(JtJN, JtJW, 0, JtJV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);


        //cvSVBkSb(JtJW, JtJV, JtJV, JtErr, param, CV_SVD_U_T + CV_SVD_V_T);
        Core.SVBackSubst(JtJW, JtJV, JtJV, JtErr, param);
        
        for (i = 0; i < nparams; i++)
            //param->data.db[i] = prevParam->data.db[i] - (mask->data.ptr[i] ? param->data.db[i] : 0);
            if (mask.get(i, 0)[0] != 0)
                param.put(i, 0, prevParam.get(i, 0)[0] - param.get(i, 0)[0]);
            else
                param.put(i, 0, prevParam.get(i, 0)[0]);
    }


    public bool updateAlt(Mat _param, Mat _JtJ, Mat _JtErr, double _errNorm)
    {
        double change;

        //CV_Assert(err.empty() );
        if (state == State.DONE)
        {
            _param = param;
            return false;
        }

        if (state == State.STARTED)
        {
            _param = param;
            JtJ = Mat.zeros(JtJ.size(), JtJ.type());       // cvZero(JtJ );
            JtErr = Mat.zeros(JtErr.size(), JtErr.type());       // cvZero(JtErr);

            errNorm = 0;
            _JtJ = JtJ;
            _JtErr = JtErr;
            _errNorm = errNorm; //_errNorm = &errNorm;

            state = State.CALC_J;
            return true;
        }

        if (state == State.CALC_J)
        {
            prevParam.copyTo(param);//cvCopy(param, prevParam );
            step();
            _param = param;
            prevErrNorm = errNorm;
            errNorm = 0;
            _errNorm = errNorm;//_errNorm = &errNorm;    ?????????????????
            state = State.CHECK_ERR;
            return true;
        }

        //assert(state == CHECK_ERR );
        if (errNorm > prevErrNorm)
        {
            lambdaLg10++;
            step();
            _param = param;
            errNorm = 0;
            _errNorm = errNorm;//_errNorm = &errNorm;
            state = State.CHECK_ERR;
            return true;
        }

        lambdaLg10 = System.Math.Max(lambdaLg10-1, -16);

        if ( ++iters >= criteria.maxCount ||
            (change = Core.norm(param, prevParam, 12)) < criteria.epsilon)   //#define CV_RELATIVE_L2  (CV_RELATIVE | CV_L2)   8 | 4 = 12
        {
            _param = param;
            state = State.DONE;
            return false;
        }

        prevErrNorm = errNorm;
        JtJ = Mat.zeros(JtJ.size(), JtJ.type());       // cvZero(JtJ );
        JtErr = Mat.zeros(JtErr.size(), JtErr.type());       // cvZero(JtErr);
        _param = param;
        _JtJ = JtJ;
        _JtErr = JtErr;
        state = State.CALC_J;
        return true;
    }


}
