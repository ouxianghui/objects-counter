/******************************************************************
** 文件名: Fitting.cpp
** Copyright (c) 2013-2015 上海犀视信息科技有限公司技术开发部
** 创 建: 王宗跃、欧湘辉
** 日 期: 2015-11-15
** 修 改: 欧湘辉
** 邮 箱: ouxianghui@xitech.cn
** 日 期: 2015-11-15
** 描 述: 曲面拟合基类
**
** 版 本: v1.0.0
**-----------------------------------------------------------------

******************************************************************/

#include "Fitting.h"

#define EPSINON (10e-8)

//采用 Z = a + b*x + c*y + d*x*y + e*x*x + f*y*y 函数进行拟合

namespace cvpr {

	CFitting::CFitting()
	{
        memset(m_pCoefX, 0, sizeof(double) * 6);
	}

	CFitting::~CFitting()
	{
	}

	bool CFitting::fitting(const cv::Mat& src, cv::Mat& dst)
	{
		CV_DbgAssert(!src.empty());
        CV_DbgAssert(src.type() == CV_16UC1);

        if (src.empty() || src.type() != CV_16UC1)
		{
			return false;
		}

        memset(m_pCoefX, 0, sizeof(double) * 6);

        dst = cv::Mat::zeros(cv::Size(src.cols, src.rows), CV_16UC1);

		int* pX = NULL;
		int* pY = NULL;
        double* pZX = NULL;
		int nC = 0;

        int nr = src.rows; // number of rows
        int nc = src.cols * src.channels(); // total number of elements per line
        for (int j = 0; j < nr; j++) {
            const uint16_t* data = src.ptr<uint16_t>(j);
            for (int i = 0; i < nc; i++) {
                //printf(" -- > val = %d\n", *data);
                if (*data != 0) {
                    nC++;
                }
                data++;
              } // end of row
        }

//		for (int i = 0; i < src.rows; ++i)
//		{
//			for (int j = 0; j < src.cols; ++j)
//			{
//                uint16_t& val = src.at<uint16_t>(i, j);
//                printf(" -- > val = %d\n", src.data);
//                if (val != 0)
//				{
//					nC++;
//				}
//			}
//		}

		pX = new int[nC];
		pY = new int[nC];
        pZX = new double[nC];

        nC = 0;
        for (int j = 0; j < nr; j++) {
            const uint16_t* data = src.ptr<uint16_t>(j);
            for (int i = 0; i < nc; i++) {
                if (*data != 0) {
                    pX[nC] = i;
                    pY[nC] = j;
                    pZX[nC] = *data;
                    nC++;
                }
                data++;
              } // end of row
        }
        //for (int i = 0; i < src.rows; i++)
        //{
        //	for (int j = 0; j < src.cols; j++)
        //	{
        //        const uint16_t& val = src.at<uint16_t>(i, j);
        //
        //        if (val != 0)
        //		{
        //			pX[nC] = j;
        //			pY[nC] = i;
        //          pZX[nC] = val;
        //			nC++;
        //		}
        //	}
        //}

		if (!_ComputeCoef(nC, pX, pY, pZX, m_pCoefX))
		{
			return false;
		}

        for (int j = 0; j < nr; j++) {
            uint16_t* data = dst.ptr<uint16_t>(j);
            for (int i = 0; i < nc; i++) {
                *data = _fZX(i, j);
                //printf(" -- > val = %d\n", *data);
                data++;
              } // end of row
        }

        //for (int i = 0; i < dst.rows; i++)
        //{
        //	for (int j = 0; j < dst.cols; j++)
        //	{
        //        uint16_t& val = dst.at<uint16_t>(i, j);
        //
        //        val = _fZX(j, i);
        //	}
        //}

		delete[] pX;
		delete[] pY;
        delete[] pZX;

		return true;
	}

	double CFitting::_fZX(int x, int y)
	{
		return (double)(m_pCoefX[0] + m_pCoefX[1] * x + m_pCoefX[2] * y + m_pCoefX[3] * x*y + m_pCoefX[4] * x*x + m_pCoefX[5] * y*y);
	}

	/* 误差方程法化
	* param aa 是输入方程系数
	* param n是a的阶数
	* param bb 是与aa对应的常数
	* param a 是输出系数阵
	* param b 是输出常数阵
	*/
	void CFitting::_TNrml(double *aa, int n, double bb, double *a, double *b)
	{
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n - i; j++)
			{
				*a += *aa * *(aa + j);
				a++;
			}
			*b += *aa * bb;
			b++;
			aa++;
		}
	}

	/* 利用对称矩阵求解线性方程组
	* param a 是系数矩阵
	* param b 是常数列
	* param x 是方程的解
	* param n 是系数矩阵阶数(一阶、二阶、三阶)
	* param wide 是列的元素个数
	*/
	bool CFitting::_TSolve(double* a, double* b, double* x, int n, int wide)
	{
		int m = 0;
		double* d = 0;
		double* l = 0;
		m = n * (n + 1) / 2;
		d = new double[n];
		l = new double[m - n];
		if (!d || !l)
		{
			delete[] d;
			delete[] l;
			return false;
		}
		memset(d, 0, sizeof(double)*n);
		memset(l, 0, sizeof(double)*(m - n));

		if (!_TRldltban1(a, d, l, n, wide))
		{
			delete[] d; d = NULL;
			delete[] l; l = NULL;
			return false;
		}
		if (!_TRldltban2(l, d, b, x, n, wide))
		{
			delete[] d; d = NULL;
			delete[] l; l = NULL;
			return false;
		}
		delete[] d; d = NULL;
		delete[] l; l = NULL;
		return true;
	}

	//利用对称矩阵求解线性方程组，这是子函数一
	bool CFitting::_TRldltban1(double* a, double* d, double* l, int n, int wide)
	{
		int i, j, k, kk, km, m;
		double* ao = NULL;
		double* aa = NULL;
		double* co = NULL;

		m = wide * (2 * n + 1 - wide) / 2;

		double* c = NULL;
		c = new double[m - wide];
		if (!c)
		{
			return false;
		}

		ao = a;
		co = c;
		a += wide;

		for (i = 0; i < m - wide; i++)
		{
			*c++ = *a++;
		}
		c = co; a = ao;

		for (k = 1; k < n; k++)
		{
			if (k < n - wide + 2)
			{
				kk = wide - 1;
			}
			else
			{
				kk--;
			}

			*d = *a++;
			aa = a;
			a += kk;

			if (k < n - wide + 1)
			{
				km = wide;
			}
			else
			{
				km = n - k + 1;
			}

			if ((*d + 1.0) == 1.0000000)
			{
				*d = 0.01;
			}

			for (i = 1; i < kk + 1; i++)
			{
				*l = *aa++ / *d;
				for (j = 0; j < kk - i + 1; j++)
				{
					*(a + j) -= *l * *(aa + j - 1);
				}

				l++;

				if (k + i > n - wide + 1)
				{
					km--;
				}

				a += km;
			}

			a = aa;
			d++;
			if (k == n - 1)
			{
				*d = *a;
			}
		}

		a = ao;
		a += wide;
		for (i = 0; i < m - wide; i++)
		{
			*a++ = *c++;
		}

		c = co;
		delete[] c;
		return true;
	}

	//利用对称矩阵求解线性方程组，这是子函数二
	bool CFitting::_TRldltban2(double* l, double* d, double* b, double* x, int n, int wide)
	{
		int i, j, kk, m;
		double* bo = NULL;
		double* lo = NULL;
		double* xx = NULL;
		double* bbo = NULL;
		double* bb = NULL;

		bb = new double[n];

		if (!bb)
		{
			return false;
		}

		bbo = bb;

		bo = b; lo = l;

		for (i = 0; i < n; i++)
		{
			*bb++ = *b++;
		}

		b = bo;
		bb = bbo;
		m = wide*(2 * n + 1 - wide) / 2;

		for (i = 1; i < n; i++)
		{
			if (i < n - wide + 2)
			{
				kk = wide;
			}
			else
			{
				kk--;
			}

			b = bo + i;
			for (j = 1; j < kk; j++)
			{
				*b++ -= *(b - j) * *l++;
			}
		}

		kk = 0;
		b = bo + n - 1;
		l = lo + m - n - 1;
		x += n - 1;
		xx = x;
		d += n - 1;

		if ((*d + 1.0) == 1.0000000)
		{
			*d = 0.01;
		}

		*x-- = *b-- / *d--;

		for (i = 1; i < n; i++)
		{
			if (i < wide)
			{
				kk++;
			}
			else
			{
				kk = wide - 1;
				xx--;
			}

			if ((*d + 1.0) == 1.0000000)
			{
				*d = 0.01;
			}

			*x = *b-- / *d--;

			for (j = 1; j < kk + 1; j++)
			{
				*x -= *l-- * *(xx - j + 1);
			}

			x--;
		}

		b = bo;

		for (i = 0; i < n; i++)
		{
			*b++ = *bb++;
		}

		bb = bbo;

		delete[] bb;
		bb = NULL;

		return true;
	}

	bool CFitting::_ComputeCoef(int nPts, int *pX, int *pY, double *pZ, double pCoef[6])
	{
		CV_DbgAssert(nPts >= 6);
		if (nPts < 6)
		{
			printf("Points for fitting not enough.\n");
			return false;
		}

		//矩阵系数
		double a1[6], b1[6], aa1[36], x1[6];
		memset(aa1, 0, sizeof(aa1));
		memset(a1, 0, sizeof(a1));
		memset(b1, 0, sizeof(b1));
		memset(x1, 0, sizeof(x1));
		int k = 6;

		//组成法方程式
		for (int i = 0; i < nPts; i++)
		{
			a1[0] = 1.0;
			a1[1] = pX[i];
			a1[2] = pY[i];
			a1[3] = pX[i] * pY[i];
			a1[4] = pX[i] * pX[i];
			a1[5] = pY[i] * pY[i];
			_TNrml(a1, k, pZ[i], aa1, b1);
		}

		if (!_TSolve(aa1, b1, x1, k, k))
		{
			printf("Precision not enough!\n");
			return false;
		}

		for (int i = 0; i < k; i++)
		{
			pCoef[i] = x1[i];
		}

		return true;
	}
}
