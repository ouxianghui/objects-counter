/******************************************************************
** 文件名: Fitting.h
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

#pragma once

#include "opencv2/core/core.hpp"

namespace cvpr {

	class CFitting
	{
	public:
		CFitting();
		virtual ~CFitting();

		/* 函数描述: 整个矩阵一次性拟合
		* @param src: 二维双通道（CV_64FC2）矩阵，存储X、Y值
		* @param dst: 拟合后的矩阵，二维双通道（CV_64FC2）矩阵，存储X、Y值
		* return: true，拟合成功，否则失败
		*/
		bool fitting(const cv::Mat& src, cv::Mat& dst);

	protected:
        double _fZX(int x, int y);

		bool _ComputeCoef(int nPts, int *pX, int *pY, double *pZ, double pCoef[]);
		void _TNrml(double *aa, int n, double bb, double *a, double *b);
		bool _TRldltban1(double *a, double *d, double *l, int n, int wide);
		bool _TRldltban2(double *l, double *d, double *b, double *x, int n, int wide);
		bool _TSolve(double *a, double *b, double *x, int n, int wide);

	protected:
        double m_pCoefX[6];
	};

}
