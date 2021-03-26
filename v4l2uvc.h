/*
* v4l2uvc.h
*
*  Created on: 2013-2-26
*      Author: nickman
*/

#ifndef V4L2UVC_H_
#define V4L2UVC_H_

/*******************************************************************************
#             uvccapture: USB UVC Video Class Snapshot Software                #
#This package work with the Logitech UVC based webcams with the mjpeg feature  #
#.                                                                             #
# 	Orginally Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard   #
#       Modifications Copyright (C) 2006  Gabriel A. Devenyi                   #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#define NB_BUFFER 32
#define DHT_SIZE 420

#include <linux/videodev2.h>

	//#define V4L2_CID_BACKLIGHT_COMPENSATION	(V4L2_CID_PRIVATE_BASE+0)
	//#define V4L2_CID_POWER_LINE_FREQUENCY	(V4L2_CID_PRIVATE_BASE+1)
	//#define V4L2_CID_SHARPNESS		(V4L2_CID_PRIVATE_BASE+2)
	//#define V4L2_CID_HUE_AUTO		(V4L2_CID_PRIVATE_BASE+3)
	//#define V4L2_CID_FOCUS_AUTO		(V4L2_CID_PRIVATE_BASE+4)
	//#define V4L2_CID_FOCUS_ABSOLUTE		(V4L2_CID_PRIVATE_BASE+5)
	//#define V4L2_CID_FOCUS_RELATIVE		(V4L2_CID_PRIVATE_BASE+6)

	//#define V4L2_CID_PANTILT_RELATIVE	(V4L2_CID_PRIVATE_BASE+7)
	//#define V4L2_CID_PANTILT_RESET		(V4L2_CID_PRIVATE_BASE+8)

	struct vdIn {
		int fd;
		char *videodevice;
		char *status;
		char *pictName;
		struct v4l2_capability cap;
		struct v4l2_format fmt;
		struct v4l2_buffer buf;
		struct v4l2_requestbuffers rb;
		void *mem[NB_BUFFER];
		unsigned char *tmpbuffer;
		unsigned char *framebuffer;
		int isstreaming;
		int grabmethod;
		int width;
		int height;
		int formatIn;
		int formatOut;
		int framesizeIn;
		int signalquit;
		int toggleAvi;
		int getPict;
	};

	class V4L2Uvc{
	public:
		int init_videoIn(struct vdIn *vd,const char *device, int width, int height, int format, int grabmethod);
		int uvcGrab(struct vdIn *vd);
		int close_v4l2(struct vdIn *vd);
		int getSurpportFmt(struct vdIn *vd);
	private:
		int init_v4l2(struct vdIn *vd);
		int video_enable(struct vdIn *vd);
		int video_disable(struct vdIn *vd);

	};

#endif /* V4L2UVC_H_ */
