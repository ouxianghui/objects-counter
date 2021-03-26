/*******************************************************************************
#             uvccapture: USB UVC Video Class Snapshot Software                #
#This package work with the Logitech UVC based webcams with the mjpeg feature  #
#.                                                                             #
#   Orginally Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard   #
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "v4l2uvc.h"

static int debug = 0;

static unsigned char dht_data[DHT_SIZE] = {
	0xff, 0xc4, 0x01, 0xa2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
	0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x01, 0x00, 0x03,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0a, 0x0b, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05,
	0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04,
	0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22,
	0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15,
	0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36,
	0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
	0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
	0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
	0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95,
	0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8,
	0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2,
	0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5,
	0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
	0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9,
	0xfa, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05,
	0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04,
	0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22,
	0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33,
	0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25,
	0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36,
	0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
	0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
	0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94,
	0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
	0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
	0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa
};


int V4L2Uvc::init_videoIn(struct vdIn *vd,const char *device, int width, int height, int format, int grabmethod)
{
	if (vd == NULL || device == NULL)
		return -1;
	if (width == 0 || height == 0)
		return -1;
	if (grabmethod < 0 || grabmethod > 1)
		grabmethod = 1;   //mmap by default;
	vd->videodevice = NULL;
	vd->status = NULL;
	vd->pictName = NULL;
	vd->videodevice = (char *)calloc(1, 16 * sizeof (char));
	vd->status = (char *)calloc(1, 100 * sizeof (char));
	vd->pictName = (char *)calloc(1, 80 * sizeof (char));
	snprintf(vd->videodevice, 12, "%s", device);
	vd->toggleAvi = 0;
	vd->getPict = 0;
	vd->signalquit = 1;
	vd->width = width;
	vd->height = height;
	vd->formatIn = format;
	vd->grabmethod = grabmethod;
	if (init_v4l2(vd) < 0) {
		fprintf(stderr, " Init v4L2 failed !! exit fatal \n");
		goto error;;
	}
	/* alloc a temp buffer to reconstruct the pict */
	vd->framesizeIn = (vd->width * vd->height << 1);
	switch (vd->formatIn) {
	case V4L2_PIX_FMT_MJPEG:
		vd->tmpbuffer = (unsigned char *)calloc(1, (size_t)vd->framesizeIn);
		if (!vd->tmpbuffer)
			goto error;
		vd->framebuffer =
			(unsigned char *)calloc(1, (size_t)vd->width * (vd->height + 8) * 2);
		break;
	case V4L2_PIX_FMT_YUYV:
		vd->framebuffer = (unsigned char *)calloc(1, (size_t)vd->framesizeIn);
		break;
	default:
		fprintf(stderr, " should never arrive exit fatal !!\n");
		goto error;
		break;
	}
	if (!vd->framebuffer)
		goto error;
	return 0;
error:
	free(vd->videodevice);
	free(vd->status);
	free(vd->pictName);
	close(vd->fd);
	return -1;
}

int V4L2Uvc::init_v4l2(struct vdIn *vd)
{
	int i;
	int ret = 0;

	if ((vd->fd = open(vd->videodevice, O_RDWR)) == -1) {
		perror("ERROR opening V4L interface \n");
		exit(1);
	}
	memset(&vd->cap, 0, sizeof (struct v4l2_capability));
	ret = ioctl(vd->fd, VIDIOC_QUERYCAP, &vd->cap);
	if (ret < 0) {
		fprintf(stderr, "Error opening device %s: unable to query device.\n",
			vd->videodevice);
		goto fatal;
	}

	if ((vd->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
		fprintf(stderr,
			"Error opening device %s: video capture not supported.\n",
			vd->videodevice);
		goto fatal;;
	}
	if (vd->grabmethod) {
		if (!(vd->cap.capabilities & V4L2_CAP_STREAMING)) {
			fprintf(stderr, "%s does not support streaming i/o\n",
				vd->videodevice);
			goto fatal;
		}
	}
	else {
		if (!(vd->cap.capabilities & V4L2_CAP_READWRITE)) {
			fprintf(stderr, "%s does not support read i/o\n", vd->videodevice);
			goto fatal;
		}
	}
	/* set format in */
	memset(&vd->fmt, 0, sizeof (struct v4l2_format));
	vd->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vd->fmt.fmt.pix.width = vd->width;
	vd->fmt.fmt.pix.height = vd->height;
	vd->fmt.fmt.pix.pixelformat = vd->formatIn;
	vd->fmt.fmt.pix.field = V4L2_FIELD_ANY;
	ret = ioctl(vd->fd, VIDIOC_S_FMT, &vd->fmt);
	if (ret < 0) {
		fprintf(stderr, "Unable to set format: %d.\n", errno);
		goto fatal;
	}
	if ((vd->fmt.fmt.pix.width != vd->width) ||
		(vd->fmt.fmt.pix.height != vd->height)) {
		fprintf(stderr, " format asked unavailable get width %d height %d \n",
			vd->fmt.fmt.pix.width, vd->fmt.fmt.pix.height);
		vd->width = vd->fmt.fmt.pix.width;
		vd->height = vd->fmt.fmt.pix.height;
		/* look the format is not part of the deal ??? */
		//vd->formatIn = vd->fmt.fmt.pix.pixelformat;
	}
	/* request buffers */
	memset(&vd->rb, 0, sizeof (struct v4l2_requestbuffers));
	vd->rb.count = NB_BUFFER;
	vd->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vd->rb.memory = V4L2_MEMORY_MMAP;

	ret = ioctl(vd->fd, VIDIOC_REQBUFS, &vd->rb);
	if (ret < 0) {
		fprintf(stderr, "Unable to allocate buffers: %d.\n", errno);
		goto fatal;
	}
	/* map the buffers */
	for (i = 0; i < NB_BUFFER; i++) {
		memset(&vd->buf, 0, sizeof (struct v4l2_buffer));
		vd->buf.index = i;
		vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd->buf.memory = V4L2_MEMORY_MMAP;
		ret = ioctl(vd->fd, VIDIOC_QUERYBUF, &vd->buf);
		if (ret < 0) {
			fprintf(stderr, "Unable to query buffer (%d).\n", errno);
			goto fatal;
		}
		if (debug)
			fprintf(stderr, "length: %u offset: %u\n", vd->buf.length,
			vd->buf.m.offset);
		vd->mem[i] = mmap(0 /* start anywhere */,
			vd->buf.length, PROT_READ, MAP_SHARED, vd->fd,
			vd->buf.m.offset);
		if (vd->mem[i] == MAP_FAILED) {
			fprintf(stderr, "Unable to map buffer (%d)\n", errno);
			goto fatal;
		}
		if (debug)
			fprintf(stderr, "Buffer mapped at address %p.\n", vd->mem[i]);
	}
	/* Queue the buffers. */
	for (i = 0; i < NB_BUFFER; ++i) {
		memset(&vd->buf, 0, sizeof (struct v4l2_buffer));
		vd->buf.index = i;
		vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd->buf.memory = V4L2_MEMORY_MMAP;
		ret = ioctl(vd->fd, VIDIOC_QBUF, &vd->buf);
		if (ret < 0) {
			fprintf(stderr, "Unable to queue buffer (%d).\n", errno);
			goto fatal;;
		}
	}
	return 0;
fatal:
	return -1;

}
int V4L2Uvc::video_enable(struct vdIn *vd)
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;

	ret = ioctl(vd->fd, VIDIOC_STREAMON, &type);
	if (ret < 0) {
		fprintf(stderr, "Unable to %s capture: %d.\n", "start", errno);
		return ret;
	}
	vd->isstreaming = 1;
	return 0;
}

int V4L2Uvc::video_disable(struct vdIn *vd)
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;

	ret = ioctl(vd->fd, VIDIOC_STREAMOFF, &type);
	if (ret < 0) {
		fprintf(stderr, "Unable to %s capture: %d.\n", "stop", errno);
		return ret;
	}
	vd->isstreaming = 0;
	return 0;
}

int V4L2Uvc::uvcGrab(struct vdIn *vd)
{
#define HEADERFRAME1 0xaf
	int ret;
	int readLen = -1;
	if (!vd->isstreaming)
	if (video_enable(vd))
		goto err;
	memset(&vd->buf, 0, sizeof (struct v4l2_buffer));
	vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vd->buf.memory = V4L2_MEMORY_MMAP;
	vd->buf.length = vd->framesizeIn;
	ret = ioctl(vd->fd, VIDIOC_DQBUF, &vd->buf);
	if (ret < 0) {
		fprintf(stderr, "Unable to dequeue buffer (%d).\n", errno);
		goto err;
	}
	switch (vd->formatIn) {
	case V4L2_PIX_FMT_MJPEG:
		memcpy(vd->tmpbuffer, vd->mem[vd->buf.index], HEADERFRAME1);
		memcpy(vd->tmpbuffer + HEADERFRAME1, dht_data, DHT_SIZE);
		memcpy(vd->tmpbuffer + HEADERFRAME1 + DHT_SIZE, (unsigned char*)(vd->mem[vd->buf.index]) + HEADERFRAME1, (vd->buf.bytesused - HEADERFRAME1));
		if (debug)
			fprintf(stderr, "bytes in used %d \n", vd->buf.bytesused);
		break;
	case V4L2_PIX_FMT_YUYV:
		if (vd->buf.bytesused > vd->framesizeIn)
		{
			memcpy(vd->framebuffer, vd->mem[vd->buf.index], (size_t)vd->framesizeIn);
		}
		else
		{
			memcpy(vd->framebuffer, vd->mem[vd->buf.index], (size_t)vd->buf.bytesused);
		}
		readLen = (size_t)vd->buf.bytesused;

#if(0)
		if (vd->buf.bytesused != vd->framesizeIn)
		{
			LOGE("V4L2_PIX_FMT_YUYV uvcGrab size = %d", vd->buf.bytesused);
		}
		else
		{
			LOGE("V4L2_PIX_FMT_YUYV uvcGrab ok %d", vd->framesizeIn);
		}
#endif
			break;
	default:
		goto err;
		break;
	}
	ret = ioctl(vd->fd, VIDIOC_QBUF, &vd->buf);
	if (ret < 0) {
		fprintf(stderr, "Unable to requeue buffer (%d).\n", errno);
		goto err;
	}

	return readLen;
err:
	vd->signalquit = 0;
	return -1;
}

int V4L2Uvc::getSurpportFmt(struct vdIn *vd)
{
	if (vd->fd > 0)
	{
		struct v4l2_format fmt;
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ioctl(vd->fd, VIDIOC_G_FMT, &fmt);
		int pp;
		memcpy(&pp, &(fmt.fmt.pix.pixelformat), 4);
		return pp;
	}
	return -1;
}

int V4L2Uvc::close_v4l2(struct vdIn *vd)
{
	int i;

	if (vd->isstreaming)
		video_disable(vd);
	for (i = 0; i < NB_BUFFER; i++) {
		munmap(vd->mem[i], vd->buf.length);
	}

	if (vd->tmpbuffer)
	free(vd->tmpbuffer);
	vd->tmpbuffer = NULL;
	free(vd->framebuffer);
	vd->framebuffer = NULL;
	free(vd->videodevice);
	free(vd->status);
	free(vd->pictName);
	vd->videodevice = NULL;
	vd->status = NULL;
	vd->pictName = NULL;
	close(vd->fd);
	return 0;
}

