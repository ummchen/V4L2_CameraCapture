/********************************
* Module:	V4L2_CAM	*
* Author:	Josh Chen	*
* Date:		2015/07/06	*
********************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include "v4l2_cam.h"

#define MEM_USER_PTR	0

#define LOWER           0
#define UPPER           255
#define CLAMP(n)        (n <= LOWER ? LOWER : n >= UPPER ? UPPER : n) 

#ifdef DEBUG
#define DEBUG_PRINT(format, args...) printf("[%s:%d] "format, __FILE__, __LINE__, ##args)
#else
#define DEBUG_PRINT(args...)
#endif

static int xioctl(int fd, int request, void *arg)
{
	int r;
	do
		r =ioctl(fd, request, arg);
	while (-1 == r && EINTR == errno);
	return r;
}

static int float_to_fraction_recursive(double f, double p, int *num, int *den)
{
	int whole = (int) f;
	f = fabs(f - whole);
	
	if (f > p) {
		int n, d;
		int a = float_to_fraction_recursive(1 / f, p + p / f, &n, &d);
		*num = d;
		*den = d * a + n;
	}
	else {
		*num = 0;
		*den = 1;
	}
	return whole;
}

static void float_to_fraction(float f, int *num, int *den)
{
	int whole = float_to_fraction_recursive(f, FLT_EPSILON, num, den);
	*num += whole * *den;
}

static inline void __convert_yuv2rgb(unsigned char y, unsigned char u, unsigned char v, unsigned char *rgb)
{
	//Y'UV422 to RGB888 conversion: http://en.wikipedia.org/wiki/YUV#Y.27UV422_to_RGB888_conversion
	rgb[2] = (unsigned char) CLAMP((298*(y-16) + 516*(u-128) + 128) >> 8);			//blue
	rgb[1] = (unsigned char) CLAMP((298*(y-16) - 100*(u-128) - 208*(v-128) + 128) >> 8);	//green
	rgb[0] = (unsigned char) CLAMP((298*(y-16) + 409*(v-128) + 128) >> 8);			//red   
}

static inline void __convert_yuv2bgr(unsigned char y, unsigned char u, unsigned char v, unsigned char *bgr)
{
	//Y'UV422 to RGB888 conversion: http://en.wikipedia.org/wiki/YUV#Y.27UV422_to_RGB888_conversion
	bgr[0] = (unsigned char) CLAMP((298*(y-16) + 516*(u-128) + 128) >> 8);			//blue
	bgr[1] = (unsigned char) CLAMP((298*(y-16) - 100*(u-128) - 208*(v-128) + 128) >> 8);	//green
	bgr[2] = (unsigned char) CLAMP((298*(y-16) + 409*(v-128) + 128) >> 8);			//red   
}

void yuv422_to_rgb24(unsigned char *yuv422_buf, unsigned char *bgr24_buf, unsigned int x_res, unsigned int y_res, unsigned short bgr_mode)
{
	unsigned int i = 0, macropixel = (x_res * y_res) >> 1;  //macropixel = 4 byte = 2 pixel
	unsigned char yuyv[4] = {0}, bgra[4] = {0};

	if (!yuv422_buf || !bgr24_buf || x_res==0 || y_res==0)
		return;

	for (i = 0; i < macropixel; i++)
	{
		memcpy(yuyv, yuv422_buf+(i*4), 4);
		if (bgr_mode)
			__convert_yuv2bgr(yuyv[0], yuyv[1], yuyv[3], bgra);
		else
			__convert_yuv2rgb(yuyv[0], yuyv[1], yuyv[3], bgra);
		memcpy(bgr24_buf+(i*6), bgra, 3);
		if (bgr_mode)
			__convert_yuv2bgr(yuyv[2], yuyv[1], yuyv[3], bgra);
		else
			__convert_yuv2rgb(yuyv[2], yuyv[1], yuyv[3], bgra);
		memcpy(bgr24_buf+(i*6)+3, bgra, 3);
	}
}

int v4l2_open_cam(struct v4l2_cam *cam_data)
{
	struct v4l2_capability		caps;
	struct v4l2_format		fmt;
	struct v4l2_streamparm		parm;
	struct v4l2_requestbuffers	req;
	struct v4l2_buffer		buf;
	char sz_buf[32] = {0};
	int num = 0, den = 0;

	if (cam_data == NULL)
	{
		DEBUG_PRINT("struct v4l2_cam == NULL\n");
		return 0;
	}

	memset(&caps,	0, sizeof(caps));
	memset(&fmt,	0, sizeof(fmt));
	memset(&parm,	0, sizeof(parm));
	memset(&req,	0, sizeof(req));
	memset(&buf,	0, sizeof(buf));

	//open device
	sprintf(sz_buf, "/dev/video%d", cam_data->cam_id);
	cam_data->cam_fd = open(sz_buf, O_RDWR);
	if (cam_data->cam_fd == -1)
	{
		DEBUG_PRINT("open device fail\n");
		return 0;
	}

	//query capability
	if (xioctl(cam_data->cam_fd, VIDIOC_QUERYCAP, &caps) == -1)
	{
		DEBUG_PRINT("query capability fail\n");
		return 0;
	}

	//setup format
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = cam_data->img_width;
	fmt.fmt.pix.height = cam_data->img_height;
	if (cam_data->grab_fmt == V4L2_CAM_FMT_YUV)
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	else if (cam_data->grab_fmt == V4L2_CAM_FMT_MJPG)
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	else if (cam_data->grab_fmt == V4L2_CAM_FMT_GREY)
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
	else if (cam_data->grab_fmt == V4L2_CAM_FMT_Y16)
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
	else
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(cam_data->cam_fd, VIDIOC_S_FMT, &fmt) == -1)
	{
		DEBUG_PRINT("setup format fail\n");
		return 0;
	}

	//setup frame rate
	if (cam_data->grab_fps > 0.0)
	{
		parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		float_to_fraction(cam_data->grab_fps, &num, &den);
		parm.parm.capture.timeperframe.numerator = den;		//that assignment
		parm.parm.capture.timeperframe.denominator = num;	//is right
		if (xioctl(cam_data->cam_fd, VIDIOC_S_PARM, &parm) == -1)
		{
			DEBUG_PRINT("setup frame rate fail\n");
			return 0;
		}
	}

	//request buffer
	req.count = 1;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#if MEM_USER_PTR
	req.memory = V4L2_MEMORY_USERPTR;
#else
	req.memory = V4L2_MEMORY_MMAP;
#endif
	if (xioctl(cam_data->cam_fd, VIDIOC_REQBUFS, &req) == -1)
	{
		DEBUG_PRINT("request buffer fail\n");
		return 0;
	}

	//query buffer
#if MEM_USER_PTR
	if (!cam_data->data_buf)
#else
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;
	if (xioctl(cam_data->cam_fd, VIDIOC_QUERYBUF, &buf) == -1)
#endif
	{
		DEBUG_PRINT("query buffer fail\n");
		return 0;
	}

	//map buffer
#if !MEM_USER_PTR
	cam_data->data_buf = (unsigned char *) mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_data->cam_fd, buf.m.offset);
	if (cam_data->data_buf == MAP_FAILED)
	{
		DEBUG_PRINT("map buffer fail\n");
		return 0;
	}
#endif

	//queue buffer
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.index = 0;
#if MEM_USER_PTR
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.m.userptr = (unsigned long) cam_data->data_buf;
	buf.length = cam_data->buf_len;
#else
	buf.memory = V4L2_MEMORY_MMAP;
	cam_data->buf_len = buf.length;
#endif
	if (xioctl(cam_data->cam_fd, VIDIOC_QBUF, &buf) == -1)
	{
		DEBUG_PRINT("queue buffer fail\n");
		return 0;
	}

	//start stream
	if (xioctl(cam_data->cam_fd, VIDIOC_STREAMON, &buf.type) == -1)
	{
		DEBUG_PRINT("start stream fail\n");
		return 0;
	}

	return 1;
}

int v4l2_grab_cam(struct v4l2_cam *cam_data, unsigned int *grab_len)
{
	struct v4l2_buffer buf = {0};

	if (cam_data == NULL)
	{
		DEBUG_PRINT("struct v4l2_cam == NULL\n");
		return 0;
	}

	//dequeue buffer
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.length = cam_data->buf_len;
#if MEM_USER_PTR
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.m.userptr = (unsigned long) cam_data->data_buf;
#else
	buf.memory = V4L2_MEMORY_MMAP;
#endif
	if (xioctl(cam_data->cam_fd, VIDIOC_DQBUF, &buf) == -1)
	{
		DEBUG_PRINT("dequeue buffer fail\n");
		return 0;
	}

	if (grab_len != NULL)
		*grab_len = buf.bytesused;

	//queue buffer
	if (xioctl(cam_data->cam_fd, VIDIOC_QBUF, &buf) == -1)
	{
		DEBUG_PRINT("queue buffer fail\n");
		return 0;
	}

	return 1;
}

int v4l2_close_cam(struct v4l2_cam *cam_data)
{
	enum v4l2_buf_type type;

	if (cam_data == NULL)
	{
		DEBUG_PRINT("struct v4l2_cam == NULL\n");
		return 0;
	}

	//stop stream
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(cam_data->cam_fd, VIDIOC_STREAMOFF, &type) == -1)
	{
		DEBUG_PRINT("stop stream fail\n");
		return 0;
	}

	//unmap buffer
#if !MEM_USER_PTR
	if (cam_data->data_buf  && cam_data->buf_len && munmap(cam_data->data_buf, cam_data->buf_len) == -1)
	{
		DEBUG_PRINT("unmap buffer fail\n");
		return 0;
	}
	cam_data->data_buf = 0;
	cam_data->buf_len = 0;
#endif

	//close device
	if (close(cam_data->cam_fd) == -1)
	{
		DEBUG_PRINT("close device fail\n");
		return 0;
	}
	cam_data->cam_fd = 0;

	return 1;
}


