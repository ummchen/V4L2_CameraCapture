#ifndef V4L2_CAM_H
#define V4L2_CAM_H

#ifdef __cplusplus
extern "C" {
#endif

enum V4L2_CAM_FMT
{
	V4L2_CAM_FMT_YUV,
	V4L2_CAM_FMT_MJPG,
	V4L2_CAM_FMT_GREY
};

struct v4l2_cam {
	int	cam_fd;
	unsigned int	cam_id;
	unsigned int	img_width;
	unsigned int	img_height;
	float	grab_fps;
	unsigned short	grab_fmt;
	unsigned char	*data_buf;
	unsigned int	buf_len;
};

void yuv422_to_rgb24(unsigned char *yuv422_buf, unsigned char *bgr24_buf, unsigned int x_res, unsigned int y_res, unsigned short bgr_mode);
int v4l2_open_cam(struct v4l2_cam *cam_data);
int v4l2_grab_cam(struct v4l2_cam *cam_data, unsigned int *data_len);
int v4l2_close_cam(struct v4l2_cam *cam_data);

#ifdef __cplusplus
}
#endif

#endif
