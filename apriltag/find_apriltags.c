/**
 * Bernd Pfrommer 
 * matlab mex file to access april tags using umich library
 */

#include <mex.h>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>


#include "apriltag.h"
#include "image_u8.h"
#include "tag36h11.h"
#include "common/homography.h"
#include "zarray.h"

#define NUMBER_OF_FIELDS 7

/*
 * Convert matlab image to image_u8_t.
 * 
 * Assumes stride (line size) equals to width, i.e. no padding for matlab image.
 * Returns NULL if failed, pointer to image otherwise.
 * Caller is responsible for deallocating the image.
 */
image_u8_t *matlab_to_image_u8(const mxArray *src) {
    if(!mxIsClass(src, "uint8")) {
		mexErrMsgTxt("image must have uint8 type");
        return (NULL);
    }
	// only handle 3-dim right now!
	if (mxGetNumberOfDimensions(src) != 2) {
		mexErrMsgTxt("image must be gray level with x and y dimensions only!");
        return (NULL);
	}
    int		*dims = (int*) mxGetDimensions(src);
	uint8_t	*grey = (uint8_t *) mxGetPr(src);
	int height  = dims[0];
	int width   = dims[1];
	image_u8_t *im = image_u8_create(width, height);	// may have stride!
	// assume matlab image has line size == width
	// convert from row to column major
	for (int y = 0; y < im->height; y++) {
		for (int x = 0; x < im->width; x++) {
			im->buf[y * im->stride + x] = grey[y + height * x];
		}
    }
	return (im);
}

void detection_to_pose(const apriltag_detection_t *det,
					   double tagSize, const double *camParams,
					   double *rot, double *trans) {
	double flenX	= camParams[0];
	double flenY	= camParams[1];
	double pPointX	= camParams[2];
	double pPointY	= camParams[3];
	matd_t *pose = homography_to_pose(det->H, flenX, flenY, pPointX, pPointY);
	// Pose is a 4x4 matrix:
	// [R|T]  (and last line [0 0 0 1])
	//
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rot[i * 3 + j] = MATD_EL(pose, i, j);
		}
	}
	//
	// MATD_EL(pose, :, 3) contains the distance in world units. In world units,
	// the tag is at locations [+-1, +-1], which means that one world unit actually
	// is half of the APRIL tag's edge length.

	for (int i = 0; i < 3; i++) {
		trans[i] = MATD_EL(pose, i, 3) * 0.5 * tagSize;
	}
	matd_destroy(pose);
}

void detection_to_matlab_struct(const apriltag_detection_t *det, mxArray *a, int idx,
								double tagSize, const double *camParams) {

	// create matlab arrays

	mxArray *id 				= mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray *hammingDistance	= mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray *rotation			= mxCreateDoubleMatrix(3, 3, mxREAL);
	mxArray *translation		= mxCreateDoubleMatrix(3, 1, mxREAL);
	mxArray *center				= mxCreateDoubleMatrix(2, 1, mxREAL);
	mxArray *corners			= mxCreateDoubleMatrix(2, 4, mxREAL);
	mxArray *homography			= mxCreateDoubleMatrix(3, 3, mxREAL);

	// populate matlab arrays with data

	*mxGetPr(id) 				= (double) det->id;
	*mxGetPr(hammingDistance) 	= det->hamming;

	double rot[9], trans[3];
	detection_to_pose(det, tagSize, camParams, rot, trans);
	memcpy(mxGetPr(rotation), rot, 9 * sizeof(double)); 
	memcpy(mxGetPr(translation), trans, 3 * sizeof(double));
	memcpy(mxGetPr(center), det->c, 2 * sizeof(double));
	memcpy(mxGetPr(corners), det->p, 8 * sizeof(double));
	double *mxh = mxGetPr(homography);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			mxh[i + 3 * j] = MATD_EL(det->H, i, j);
		}
	}

	// tie matlab arrays into structure

	mxSetField(a, idx, "id", id);
	mxSetField(a, idx, "hammingDistance", hammingDistance);
	mxSetField(a, idx, "rotation", rotation);
	mxSetField(a, idx, "translation", translation);
	mxSetField(a, idx, "center", center);
	mxSetField(a, idx, "corners", corners);
	mxSetField(a, idx, "homography", homography);
}

mxArray *create_struct_array(mwSize nelements) {
	const char *field_names[NUMBER_OF_FIELDS] =
		{"id", "hammingDistance", "rotation", "translation",
		 "center", "corners", "homography"};
    mwSize dims[2] = {1, nelements};
    /* Create a 1-by-n array of structs. */ 
    mxArray *s = mxCreateStructArray(2, dims, NUMBER_OF_FIELDS, field_names);
	return (s);
}


double get_tag_size(const mxArray *ts) {
	if (mxGetNumberOfDimensions(ts) != 2) {
		mexErrMsgTxt("tag size must be scalar!");
        return (-1);
	}
	int		*dims = (int*) mxGetDimensions(ts);
	if (dims[0] != 1 || dims[1] != 1) {
		mexErrMsgTxt("tag size must be scalar!");
        return (-1);
	}
	double *data = mxGetPr(ts);
	if (data == NULL) {
		mexErrMsgTxt("tag size must have real value!");
		return (-1);
	}
	return *data;
}

const double *get_cam_params(const mxArray *cp) {
	if (mxGetNumberOfDimensions(cp) != 2) {
		mexErrMsgTxt("cam params must be row vector!");
        return (NULL);
	}
	int		*dims = (int*) mxGetDimensions(cp);
	if (dims[0] != 1 || dims[1] != 4) {
		mexErrMsgTxt("cam params must have 4 values!");
        return (NULL);
	}
	const double *data = mxGetPr(cp);
	if (data == NULL) {
		mexErrMsgTxt("cam params must be of type real!");
		return (NULL);
	}
	return data;
}

/*
 * The gateway function for the matlab mex compiler.
 * 
 * apriltags = find_apriltags(image, tagsize, camera_parameters)
 *
 * image	matlab grey image of type uint8
 * tagsize	edge length of april tag (in meters!)
 * camera_parameters (in pixels) [focal_length_x, focal_length_y, principal_point_x, principal_point_y]
 */

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs != 3) {
		mexErrMsgTxt("must have one 3 input arguments (image, tagsize, camera_parameters)");
		return;
	}

	if (nlhs != 1) {
		mexErrMsgTxt("one output required.");
		return;
	}

	apriltag_family_t	*tf = tag36h11_create();
    apriltag_detector_t	*td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
	// set various options for detector

    td->quad_decimate	= 1.0;	// decimate input image by this factor
    td->quad_sigma		= 0;	// blur
    td->nthreads		= 4;	// use this many cpu threads
    td->debug			= 0;
    td->refine_edges	= 1;	// spend more time trying to align edges
    td->refine_decode	= 0;	// spend more time decoding tags
    td->refine_pose		= 0;	// spend more time trying to precisely localize tags

	image_u8_t *im = matlab_to_image_u8(prhs[0]);
	if (!im) {
		mexErrMsgTxt("image conversion failed!");
		return;
	}
	zarray_t *detections = apriltag_detector_detect(td, im);
	int ntags	= zarray_size(detections);

	mxArray *a	= create_struct_array((mwSize)ntags);

	double tagSize = get_tag_size(prhs[1]);
	if (tagSize <= 0) {
		mexErrMsgTxt("invalid tag size");
		return;
	}

	const double *camParams = get_cam_params(prhs[2]);
	if (camParams == NULL) {
		mexErrMsgTxt("invalid cam parameters");
		return;
	}
	for (int i = 0; i < ntags; i++) {
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);
		detection_to_matlab_struct(det, a, i, tagSize, camParams);
	}
	// clean up
	apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
	image_u8_destroy(im);

	// assign to output argument
	plhs[0] = a;
}
