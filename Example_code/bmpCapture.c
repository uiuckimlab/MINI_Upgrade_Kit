#include <stdio.h>
#include <opencv/cv.h>
#include <opencv2/videoio/videoio_c.h>
#include <highgui.h>
#include <stdlib.h>
#include <math.h>

void findCoM(IplImage *threshImage, IplImage *origImage, long *xloc, long *yloc, long *numPix);

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("Function takes 1 argument, the desired filename \n");
    return 1;
  }

  char filename[50];

  CvCapture *capture;

  capture = cvCaptureFromCAM(1);
  if (!capture)
    printf("No camera detected \n");

  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 160);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 120);

  IplImage *frame;
  int i;
  for (i = 0; i < 20; i++)
  {
    cvWaitKey(20); //we take 20 pictures, this allows camera to adjust to lighting before taking a the final photo (otherwise pictures look too yellow)
    frame = cvQueryFrame(capture);
  }

  argv++;
  sprintf(filename, "%s.bmp", *argv);

  cvSaveImage(filename, frame, 0);

  cvReleaseCapture(&capture);

  return 0;
}
