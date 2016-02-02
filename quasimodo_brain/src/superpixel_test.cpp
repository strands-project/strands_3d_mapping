// ****************************************************************************** 
// SEEDS Superpixels
// ******************************************************************************
// Author: Beat Kueng based on Michael Van den Bergh's code
// Contact: vamichae@vision.ee.ethz.ch
//
// This code implements the superpixel method described in:
// M. Van den Bergh, X. Boix, G. Roig, B. de Capitani and L. Van Gool, 
// "SEEDS: Superpixels Extracted via Energy-Driven Sampling",
// ECCV 2012
// 
// Copyright (c) 2012 Michael Van den Bergh (ETH Zurich). All rights reserved.
// ******************************************************************************

#include <vector>
#include <string>

#include "seeds2.h"

#include <cv.h>
#include <highgui.h>
#include <fstream>


#include "helper.h"

#include <ctime>
using namespace std;


string getNameFromPathWithoutExtension(string path){
  string nameWith =  path.substr(path.find_last_of("/\\")+1);
  string nameWithout = nameWith.substr(0,nameWith.find_last_of("."));
  return nameWithout;
}


int main(int argc, char* argv[])
{
  char* input_file1;
  if(argc > 1)
  {
    input_file1 = argv[1];   // color image
  }
  else
    {
      printf("Error : no filename given as input");
      printf("Usage : %s image_name [number_of_superpixels]\n",argv[0]);
      return -1;
    }

  int NR_SUPERPIXELS = 200;
  if(argc > 2)
    NR_SUPERPIXELS = atoi(argv[2]);

  int numlabels = 10;
  int width(0), height(0);

  IplImage* img = cvLoadImage(input_file1);
  if ((!img))
    {
      printf("Error while opening %s\n", input_file1);
      return -1;
    }

  width = img->width;
  height = img->height;
  int sz = height*width;


  printf("Image loaded %d\n",img->nChannels);

  UINT* ubuff = new UINT[sz];
  UINT* ubuff2 = new UINT[sz];
  UINT* dbuff = new UINT[sz];
  UINT pValue;
  UINT pdValue;
  char c;
  UINT r,g,b,d,dx,dy;
  int idx = 0;
  for(int j=0;j<img->height;j++)
    for(int i=0;i<img->width;i++)
      {
        if(img->nChannels == 3)
          {
            // image is assumed to have data in BGR order
            b = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels];
            g = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels+1];
            r = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels+2];
			if (d < 128) d = 0;
            pValue = b | (g << 8) | (r << 16);
          }
        else if(img->nChannels == 1)
          {
            c = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels];
            pValue = c | (c << 8) | (c << 16);
          }
        else
          {
            printf("Unknown number of channels %d\n", img->nChannels);
            return -1;
          }          
        ubuff[idx] = pValue;
        ubuff2[idx] = pValue;
        idx++;
      }




/*******************************************
 * SEEDS SUPERPIXELS                       *
 *******************************************/
int NR_BINS = 5; // Number of bins in each histogram channel

printf("Generating SEEDS with %d superpixels\n", NR_SUPERPIXELS);
SEEDS seeds(width, height, 3, NR_BINS);

// SEEDS INITIALIZE
int nr_superpixels = NR_SUPERPIXELS;

// NOTE: the following values are defined for images from the BSD300 or BSD500 data set.
// If the input image size differs from 480x320, the following values might no longer be 
// accurate.
// For more info on how to select the superpixel sizes, please refer to README.TXT.
int seed_width = 3; int seed_height = 4; int nr_levels = 4;
if (width >= height)
{
	if (nr_superpixels == 600) {seed_width = 2; seed_height = 2; nr_levels = 4;}
	if (nr_superpixels == 400) {seed_width = 3; seed_height = 2; nr_levels = 4;}
	if (nr_superpixels == 266) {seed_width = 3; seed_height = 3; nr_levels = 4;}
	if (nr_superpixels == 200) {seed_width = 3; seed_height = 4; nr_levels = 4;}
	if (nr_superpixels == 150) {seed_width = 2; seed_height = 2; nr_levels = 5;}
	if (nr_superpixels == 100) {seed_width = 3; seed_height = 2; nr_levels = 5;}
	if (nr_superpixels == 50)  {seed_width = 3; seed_height = 4; nr_levels = 5;}
	if (nr_superpixels == 25)  {seed_width = 3; seed_height = 2; nr_levels = 6;}
	if (nr_superpixels == 17)  {seed_width = 3; seed_height = 3; nr_levels = 6;}
	if (nr_superpixels == 12)  {seed_width = 3; seed_height = 4; nr_levels = 6;}
	if (nr_superpixels == 9)  {seed_width = 2; seed_height = 2; nr_levels = 7;}
	if (nr_superpixels == 6)  {seed_width = 3; seed_height = 2; nr_levels = 7;}
}
else
{
	if (nr_superpixels == 600) {seed_width = 2; seed_height = 2; nr_levels = 4;}
	if (nr_superpixels == 400) {seed_width = 2; seed_height = 3; nr_levels = 4;}
	if (nr_superpixels == 266) {seed_width = 3; seed_height = 3; nr_levels = 4;}
	if (nr_superpixels == 200) {seed_width = 4; seed_height = 3; nr_levels = 4;}
	if (nr_superpixels == 150) {seed_width = 2; seed_height = 2; nr_levels = 5;}
	if (nr_superpixels == 100) {seed_width = 2; seed_height = 3; nr_levels = 5;}
	if (nr_superpixels == 50)  {seed_width = 4; seed_height = 3; nr_levels = 5;}
	if (nr_superpixels == 25)  {seed_width = 2; seed_height = 3; nr_levels = 6;}
	if (nr_superpixels == 17)  {seed_width = 3; seed_height = 3; nr_levels = 6;}
	if (nr_superpixels == 12)  {seed_width = 4; seed_height = 3; nr_levels = 6;}
	if (nr_superpixels == 9)  {seed_width = 2; seed_height = 2; nr_levels = 7;}
	if (nr_superpixels == 6)  {seed_width = 2; seed_height = 3; nr_levels = 7;}
}

seeds.initialize(seed_width, seed_height, nr_levels);



clock_t begin = clock();

seeds.update_image_ycbcr(ubuff);

seeds.iterate();

clock_t end = clock();
double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
printf("    elapsed time=%lf sec\n", elapsed_secs);

printf("SEEDS produced %d labels\n", seeds.count_superpixels());

// DRAW SEEDS OUTPUT

	sz = 3*width*height;

  UINT* output_buff = new UINT[sz];
  for (int i = 0; i<sz; i++) output_buff[i] = 0;


  //printf("Draw Contours Around Segments\n");
  DrawContoursAroundSegments(ubuff, seeds.labels[nr_levels-1], width, height, 0xff0000, false);//0xff0000 draws red contours
  DrawContoursAroundSegments(output_buff, seeds.labels[nr_levels-1], width, height, 0xffffff, true);//0xff0000 draws white contours

  string imageFileName = getNameFromPathWithoutExtension(string(input_file1));
  imageFileName = "./" + imageFileName + "_labels.png";
  //printf("Saving image %s\n",imageFileName.c_str());
  SaveImage(ubuff, width, height,
            imageFileName.c_str());

  imageFileName = getNameFromPathWithoutExtension(string(input_file1));
  imageFileName = "./" + imageFileName + "_boundary.png";
  //printf("Saving image %s\n",imageFileName.c_str());
  SaveImage(output_buff, width, height,
            imageFileName.c_str());
  
  string labelFileNameTxt = getNameFromPathWithoutExtension(string(input_file1));
  labelFileNameTxt = "./" + labelFileNameTxt + ".seg";
  seeds.SaveLabels_Text(labelFileNameTxt);



  delete[] ubuff;
  delete[] output_buff;



  printf("Done!\n");

  return 0;
}
