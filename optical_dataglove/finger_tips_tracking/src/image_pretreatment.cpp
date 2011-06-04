/**
 * @file   image_pretreatment.cpp
 * @author Ugo Cupcic <ugocupcic@gmail.com>
 * @date   Sun May 29 15:16:04 2011
 * 
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 * @brief  Pretreats the images to get an image which is easier to interpret.
 * 
 * 
 */
#include <finger_tips_tracking/image_pretreatment.hpp>

namespace optical_dataglove
{
  ImagePretreater::ImagePretreater()
  {
  }

  ImagePretreater::~ImagePretreater()
  {
  }

  cv::Mat ImagePretreater::pretreat(cv::Mat image_mat)
  {
    start_image = image_mat;
    convert_to_hsv();
    filter_saturation();
    erode_then_dilate();

    return transformed_image;
  }

  void ImagePretreater::convert_to_hsv()
  {    
    cv::cvtColor (start_image, transformed_image, CV_BGR2HSV);
  }

  void ImagePretreater::filter_saturation()
  {   
    for(int i = 0; i < transformed_image.rows; i++)
    {
      for(int j = 0; j < transformed_image.cols; j++)
      {
        if( transformed_image.at<cv::Vec3b>(i,j)[1] < 200 )
        {
          // Clear pixel blue output channel
          transformed_image.at<cv::Vec3b>(i,j)[0] = 0;
          transformed_image.at<cv::Vec3b>(i,j)[1] = 0;
          transformed_image.at<cv::Vec3b>(i,j)[2] = 0;
        }
      }
    }
  }  

  void ImagePretreater::erode_then_dilate()
  {
    cv::erode(transformed_image, tmp_image, cv::Mat(), cv::Point(-1,-1), 10);
    transformed_image = tmp_image;
    cv::dilate(transformed_image, tmp_image, cv::Mat(), cv::Point(-1,-1), 10);
    transformed_image = tmp_image;
    
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

