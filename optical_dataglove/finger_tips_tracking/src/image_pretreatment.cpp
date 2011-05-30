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
    convert_to_hsv(image_mat);

    img_H = cv::Mat::zeros(transformed_image.rows, transformed_image.cols, CV_8U);
    img_S = cv::Mat::zeros(transformed_image.rows, transformed_image.cols, CV_8U);
    img_V = cv::Mat::zeros(transformed_image.rows, transformed_image.cols, CV_8U);

    return transformed_image;
  }

  cv::Mat ImagePretreater::convert_to_hsv(cv::Mat image_mat)
  {    
    cv::cvtColor (image_mat, transformed_image, CV_BGR2HSV);
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

