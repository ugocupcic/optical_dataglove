/**
 * @file   image_segmentation.cpp
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
 * @brief  Segments the image to extract the finger tip position.
 * 
 * 
 */

#include <finger_tips_tracking/image_segmentation.hpp>

namespace optical_dataglove
{
  ImageSegmenter::ImageSegmenter()
  {
  }

  ImageSegmenter::~ImageSegmenter()
  {
  }

  finger_tip::Position ImageSegmenter::segment_finger_tips(cv::Mat image_mat)
  {
    finger_tip::Position position_in_image;
    position_in_image.x_img = 100;
    position_in_image.y_img = 100;

    position_in_image.radius = 10;

    return position_in_image;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


