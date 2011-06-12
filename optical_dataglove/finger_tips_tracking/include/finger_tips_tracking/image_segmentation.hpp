/**
 * @file   image_segmentation.hpp
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
 * @brief  Segments to image to extract the finger tip position.
 * 
 * 
 */

#ifndef _IMAGE_SEGMENTATION_HPP_
#define _IMAGE_SEGMENTATION_HPP_

#include <ros/ros.h>
#include <opencv/cv.h>

namespace optical_dataglove
{
  class ImageSegmenter
  {
  public:
    ImageSegmenter();
    ~ImageSegmenter();

    std::vector<cv::Rect> segment_finger_tips(cv::Mat image_mat);
    std::vector<cv::Rect> flood_fill();
  private:    
    cv::Mat image_mat_;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
