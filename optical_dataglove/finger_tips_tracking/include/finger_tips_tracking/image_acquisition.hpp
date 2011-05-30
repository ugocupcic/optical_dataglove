/**
 * @file   image_acquisition.hpp
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
 * @brief  This is where the images are acquired
 * 
 * 
 */

#ifndef _IMAGE_ACQUIRER_HPP_
#define _IMAGE_ACQUIRER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <boost/smart_ptr.hpp>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>

#include "finger_tips_tracking/image_pretreatment.hpp"

namespace optical_dataglove
{
  class ImageAcquirer
  {
  public:
    ImageAcquirer();
    ~ImageAcquirer();

  private:
    void new_image_cb(boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > const> const& image_ptr);

    boost::shared_ptr<image_transport::ImageTransport> image_transport;
    image_transport::Subscriber image_subscriber;

    boost::shared_ptr<ImagePretreater> image_pretreater;

    IplImage* last_image;
    cv::Mat last_image_mat;
    sensor_msgs::CvBridge cv_bridge;
    
    ros::NodeHandle node_handle, node_handle_private;

    bool display_debug_image;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
