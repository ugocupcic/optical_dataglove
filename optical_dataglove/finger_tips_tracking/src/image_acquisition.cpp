#/**
 * @file   image_acquisition.cpp
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
#include <finger_tips_tracking/image_acquisition.hpp>
#include <opencv/highgui.h>

namespace optical_dataglove
{
  ImageAcquirer::ImageAcquirer() :
    node_handle_private("~"), display_debug_image(true)
  {
    image_pretreater = boost::shared_ptr<ImagePretreater>( new ImagePretreater() );
    image_segmenter  = boost::shared_ptr<ImageSegmenter>( new ImageSegmenter() );

    image_transport = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_handle_private) );
    image_subscriber = image_transport->subscribe( "/logitech_usb_webcam/image_raw", 1, &ImageAcquirer::new_image_cb, this);
    
    if(display_debug_image)
    {
      //DISPLAYING THE IMAGE IN DEBUG
      cvNamedWindow("Pretreated Image");
      cvNamedWindow("Segmentation");
      cvStartWindowThread();
    }
  }

  ImageAcquirer::~ImageAcquirer()
  {
    if(display_debug_image)
      cvDestroyWindow("Pretreated Image");
      cvDestroyWindow("Segmentation");
  }

  void ImageAcquirer::new_image_cb(boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > const> const& image_ptr)
  {
    try
    {
      last_image = cv_bridge.imgMsgToCv(image_ptr, "rgb8");
      original_image_mat = cv::Mat(last_image);
      
      //pretreat the image
      last_image_mat = image_pretreater->pretreat(cv::Mat(last_image));
      if(display_debug_image)
        cv::imshow("Pretreated Image", last_image_mat);
      
      //segment the binary image
      std::vector<cv::Rect> position_in_image = image_segmenter->segment_finger_tips(last_image_mat);
      
      if(display_debug_image)
      {
        for (int i = 0; i < position_in_image.size(); i++) {
          cv::rectangle( original_image_mat, position_in_image[i], cv::Scalar(0,0,255), 1 );
        }
        cv::imshow("Segmentation", original_image_mat);
      }
      
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_ptr->encoding.c_str());
    }
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

