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

#include "finger_tips_tracking/image_acquisition.hpp"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "finger_tips_tracking");

  boost::shared_ptr<optical_dataglove::ImageAcquirer> im_acq = boost::shared_ptr<optical_dataglove::ImageAcquirer>( new optical_dataglove::ImageAcquirer() );

  ros::spin();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
