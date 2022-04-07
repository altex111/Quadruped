/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#pragma once
#include <math.h>
#include "ydlidar_protocol.h"

#define MAX_SCAN_NODES (3600)

class CYdLidar {

  PropertyBuilderByName(float, MaxRange,
                        private) ///< 设置和获取激光最大测距范围(m)
  PropertyBuilderByName(float, MinRange,
                        private) ///< 设置和获取激光最小测距范围(m)
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< 设置和获取激光最大角度, 最大值180度(度)
  PropertyBuilderByName(float, MinAngle,
                        private) ///< 设置和获取激光最小角度, 最小值-180度(度)
  PropertyBuilderByName(bool, FixedResolution,
                        private) ///< 设置和获取激光是否是固定角度分辨率
  PropertyBuilderByName(bool, Reversion,
                        private) ///< 设置和获取是否旋转激光180度
  PropertyBuilderByName(bool, Inverted,
                        private)///< 设置是否反转激光方向(顺时针，　逆时针）
  PropertyBuilderByName(std::vector<float>, IgnoreArray,
                        private) ///< 设置和获取激光剔除点
  PropertyBuilderByName(float, OffsetTime,
                        private) ///<
  PropertyBuilderByName(int, LidarType,
                        private) ///< 雷达类型

 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan,
                       bool &hardwareError,
					   size_t count);

  //get zero angle offset value
  float getAngleOffset() const;

  node_info* Get_globalNodes() {return global_nodes;}

 protected:
  /*!
    * @brief isRangeValid
    * @param reading
    * @return
    */
  bool isRangeValid(double reading) const;

  /*!
   * @brief isRangeIgnore
   * @param angle
   * @return
   */
  bool isRangeIgnore(double angle) const;

 private:
  int     m_FixedSize ;
  float   m_AngleOffset;
  uint64_t m_PointTime;
  uint64_t last_node_time;
  node_info *global_nodes;
};	// End of class

