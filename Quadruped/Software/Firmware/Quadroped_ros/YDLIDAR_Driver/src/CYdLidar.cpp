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
#include "CYdLidar.h"
#include <map>
#include <angles.h>
#include <numeric>

using namespace std;
using namespace angles;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar()
{
 	//m_FixedResolution   = true;
	m_FixedResolution   = false;
	m_Reversion         = false;
	m_Inverted          = false;//
	m_LidarType         = TYPE_TRIANGLE;
	m_MaxAngle          = 180.f;
	m_MinAngle          = -180.f;
	m_MaxRange          = 12.0;
	m_MinRange          = 0.10;
	m_FixedSize         = 720;			//todo for this all
	m_IgnoreArray.clear();
	m_PointTime         = 1e9 / 5000;
	m_OffsetTime        = 0.0;
	m_AngleOffset       = 0.0;
	last_node_time = 0;//todo getTime();
	//todo global_nodes = new node_info[MAX_SCAN_NODES];
	global_nodes = new node_info[LIDAR_MAX_PACKET_NUMBER];
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
	if (global_nodes)
	{
		delete[] global_nodes;
		global_nodes = NULL;
	}
}

//get zero angle offset value
float CYdLidar::getAngleOffset() const {
	return m_AngleOffset;
}

bool CYdLidar::isRangeValid(double reading) const {
  if (reading >= m_MinRange && reading <= m_MaxRange) {
    return true;
  }

  return false;
}

bool CYdLidar::isRangeIgnore(double angle) const {
  bool ret = false;

  for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
    if ((angles::from_degrees(m_IgnoreArray[j]) <= angle) &&
        (angle <= angles::from_degrees(m_IgnoreArray[j + 1]))) {
      ret = true;
      break;
    }
  }

  return ret;
}


/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan,
                                bool &hardwareError,
								size_t count) {
  hardwareError			= false;

#if 0
  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(200 / m_ScanFrequency);
    return false;
  }
#endif

  //wait Scan data:
  uint64_t tim_scan_start = 0;//todo getTime();
  uint64_t startTs = tim_scan_start;
  bool op_result = true;// =  lidarPtr->grabScanData(global_nodes, count); todo
  uint64_t tim_scan_end = 0;//todo getTime();

  // Fill in scan data:
  if (op_result) {
    uint64_t scan_time = m_PointTime * (count - 1);
    tim_scan_end += m_OffsetTime * 1e9;
    tim_scan_end -= m_PointTime;
    tim_scan_end -= global_nodes[0].stamp;
    tim_scan_start = tim_scan_end -  scan_time ;

    if (tim_scan_start < startTs) {
      tim_scan_start = startTs;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if ((last_node_time + m_PointTime) >= tim_scan_start) {
      tim_scan_start = last_node_time + m_PointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    int all_node_count = count;

    outscan.config.min_angle = angles::from_degrees(m_MinAngle);
    outscan.config.max_angle =  angles::from_degrees(m_MaxAngle);
    outscan.config.scan_time =  static_cast<float>(scan_time * 1.0 / 1e9);
    outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;
    outscan.stamp = tim_scan_start;
    outscan.pointsSize = 0u;

    if (m_FixedResolution) {
      all_node_count = m_FixedSize;
    }

    outscan.config.angle_increment = (outscan.config.max_angle -
                                      outscan.config.min_angle) / (all_node_count - 1);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;

    for (int i = 0; i < count; i++) {
      angle = static_cast<float>((global_nodes[i].angle_q6_checkbit >>
                                  LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;

	  range = static_cast<float>(global_nodes[i].distance_q2 / 4000.f);

      intensity = static_cast<float>(global_nodes[i].sync_quality);
      angle = angles::from_degrees(angle);

      //Rotate 180 degrees or not
      if (m_Reversion) {
        angle = angle + M_PI;
      }

      //Is it counter clockwise
      if (m_Inverted) {
        angle = 2 * M_PI - angle;
      }

      angle = angles::normalize_angle(angle);

      //ignore angle
      if (isRangeIgnore(angle)) {
        range = 0.0;
      }

      //valid range
      if (!isRangeValid(range)) {
        range = 0.0;
        intensity = 0.0;
      }

      if (angle >= outscan.config.min_angle &&
          angle <= outscan.config.max_angle) {
        LaserPoint point;
        point.angle = angle;
        point.range = range;
        point.intensity = intensity;

        if (0 == outscan.pointsSize) {
          outscan.stamp = tim_scan_start + i * m_PointTime;
        }

        outscan.points[outscan.pointsSize] = point;
        outscan.pointsSize++;
      }
    }

    return true;
  } else {
    if (op_result) {
      // Error? Retry connection
    }
  }

  return false;

}
