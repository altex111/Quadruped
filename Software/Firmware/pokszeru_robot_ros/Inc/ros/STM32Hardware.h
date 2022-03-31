/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include "perif/esp.h"
#include "stm32l4xx_it.h"
#include <cassert>

extern uint32_t systick;

namespace quad
{
	class STM32Hardware {
	  protected:

		ESP* esp_Wifi;

	  public:
		STM32Hardware():
		  esp_Wifi(0){
		}

		void setESP(ESP* esp_Wifi) {
			this->esp_Wifi = esp_Wifi;
		}

		void init(){
		}

		int read()
		{
			assert(esp_Wifi);

			char a;
			if (esp_Wifi->GetRos_inData().pop(a))
			{
				return a;
			}
			return -1;
		}

		void flush(void){
			//esp_Wifi->sendFlush();
		}

		void write(uint8_t* data, int length){
			assert(esp_Wifi);

			for(uint16_t looper = 0; looper < length; looper++)
			{
				esp_Wifi->GetRos_outData().push((char&)data[looper]);
			}
		}

		unsigned long time()
		{
			return xTaskGetTickCount();
		}

	};
}

#endif

