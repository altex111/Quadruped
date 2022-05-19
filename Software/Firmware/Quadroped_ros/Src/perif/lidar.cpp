#include "perif/lidar.h"

#define LIDAR_1_MS_WAIT 1
#define LIDAR_2_MS_WAIT 2

namespace quadroped
{
LIDAR::LIDAR(USART_TypeDef *uart):
		m_LIDARcomm(uart),
		laser(),
		scan_msg(),
		frame_id("base_laser")
{
	scan_msg.ranges = scan_msg_ranges;

	scan_msg.header.frame_id = frame_id;
}

UART& LIDAR::getUART()
{
	return m_LIDARcomm;
}

void LIDAR::Start()
{
	m_LIDARcomm.ClearInFifo();
}

bool LIDAR::Process()
{
	if(false == Rx_ProcessFindStart())
	{
		return false;
	}

	/* Wait for packets, until second start packet arrives */
	rxProcessCounter = 0u;
	if(false == (Rx_ProcessMeasurement(laser.Get_globalNodes(), rxProcessCounter)))
	{
		m_LIDARcomm.ClearInFifo();
		lidar_inData.clear();
		return false;
	}

	/* Call doProcessSimple */
	bool hardError;
	laser.doProcessSimple(scan, hardError, rxProcessCounter);

	/* Create ROS compatible data */
	//todo scan_msg.header.frame_id = "laser_frame";
	//scan_msg.header.frame_id = "map";
    scan_msg.angle_min =(scan.config.min_angle);
    scan_msg.angle_max = (scan.config.max_angle);
    scan_msg.angle_increment = (scan.config.angle_increment);
    scan_msg.scan_time = scan.config.scan_time;
    scan_msg.time_increment = scan.config.time_increment;
    scan_msg.range_min = (scan.config.min_range);
    scan_msg.range_max = (scan.config.max_range);
    int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
    //scan_msg.ranges.resize(size);
    //scan_msg.intensities.resize(size);
    for(int i=0; i < scan.pointsSize; i++)
    {
		int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
		if(index >=0 && index < size)
		{
			scan_msg.ranges[index] = scan.points[i].range;
        }
    }
    scan_msg.ranges_length = scan.pointsSize;

	m_LIDARcomm.ClearInFifo();
	lidar_inData.clear();
    return true;
}

bool LIDAR::Rx_ProcessFindStart()
{
	char packetHeaderLow = 0;
	char packetHeaderHigh = 0u;

	/* Find the first packet header by byte to align them properly */
	while(m_LIDARcomm.ReadBuffer(&packetHeaderLow,1))
	{
		if((LIDAR_PACKET_HEADER & 0xFF) == packetHeaderLow)
		{
			/* If the read in byte is equal to the lower byte of the packet header, than the next byte should be tested as well */
			/* Set packetHeaderHigh to meet the while statement requirement for the first cycle - i dont like do-while :( */
			packetHeaderHigh = LIDAR_PACKET_HEADER & 0xFF;
			while((LIDAR_PACKET_HEADER & 0xFF) == packetHeaderHigh)
			{
				/* Function should not return until if the lower byte has already been found */
				while(false == m_LIDARcomm.ReadBuffer(&packetHeaderHigh,1))
				{
					vTaskDelay( LIDAR_1_MS_WAIT / portTICK_PERIOD_MS );
				}

				if((LIDAR_PACKET_HEADER >> 8) == packetHeaderHigh)
				{
					/* Packet header found */
					uint16_t firstPacketHeader = packetHeaderLow + (packetHeaderHigh << 8);
					uint16_t firstPacketCTLS = 0u;

					while(false == m_LIDARcomm.ReadBuffer((char*)&firstPacketCTLS,2))
					{
						vTaskDelay( LIDAR_1_MS_WAIT / portTICK_PERIOD_MS );
					}

					if(CT_RingStart == (firstPacketCTLS & 0x01))
					{
						/* Push the first packet header and CT+LS into the inData buffer */
						lidar_inData.push(firstPacketHeader);
						lidar_inData.push(firstPacketCTLS);
						return true;
					}
				}
			}
		}
	}

	return false;
}

bool LIDAR::Rx_ProcessMeasurement(node_info *nodebuffer, size_t &count)
{
	uint8_t startPacketCounter = 0u;

	/* Create local variable to store the node info */
	node_info local_node;
	node_packages local_package;
	uint16_t inData_word = 0u;

	m_LIDARcomm.CopyInFifo(lidar_inData);
	count = 0u;

	/* This cycle should terminate when the second start message is found */
	while(startPacketCounter < 2)
	{
		Pop_InData(inData_word);

		if(LIDAR_PACKET_HEADER == inData_word)
		{
			/* Packet header found */
			uint8_t scan_frequence = 0u;
			uint8_t packageSampleCounter = 0u;
			uint16_t FirstSampleAngle = 0u;
			uint16_t LastSampleAngle = 0u;
			float IntervalSampleAngle = 0u;
			uint16_t CheckSum;
			int32_t AngleCorrectForDistance = 0;
			uint16_t CheckSumCal = LIDAR_PACKET_HEADER;

			local_package.package_Head = inData_word;

			/* Get CT+LSN word */
			Pop_InData(inData_word);
			uint8_t package_type = inData_word & 0x01;
			local_package.package_CT = package_type;
			if(CT_RingStart == package_type)
			{
				scan_frequence = (inData_word & 0xFE) >> 1;

				/* Start packet found, set return value to true */
				startPacketCounter++;

				if(startPacketCounter >= 2)
				{
					/* If second start packet found, stop the execution */
					break;
				}
			}
			packageSampleCounter = inData_word >> 8u;
			local_package.nowPackageNum = packageSampleCounter;
			CheckSumCal ^= inData_word;

			if((count + packageSampleCounter) > LIDAR_MAX_PACKET_NUMBER)
			{
				/* This line is for debug purposes */
				count += packageSampleCounter;
				return false;
			}

			/* Get FSA (Starting angle) word */
			Pop_InData(inData_word);
			local_package.packageFirstSampleAngle = inData_word;
			if(inData_word & LIDAR_RESP_MEASUREMENT_CHECKBIT)
			{
				/* todo error report */
			}
			FirstSampleAngle = inData_word >> 1;
			CheckSumCal ^= inData_word;

			/* Get LSA (End angle) word */
			Pop_InData(inData_word);
			local_package.packageLastSampleAngle = inData_word;
			if(inData_word & LIDAR_RESP_MEASUREMENT_CHECKBIT)
			{
				/* todo error report */
			}
			LastSampleAngle = inData_word >> 1;
			CheckSumCal ^= inData_word;

			/* Calculate interval sample angle */
			if (packageSampleCounter == 1)
			{
			  IntervalSampleAngle = 0;
			}
			else
			{
				if (LastSampleAngle < FirstSampleAngle)
				{
					if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64))
					{
						IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle - FirstSampleAngle) / ((packageSampleCounter - 1) * 1.0));
						IntervalSampleAngle_LastPackage = IntervalSampleAngle;
					}
					else
					{
						IntervalSampleAngle = IntervalSampleAngle_LastPackage;
					}
				}
				else
				{
					IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((packageSampleCounter - 1) * 1.0));
					IntervalSampleAngle_LastPackage = IntervalSampleAngle;
				}
			}

			/* Get CS (Check code) word */
			Pop_InData(inData_word);
			local_package.checkSum = inData_word;
			CheckSum = inData_word;

			/* Get Si (Sampled data) */
			/*for(uint32_t sampleCounter = 0u; sampleCounter < packageSampleCounter; sampleCounter++)
			{
				Pop_InData(inData_word);
				local_package.packageSampleDistance[sampleCounter] = inData_word;
			}*/

			/* --- The whole packet has arrived --- */
			/* --- Create node info for every packet --- */

			/* Create a node info from every sample */
			for(uint32_t sampleCounter = 0u; sampleCounter < packageSampleCounter; sampleCounter++)
			{
				Pop_InData(inData_word);
				CheckSumCal ^= inData_word;
				local_package.packageSampleDistance[sampleCounter] = inData_word;

				local_node.scan_frequence = scan_frequence;
				local_node.stamp = 0;

				local_node.distance_q2 = local_package.packageSampleDistance[sampleCounter];
				local_node.sync_quality = ((uint16_t)(0xfc | local_package.packageSampleDistance[sampleCounter] & 0x0003)) << LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

			    if (local_node.distance_q2 != 0)
			    {
					AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - (local_node.distance_q2 / 4.0))) / 155.3) / (local_node.distance_q2 / 4.0))) * 180.0 / 3.1415) * 64.0);
			    }
			    else
			    {
			    	AngleCorrectForDistance = 0;
			    }

			    float sampleAngle = IntervalSampleAngle * sampleCounter;

				if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0)
				{
					local_node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
				}
				else
				{
					if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040)
					{
						local_node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
					}
					else
					{
						local_node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
					}
				}

				nodebuffer[count+sampleCounter] = local_node;
			}

			/* Set the count argument */
			count += packageSampleCounter;

			if(CheckSumCal != CheckSum)
			{
				return false;
			}
		}
		else
		{
			/* No packet header, check next word */
		}

		/* todo test wait for next packet */
		vTaskDelay( 5 / portTICK_PERIOD_MS );
	}

	return true;
}

void LIDAR::Pop_InData(uint16_t& inData)
{
	while(false == lidar_inData.pop(inData))
	{
		vTaskDelay( LIDAR_1_MS_WAIT / portTICK_PERIOD_MS );
		m_LIDARcomm.CopyInFifo(lidar_inData);
	}
}

}
