/*! @file camera_gimbal_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Camera and Gimbal Control API usage in a Linux environment.
 *  Shows example usage of camera commands and gimbal position/speed control
 *  APIs
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_gimbal.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using std::hex;

bool
gimbalCameraControl(Vehicle* vehicle)
{
	if (!vehicle->gimbal)
	{
		DERROR("Gimbal object does not exist.\n");
		return false;
	}
	
	else
		std::cout << "asdasd" << std::endl;		

	int responseTimeout = 0;

	GimbalContainer              gimbal;
	RotationAngle                initialAngle;
	RotationAngle                currentAngle;
	DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
	int                          pkgIndex;

	/*
	 * Subscribe to gimbal data not supported in MAtrice 100
	 */

	if (!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		// Telemetry: Verify the subscription
		ACK::ErrorCode subscribeStatus;
		subscribeStatus = vehicle->subscribe->verify(responseTimeout);
		if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
		{
			ACK::getErrorCodeMessage(subscribeStatus, __func__);
			return false;
		}

		// Telemetry: Subscribe to gimbal status and gimbal angle at freq 10 Hz
		pkgIndex                  = 0;
		int       freq            = 10;
		TopicName topicList10Hz[] = { TOPIC_GIMBAL_ANGLES, TOPIC_GIMBAL_STATUS };
		int       numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
		bool      enableTimestamp = false;

		bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
		  pkgIndex,
			numTopic,
			topicList10Hz,
			enableTimestamp,
			freq);
		if (!(pkgStatus))
		{
			return pkgStatus;
		}
		subscribeStatus =
		  vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
		if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
		{
			ACK::getErrorCodeMessage(subscribeStatus, __func__);
			// Cleanup before return
			vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
			return false;
		}
	}

	sleep(1);

	std::cout
	  << "Please note that the gimbal yaw angle you see in the telemetry is "
	     "w.r.t absolute North"
	     ", and the accuracy depends on your magnetometer calibration.\n\n";

	// Get Gimbal initial values
	if(!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		initialAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
		initialAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
		initialAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	}
	else
	{
		initialAngle.roll  = vehicle->broadcast->getGimbal().roll;
		initialAngle.pitch = vehicle->broadcast->getGimbal().pitch;
		initialAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	}

	std::cout << "Initial Gimbal rotation angle: [" << initialAngle.roll << ", "
	          << initialAngle.pitch << ", " << initialAngle.yaw << "]\n\n";

	// Re-set Gimbal to initial values
	gimbal = GimbalContainer(0, 0, 0, 20, 1, initialAngle);
	doSetGimbalAngle(vehicle, &gimbal);

	std::cout << "Setting new Gimbal rotation angle to [0,20,180] using "
	             "incremental control:\n";

	// Get current gimbal data to calc precision error in post processing
	if(!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
		currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
		currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	}
	else
	{
		currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
		currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
		currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	}

	gimbal = GimbalContainer(0, 200, 1800, 20, 0, initialAngle, currentAngle);
	doSetGimbalAngle(vehicle, &gimbal);

	if (!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
		currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
		currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	}
	else
	{
		currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
		currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
		currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	}

	displayResult(&currentAngle);


	gimbal = GimbalContainer(0, -500, 0, 20, 1, initialAngle);
	doSetGimbalAngle(vehicle, &gimbal);

	if (!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
		currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
		currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	}
	else
	{
		currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
		currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
		currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	}

	displayResult(&currentAngle);

	gimbalSpeed.roll  = 00;
	gimbalSpeed.pitch = 0;
	gimbalSpeed.yaw   = -00;
	gimbalSpeed.gimbal_control_authority = 1;
	gimbalSpeed.disable_fov_zoom = 0;
	gimbalSpeed.ignore_user_stick = 0;
	gimbalSpeed.extend_control_range = 0;
	gimbalSpeed.ignore_aircraft_motion = 0;
	gimbalSpeed.yaw_return_neutral = 0;
	gimbalSpeed.reserved0 = 0;
	gimbalSpeed.reserved1 = 0;

	int speedControlDurationMs = 4000;
	int incrementMs            = 100;
	for (int timer = 0; timer < speedControlDurationMs; timer += incrementMs)
	{
		vehicle->gimbal->setSpeed(&gimbalSpeed);
		usleep(incrementMs * 1000);
	}

	if (!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
		currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
		currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	}
	else
	{
		currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
		currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
		currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	}

	displayResult(&currentAngle);

	// Reset the position
	std::cout << "Resetting position...\n";
	gimbal = GimbalContainer(0, 0, 0, 20, 1, initialAngle);
	doSetGimbalAngle(vehicle, &gimbal);

	if (!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
		currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
		currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	}
	else
	{
		currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
		currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
		currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	}

	displayResult(&currentAngle);

	// Cleanup and exit gimbal sample
	if(!vehicle->isM100() && !vehicle->isLegacyM600())
	{
		ACK::ErrorCode ack =
		  vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
		if (ACK::getError(ack))
		{
			std::cout
			  << "Error unsubscribing; please restart the drone/FC to get back "
			     "to a clean state.\n";
		}
	}

	return true;
}

void
doSetGimbalAngle(Vehicle* vehicle, GimbalContainer* gimbal)
{
	DJI::OSDK::Gimbal::AngleData gimbalAngle;
	gimbalAngle.roll     = gimbal->roll;
	gimbalAngle.pitch    = gimbal->pitch;
	gimbalAngle.yaw      = gimbal->yaw;
	gimbalAngle.duration = gimbal->duration;
	gimbalAngle.mode |= 0;
	gimbalAngle.mode |= gimbal->isAbsolute;
	gimbalAngle.mode |= gimbal->yaw_cmd_ignore << 1;
	gimbalAngle.mode |= gimbal->roll_cmd_ignore << 2;
	gimbalAngle.mode |= gimbal->pitch_cmd_ignore << 3;

	vehicle->gimbal->setAngle(&gimbalAngle);
	// Give time for gimbal to sync
	
}

void
displayResult(RotationAngle* currentAngle)
{
	std::cout << "New Gimbal rotation angle is [";
	std::cout << currentAngle->roll << " ";
	std::cout << currentAngle->pitch << " ";
	std::cout << currentAngle->yaw;
	std::cout << "]\n\n";
}


void cameraZoomControl(Vehicle* vehicle)
{
	camera_zoom_data_type zoom_data = { 0 };

	memset(&zoom_data, 0, sizeof(zoom_data));
	zoom_data.func_index = 19;
	zoom_data.cam_index = 1;
	zoom_data.zoom_config.optical_zoom_mode = 0;
	zoom_data.zoom_config.optical_zoom_enable = 1;
	zoom_data.optical_zoom_param.step_param.zoom_step_level = 200;
	zoom_data.optical_zoom_param.step_param.zoom_step_direction = 1;
	z30_zoom_test(vehicle, &zoom_data);
	sleep(3);

	memset(&zoom_data, 0, sizeof(zoom_data));
	zoom_data.func_index = 19;
	zoom_data.cam_index = 1;
	zoom_data.zoom_config.optical_zoom_mode = 1;
	zoom_data.zoom_config.optical_zoom_enable = 1;
	zoom_data.optical_zoom_param.pos_param.zoom_pos_level = 500;
	z30_zoom_test(vehicle, &zoom_data);
	sleep(3);

	memset(&zoom_data, 0, sizeof(zoom_data));
	zoom_data.func_index = 19;
	zoom_data.cam_index = 1;
	zoom_data.zoom_config.optical_zoom_mode = 2;
	zoom_data.zoom_config.optical_zoom_enable = 1;
	zoom_data.optical_zoom_param.cont_param.zoom_cont_speed = 50;
	zoom_data.optical_zoom_param.cont_param.zoom_cont_direction = 0;
	z30_zoom_test(vehicle, &zoom_data);
	sleep(3);
}

void z30_zoom_cb(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData)
{
	uint16_t ack_data_len = recvFrame.recvInfo.len - OpenProtocol::PackageMin;
	std::cout << "ack data: " <<  __func__  << " ";
	for (uint16_t i = 0; i < ack_data_len; i++)
	{
		std::cout << "0x" << hex << (unsigned int) recvFrame.recvData.raw_ack_array[i] << " ";
	}
	std::cout << std::endl;
}

void z30_zoom_test(Vehicle* vehicle, camera_zoom_data_type *zoom)
{
	const uint8_t z30_zoom_cmd[] = { 0x01, 0x30 };
	int cbIndex = vehicle->callbackIdIndex();

	vehicle->nbCallbackFunctions[cbIndex] = (void*)z30_zoom_cb;
	vehicle->nbUserData[cbIndex] = NULL;

	vehicle->protocolLayer->send(2, vehicle->getEncryption(), z30_zoom_cmd, (unsigned char*)zoom, sizeof(camera_zoom_data_type), 500, 2, true, cbIndex);
	std::cout << "z30_zoom_test running" << std::endl;
}