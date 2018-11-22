/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include "uORBUtils.hpp"
#include "uORBManager.hpp"
#include "px4_config.h"
#include "uORBDevices.hpp"


//=========================  Static initializations =================
uORB::Manager *uORB::Manager::_Instance = nullptr;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool uORB::Manager::initialize()
{
	if (_Instance == nullptr) {
		_Instance = new uORB::Manager();
	}

	return _Instance != nullptr;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
uORB::Manager::Manager()
	: _comm_channel(nullptr)
{
	_device_master = nullptr;

#ifdef ORB_USE_PUBLISHER_RULES
	const char *file_name = "./rootfs/orb_publisher.rules";
	int ret = readPublisherRulesFromFile(file_name, _publisher_rule);

	if (ret == PX4_OK) {
		_has_publisher_rules = true;
		PX4_INFO("Using orb rules from %s", file_name);

	} else {
		PX4_ERR("Failed to read publisher rules file %s (%s)", file_name, strerror(-ret));
	}

#endif /* ORB_USE_PUBLISHER_RULES */

}

uORB::Manager::~Manager()
{
	delete _device_master;
}

uORB::DeviceMaster *uORB::Manager::get_device_master()
{
	if (!_device_master) {
		_device_master = new DeviceMaster();

		if (_device_master) {
			int ret = _device_master->init();

			if (ret != PX4_OK) {
				PX4_ERR("Initialization of DeviceMaster failed (%i)", ret);
				errno = -ret;
				delete _device_master;
				_device_master = nullptr;
			}

		} else {
			PX4_ERR("Failed to allocate DeviceMaster");
			errno = ENOMEM;
		}
	}

	return _device_master;
}

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance)
{
	/*
	 * node에 대한 path 생성 및 open 시도
	 */
	char path[orb_maxpath];
	int inst = instance;
	int ret = uORB::Utils::node_mkpath(path, meta, &inst);

	if (ret != OK) {
		errno = -ret;
		return PX4_ERROR;
	}

#if defined(__PX4_NUTTX)
	struct stat buffer;
	ret = stat(path, &buffer);
#else
	ret = px4_access(path, F_OK);

	if (ret == -1 && meta != nullptr && !_remote_topics.empty()) {
		ret = (_remote_topics.find(meta->o_name) != _remote_topics.end()) ? OK : PX4_ERROR;
	}

#endif

	if (ret == 0) {
		// topic이 있다는 것을 알지만 advertised나 publish가 안된 경우로 subscriber만 있는 경우
		// open()에서 메모리 할당은 이뤄지지 않음
		int fd = px4_open(path, 0);

		if (fd >= 0) {
			unsigned long is_published;

			if (px4_ioctl(fd, ORBIOCISPUBLISHED, (unsigned long)&is_published) == 0) {
				if (!is_published) {
					ret = PX4_ERROR;
				}
			}

			px4_close(fd);
		}
	}

	return ret;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
		int priority, unsigned int queue_size)
{
#ifdef ORB_USE_PUBLISHER_RULES

	// publish 방법 검사
	if (_has_publisher_rules) {
		const char *prog_name = px4_get_taskname();

		if (strcmp(_publisher_rule.module_name, prog_name) == 0) {
			if (_publisher_rule.ignore_other_topics) {
				if (!findTopic(_publisher_rule, meta->o_name)) {
					PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
					return (orb_advert_t)_Instance;
				}
			}

		} else {
			if (findTopic(_publisher_rule, meta->o_name)) {
				PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
				return (orb_advert_t)_Instance;
			}
		}
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	int result, fd;
	orb_advert_t advertiser;
	// advertiser를 node로 열기
	fd = node_open(meta, data, true, instance, priority);

	if (fd == PX4_ERROR) {
		PX4_ERR("%s advertise failed", meta->o_name);
		return nullptr;
	}

	// queue 크기 설정. 처음 publication 전에 완료해야만 함. 처음 advertiser가 아닌 경우 실패.
	result = px4_ioctl(fd, ORBIOCSETQUEUESIZE, (unsigned long)queue_size);

	if (result < 0 && queue_size > 1) {
		PX4_WARN("orb_advertise_multi: failed to set queue size");
	}

	// advertiser handle을 얻어서 node를 닫기
	result = px4_ioctl(fd, ORBIOCGADVERTISER, (unsigned long)&advertiser);
	px4_close(fd);

	if (result == PX4_ERROR) {
		PX4_WARN("px4_ioctl ORBIOCGADVERTISER failed. fd = %d", fd);
		return nullptr;
	}

	// remote system call에 대해서 알림
	uORB::DeviceNode::topic_advertised(meta, priority);

	// advertiser는 object를 초기화하기 위해서 반드시 초기 publish를 수행해야만 함. 
	result = orb_publish(meta, advertiser, data);

	if (result == PX4_ERROR) {
		PX4_WARN("orb_publish failed");
		return nullptr;
	}

	return advertiser;
}

int uORB::Manager::orb_unadvertise(orb_advert_t handle)
{
#ifdef ORB_USE_PUBLISHER_RULES

	if (handle == _Instance) {
		return PX4_OK; //pretend success
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	return uORB::DeviceNode::unadvertise(handle);
}

int uORB::Manager::orb_subscribe(const struct orb_metadata *meta)
{
	return node_open(meta, nullptr, false);
}

int uORB::Manager::orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	int inst = instance;
	return node_open(meta, nullptr, false, &inst);
}

int uORB::Manager::orb_unsubscribe(int fd)
{
	return px4_close(fd);
}

int uORB::Manager::orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
#ifdef ORB_USE_PUBLISHER_RULES

	if (handle == _Instance) {
		return PX4_OK; //pretend success
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	return uORB::DeviceNode::publish(meta, handle, data);
}

int uORB::Manager::orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	int ret;

	ret = px4_read(handle, buffer, meta->o_size);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int uORB::Manager::orb_check(int handle, bool *updated)
{
	// false초기화하여 px4_ioctl 수행
	*updated = false;
	return px4_ioctl(handle, ORBIOCUPDATED, (unsigned long)(uintptr_t)updated);
}

int uORB::Manager::orb_stat(int handle, uint64_t *time)
{
	return px4_ioctl(handle, ORBIOCLASTUPDATE, (unsigned long)(uintptr_t)time);
}

int uORB::Manager::orb_priority(int handle, int32_t *priority)
{
	return px4_ioctl(handle, ORBIOCGPRIORITY, (unsigned long)(uintptr_t)priority);
}

int uORB::Manager::orb_set_interval(int handle, unsigned interval)
{
	return px4_ioctl(handle, ORBIOCSETINTERVAL, interval * 1000);
}

int uORB::Manager::orb_get_interval(int handle, unsigned *interval)
{
	int ret = px4_ioctl(handle, ORBIOCGETINTERVAL, (unsigned long)interval);
	*interval /= 1000;
	return ret;
}


int uORB::Manager::node_advertise
(
	const struct orb_metadata *meta,
	int *instance,
	int priority
)
{
	int fd = -1;
	int ret = PX4_ERROR;

	// advertiser 데이터 채우기
	const struct orb_advertdata adv = { meta, instance, priority };

	/* open the control device */
	fd = px4_open(TOPIC_MASTER_DEVICE_PATH, 0);

	if (fd < 0) {
		goto out;
	}

	// 해당 object를 advertise하기
	ret = px4_ioctl(fd, ORBIOCADVERTISE, (unsigned long)(uintptr_t)&adv);

	// 이미 존재하느 ㄴ경우 PX4_OK 반환
	if ((PX4_OK != ret) && (EEXIST == errno)) {
		ret = PX4_OK;
	}

out:

	if (fd >= 0) {
		px4_close(fd);
	}

	return ret;
}

int uORB::Manager::node_open(const struct orb_metadata *meta, const void *data, bool advertiser, int *instance,
			     int priority)
{
	char path[orb_maxpath];
	int fd = -1, ret;

	// meta가 null인 경우 object는 정의되어 있지 않음. 시스템이 모르기 때문에 advertise/subscribe를 할 수 없음.
	if (nullptr == meta) {
		errno = ENOENT;
		return PX4_ERROR;
	}

	// advertiser는 반드시 초기 값을 publish 해야만 함

	if (advertiser && (data == nullptr)) {
		errno = EINVAL;
		return PX4_ERROR;
	}

	// instance를 가지고 있는 경우 새로운 node를 생성하고 instance를 설정. 여기서 open할 필요는 없음
	if (!instance || !advertiser) {
		// node에 대한 path 생성하고 open 시도
		ret = uORB::Utils::node_mkpath(path, meta, instance);

		if (ret != OK) {
			errno = -ret;
			return PX4_ERROR;
		}

		// path를 advertiser나 subscriber로 open
		fd = px4_open(path, advertiser ? PX4_F_WRONLY : PX4_F_RDONLY);

	} else {
		*instance = 0;
	}

	// node를 advertise
	if (fd < 0) {

		// node 생성
		ret = node_advertise(meta, instance, priority);

		if (ret == PX4_OK) {
			// path를 업데이트해서 node_advertise 호출하는 동안 업데이트 되도록
			ret = uORB::Utils::node_mkpath(path, meta, instance);

			if (ret != PX4_OK) {
				errno = -ret;
				return PX4_ERROR;
			}
		}
		// 성공시 다시 open 시도
		if (ret == PX4_OK) {
			fd = px4_open(path, (advertiser) ? PX4_F_WRONLY : PX4_F_RDONLY);
		}
	}


	if (fd < 0) {
		errno = EIO;
		return PX4_ERROR;
	}

	// 모든 동작이 OK인 경우, handle을 반환
	return fd;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::Manager::set_uorb_communicator(uORBCommunicator::IChannel *channel)
{
	_comm_channel = channel;

	if (_comm_channel != nullptr) {
		_comm_channel->register_handler(this);
	}
}

uORBCommunicator::IChannel *uORB::Manager::get_uorb_communicator()
{
	return _comm_channel;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_remote_topic(const char *topic_name, bool isAdvertisement)
{
	int16_t rc = 0;

	if (isAdvertisement) {
		_remote_topics.insert(topic_name);

	} else {
		_remote_topics.erase(topic_name);
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_add_subscription(const char *messageName, int32_t msgRateInHz)
{
	PX4_DEBUG("entering Manager_process_add_subscription: name: %s", messageName);

	int16_t rc = 0;
	_remote_subscriber_topics.insert(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		if (node == nullptr) {
			PX4_DEBUG("DeviceNode(%s) not created yet", messageName);

		} else {
			// node is present.
			node->process_add_subscription(msgRateInHz);
		}

	} else {
		rc = -1;
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_remove_subscription(const char *messageName)
{
	int16_t rc = -1;
	_remote_subscriber_topics.erase(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// node 이름 얻기
		if (node == nullptr) {
			PX4_DEBUG("[posix-uORB::Manager::process_remove_subscription(%d)]Error No existing subscriber found for message: [%s]",
				  __LINE__, messageName);

		} else {
			// node가 있는 경우
			node->process_remove_subscription();
			rc = 0;
		}
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_received_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = -1;
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			PX4_DEBUG("No existing subscriber found for message: [%s] nodepath:[%s]", messageName, nodepath);

		} else {
			// node is present.
			node->process_received_message(length, data);
			rc = 0;
		}
	}

	return rc;
}

bool uORB::Manager::is_remote_subscriber_present(const char *messageName)
{
#ifdef __PX4_NUTTX
	return _remote_subscriber_topics.find(messageName);
#else
	return (_remote_subscriber_topics.find(messageName) != _remote_subscriber_topics.end());
#endif
}


#ifdef ORB_USE_PUBLISHER_RULES

bool uORB::Manager::startsWith(const char *pre, const char *str)
{
	size_t lenpre = strlen(pre),
	       lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

bool uORB::Manager::findTopic(const PublisherRule &rule, const char *topic_name)
{
	const char **topics_ptr = rule.topics;

	while (*topics_ptr) {
		if (strcmp(*topics_ptr, topic_name) == 0) {
			return true;
		}

		++topics_ptr;
	}

	return false;
}

void uORB::Manager::strTrim(const char **str)
{
	while (**str == ' ' || **str == '\t') { ++(*str); }
}

int uORB::Manager::readPublisherRulesFromFile(const char *file_name, PublisherRule &rule)
{
	FILE *fp;
	static const int line_len = 1024;
	int ret = PX4_OK;
	char *line = new char[line_len];

	if (!line) {
		return -ENOMEM;
	}

	fp = fopen(file_name, "r");

	if (fp == NULL) {
		delete[](line);
		return -errno;
	}

	const char *restrict_topics_str = "restrict_topics:";
	const char *module_str = "module:";
	const char *ignore_others = "ignore_others:";

	rule.ignore_other_topics = false;
	rule.module_name = nullptr;
	rule.topics = nullptr;

	while (fgets(line, line_len, fp) && ret == PX4_OK) {

		if (strlen(line) < 2 || line[0] == '#') {
			continue;
		}

		if (startsWith(restrict_topics_str, line)) {
			//read topics list
			char *start = line + strlen(restrict_topics_str);
			strTrim((const char **)&start);
			char *topics = strdup(start);
			int topic_len = 0, num_topics = 0;

			for (int i = 0; topics[i]; ++i) {
				if (topics[i] == ',' || topics[i] == '\n') {
					if (topic_len > 0) {
						topics[i] = 0;
						++num_topics;
					}

					topic_len = 0;

				} else {
					++topic_len;
				}
			}

			if (num_topics > 0) {
				rule.topics = new const char *[num_topics + 1];
				int topic = 0;
				strTrim((const char **)&topics);
				rule.topics[topic++] = topics;

				while (topic < num_topics) {
					if (*topics == 0) {
						++topics;
						strTrim((const char **)&topics);
						rule.topics[topic++] = topics;

					} else {
						++topics;
					}
				}

				rule.topics[num_topics] = nullptr;
			}

		} else if (startsWith(module_str, line)) {
			//read module name
			char *start = line + strlen(module_str);
			strTrim((const char **)&start);
			int len = strlen(start);

			if (len > 0 && start[len - 1] == '\n') {
				start[len - 1] = 0;
			}

			rule.module_name = strdup(start);

		} else if (startsWith(ignore_others, line)) {
			const char *start = line + strlen(ignore_others);
			strTrim(&start);

			if (startsWith("true", start)) {
				rule.ignore_other_topics = true;
			}

		} else {
			PX4_ERR("orb rules file: wrong format: %s", line);
			ret = -EINVAL;
		}
	}

	if (ret == PX4_OK && (!rule.module_name || !rule.topics)) {
		PX4_ERR("Wrong format in orb publisher rules file");
		ret = -EINVAL;
	}

	delete[](line);
	fclose(fp);
	return ret;
}
#endif /* ORB_USE_PUBLISHER_RULES */
