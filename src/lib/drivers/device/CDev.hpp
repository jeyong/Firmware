/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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

/**
 * @file CDev.hpp
 *
 * Definitions for the generic base classes in the device framework.
 */

#ifndef _DEVICE_CDEV_HPP
#define _DEVICE_CDEV_HPP

#include "Device.hpp"

#include <px4_config.h>
#include <px4_posix.h>

#ifdef __PX4_NUTTX
#include "nuttx/cdev_platform.hpp"
#else
#include "posix/cdev_platform.hpp"
#endif

/**
 * Namespace encapsulating all device framework classes, functions and data.
 */
namespace device
{

/**
 * Abstract class for any character device
 */
class __EXPORT CDev : public Device
{
public:
	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 */
	CDev(const char *name, const char *devname); // TODO: dagar remove name and Device inheritance

	virtual ~CDev();

	virtual int	init();

	/**
	 * device의 open 처리
	 * Handle an open of the device.
	 *
	 * 이 함수는 device의 모든 open에 대해서 호출됨. 기본 구현은 _open_count를 유지하고 항상 OK 반환.
	 * This function is called for every open of the device. The default
	 * implementation maintains _open_count and always returns OK.
	 *
	 * @param filep		Pointer to the NuttX file structure. // NuttX 파일 구조체에 대한 포인터
	 * @return		OK if the open is allowed, -errno otherwise.
	 */
	virtual int	open(file_t *filep);

	/**
	 * device의 close에 대한 처리
	 * Handle a close of the device.
	 *
	 * 기본 구현은 _open_count를 유지하며 0이 아닌한 OK를 반환
	 * This function is called for every close of the device. The default
	 * implementation maintains _open_count and returns OK as long as it is not zero.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @return		OK if the close was successful, -errno otherwise.
	 */
	virtual int	close(file_t *filep);

	/**
	 * device의 read 구현
	 * Perform a read from the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filep		Pointer to the NuttX file structure. // NuttX 파일 구조체에 대한 포인터
	 * @param buffer	Pointer to the buffer into which data should be placed. // data가 들어가는 buffer에 대한 포인터
	 * @param buflen	The number of bytes to be read. // 읽은 byte의 수
	 * @return		The number of bytes read or -errno otherwise.
	 */
	virtual ssize_t	read(file_t *filep, char *buffer, size_t buflen);

	/**
	 * Perform a write to the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @param buffer	Pointer to the buffer from which data should be read.
	 * @param buflen	The number of bytes to be written.
	 * @return		The number of bytes written or -errno otherwise.
	 */
	virtual ssize_t	write(file_t *filep, const char *buffer, size_t buflen);

	/**
	 * Perform a logical seek operation on the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @param offset	The new file position relative to whence.
	 * @param whence	SEEK_OFS, SEEK_CUR or SEEK_END.
	 * @return		The previous offset, or -errno otherwise.
	 */
	virtual off_t	seek(file_t *filep, off_t offset, int whence);

	/**
	 * ioctl 동작 구현
	 * Perform an ioctl operation on the device.
	 *
	 * 기본 구현은 DIOC_GETPRIV 처리. 하위 class는 자기가 처리안하는 command에 대해서 기본 구현을 호출해야함.
	 * The default implementation handles DIOC_GETPRIV, and otherwise
	 * returns -ENOTTY. Subclasses should call the default implementation
	 * for any command they do not handle themselves.
	 *
	 * @param filep		Pointer to the NuttX file structure. // NuttX 파일 구조체
	 * @param cmd		The ioctl command value. //명령
	 * @param arg		The ioctl argument value. // 인자
	 * @return		OK on success, or -errno otherwise.
	 */
	virtual int	ioctl(file_t *filep, int cmd, unsigned long arg);

	/**
	 * poll setup/teardown 동작 수행
	 * Perform a poll setup/teardown operation.
	 *
	 * 내부적인 처리로 일반적으로 override하지 말아야 함. 
	 * This is handled internally and should not normally be overridden.
	 *
	 * @param filep		Pointer to the internal file structure. // 내부 파일 구조체에 대한 포인터
	 * @param fds		Poll descriptor being waited on. // 대기하고 있는 Poll descriptor 
	 * @param setup		True if this is establishing a request, false if // 요청이 성립되면 true 그 외에는 false
	 *			it is being torn down.
	 * @return		OK on success, or -errno otherwise.
	 */
	virtual int	poll(file_t *filep, px4_pollfd_struct_t *fds, bool setup);

protected:
	/**
	 * 기본 cdev 파일 동작 테이블에 대한 table. clone devices를 등록하는데 유용.
	 * Pointer to the default cdev file operations table; useful for
	 * registering clone devices etc.
	 */
	static const px4_file_operations_t	fops;

	/**
	 * poll event에 대한 device 현재 상태를 검사
	 * Check the current state of the device for poll events from the
	 * perspective of the file.
	 *
	 * poll이 즉시 return해야하는지를 결정하는 setup이 되는 경우 이 함수는 기본 poll() 구현에서 호출됨. 
	 * This function is called by the default poll() implementation when
	 * a poll is set up to determine whether the poll should return immediately.
	 *
	 * The default implementation returns no events.
	 *
	 * @param filep		The file that's interested. // 관심있는 file
	 * @return		The current set of poll events. // 현재 poll event의 집합
	 */
	virtual pollevent_t poll_state(file_t *filep);

	/**
	 * 새로운 poll event들을 report
	 * Report new poll events.
	 *
	 * 이 함수는 poll waiter가 원하는 경우, device 변경되는 경우 언제든 호출되어야 함. 
	 * This function should be called anytime the state of the device changes
	 * in a fashion that might be interesting to a poll waiter.
	 *
	 * @param events	The new event(s) being announced.
	 */
	virtual void	poll_notify(pollevent_t events);

	/**
	 * poll_notify의 내부 구현
	 * Internal implementation of poll_notify.
	 *
	 * @param fds		A poll waiter to notify. // notify할 poll waiter
	 * @param events	The event(s) to send to the waiter. // waiter에게 전달할 event
	 */
	virtual void	poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events);

	/**
	 * 처음 open하는 경우 noti
	 * Notification of the first open.
	 *
	 * 이 함수는 device가 open될때 호출된다. driver lock은 해당 호출 동안 hold 상태가 된다.  
	 * This function is called when the device open count transitions from zero
	 * to one.  The driver lock is held for the duration of the call.
	 *
	 * 기본 구현에서는 OK 반환
	 * The default implementation returns OK.
	 *
	 * @param filep		Pointer to the NuttX file structure // NuttX 파일 structure에 대한 pointer
	 * @return		OK if the open should proceed, -errno otherwise. // open이 처리되면 OK
	 */
	virtual int	open_first(file_t *filep);

	/**
	 * 마지막 close시에 noti
	 * Notification of the last close.
	 *
	 * This function is called when the device open count transitions from
	 * one to zero.  The driver lock is held for the duration of the call.
	 *
	 * The default implementation returns OK.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @return		OK if the open should return OK, -errno otherwise.
	 */
	virtual int	close_last(file_t *filep);

	/**
	 * class device 이름을 등록하고 자동으로 device class instance suffix를 추가.
	 * Register a class device name, automatically adding device
	 * class instance suffix if need be.
	 *
	 * @param class_devname   Device class name
	 * @return class_instamce Class instance created, or -errno on failure // 생성된 class instance
	 */
	virtual int register_class_devname(const char *class_devname);

	/**
	 * Register a class device name, automatically adding device
	 * class instance suffix if need be.
	 *
	 * @param class_devname   Device class name
	 * @param class_instance  Device class instance from register_class_devname()
	 * @return		  OK on success, -errno otherwise
	 */
	virtual int unregister_class_devname(const char *class_devname, unsigned class_instance);

	/**
	 * Get the device name.
	 *
	 * @return the file system string of the device handle
	 */
	const char	*get_devname() { return _devname; }

	/**
	 * driver lock을 가지기
	 * Take the driver lock.
	 *
	 * 각 dirver instance는 자신의 lock/semaphore를 가짐.
	 * Each driver instance has its own lock/semaphore.
	 *
	 * signal에 의해서 interrupt될 수 있음. 
	 * Note that we must loop as the wait may be interrupted by a signal.
	 *
	 * Careful: lock() calls cannot be nested!
	 */
	void		lock()
	{
		do {} while (px4_sem_wait(&_lock) != 0);
	}

	/**
	 * driver lock 해제
	 * Release the driver lock.
	 */
	void		unlock()
	{
		px4_sem_post(&_lock);
	}

	px4_sem_t	_lock; /**< lock to protect access to all class members (also for derived classes) */

	bool		_pub_blocked{false};		/**< true if publishing should be blocked */

private:
	const char	*_devname;		/**< device node name */
	bool		_registered{false};		/**< true if device name was registered */

	uint8_t		_max_pollwaiters{0}; /**< size of the _pollset array */
	uint16_t	_open_count{0};		/**< number of successful opens */

	px4_pollfd_struct_t	**_pollset{nullptr};

	/**
	 * slot에 pollwaiter를 저장하며ㅑ 나중에 찾을 수 있음.
	 * Store a pollwaiter in a slot where we can find it later.
	 *
	 * pollset을 필요에 따라서 확장. driver locked을 가지고 호출해야만 함.
	 * Expands the pollset as required.  Must be called with the driver locked.
	 *
	 * @return		OK, or -errno on error.
	 */
	int		store_poll_waiter(px4_pollfd_struct_t *fds);

	/**
	 * poll waiter 제거
	 * Remove a poll waiter.
	 *
	 * @return		OK, or -errno on error.
	 */
	int		remove_poll_waiter(px4_pollfd_struct_t *fds);

	/* do not allow copying this class */
	CDev(const CDev &);
	CDev operator=(const CDev &);
};

} // namespace device

// class instance for primary driver of each class
enum CLASS_DEVICE {
	CLASS_DEVICE_PRIMARY = 0,
	CLASS_DEVICE_SECONDARY = 1,
	CLASS_DEVICE_TERTIARY = 2
};

#endif /* _DEVICE_CDEV_HPP */
