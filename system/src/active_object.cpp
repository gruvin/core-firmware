/**
 ******************************************************************************
  Copyright (c) 2015 Particle Industries, Inc.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

#include "active_object.h"

#include "spark_wiring_interrupts.h"

#if PLATFORM_THREADING

#include <string.h>
#include "concurrent_hal.h"
#include "timer_hal.h"

void ActiveObjectBase::start_thread()
{
    // prevent the started thread from running until the thread id has been assigned
    // so that calls to isCurrentThread() work correctly
    set_thread(std::thread(run_active_object, this));
    while (!started) {
        os_thread_yield();
    }
}


void ActiveObjectBase::run()
{
    std::lock_guard<std::mutex> lck (_start);
    started = true;

    uint32_t last_background_run = 0;
    for (;;)
    {
    	uint32_t now;
        if (!process())
		{
        	configuration.background_task();
        }
        else if ((now=HAL_Timer_Get_Milli_Seconds())-last_background_run > configuration.take_wait)
        {
        	last_background_run = now;
        	configuration.background_task();
        }
    }
}

bool ActiveObjectBase::process()
{
    bool result = false;
    Item item = nullptr;
    if (take(item) && item)
    {
        Message& msg = *item;
        msg();
        result = true;
    }
    return result;
}

void ActiveObjectBase::run_active_object(ActiveObjectBase* object)
{
    object->run();
}

#endif // PLATFORM_THREADING

ISRTaskQueue::ISRTaskQueue(size_t size) :
        tasks_(nullptr),
        availTask_(nullptr),
        firstTask_(nullptr),
        lastTask_(nullptr) {
    if (size) {
        // Initialize pool of task objects
        tasks_ = new(std::nothrow) Task[size];
        if (tasks_) {
            for (size_t i = 0; i < size; ++i) {
                Task* t = tasks_ + i;
                if (i != size - 1) {
                    t->next = t + 1;
                } else {
                    t->next = nullptr;
                }
            }
        }
        availTask_ = tasks_;
    }
}

ISRTaskQueue::~ISRTaskQueue() {
    delete[] tasks_;
}

bool ISRTaskQueue::enqueue(TaskFunc func, void* data) {
    SPARK_ASSERT(func && HAL_IsISR());
    ATOMIC_BLOCK() { // Prevent preemption of the current ISR
        // Take task object from the pool
        Task* t = availTask_;
        if (!t) {
            return false;
        }
        availTask_ = t->next;
        // Initialize task object
        t->func = func;
        t->data = data;
        t->next = nullptr;
        // Add task object to the queue
        if (lastTask_) {
            lastTask_->next = t;
        } else { // The queue is empty
            firstTask_ = t;
        }
        lastTask_ = t;
    }
    return true;
}

bool ISRTaskQueue::process() {
    SPARK_ASSERT(!HAL_IsISR());
    TaskFunc func = nullptr;
    void* data = nullptr;
    ATOMIC_BLOCK() {
        // Take task object from the queue
        Task *t = firstTask_;
        if (!t) {
            return false;
        }
        firstTask_ = firstTask_->next;
        if (!firstTask_) {
            lastTask_ = nullptr;
        }
        func = t->func;
        data = t->data;
        // Return task object to the pool
        t->next = availTask_;
        availTask_ = t;
    }
    // Invoke task function
    func(data);
    return true;
}
