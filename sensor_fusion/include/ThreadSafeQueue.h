#pragma once

#include <glog/logging.h>

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#define _GLIBCXX_DTS2_CONDITION_VARIABLE_ANY

template <typename QueueType>
class ThreadSafeQueue {
public:

// Constructor
ThreadSafeQueue() {
  shut_down_ = false;
}

// Destructor
~ThreadSafeQueue() {
  shut_down_ = true;
  cond_not_empty_.notify_all();
  cond_not_full_.notify_all();
}

void ShutDown() {
  shut_down_ = true;
  cond_not_empty_.notify_all();
  cond_not_full_.notify_all();
}

void Resume() {
  shut_down_ = false;
  cond_not_empty_.notify_all();
  cond_not_full_.notify_all();
}

size_t Size() {
  std::unique_lock<std::mutex> lk(mutex_);
  size_t size = queue_.size();
  return size;
}

bool Empty() {
  std::unique_lock<std::mutex> lk(mutex_);
  bool empty = queue_.empty();
  return empty;
}

// Push the value to queue if the queue actual size is less than max_queue_size(not full), else block 
bool PushBlockingIfFull(const QueueType& value, size_t max_queue_size) {
  while (!shut_down_) {
    std::unique_lock<std::mutex> lk(mutex_);
    size_t size = queue_.size();
    if (size >= max_queue_size) {
      cond_not_full_.wait(lk);
    }
    if (size >= max_queue_size) {
      // A spurious wakeup
      //lock.unlock();
      continue;
    }
    // Push to queue
    queue_.push(value);
    // Signal other thread that queue is available
    //lock.unlock();
    cond_not_empty_.notify_one();
    return true;
  }
  return false;
}

void PushNonBlocking(const QueueType& value) {
  std::unique_lock<std::mutex> lk(mutex_);
  queue_.push(value);
  cond_not_empty_.notify_one();
}

bool PopBlocking(QueueType& value) {
  while (!shut_down_) {
    std::unique_lock<std::mutex> lk(mutex_);
    if (queue_.empty()) {
      cond_not_empty_.wait(lk);
    }
    if (queue_.empty()) {
      //lock.unlock();
      continue;
    }
    value = std::move(queue_.front());
    queue_.pop();
    //lock.unlock();
    cond_not_full_.notify_one();
    return true;
  }
  return false;
}

bool PopNonBlocking(QueueType& value) {
  std::unique_lock<std::mutex> lk(mutex_);
  if (queue_.empty()) {
    return false;
  }
  value = std::move(queue_.front());
  queue_.pop();
  return true;
}

bool PopNonBlocking() {
  std::unique_lock<std::mutex> lk(mutex_);
  if (queue_.empty()) {
    return false;
  }
  queue_.pop();
  cond_not_full_.notify_one();
  return true;
}

bool GetCopyOfFrontBlocking(QueueType& value) {
  while (!shut_down_) {
    std::unique_lock<std::mutex> lk(mutex_);
    if (queue_.empty()) {
      cond_not_empty_.wait(lk);
    }
    if (queue_.empty()) {
      continue;
    }
    value = queue_.front();
    return true;
  }
  return false;
}

private:
  std::queue<QueueType> queue_; // Actual queue
  std::mutex mutex_; // The queue mutex
  std::condition_variable cond_not_empty_; 
  std::condition_variable cond_not_full_;
  std::atomic_bool shut_down_; // Flag if shutdown is requested
};

