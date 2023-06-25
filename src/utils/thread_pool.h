//
// Created by chuanyi.yang@hobot.cc on 06/15/2018.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#ifndef J5_SAMPLE_UTILS_THREADPOOL_THREADPOOL_H_
#define J5_SAMPLE_UTILS_THREADPOOL_THREADPOOL_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace J5Sample {

typedef std::function<void()> TaskFunction;
struct Task {
  TaskFunction func;
  explicit Task(const TaskFunction &_task) : func(_task) {}
};

class CThreadPool {
 public:
  CThreadPool() { stop_ = false; }
  virtual ~CThreadPool() {
    stop_ = true;
    m_varCondition.notify_all();
    std::lock_guard<std::mutex> lck(m_mutThread);
    for (int i = 0; i < m_nMaxThreads; ++i) {
      m_vecThreads[i]->join();
    }
  }
  void CreatThread(int threadCount, const char* threadName = nullptr) {
    std::lock_guard<std::mutex> lck(m_mutThread);
    m_nMaxThreads = threadCount;
    m_nNumRunningThreads = 0;
    m_vecThreads.reserve(m_nMaxThreads);
    for (int i = 0; i < m_nMaxThreads; ++i) {
      auto thread = std::make_shared<std::thread>(
          std::bind(&CThreadPool::exec_loop, this, threadName));
      m_vecThreads.push_back(thread);
    }
    //  wait all threads to start, enter main loop
    while (m_nNumRunningThreads < static_cast<int>(m_vecThreads.size())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  // post an async task
  void PostTask(const TaskFunction &fun) {
    {
      std::lock_guard<std::mutex> lck(m_mutTaskQuene);
      auto task = std::make_shared<Task>(fun);
      m_setTaskQuenes.push_back(task);
    }
    m_varCondition.notify_one();  // wake worker thread(s)
  }
  size_t GetTaskNum() { return m_setTaskQuenes.size(); }
  void ClearTask() {
    std::lock_guard<std::mutex> lck(m_mutTaskQuene);
    m_setTaskQuenes.clear();
  }
  void Stop() {}
  void Start() {}

 protected:
  void exec_loop(std::string threadName) {
    ++m_nNumRunningThreads;
    if (!threadName.empty()) {
      threadName = std::to_string(m_nNumRunningThreads) + "_" + threadName;
      pthread_setname_np(pthread_self(), threadName.substr(0, 15).c_str());
    }

    while (!stop_) {
      std::shared_ptr<Task> tsk;
      {
        std::unique_lock<std::mutex> lck(m_mutTaskQuene);
        if (!stop_ && m_setTaskQuenes.size() <= 0) {
          m_varCondition.wait(lck);
        }

        if (stop_ || m_setTaskQuenes.size() <= 0) {
          continue;
        }
        tsk = m_setTaskQuenes.front();
        m_setTaskQuenes.pop_front();
      }
      //  Exec one task, wake other threads.
      tsk->func();
    }
  }

 private:
  typedef std::list<std::shared_ptr<Task>> TaskContainer;
  TaskContainer m_setTaskQuenes;
  mutable std::mutex m_mutThread;
  // a mutex for task quene operations only
  mutable std::mutex m_mutTaskQuene;

  std::condition_variable m_varCondition;
  std::atomic<int> m_nNumRunningThreads;
  typedef std::shared_ptr<std::thread> CThreadPtr;

  std::vector<CThreadPtr> m_vecThreads;
  std::atomic<bool> stop_;
  // cound set the value before starting the thread pool only
  int m_nMaxThreads;
};
}  // namespace J5Sample
#endif  // J5_SAMPLE_UTILS_THREADPOOL_THREADPOOL_H_
