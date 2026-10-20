//
// Created by xiang on 2022/2/9.
//

#ifndef ASYNC_MESSAGE_PROCESS_H
#define ASYNC_MESSAGE_PROCESS_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include <glog/logging.h>

namespace lightning::sys {
using UL = std::unique_lock<std::mutex>;

/**
 * 异步消息处理类
 * 内部有线程和队列机制，保证回调是串行的
 * @tparam T
 *
 * NOTE skip设为1的时候实际不会跳帧。。设为2的时候实际走一帧跳一帧
 */
template <typename T>
class AsyncMessageProcess {
   public:
    using ProcFunc = std::function<void(const T&)>;  // 消息回调函数
    AsyncMessageProcess() = default;
    AsyncMessageProcess(ProcFunc proc_func, std::string name = "");
    // std::thread 析构时若仍 joinable 会直接 std::terminate；本类此前缺少析构
    ~AsyncMessageProcess() { Quit(); }

    /// 设置处理函数
    void SetProcFunc(ProcFunc proc_func) { custom_func_ = proc_func; }

    /// 设置队列最大长度
    void SetMaxSize(size_t size) { max_size_ = size; }

    /// 开始处理消息
    void Start();

    /// 添加一条消息
    void AddMessage(const T& msg);

    /// 退出
    void Quit();

    /// 清空跳帧计数器，下一个数据会立即执行
    void CleanSkipCnt();

    void SetName(std::string name) { name_ = std::move(name); }
    void SetSkipParam(bool enable_skip, int skip_num) { enable_skip_ = enable_skip, skip_num_ = skip_num; }

    AsyncMessageProcess(const AsyncMessageProcess&) = delete;
    void operator=(const AsyncMessageProcess&) = delete;

   private:
    void ProcLoop();

    std::thread proc_;
    std::mutex mutex_;
    std::condition_variable cv_msg_;
    std::deque<T> msg_buffer_;
    // 工作线程与调用线程（AddMessage/Quit）无锁并发访问，必须用原子量：
    // 非原子 bool 的数据竞争是 UB，且编译器可能把 exit_flag_ 缓存在寄存器中，
    // 使 ProcLoop 的 while (!exit_flag_) 永不退出、join() 永久阻塞。
    std::atomic_bool update_flag_{false};
    std::atomic_bool exit_flag_{false};
    size_t max_size_ = 100;  // 40
    std::string name_;

    /// 跳帧
    bool enable_skip_ = false;
    int skip_num_ = 0;
    int skip_cnt_ = 0;

    ProcFunc custom_func_;
};

template <typename T>
void AsyncMessageProcess<T>::CleanSkipCnt() {
    UL lock(mutex_);
    skip_cnt_ = 0;
}

template <typename T>
AsyncMessageProcess<T>::AsyncMessageProcess(AsyncMessageProcess::ProcFunc proc_func, std::string name) {
    custom_func_ = std::move(proc_func);
    name_ = name;
}

template <typename T>
void AsyncMessageProcess<T>::Start() {
    exit_flag_.store(false);
    update_flag_.store(false);
    proc_ = std::thread([this]() { ProcLoop(); });
}

template <typename T>
void AsyncMessageProcess<T>::ProcLoop() {
    while (!exit_flag_) {
        UL lock(mutex_);
        cv_msg_.wait(lock, [this]() { return update_flag_.load() || exit_flag_.load(); });

        // take the message and process it
        auto buffer = msg_buffer_;
        msg_buffer_.clear();
        update_flag_ = false;
        lock.unlock();

        // 处理之
        for (const auto& msg : buffer) {
            custom_func_(msg);
        }
    }
}

template <typename T>
void AsyncMessageProcess<T>::AddMessage(const T& msg) {
    UL lock(mutex_);
    if (enable_skip_ && skip_num_ > 0) {
        if (skip_cnt_ != 0) {
            skip_cnt_++;
            skip_cnt_ = skip_cnt_ % skip_num_;
            return;
        }

        skip_cnt_++;
        skip_cnt_ = skip_cnt_ % skip_num_;
    }

    msg_buffer_.push_back(msg);
    while (msg_buffer_.size() > max_size_) {
        LOG(INFO) << name_ << " exceeds largest size: " << max_size_;
        msg_buffer_.pop_front();
    }

    update_flag_ = true;
    cv_msg_.notify_one();
}

template <typename T>
void AsyncMessageProcess<T>::Quit() {
    // 持锁写入标志：AddMessage/ProcLoop 均在锁内访问它们，锁外写会构成数据竞争。
    // notify_all 以唤醒可能存在的多个等待者。
    {
        UL lock(mutex_);
        update_flag_.store(true);
        exit_flag_.store(true);
    }
    cv_msg_.notify_all();

    if (proc_.joinable()) {
        proc_.join();
    }
}

}  // namespace lightning::sys

#endif
