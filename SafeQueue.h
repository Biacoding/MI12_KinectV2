#include <mutex>
#include <queue>
#include <boost/optional.hpp>

#pragma once
std::atomic<bool> threadFinished{ false };
template <typename T>
class SafeQueue
{
private:
	std::queue<T> queue;
	std::mutex mtx;
	std::condition_variable cv;
public:

	void push(T value) {
		std::lock_guard<std::mutex> lock(mtx);
		this->queue.push(std::move(value));
		cv.notify_one();
	}

	boost::optional<T> pop() {
		std::unique_lock<std::mutex> lock(mtx);
		cv.wait(lock, [this]() { return !this->queue.empty() || threadFinished.load();; });
		if (this->queue.empty()) {
			return boost::none;
		}
		T tmp = std::move(this->queue.front());
		this->queue.pop();
		return tmp;
	}
	void notify_all() {
		this->cv.notify_all();
	}
};

