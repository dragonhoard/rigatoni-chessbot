//
// Created by Cole Ten on 3/23/24.
//

#ifndef FIXED_FREQUENCY_LOOP_MANAGER_HPP
#define FIXED_FREQUENCY_LOOP_MANAGER_HPP

#include <chrono>
#include <functional>
#include <thread>
#include <type_traits>
#include <pybind11/pybind11.h>

namespace py = pybind11;

using namespace std::chrono_literals;


class FixedFrequencyLoopManager {
public:
	FixedFrequencyLoopManager(int64_t period_ns);

	void sleep();

private:
	std::chrono::steady_clock::time_point prev_time;
	std::chrono::steady_clock::duration period;
};


#endif // FIXED_FREQUENCY_LOOP_MANAGER_HPP
