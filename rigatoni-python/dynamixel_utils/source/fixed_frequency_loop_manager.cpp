//
// Created by Cole Ten on 5/10/24.
//

#include "fixed_frequency_loop_manager.h"


FixedFrequencyLoopManager::FixedFrequencyLoopManager(
	int64_t period_ns
): prev_time(std::chrono::steady_clock::now()), period(period_ns)
{
}

void FixedFrequencyLoopManager::sleep()
{
	using clock = std::chrono::steady_clock;

	clock::time_point curr_time, next_time;
	curr_time = clock::now();

	// Calculate expected `next_time` for this loop iteration
	next_time = prev_time + period;

	// If the `prev_time` is after the `curr_time`, then set `next_time` to be
	// `period` after `current_time`
	if (curr_time < prev_time)
		next_time = curr_time + period;

	// Update `prev_time` by user specified period
	prev_time += period;

	// If `curr_time` is after the calculated `next_time`, then loop frequency
	// is below desired.
	if (next_time <= curr_time)
	{
		if (curr_time > next_time + period)
			prev_time = curr_time + period;

		return;
	}

	// Sleep for `next_time` - `curr_time`;
	std::this_thread::sleep_for(next_time - curr_time);
}


PYBIND11_MODULE(_dynamixel_utils, m) {
	py::class_<FixedFrequencyLoopManager>(m, "FixedFrequencyLoopManager")
		.def(py::init<int64_t>(), py::arg("period_ns"))
		.def("sleep", &FixedFrequencyLoopManager::sleep);
}
