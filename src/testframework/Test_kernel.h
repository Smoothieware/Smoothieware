#pragma once

#include "Module.h"
#include <functional>

void test_kernel_setup_config(const char* start, const char* end);
void test_kernel_teardown();
void test_kernel_trap_event(_EVENT_ENUM id_event, std::function<void(void*)> fnc);
void test_kernel_untrap_event(_EVENT_ENUM id_event);
