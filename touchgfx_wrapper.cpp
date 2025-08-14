#include <touchgfx/hal/OSWrappers.hpp>

extern "C" {
	void signalVSync(void) {
		touchgfx::OSWrappers::signalVSync();
	}
}
