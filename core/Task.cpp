#include "Task.hpp"

using namespace atmc;

unsigned char TaskAllocator::stack[Config::TaskPromiseStackSize];
TaskAllocator::ChunkHeader* TaskAllocator::stackTop;