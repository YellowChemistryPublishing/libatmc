#include "Task.h"

using namespace sys;
using namespace atmc;

alignas(std::max_align_t) unsigned char TaskAllocator::stack[Config::TaskPromiseStackSize];
TaskAllocator::ChunkHeader* TaskAllocator::stackTop;