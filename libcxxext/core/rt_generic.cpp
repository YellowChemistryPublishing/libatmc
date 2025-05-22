#include "rt.h"

extern "C" _weak void* __task_operator_new(size_t sz)
{
    return ::operator new(sz);
}
extern "C" _weak void __task_operator_delete(void* ptr)
{
    ::operator delete(ptr);
}

extern "C" _weak void __launch_async(void*)
{ }
