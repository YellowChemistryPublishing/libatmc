#include "cxxsysint.h"

void* __buf_new(size_t size)
{
    return new char[size];
}
void __buf_delete(void* ptr)
{
    delete[] static_cast<char*>(ptr);
}
