#pragma once

namespace sys
{
    using ThreadID = __thread_id;

    using ThreadCriticalSectionISR = __tcs_isr;
    using ThreadCriticalSection = __tcs;

    using Thread = __thread_type;
} // namespace sys