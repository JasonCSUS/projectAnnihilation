#ifndef GLOBAL_TASKS_H
#define GLOBAL_TASKS_H

#include <tbb/task_group.h>

// Declare a global task group to be used by the entire application.
extern tbb::task_group g_taskGroup;

#endif // GLOBAL_TASKS_H
