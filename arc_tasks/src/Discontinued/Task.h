#ifndef ARC_TASKS_TASK_H
#define ARC_TASKS_TASK_H

namespace arc_tasks{
  
class Task{
 private:
  int task;
  int priority;
 public:
  Task(int task, int priority);
  int get_task();
  int get_priority();

  
};
 
}
#endif //ARC_TASKS_TASK_H
