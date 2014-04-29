#ifndef TBB_H
#define TBB_H

/*
 * Helper function for tbb::task::enqueue
 * 
 * Enqueue a task for execution by the TBB task scheduler using lambda expressions
 * 
 * Usage:
 * 
 *   tbb_enqueue_lambda( []{ 
 *       // put code here
 *   } );
 * 
 * More information:
 *   http://www.threadingbuildingblocks.org/docs/help/index.htm#reference/task_scheduler/task_cls.htm
 */

template<typename F>
class lambda_task : public tbb::task {
    F my_func;
    tbb::task* execute() {
        my_func();
        return NULL;
    }
public:
    lambda_task(const F& f) : my_func(f) {}
};

template<typename F>
void tbb_enqueue_lambda(const F& f) {
    tbb::task::enqueue(*new(tbb::task::allocate_root()) lambda_task<F>(f));
}

#endif // TBB_H
