#ifndef USE_OMP
int omp_get_max_threads(){return 1;}
int omp_get_thread_num(){return 0;}
#endif
