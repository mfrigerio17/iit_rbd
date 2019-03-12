#ifndef IIT_ROBCOGEN_MACROS_H_
#define IIT_ROBCOGEN_MACROS_H_

#ifdef IITRBD_NO_DEBUG
#define iit_rbd_matrix_assert(x)
#else
#define iit_rbd_matrix_assert(x) eigen_assert(x)
#endif

#endif
