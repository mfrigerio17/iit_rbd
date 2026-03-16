/* CPYHDR { */
/*
 * SPDX-FileCopyrightText: © 2019 Marco Frigerio
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of the 'iit-rbd' library.
 */
/* } CPYHDR */

#ifndef IIT_ROBCOGEN_MACROS_H_
#define IIT_ROBCOGEN_MACROS_H_

#ifdef IITRBD_NO_DEBUG
#define iit_rbd_matrix_assert(x)
#define iit_rbd_matrix_specific_size(type,rows,cols)
#else
#define iit_rbd_matrix_assert(x) eigen_assert(x)
#define iit_rbd_matrix_specific_size(type,rows,cols) EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(type,rows,cols)
#endif

#endif
