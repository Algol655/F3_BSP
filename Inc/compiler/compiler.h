/*! ----------------------------------------------------------------------------
 * @file	compiler.h
 * @brief
 *
 * @author	Tommaso Sabatini, 2016
 */

#ifndef COMPILER_H_
#define COMPILER_H_

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __GNUC__
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <ctype.h>
#include <sys/types.h>
#include "arm_math.h"

#ifndef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC	1000
#endif

#ifndef TRUE
	#define TRUE 		true
#endif
#ifndef FALSE
	#define FALSE		false
#endif

#ifndef __align4
	#define __align4            __attribute__((aligned (4)))
#endif
#ifndef __weak
	#define __weak              __attribute__((weak))
#endif
#ifndef __always_inline
	#define __always_inline     __attribute__((always_inline))
#endif

#endif


#ifdef __cplusplus
}
#endif

#endif /* COMPILER_H_ */
