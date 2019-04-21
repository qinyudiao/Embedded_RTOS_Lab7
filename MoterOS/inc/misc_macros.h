/**
 * @file misc_macros.h
 * @brief Some helper macros
 */

#ifndef MISC_MACROS_H
#define MISC_MACROS_H

/**
 * @brief Get the number of elements in an array
 */
#define lengthof(array) (sizeof(array)/sizeof((array)[0]))

/**
 * @brief Zeroes out an array
 */
#define zeroes(array)   memset(array, 0, sizeof(array))

#endif
