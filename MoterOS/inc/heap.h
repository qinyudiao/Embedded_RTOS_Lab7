/**
 * @file
 * @author Jacob Egner
 * @date 2008-07-31
 * @brief Implements memory heap for dynamic memory allocation.
 * Follows standard malloc/calloc/realloc/free interface
 * for allocating/unallocating memory.
 * modified 8/31/08 Jonathan Valvano for style
 * modified 12/16/11 Jonathan Valvano for 32-bit machine
 * modified August 10, 2014 for C99 syntax
 * This example accompanies the book
 * "Embedded Systems: Real Time Operating Systems for ARM Cortex M Microcontrollers",
 * ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2014
 * Implementation Notes:
 * This is a Knuth Heap. Each block has a header and a trailer, which I shall
 * call the meta-sections.  The meta-sections are each a single int32_t that tells
 * how many int32_ts/words can be stored between the header and trailer.
 * If the block is used, the meta-sections record the room as a positive
 * number.  If the block is unused, the meta-sections record the room as a
 * negative number.
 * Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
 * You may use, edit, run or distribute this file
 * as long as the above copyright notice remains
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 * For more information about my classes, my research, and my books, see
 * http://users.ece.utexas.edu/~valvano/
 */


#ifndef HEAP_H
#define HEAP_H

// feel free to change HEAP_SIZE_BYTES to however
// big you want the heap to be
#define HEAP_SIZE_BYTES (8)
#define HEAP_SIZE_WORDS (HEAP_SIZE_BYTES / sizeof(int32_t))

#define HEAP_OK 0
#define HEAP_ERROR_CORRUPTED_HEAP 1
#define HEAP_ERROR_POINTER_OUT_OF_RANGE 2

// struct for holding statistics on the state of the heap
typedef struct heap_stats {
  int32_t wordsAllocated;
  int32_t wordsAvailable;
  int32_t wordsOverhead;
  int32_t blocksUsed;
  int32_t blocksUnused;
} heap_stats_t;

/**
 * @brief Initialize the Heap
 * notes: Initializes/resets the heap to a clean state where no memory is allocated.
 *
 * @return always HEAP_OK
 */
int32_t Heap_Init(void);


/**
 * @brief Allocate memory, data not initialized
 *
 * @param desiredBytes desired number of bytes to allocate
 *
 * @return void* pointing to the allocated memory or will return NULL
 * if there isn't sufficient space to satisfy allocation request
 */
void* Heap_Malloc(int32_t desiredBytes);


/**
 * @brief Allocate memory, data are initialized to 0
 * notes: the allocated memory block will be zeroed out
 *
 * @param desiredBytes desired number of bytes to allocate
 *
 * @return void* pointing to the allocated memory block or will return NULL
 * if there isn't sufficient space to satisfy allocation request
 */
void* Heap_Calloc(int32_t desiredBytes);


/**
 * @brief Reallocate buffer to a new size
 * notes: the given block will be unallocated after its
 * contents are copied to the new block
 *
 * @param oldBlock pointer to a block
 *
 * @param desiredBytes a desired number of bytes for a new block
 * where the contents of the old block will be copied to
 *
 * @return void* pointing to the new block or will return NULL
 * if there is any reason the reallocation can't be completed
 */
void* Heap_Realloc(void* oldBlock, int32_t desiredBytes);


/**
 * @brief return a block to the heap
 *
 * @param pointer the pointer to memory to unallocate
 *
 * @return HEAP_OK if everything is ok;
 * HEAP_ERROR_POINTER_OUT_OF_RANGE if pointer points outside the heap;
 * HEAP_ERROR_CORRUPTED_HEAP if heap has been corrupted or trying to
 * unallocate memory that has already been unallocated;
 */
int32_t Heap_Free(void* pointer);


/**
 * @brief Test the heap
 *
 * @return validity of the heap - either HEAP_OK or HEAP_ERROR_HEAP_CORRUPTED
 */
int32_t Heap_Test(void);


/**
 * @brief return the current status of the heap
 *
 * @return a heap_stats_t that describes the current usage of the heap
 */
heap_stats_t Heap_Stats(void);


#endif //#ifndef HEAP_H
