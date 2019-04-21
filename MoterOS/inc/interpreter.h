/**
 * @file 
 * 
 * List of commands
 *   - adc
 *     - Prints 2 consecutive ADC samples of channel 0 to the LCD and UART0
 *   - lcd
 *     - Prints strings on each line of each logical display on the LCD.
 *   - critical
 *     - Prints the percentage of CPU time spent with interrupts disabled.
 *   - log
 *     - Prints profiler events logged
 *   - clear
 *     - Clears the profiler event log and restarts the profiler
 *   - format
 *     - Formats the filesystem on the SD card
 *   - ls
 *     - List all files in the filesystem
 *   - cat
 *     - Print out file in the filesystem.
 *     - Takes one argument: the name of the file to print
 *   - rm
 *     - Remove file in the filesystem.
 *     - Takes one argument: the name of the file to remove
 *   - touch
 *     - Create a file in the filesystem.
 *     - Takes one argument: the name of the file to create
 *   - echo
 *     - Append characters to a file
 *     - Takes two arguments in this order:
 *        - The name of the file to append to
 *        - Remaining characters are written to the file
 *   - increase
 *     - Artificially increase the time spent in critical sections
 *       to test the "critical" command.
 */

#ifndef INTERPRETER_H
#define INTERPRETER_H

/**
 * @brief OS Task that sends characters to the interpreter
 * 
 */
void interpreter_task(void);


/**
 * @brief Pass user input to the interpreter and act on their command.
 * @param cmd_str String containing the entire user command.
 */
void interpreter_cmd(char* cmd_str);

#endif
