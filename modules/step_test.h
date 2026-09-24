#ifndef STEP_TEST_H
#define STEP_TEST_H

/* Initializes deadband characterization state and opens step_response.csv.
 * Call once before the main control loop begins. */
void step_test_init(void);
/*oiulkjesds*/
/* Call once per 1 ms loop iteration with the current loop_count (starting at 0).
 * Runs deadband characterization first, then the six-phase step-response
 * sequence, logging to step_response.csv each iteration.
 * Returns 1 while the test is still active, 0 once the test has completed. */
int step_test(unsigned int loop_count);

/* Prints the identified deadband values, flushes/closes the CSV file. */
void step_test_cleanup(void);

#endif // STEP_TEST_H