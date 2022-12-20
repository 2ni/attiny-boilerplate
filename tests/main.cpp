/*
 * gcc -DF_CPU=10000000 -std=c++11 -I mocks -I ../common uart.cpp test.cpp main.cpp -o test && ./test
 *
 */
#include <stdio.h>

#include "test.h"
#include "struct.h"

#include "tests_uart.h"

int main() {
  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;
  Test_Outcome outcome;
  Test_Result test_result;

  // test uart
  test_result = tests_uart();
  number_of_passed += test_result.passed;
  number_of_tests += test_result.total;

  // summary
  uint16_t number_of_failed = number_of_tests-number_of_passed;
  printf("\n");
  if (number_of_failed) {
    printf(NOK("Failures!!!: %u/%u.") "\n", number_of_failed, number_of_tests);
  } else {
    printf(OK("All good: %u/%u.") "\n", number_of_passed, number_of_tests);
  }

  return number_of_failed;
}
