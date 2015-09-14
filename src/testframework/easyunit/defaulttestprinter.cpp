/*
EasyUnit : Simple C++ Unit testing framework
Copyright (C) 2004 Barthelemy Dagenais

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Barthelemy Dagenais
barthelemy@prologique.com
*/

#include "defaulttestprinter.h"

#include "testpartresult.h"

#include <stdio.h>


DefaultTestPrinter::DefaultTestPrinter()
: testsTotal_(0),testFailuresTotal_(0),failuresTotal_(0),
	level_(normal), showSuccessDetail_(false), output_(stdout)
{
}

DefaultTestPrinter::~DefaultTestPrinter()
{
}

void DefaultTestPrinter::print(const TestResult *testResult)
{
	int failures;
	int successes;
	int errors;
	SimpleString state;
	SimpleString name;
	TestCase *testCase = testResult->getTestCases();
	int size = testResult->getTestCaseCount();

	printHeader(testResult);

	if (testResult->getTestCaseRanCount() == 0) {
		fprintf(output_,"\nNo test ran\n");
	}

	for (int i=0;i<size;i++) {

	  if (testCase->ran()) {

  		name = testCase->getName();
  		failures = testCase->getFailuresCount();
  		successes = testCase->getSuccessesCount();
  		errors = testCase->getErrorsCount();

  		if (failures > 0 || errors > 0) {
  			state = "FAILED";
  		}
  		else {
  			state = "SUCCEEDED";
  		}

  		fprintf(output_, "\n\nTest case \"%s\" %s with %d error(s), %d failure(s) and %d success(es): \n",name.asCharString(),state.asCharString(),errors,failures,successes);

  		printTests(testCase);
		}

		testCase = testCase->getNext();
	}
}

void DefaultTestPrinter::setHeaderLevel(headerLevel level)
{
	level_ = level;
}

void DefaultTestPrinter::showSuccessDetail(bool show)
{
	showSuccessDetail_ = show;
}

void DefaultTestPrinter::setOutput(FILE *output)
{
	output_ = output;
}

void DefaultTestPrinter::printHeader(const TestResult *testResult)
{
	fprintf(output_ , "-- EasyUnit Results --\n");

	if (level_ != off) {
		fprintf(output_ , "\nSUMMARY\n\n");
		fprintf(output_ , "Test summary: ");

		if (testResult->getErrors() > 0 || testResult->getFailures() > 0) {
			fprintf(output_ , "FAIL\n");
		}
		else {
			fprintf(output_ , "SUCCESS\n");
		}

		if (level_ == normal) {
			printNormalHeader(testResult);
		}
		else {
			printCompleteHeader(testResult);
		}
	}

	fprintf(output_ , "\n");
	fprintf(output_ , "\nDETAILS");
}

void DefaultTestPrinter::printCompleteHeader(const TestResult *testResult)
{
	fprintf(output_ , "Number of test cases: %d\n",testResult->getTestCaseCount());
	fprintf(output_ , "Number of test cases ran: %d\n",testResult->getTestCaseRanCount());
	fprintf(output_ , "Test cases that succeeded: %d\n",testResult->getSuccesses());
	fprintf(output_ , "Test cases with errors: %d\n",testResult->getErrors());
	fprintf(output_ , "Test cases that failed: %d\n",testResult->getFailures());
	fprintf(output_ , "Number of tests ran: %d\n",testResult->getTestRanCount());
	fprintf(output_ , "Tests that succeeded: %d\n",testResult->getTotalSuccesses());
	fprintf(output_ , "Tests with errors: %d\n",testResult->getTotalErrors());
	fprintf(output_ , "Tests that failed: %d\n",testResult->getTotalFailures());

}

void DefaultTestPrinter::printNormalHeader(const TestResult *testResult)
{
	fprintf(output_ , "Number of test cases ran: %d\n",testResult->getTestCaseRanCount());
	fprintf(output_ , "Test cases that succeeded: %d\n",testResult->getSuccesses());
	fprintf(output_ , "Test cases with errors: %d\n",testResult->getErrors());
	fprintf(output_ , "Test cases that failed: %d\n",testResult->getFailures());
}

void DefaultTestPrinter::printTests(TestCase *testCase)
{
	const char *indent = " ";
	Test *test = testCase->getTests();
	int size = testCase->getTestsCount();
	SimpleString state;



	for (int i=0;i<size;i++) {
		if (test->getFailuresCount() > 0 || test->getErrorsCount() > 0) {
			state = "FAILED :";
		}
		else {
			state = "SUCCEEDED!";
		}

		fprintf(output_, "%s Test \"%s\" %s\n",indent,test->getTestName().asCharString(),state.asCharString());
		printResults(test);
		test = test->getNext();
	}
}

void DefaultTestPrinter::printResults(Test *test)
{
	const char *indent = "    ";
	TestPartResult *testPR = test->getTestPartResult();
	int size = test->getFailuresCount() + test->getSuccessesCount() + test->getErrorsCount();
	int type;

	for (int i=0;i<size;i++) {

		type = testPR->getType();

		if (type == failure) {
			fprintf (output_, "%s%s%s%s%s%ld%s%s\n",
				indent,
				"Failure: \"",
				testPR->getMessage().asCharString (),
				"\" " ,
				"line ",
				testPR->getLineNumber(),
				" in ",
				testPR->getFileName().asCharString ());
		}
		else if (type == error) {
			fprintf (output_, "%s%s%s%s%s%s\n",
				indent,
				"Error in ",
				test->getTestName().asCharString(),
				": \"",
				testPR->getMessage().asCharString (),
				"\"");
		}
		else if (type == success && showSuccessDetail_) {
			fprintf (output_, "%s%s%s%s%s%ld%s%s\n",
				indent,
				"Success: \"",
				testPR->getMessage().asCharString (),
				"\" " ,
				"line ",
				testPR->getLineNumber(),
				" in ",
				testPR->getFileName().asCharString ());
		}
		testPR = testPR->getNext();
	}
}



