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

#ifndef TESTREGISTRY_H
#define TESTREGISTRY_H

#include "test.h"
#include "testcase.h"
#include "testprinter.h"
#include "simplestring.h"
#include "testrunner.h"
#include "testresult.h"


/**
 * The TestRegistry is the main class used to register all tests,
 * and create appropriate TestCase. It can then be used to run
 * tests and print results. All methods that should be used by
 * the user are static.
 */
class TestRegistry
{
	public:
	  TestRegistry();
		~TestRegistry();

		/**
		 * Add a test in the registry. If the previous TestCase was not the same
		 * as the one of the current test, a new TestCase is created.
		 *
		 * @param test Test to be added
		 */
		static void addTest (Test *test);

		/**
		 * Run all tests in the registry (default test runner) and return
		 * the test results.
		 *
		 * @return The test results
		 */
		static const TestResult* run();

		/**
		 * Pass all tests in the registry to the TestRunner runner and
		 * return the results of all tests ran.
		 *
		 * @param runner The custom runner used to decided which test to run
		 * @return The test results of all tests ran
		 */
		static const TestResult* run(TestRunner *runner);

		/**
		 * Run all tests in the registry (default test runner) and return
		 * the test results. This will also print the results using the
		 * default test printer (normal level of details and to the standard
		 * output).
		 *
		 * @return The test results
		 */
		static const TestResult* runAndPrint();

		/**
		 * Pass all tests in the registry to the TestRunner runner and
		 * return the results of all tests ran. This will also print the results
		 * using the default test printer (normal level of details and to the
		 * standard output).
		 *
		 * @param runner The custom runner used to decided which test to run
		 * @return The test results
		 */
		static const TestResult* runAndPrint(TestRunner *runner);

		/**
		 * Run all tests in the registry (default test runner) and return
		 * the test results. Results will also be given to
		 * to the TestPrinter printer.
		 *
		 * @param printer The custom printer used to print the test results
		 * @return The test results
		 */
		static const TestResult* runAndPrint(TestPrinter *printer);

		/**
		 * Pass all tests in the registry to the TestRunner runner and
		 * return the results of all tests ran. Results will also be given to
		 * to the TestPrinter printer.
		 *
		 * @param printer The custom printer used to print the test results
		 * @param runner The custom runner used to decided which test to run
		 * @return The test results
		 */
		static const TestResult* runAndPrint(TestPrinter *printer, TestRunner *runner);

	private:
		static TestRegistry& instance();
		static int nextName;
		void add(Test *test);
		void addTestCase(TestCase *testCase);
		const TestResult* runTests(TestRunner *runner);
		TestCase *currentTC_;
		TestPrinter *defaultPrinter_;
		int testCaseCount_;
		TestRunner *defaultRunner_;
		TestResult testResult_;
};

#endif // TESTREGISTRY_H


