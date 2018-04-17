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

#include "simplestring.h"

#ifndef TESTCASE_H
#define TESTCASE_H


class Test;
class TestResult;

/**
 * A TestCase is a collection of unit tests (instance of Test) and is
 * always specified by the first parameter of a Test declaration.
 */
class TestCase
{
	public:

		/**
		 * Main TestCase constructor.
		 *
		 * @param name TestCase name
		 * @param testResult Pointer to the TestResult used to report results
		 * of executed Test
		 */
		TestCase(const SimpleString& name, TestResult *testResult);

		virtual ~TestCase();

		/**
		 * Add a Test to the Test list. This method is used by TestRegistry.
		 *
		 * @param test Test instance to add to the Test list.
		 */
		void addTest(Test *test);

		/**
		 * Get the Test list.
		 *
		 * @return Test list
		 */
		Test* getTests() const;

    /**
     * Execute all Tests in the Test list of this TestCase. In fact, it calls
     * the run() method of all Tests.
     */
		void run();

    /**
     * Get the Test list size (number of Tests in this TestCase).
     *
     * @return The Test list size
     */
		int getTestsCount() const;

		/**
		 * Get the total number of failures reported by all Tests.
		 *
		 * @return The total number of failures reported by all Tests. 0
		 * if no test were run or if no failures were reported.
		 */
		int getFailuresCount() const;

		/**
		 * Get the total number of successes reported by all Tests.
		 *
		 * @return The total number of successes reported by all Tests. 0
		 * if no test were run or if no successes were reported.
		 */
		int getSuccessesCount() const;

		/**
		 * Get the total number of errors reported by all Tests.
		 *
		 * @return The total number of errors reported by all Tests. 0
		 * if no test were run, if this is the embedded version or if
		 * no errors were reported.
		 */
		int getErrorsCount() const;

		/**
		 * Indicates whether or not this TestCase was executed.
		 *
		 * @return true if the method run() of this TestCase was called. false
		 * otherwise
		 */
		bool ran() const;

		/**
		 * Get the TestCase name. This name is specified by the first parameter
		 * of the Test declaration. For example, if a test was declared as
		 * TEST(TESTCASE1, TEST1), the TestCase name would be "TESTCASE1".
		 *
		 * @return The name of the TestCase
		 */
		const SimpleString& getName() const;

		/**
		 * Get the next TestCase in the list.
		 *
		 * @return The next TestCase in the TestCase linked list
		 */
		TestCase* getNext() const;

		/**
		 * Set the next TestCase in the list.
		 *
		 * @return The next TestCase in the TestCase linked list
		 */
		void setNext(TestCase *testCase);

	protected:
		int failuresCount_{0};
		int successesCount_{0};
		int errorsCount_{0};
		int testsCount_{0};
		Test *tests_{0};
		SimpleString name_;
		TestCase *nextTestCase_{0};
		TestResult *testResult_;

	private:
	  void updateCount(Test *test);
	  void runTests(Test *test);
	  void runTest(Test *test);
	  bool ran_{false};

};

#endif // TESTCASE_H


