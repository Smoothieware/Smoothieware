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

#ifndef testresult_H
#define testresult_H

#include "testcase.h"


class TestResult
{
public:
	TestResult();
	virtual ~TestResult();


	/**
	 * Get the total number of successes registered by all
	 * test cases ran. This is the sum of all TestCase->getSuccessesCount().
	 *
	 *@return The number of successes registered by all testcases.
	 */
	int getTotalSuccesses() const;

	/**
	 * Get the total number of errors registered by all
	 * test cases ran. This is the sum of all TestCase->getErrorsCount().
	 *
	 *@return The number of errors registered by all testcases.
	 */
	int getTotalErrors() const;

	/**
	 * Get the total number of failures registered by all
	 * test cases ran. This is the sum of all TestCase->getFailuresCount().
	 *
	 * @return The number of failures registered by all testcases.
	 */
	int getTotalFailures() const;

	/**
	 * Get the number of testcases ran that succeeded.
	 *
	 * @return The number of testcases ran that succeeded.
	 */
	int getSuccesses() const;

	/**
	 * Get the number of testcases ran that failed.
	 *
	 * @return The number of testcases ran that failed.
	 */
	int getFailures() const;

	/**
	 * Get the number of testcases ran that reported an error.
	 *
	 * @return The number of testcases ran that reported an error.
	 */
	int getErrors() const;

	/**
	 * Get the number of testcases in the TestCase list.
	 *
	 * @return The size of the TestCase list
	 */
	int getTestCaseCount() const;

	/**
	 * Get the number of tests
	 *
	 * @return The number of tests ran that succeeded
	 */
	int getTestRanCount() const;

	/**
	 * Get the number of testcases ran.
	 *
	 * @return The number of testcases ran
	 */
	int getTestCaseRanCount() const;

	/**
	 * Get the TestCase list. This list contains all TestCase registered and
	 * not only those that were ran.
	 *
	 * @return The TestCase list
	 */
	TestCase* getTestCases() const;

	/**
	 * Set the TestCase list and the size of the list.
	 *
	 * @param testCases TestCase list
	 * @param testCaseCount size of the TestCase list
	 */
	void setTestCases(TestCase *testCases, int testCaseCount);

	/**
	 * Add a TestCase result. This is used by a TestCase after it has
	 * completed.
	 *
	 * @param testCase TestCase that ran and contains results to add to
	 * global results
	 */
	virtual void addResult(TestCase *testCase);

protected:
	int testCaseCount_{0};
	int testRanCount_{0};
	int testCaseRanCount_{0};

	int totalSuccesses_{0};
	int totalErrors_{0};
	int totalFailures_{0};

	int successes_{0};
	int errors_{0};
	int failures_{0};

	TestCase* testCases_{0};

};


#endif	// testresult_H

