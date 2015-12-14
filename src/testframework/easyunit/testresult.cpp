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

#include "testresult.h"


TestResult::TestResult()
{
}


TestResult::~TestResult()
{
}

int TestResult::getTotalSuccesses() const
{
	return totalSuccesses_;
}

int TestResult::getTotalErrors() const
{
	return totalErrors_;
}

int TestResult::getTotalFailures() const
{
	return totalFailures_;
}


int TestResult::getSuccesses() const
{
	return successes_;
}

int TestResult::getFailures() const
{
	return failures_;
}

int TestResult::getErrors() const
{
	return errors_;
}

int TestResult::getTestCaseCount() const
{
	return testCaseCount_;
}

int TestResult::getTestRanCount() const
{
	return testRanCount_;
}

int TestResult::getTestCaseRanCount() const
{
	return testCaseRanCount_;
}

TestCase* TestResult::getTestCases() const
{
	return testCases_;
}

void TestResult::setTestCases(TestCase *testCases, int testCaseCount)
{
	testCases_ = testCases;
	testCaseCount_ = testCaseCount;
}

void TestResult::addResult(TestCase *testCase)
{
	int tcSuccesses = testCase->getSuccessesCount();
	int tcErrors = testCase->getErrorsCount();
	int tcFailures = testCase->getFailuresCount();

	testCaseRanCount_++;

	totalSuccesses_ += tcSuccesses;
	totalErrors_ += tcErrors;
	totalFailures_ += tcFailures;
	testRanCount_ += testCase->getTestsCount();

	if (tcErrors == 0 && tcFailures == 0) {
		successes_++;
	}
	else if (tcErrors > 0) {
		errors_++;
	}
	else {
		failures_++;
	}
}



