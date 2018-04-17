#define ECPP
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

#include "testcase.h"
#include "test.h"
#include "testresult.h"

#ifndef ECPP
#include <exception>
#endif


TestCase::TestCase(const SimpleString& name, TestResult *testResult)
: name_(name), testResult_(testResult)
{
}

TestCase::~TestCase()
{
}

void TestCase::addTest(Test *test)
{
	Test *tmp;

 	if (tests_ == 0) {
		tests_ = test;
		tests_->setNext(tests_);
	}
	else {
		tmp = tests_;
		tests_ = test;
		tests_->setNext(tmp->getNext());
		tmp->setNext(tests_);
	}

	testsCount_++;
}

Test* TestCase::getTests() const
{
	Test *test = tests_;

	if (test != 0) {
		test = test->getNext();
	}

	return test;
}

void TestCase::run()
{
	Test *test = tests_->getNext();

	runTests(test);

	ran_ = true;

	testResult_->addResult(this);
}

int TestCase::getTestsCount() const
{
	return testsCount_;
}

int TestCase::getFailuresCount() const
{
  return failuresCount_;
}

int TestCase::getSuccessesCount() const
{
  return successesCount_;
}

int TestCase::getErrorsCount() const
{
	return errorsCount_;
}

bool TestCase::ran() const
{
	return ran_;
}

const SimpleString& TestCase::getName() const
{
	return name_;
}

void TestCase::updateCount(Test *test)
{
  if (test->getErrorsCount() > 0) {
  	errorsCount_++;
  }
  else if (test->getFailuresCount() > 0) {
  	failuresCount_++;
  }
  else {
  	successesCount_++;
  }
}

TestCase* TestCase::getNext() const
{
	return nextTestCase_;
}

void TestCase::setNext(TestCase *testCase)
{
	nextTestCase_ = testCase;
}

void TestCase::runTests(Test *test)
{

	for (int i = 0; i<testsCount_; i++) {
		test->setUp();
		runTest(test);
		test->tearDown();
		updateCount(test);
		test = test->getNext();
	}

}

#ifdef ECPP

void TestCase::runTest(Test *test)
{
	test->run();
}

#else

void TestCase::runTest(Test *test)
{
	try {
		test->run();
	}
	catch (std::exception &e) {
		test->addTestPartResult(new TestPartResult(test,"",-1,e.what(),error));
	}
	catch (...) {
		test->addTestPartResult(new TestPartResult(test,"",-1,"Unexpected error occured",error));
	}
}
#endif

