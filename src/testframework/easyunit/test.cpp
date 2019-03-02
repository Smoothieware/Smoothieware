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

#include "test.h"
#include "testregistry.h"


Test::Test(const SimpleString& testCaseName, const SimpleString& testName)
: testCaseName_(testCaseName), testName_(testName), testPartResult_(0), nextTest_(0), failuresCount_(0),
	successesCount_(0)
{
  TestRegistry::addTest(this);
}

Test::~Test() {
  TestPartResult *tmp;
  int size = failuresCount_ + successesCount_;

  for (int i = 0; i<size; i++) {
  	tmp = testPartResult_;
  	testPartResult_ = testPartResult_->getNext();
  	delete tmp;
  }
}

void Test::setUp()
{
}

void Test::tearDown()
{
}

void Test::run()
{
}


TestCase* Test::getTestCase() const
{
	return testCase_;
}


void Test::setTestCase(TestCase *testCase)
{
	testCase_ = testCase;
}

void Test::addTestPartResult(TestPartResult *testPartResult)
{
  TestPartResult *tmp;
  int type = testPartResult->getType();

  if (testPartResult_ == 0) {
		testPartResult_ = testPartResult;
		testPartResult_->setNext(testPartResult_);
	}
	else {
		tmp = testPartResult_;
		testPartResult_ = testPartResult;
		testPartResult_->setNext(tmp->getNext());
		tmp->setNext(testPartResult_);
	}

	if (type == failure) {
	  failuresCount_++;
	}
	else if (type == error) {
		errorsCount_++;
	}
	else {
		successesCount_++;
	}
}

TestPartResult* Test::getTestPartResult() const
{
  TestPartResult *tpr = testPartResult_;

  if (tpr != 0) {
  	tpr = tpr->getNext();
  }

  return tpr;
}

int Test::getFailuresCount() const
{
	return failuresCount_;
}

int Test::getSuccessesCount() const
{
 	return successesCount_;
}

int Test::getErrorsCount() const
{
	return errorsCount_;
}

void Test::setNext(Test *nextTest)
{
	nextTest_ = nextTest;
}


Test* Test::getNext() const
{
	return nextTest_;
}

const SimpleString& Test::getTestName() const
{
  return testName_;
}

const SimpleString& Test::getTestCaseName() const
{
	return testCaseName_;
}


