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

#include "testregistry.h"
#include "defaulttestprinter.h"


int TestRegistry::nextName = 0;

TestRegistry::TestRegistry()
: currentTC_(0), defaultPrinter_(new DefaultTestPrinter()),testCaseCount_(0),
	defaultRunner_(new TestRunner())
{
}  

TestRegistry::~TestRegistry()
{
	TestCase *tmp;
	for (int i = 0; i<testCaseCount_; i++) {
		tmp = currentTC_;
		currentTC_ = currentTC_->getNext();
		delete tmp;
	}
	
	delete defaultPrinter_;
	delete defaultRunner_;
}

void TestRegistry::addTest(Test *test)
{
	instance().add(test);
}  

const TestResult* TestRegistry::run()
{
	return instance().runTests(instance().defaultRunner_);
}

const TestResult* TestRegistry::run(TestRunner *runner)
{
	return instance().runTests(runner);
}

const TestResult* TestRegistry::runAndPrint()
{
	return runAndPrint(instance().defaultPrinter_,instance().defaultRunner_);
}

const TestResult* TestRegistry::runAndPrint(TestRunner *runner)
{
	return runAndPrint(instance().defaultPrinter_,runner);
}

const TestResult* TestRegistry::runAndPrint(TestPrinter *printer)
{
	return runAndPrint(printer,instance().defaultRunner_);
}


const TestResult* TestRegistry::runAndPrint(TestPrinter *printer, TestRunner *runner)
{
	const TestResult *testResult = instance().runTests(runner);
	printer->print(testResult);
	return testResult;
}

		
TestRegistry& TestRegistry::instance()
{
	static TestRegistry registry;
	return registry;
}  

void TestRegistry::add(Test *test)
{
	const SimpleString tcName = test->getTestCaseName();
	const SimpleString tName = test->getTestName();
	
	if ((currentTC_ == 0) || (currentTC_->getName() != tcName)) {
			addTestCase(new TestCase(tcName,&testResult_));
	}
	
	currentTC_->addTest(test);
	
}

const TestResult* TestRegistry::runTests(TestRunner *runner)
{
	TestCase *tc = currentTC_;
	
	if (tc != 0) {
		tc = tc->getNext();
		runner->run(tc,testCaseCount_);
	}
	
	testResult_.setTestCases(tc,testCaseCount_);
	
	return &testResult_;
} 



void TestRegistry::addTestCase(TestCase *testCase)
{
	TestCase *tmp;
 	
 	if (currentTC_ == 0) {
 		currentTC_ = testCase;
 		currentTC_->setNext(currentTC_);
	}
	else {
		tmp = currentTC_;
		currentTC_ = testCase;
		currentTC_->setNext(tmp->getNext());
		tmp->setNext(currentTC_);
	}
	
	testCaseCount_++;
}   


