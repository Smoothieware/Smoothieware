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

#ifndef TEST_H
#define TEST_H

#include "testcase.h"
#include "testpartresult.h"





/**
 * EasyUnit namespace.
 * This is the namespace containing all easyunit classes.
 */

/**
 * Test class containing all macros to do unit testing.
 * A test object represents a test that will be executed. Once it has been
 * executed, it reports all results in the testPartResult linked list.
 *
 * A failure occurs when a test fails (condition is false).
 * An error occurs when an exception is thrown during a test.
 * A success occurs if a test succeed (condition is true).
 */
class Test
{
	public:

		/**
		 * Main Test constructor. Used to create a test that will register itself
		 * with TestRegistry and with its test case.
		 * @param testCaseName Name of the test case this test belongs to
		 * @param testName Name of this test
		 */
		Test(const SimpleString& testCaseName, const SimpleString& testName);

		/**
		 * Main Test desctructor
		 * Delete the testPartResult linked list. This is why the user should
		 * only use the macro provided by easyunit to report a test result.
		 */
		virtual ~Test();

		/**
		 * Fixtures that will be called after run().
		 */
		virtual void tearDown();

		/**
		 * Fixtures that will be called before run().
		 */
		virtual void setUp();

		/**
		 * Test code should be in this method.
		 * run() will be called by the Test's TestCase, hence subclasses of Test
		 * should override this method.
		 */
		virtual void run();

		/**
		 * Set the TestCase this test belongs to.
		 *
		 * @param testCase The TestCase this test belongs to
		 */
  	void setTestCase(TestCase *testCase);

  	/**
  	 * Get the TestCase this test belongs to. A test always belongs to
  	 * only one TestCase. This is the TestCase identified by the first
  	 * parameter of the test declaration. For example, if there is a
  	 * test declared as TEST(TESTCASE1, TEST1), this test will be
  	 * associated with the TestCase TESTCASE1.
  	 *
  	 * @return The TestCase this test belongs to
  	 */
		TestCase* getTestCase() const;

		/**
		 * Add a testpartresult to the testpartresult list of this test.
		 * This method is used by the assertion macros to report success,
		 * failure or error.
		 *
		 * @param testPartResult The testpartresult to be added to the list
		 */
		virtual void addTestPartResult(TestPartResult *testPartResult);

		/**
		 * Get the testpartresult list of this test. If assertion macros
		 * and TEST and TESTF macros are used, there may be more than
		 * one successful testpartresult and no more than one error or failure.
		 *
		 * @return testPartResult The list of testpartresults of this test
		 */
		TestPartResult* getTestPartResult() const;

		/**
		 * Returns number of failures found in this test.
		 * If macro TEST or TESTF is used, failuresCount <= 1.
		 * If Test class is extended and ASSERT macros are used in different
		 * test methods, than failuresCount may be more than 1.
		 *
		 * @return Number of failures in this test
		 */
		int getFailuresCount() const;

		/**
		 * Returns number of successes found in this test.
		 * There may be more than one success since each ASSERT macro
		 * that succeeded generate a success.
		 *
		 * @return Number of successes in this test
		 */
		int getSuccessesCount() const;

		/**
		 * Returns number of errors found in this test.
		 * ErrorsCount <= 1, since exception are caught
		 * for the whole run() method.
		 *
		 * @return Number of errors in this test
		 */
		int getErrorsCount() const;


    /**
     * Set the next test in the linked list.
     *
     * @param nextTest Next test in the linked list
     */
		void setNext(Test *nextTest);

		/**
  	 * Get the next test in the linked list.
  	 *
		 * @return The next test in the linked list
		 */
		Test* getNext() const;

		/**
		 * Get the name of the TestCase this test belongs to. The name of the
		 * TestCase is the first parameter of the test declaration. For example,
		 * if a test is declared as TEST(TESTCASE1, TEST1), this method will return
		 * "TESTCASE1".
		 *
		 * @return The TestCase name of this test
		 */
		const SimpleString& getTestCaseName() const;

		/**
		 * Get the name of this test. The name of the test is the second
		 * parameter of the test declaration. For example,
		 * if a test is declared as TEST(TESTCASE1, TEST1), this method will return
		 * "TEST1".
		 *
		 * @return The name of this test.
		 */
		const SimpleString& getTestName() const;

 protected:
		SimpleString testCaseName_;
		SimpleString testName_;
  		TestCase *testCase_;
		TestPartResult *testPartResult_;
		Test *nextTest_;
  		int failuresCount_;
		int successesCount_;
		int errorsCount_;
};




/*
 * Helper macros
 */

#define EQUALS_DELTA(expected,actual,delta)\
  ((actual - expected) <= delta && actual >= expected) || ((expected - actual) <= delta && expected >= actual)

#define TO_STRING_EQUALS_F(expected,actual)\
  StringFrom("Expected : ") + StringFrom(expected) + StringFrom(" but Actual : ") + StringFrom(actual)

#define TO_STRING_EQUALS_S(expected,actual)\
  StringFrom(expected) + StringFrom(" == ") + StringFrom(actual)

#define TO_S_E_DELTA_F(expected,actual,delta)\
  StringFrom("Expected : ") + StringFrom(expected) + StringFrom(" but Actual : ") + StringFrom(actual) + StringFrom(" with delta = ") + StringFrom(delta)

#define TO_S_E_DELTA_S(expected,actual,delta)\
  StringFrom(expected) + StringFrom(" == ") + StringFrom(actual) + StringFrom(" with delta = ") + StringFrom(delta)

/**
 * Asserts that a condition is true.
 * If the condition is not true, a failure is generated.
 * @param condition Condition to fullfill for the assertion to pass
 */
#define ASSERT_TRUE(condition)\
	{ if (condition) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,#condition,success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__, #condition,failure)); return;\
	}}

/**
 * Asserts that a condition is true.
 * If the condition is not true, a failure is generated.
 * @param condition Condition to fullfill for the assertion to pass
 * @param message Message that will be displayed if this assertion fails
 */
#define ASSERT_TRUE_M(condition,message)\
	{ if (condition) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,#condition,success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__, message,failure)); return;\
	}}

/**
 * Asserts that the two parameters are equals. Operator == must be defined.
 * If the two parameters are not equals, a failure is generated.
 * @param expected Expected value
 * @param actual Actual value to be compared
 */
#define ASSERT_EQUALS(expected,actual)\
{ if (expected == actual) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_STRING_EQUALS_S(#expected,#actual),success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_STRING_EQUALS_F(#expected,#actual),failure)); return;\
	}}

/**
 * Asserts that the two parameters are equals. Operator == must be defined.
 * If the two parameters are not equals, a failure is generated.
 *
 * Parameters must be primitive data types or StringFrom (custom type) must
 * be overloaded.
 *
 * @see SimpleString
 * @param expected Expected value
 * @param actual Actual value to be compared
 */
#define ASSERT_EQUALS_V(expected,actual)\
{ if (expected == actual) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_STRING_EQUALS_S(expected,actual),success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_STRING_EQUALS_F(expected,actual),failure)); return;\
	}}

/**
 * Asserts that the two parameters are equals. Operator == must be defined.
 * If the two parameters are not equals, a failure is generated.
 * @param expected Expected value
 * @param actual Actual value to be compared
 * @param message Message that will be displayed if this assertion fails
 */
#define ASSERT_EQUALS_M(expected,actual,message)\
{ if (expected == actual) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,#expected,success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,message,failure)); return;\
	}}

/**
 * Asserts that the two parameters are equals within a delta. Operators == and - must be defined.
 * If the two parameters are not equals, a failure is generated.
 * @param expected Expected value
 * @param actual Actual value to be compared
 * @param delta Delta accepted between the two values
 */
#define ASSERT_EQUALS_DELTA(expected,actual,delta)\
{ if (EQUALS_DELTA(expected,actual,delta) ) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_S_E_DELTA_S(#expected,#actual,#delta),success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_S_E_DELTA_F(#expected,#actual,#delta),failure)); return;\
	}}

/**
 * Asserts that the two parameters are equals within a delta. Operators == and - must be defined.
 * If the two parameters are not equals, a failure is generated.
 * @param expected Expected value
 * @param actual Actual value to be compared
 * @param delta Delta accepted between the two values
 * @param message Message that will be displayed if this assertion fails
 */
#define ASSERT_EQUALS_DELTA_M(expected,actual,delta,message)\
{ if (EQUALS_DELTA(expected,actual,delta)) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,#expected,success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,message,failure)); return;\
	}}

/**
 * Asserts that the two parameters are equals within a delta. Operators == and - must be defined.
 * If the two parameters are not equals, a failure is generated.
 *
 * Parameters must be primitive data types or StringFrom (custom type) must
 * be overloaded.
 *
 * @see SimpleString
 * @param expected Expected value
 * @param actual Actual value to be compared
 * @param delta Delta accepted between the two values
 */
#define ASSERT_EQUALS_DELTA_V(expected,actual,delta)\
{ if (EQUALS_DELTA(expected,actual,delta)) {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_S_E_DELTA_S(expected,actual,delta),success));\
	} else {\
	addTestPartResult(new TestPartResult(this, __FILE__,__LINE__,TO_S_E_DELTA_F(expected,actual,delta),failure)); return;\
	}}


/**
 * Make a test fails.
 */
#define FAIL()\
  { addTestPartResult(new TestPartResult(this, __FILE__, __LINE__,("Test failed."),failure)); return; }

/**
 * Make a test fails with the given message.
 * @param text Failure message
 */
#define FAIL_M(text)\
	{ addTestPartResult(new TestPartResult(this, __FILE__, __LINE__,text,failure)); return; }


/**
 * Define a test in a TestCase.
 * User should put his test code between brackets after using this macro.
 * @param testCaseName TestCase name where the test belongs to
 * @param testName Unique test name
 */
#define TEST(testCaseName, testName)\
  class testCaseName##testName##Test : public Test \
	{ public: testCaseName##testName##Test() : Test (#testCaseName , #testName) {} \
            void run(); } \
    testCaseName##testName##Instance; \
	void testCaseName##testName##Test::run ()


/**
 * Define a test in a TestCase using test fixtures.
 * User should put his test code between brackets after using this macro.
 *
 * This macro should only be used if test fixtures were declared earlier in
 * this order: DECLARE, SETUP, TEARDOWN.
 * @param testCaseName TestCase name where the test belongs to. Should be
 * the same name of DECLARE, SETUP and TEARDOWN.
 * @param testName Unique test name.
 */
#define TESTF(testCaseName, testName)\
  class testCaseName##testName##Test : public testCaseName##Declare##Test \
	{ public: testCaseName##testName##Test() : testCaseName##Declare##Test (#testCaseName , #testName) {} \
            void run(); } \
    testCaseName##testName##Instance; \
	void testCaseName##testName##Test::run ()


/**
 * Setup code for test fixtures.
 * This code is executed before each TESTF.
 *
 * User should put his setup code between brackets after using this macro.
 *
 * @param testCaseName TestCase name of the fixtures.
 */
#define SETUP(testCaseName)\
	void testCaseName##Declare##Test::setUp ()


/**
 * Teardown code for test fixtures.
 * This code is executed after each TESTF.
 *
 * User should put his setup code between brackets after using this macro.
 *
 * @param testCaseName TestCase name of the fixtures.
 */
#define TEARDOWN(testCaseName)\
	void testCaseName##Declare##Test::tearDown ()


/**
 * Location to declare variables and objets.
 * This is where user should declare members accessible by TESTF,
 * SETUP and TEARDOWN.
 *
 * User should not use brackets after using this macro. User should
 * not initialize any members here.
 *
 * @param testCaseName TestCase name of the fixtures
 * @see END_DECLARE for more information.
 */
#define DECLARE(testCaseName)\
	class testCaseName##Declare##Test : public Test \
	{ public: testCaseName##Declare##Test(const SimpleString& testCaseName, const SimpleString& testName) : Test (testCaseName , testName) {} \
	virtual void run() = 0; void setUp(); void tearDown(); \
	protected:


/**
 * Ending macro used after DECLARE.
 *
 * User should use this macro after declaring members with
 * DECLARE macro.
 */
#define END_DECLARE \
	};

#endif // TEST_H


