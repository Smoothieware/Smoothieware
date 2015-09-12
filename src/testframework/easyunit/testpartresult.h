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

#ifndef TESTPARTRESULT_H
#define TESTPARTRESULT_H

#include "simplestring.h"


/**
 * This enumeration contains the three states a TestPartResult can take.
 *
 * A failure means that an assertion failed during the test.
 *
 * A success means that all assertion succeeded during the test.
 *
 * An error means that an exception was thrown during the test.
 */
enum testType {failure,success,error};

class Test;

/**
 * This class contains details about assertion processed during the
 * execution of a Test. It contains the line and the file of the assertion, 
 * the result (success, failure or error), the condition (or message), and
 * the Test class where the assertion was processed.
 */ 
class TestPartResult
{
	public:
		/**
		 * Main constructor used to initialize all details about the result
		 * of an assertion.
		 *
		 * @param test The test where the assertion was processed
		 * @param fileName The file name where the assertion is located
		 * @param lineNumber The line number where the assertion is located
		 * @param message The assertion condition or message
		 * @param type The result of the assertion (failure, success or error)
		 */
		TestPartResult (Test *test,
		const SimpleString& fileName, 
		long lineNumber,
		const SimpleString& message,
  	testType type);

    /**
     * Set the next TestPartResult in the list.
     *
     * @param next The next TestPartResult in the linked list
     */
		void setNext(TestPartResult* next);
		
		/**
		 * Get the next TestPartResult in the list.
		 *
		 * @return The next TestPartResult in the linked list
		 */
  	TestPartResult* getNext() const;

    /**
     * Get the type of the TestPartResult. This represents the result
     * of the assertion.
     *
     * @return The type of the TestPartResult (failure, success or error)
     */
  	testType getType() const;
  	
  	/**
  	 * Get the message (or condition) of the assertion.
  	 *
  	 * @return The message (or condition) of the assertion
  	 */
  	const SimpleString& getMessage() const;
  	
  	/**
  	 * Get the Test where the assertion is located.
  	 *
  	 * @return The Test where the assertion is located
  	 */
  	Test* getTest() const;
  	
  	/**
  	 * Get the file name where the assertion is located.
  	 *
  	 * @return The file name where the assertion is located
  	 */
  	const SimpleString& getFileName() const;
  	
  	/**
  	 * Get the line number where the assertion is located.
  	 *
  	 * @return The line number where the assertion is located
  	 */
  	long getLineNumber() const;
  	
  	
  protected:
  	SimpleString message_;
		Test *test_;
  	SimpleString fileName_;
  	long lineNumber_;

	private:
		TestPartResult *next_;
  	testType type_;  	
};

#endif // TESTPARTRESULT_H


