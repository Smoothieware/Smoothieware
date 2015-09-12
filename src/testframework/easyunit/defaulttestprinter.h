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

#ifndef DEFAULTTESTPRINTER_H
#define DEFAULTTESTPRINTER_H

#include "testprinter.h"
#include "testcase.h"
#include "test.h"
#include "testresult.h"
#include <stdio.h>


/**
 * Complete header level means that a header will be printed
 * before the test details with all information available in
 * the test result.
 * 
 * Normal header level means that a header will be printed
 * before the test details with the most useful information
 * available in the test result.
 *
 * Off header level means that no header will be printed
 * before the test details.
 * 
 * Whatever the level, there will always be a clear indication
 * telling if there was a failure/error or not at the global
 * level.
 */
enum headerLevel {complete,normal,off};

/**
 * This is the default testprinter used by easyunit testregistry
 * when the user calls the runAndPrint() method without specifying
 * a testprinter.
 *
 * This testprinter writes plain text result to any supplied file.
 * The default file is the standard output.
 *
 * You may customize the outpur format by specifying the header level
 * and if you wish the testprinter to print details about each success.
 *
 * The default header level is normal and by default, the testprinter
 * does not print details about each success.
 */
class DefaultTestPrinter : public TestPrinter
{
	public:
	
	/**
	 * Default constructor that sets the header level
	 * to normal and the output source to the standard
	 * output.
	 */
		DefaultTestPrinter();
		
	/**
	 * Empty destructor.
	 */
		virtual ~DefaultTestPrinter();
	/**
	 * Prints a header depending of the header level and
	 * details about each test to the output_.
	 *
	 * @param testResult Results of all tests that were ran.
	 */
		virtual void print(const TestResult *testResult);	
		
	/**
	 * Set the header level of the printer.
	 *
	 * @param level Header level that will be used during print()
	 */
		void setHeaderLevel(headerLevel level);
		
	/**
	 * Set whether or not the printer should display the details
	 * of test that succeeded.
	 *
	 * @param show Set to true to display details about success
	 */
		void showSuccessDetail(bool show);
		
	/**
	 * Set the output to which the printer will print results.
	 *
	 * @param output Output used to print the results
	 */
		void setOutput(FILE *output);
		
	protected:
		virtual void printHeader(const TestResult *testResult);
		virtual void printTests(TestCase *testCase);
		virtual void printResults(Test *test);
		virtual void printCompleteHeader(const TestResult *testResult);
		virtual void printNormalHeader(const TestResult *testResult);
		int testsTotal_;
		int testFailuresTotal_;
		int failuresTotal_;
		headerLevel level_;
		bool showSuccessDetail_;
		FILE *output_;
};

#endif // DEFAULTTESTPRINTER_H

