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

#ifndef TESTPRINTER_H
#define TESTPRINTER_H

#include "testresult.h"


/**
 * A TestPrinter is a class used by the TestRegistry to print results
 * of executed TestCases. This is an abstract class, so no default behavior
 * for the print method is provided.
 *
 * @see DefaultTestPrinter
 */
class TestPrinter
{
	public:
        virtual ~TestPrinter(){};
	/**
	 * Print the details of a given TestResult instance. This
	 * method must be overridden by subclasses since it is
	 * abstract.
	 *
	 * @param testResult TestResult instance that the user wish to print
	 */
		virtual void print(const TestResult *testResult) = 0;
};

#endif // TESTPRINTER_H


