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

#ifndef TestRunner_H
#define TestRunner_H

#include "testcase.h"



/**
 * Test runner used to determine which test to run.
 * 
 * User may extends this class to provide a custom test runner
 * to TestRegistry.
 */
class TestRunner
{
public:
	TestRunner();
	virtual ~TestRunner();
	
	/**
	 * Method used to run testcases by TestRegistry.
	 * 
	 * User should override this method in order to provide custom
	 * behavior.
	 *
	 * @param testCase Linked list of testcases
	 * @param size Size of the linked list
	 */
	virtual void run(TestCase *testCase, int size);

};


#endif	// TestRunner_H

