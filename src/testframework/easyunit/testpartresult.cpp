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

#include "testpartresult.h"
#include "test.h"


TestPartResult::TestPartResult (Test *test,
	const SimpleString&	fileName, 
	long lineNumber,
	const SimpleString&	message,
	testType type) 
  : message_ (message), 
    test_ (test), 
    fileName_ (fileName), 
    lineNumber_ (lineNumber),
    type_ (type)
{
}

void TestPartResult::setNext(TestPartResult *next) {
  next_ = next;
}  

TestPartResult* TestPartResult::getNext() const {
  return next_;
}  

testType TestPartResult::getType() const {
	return type_; 
} 

const SimpleString& TestPartResult::getMessage() const {
  return message_;
}
  	
Test* TestPartResult::getTest() const {
  return test_;
}
  	
const SimpleString& TestPartResult::getFileName() const {
  return fileName_;
}
  	
long TestPartResult::getLineNumber() const {
  return lineNumber_;
}

