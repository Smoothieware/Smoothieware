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

This class was originally created by Michael Feathers and was modified
by Barthelemy Dagenais.
*/


#ifndef SIMPLE_STRING
#define SIMPLE_STRING


/**
 * SimpleString is a simple implementation of the std class String and is
 * provided to ease the manipulation of strings without using any other
 * libraries.
 */
class SimpleString
{
	friend bool	operator== (const SimpleString& left, const SimpleString& right);

	friend bool	operator!= (const SimpleString& left, const SimpleString& right);

  public:
    SimpleString ();
		SimpleString (const char *value);
		SimpleString (const SimpleString& other);
		~SimpleString ();

	  SimpleString operator= (const SimpleString& other);
	
    SimpleString operator+ (const SimpleString& other);
  
	  char *asCharString () const;
	  int size() const;

  private:
	  char *buffer;
};

// Those functions are provided to ease the conversion between
// primary datatypes and SimpleString. Feel free to extend this list
// to support your own datatype.
SimpleString StringFrom (bool value);
SimpleString StringFrom (const char *value);
SimpleString StringFrom (long value);
SimpleString StringFrom (int value);
SimpleString StringFrom (double value);
SimpleString StringFrom (const SimpleString& other);

#endif

