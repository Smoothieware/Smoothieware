/*
Copyright (c) 2011 Andy Kirkham
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef AJK_FPOINTER_H
#define AJK_FPOINTER_H

#ifndef NULL
#define NULL 0
#endif

#include <cstdint>

namespace AjK {

class FPointerDummy;

/** FPointer - Adds callbacks that take and return a 32bit uint32_t data type.
*
* The Mbed library supplies a callback using the FunctionPointer object as
* defined in FunctionPointer.h However, this callback system does not allow
* the caller to pass a value to the callback. Likewise, the callback itself
* cannot return a value.
*
* FPointer operates in the same way but allows the callback function to be
* passed one arg, a uint32_t value. Additionally, the callback can return
* a single uint32_t value. The reason for using uint32_t is that the Mbed
* and the microcontroller (LPC1768) have a natural data size of 32bits and
* this means we can use the uint32_t as a pointer. See example1.h for more
* information. This example passes an "int" by passing a pointer to that
* int as a 32bit value. Using this technique you can pass any value you like.
* All you have to do is pass a pointer to your value cast to (uint32_t). Your
* callback can the deference it to get the original value.
*
* example2.h shows how to do the same thing but demostrates how to specify
* the callback into a class object/method.
*
* Finally, example3.h shows how to pass multiple values. In this example we
* define a data structure and in the callback we pass a pointer to that
* data structure thus allowing the callback to again get the values.
*
* Note, when passing pointers to variables to the callback, if the callback
* function/method changes that variable's value then it will also change the
* value the caller sees. If C pointers are new to you, you are strongly
* advised to read up on the subject. It's pointers that often get beginners
* into trouble when mis-used.
*
* @see example1.h
* @see example2.h
* @see example3.h
* @see http://mbed.org/handbook/C-Data-Types
* @see http://mbed.org/projects/libraries/svn/mbed/trunk/FunctionPointer.h
*/
class FPointer {

protected:

    //! C callback function pointer.
    uint32_t (*c_callback)(uint32_t);
    
    //! C++ callback object/method pointer (the object part).
    FPointerDummy *obj_callback;
    
    //! C++ callback object/method pointer (the method part).
    uint32_t (FPointerDummy::*method_callback)(uint32_t);

public:
    
    /** Constructor
*/
    FPointer() {
        c_callback = NULL;
        obj_callback = NULL;
        method_callback = NULL;
    }
    
    /** attach - Overloaded attachment function.
*
* Attach a C type function pointer as the callback.
*
* Note, the callback function prototype must be:-
* @code
* uint32_t myCallbackFunction(uint32_t);
* @endcode
* @param A C function pointer to call.
*/
    void attach(uint32_t (*function)(uint32_t) = 0) { c_callback = function; }
    
    /** attach - Overloaded attachment function.
*
* Attach a C++ type object/method pointer as the callback.
*
* Note, the callback method prototype must be:-
* @code
* public:
* uint32_t myCallbackFunction(uint32_t);
* @endcode
* @param A C++ object pointer.
* @param A C++ method within the object to call.
*/
    template<class T>
    void attach(T* item, uint32_t (T::*method)(uint32_t)) {
        obj_callback = (FPointerDummy *)item;
        method_callback = (uint32_t (FPointerDummy::*)(uint32_t))method;
    }

    /** call - Overloaded callback initiator.
*
* call the callback function.
*
* @param uint32_t The value to pass to the callback.
* @return uint32_t The value the callback returns.
*/
    uint32_t call(uint32_t arg) {
        if (c_callback != NULL) {
            return (*c_callback)(arg);
        }
        else {
            if (obj_callback != NULL && method_callback != NULL) {
                return (obj_callback->*method_callback)(arg);
            }
        }
        return (uint32_t)NULL;
    }
    
    /** call - Overloaded callback initiator.
*
* Call the callback function without passing an argument.
* The callback itself is passed NULL. Note, the callback
* prototype should still be <b>uint32_t callback(uint32_t)</b>.
*
* @return uint32_t The value the callback returns.
*/
    uint32_t call(void) {
        if (c_callback != NULL) {
            return (*c_callback)((uint32_t)NULL);
        }
        else {
            if (obj_callback != NULL && method_callback != NULL) {
                return (obj_callback->*method_callback)((uint32_t)NULL);
            }
        }
        return (uint32_t)NULL;
    }
};

}; // namespace AjK ends

using namespace AjK;

#endif
