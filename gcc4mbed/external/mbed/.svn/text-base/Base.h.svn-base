/* mbed Microcontroller Library - Base
 * Copyright (c) 2006-2008 ARM Limited. All rights reserved.
 * sford, jbrawn
 */
 
#ifndef MBED_BASE_H
#define MBED_BASE_H

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include <cstdlib>
#include "DirHandle.h"

namespace mbed {

#ifdef MBED_RPC
struct rpc_function {
    const char *name;
    void (*caller)(const char*, char*);
};

struct rpc_class {
    const char *name;
    const rpc_function *static_functions;
    struct rpc_class *next;
};
#endif

/* Class Base
 *  The base class for most things
 */
class Base {

public: 
    
    Base(const char *name = NULL);

    virtual ~Base();

    /* Function register_object
     *  Registers this object with the given name, so that it can be
     *  looked up with lookup. If this object has already been
     *  registered, then this just changes the name.
     *
     * Variables
     *   name - The name to give the object. If NULL we do nothing.
     */
    void register_object(const char *name);

    /* Function name
     *  Returns the name of the object, or NULL if it has no name.
     */
    const char *name();

#ifdef MBED_RPC

    /* Function rpc
     *  Call the given method with the given arguments, and write the
     *  result into the string pointed to by result. The default
     *  implementation calls rpc_methods to determine the supported
     *  methods.
     *
     * Variables
     *  method - The name of the method to call.
     *  arguments - A list of arguments separated by spaces.
     *  result - A pointer to a string to write the result into. May
     *    be NULL, in which case nothing is written.
     *
     *  Returns
     *    true if method corresponds to a valid rpc method, or
     *    false otherwise.
     */
    virtual bool rpc(const char *method, const char *arguments, char *result);	

    /* Function get_rpc_methods
     *  Returns a pointer to an array describing the rpc methods
     *  supported by this object, terminated by either
     *  RPC_METHOD_END or RPC_METHOD_SUPER(Superclass).
     *
     * Example
     * > class Example : public Base {
     * >   int foo(int a, int b) { return a + b; }
     * >   virtual const struct rpc_method *get_rpc_methods() {
     * >     static const rpc_method rpc_methods[] = {
     * >       { "foo", generic_caller<int, Example, int, int, &Example::foo> },
     * >       RPC_METHOD_SUPER(Base)
     * >     };
     * >     return rpc_methods;
     * >   }
     * > };
     */
    virtual const struct rpc_method *get_rpc_methods();

    /* Function rpc
     *  Use the lookup function to lookup an object and, if
     *  successful, call its rpc method
     *
     * Variables
     *  returns - false if name does not correspond to an object,
     *    otherwise the return value of the call to the object's rpc
     *    method.
     */
    static bool rpc(const char *name, const char *method, const char *arguments, char *result);

#endif

    /* Function lookup
     *  Lookup and return the object that has the given name.
     *
     * Variables
     *  name - the name to lookup.
     *  len - the length of name.
     */
    static Base *lookup(const char *name, unsigned int len);

    static DirHandle *opendir();
    friend class BaseDirHandle;

protected: 

    static Base *_head;
    Base *_next;
    const char *_name;
    bool _from_construct;

private:

#ifdef MBED_RPC
    static rpc_class *_classes;

    static const rpc_function _base_funcs[];
    static rpc_class _base_class;
#endif

    void delete_self();
    static void list_objs(const char *arguments, char *result);
    static void clear(const char*,char*);

    static char *new_name(Base *p);

public:

#ifdef MBED_RPC
    /* Function add_rpc_class
     *  Add the class to the list of classes which can have static
     *  methods called via rpc (the static methods which can be called
     *  are defined by that class' get_rpc_class() static method).
     */
    template<class C>
    static void add_rpc_class() {
        rpc_class *c = C::get_rpc_class();
        c->next = _classes;
        _classes = c;
    }

    template<class C> 
    static const char *construct() {
        Base *p = new C();
        p->_from_construct = true;
        if(p->_name==NULL) {
            p->register_object(new_name(p));
        }
        return p->_name;
    }

    template<class C, typename A1> 
    static const char *construct(A1 arg1) {
        Base *p = new C(arg1);
        p->_from_construct = true;
        if(p->_name==NULL) {
            p->register_object(new_name(p));
        }
        return p->_name;
    }

    template<class C, typename A1, typename A2> 
    static const char *construct(A1 arg1, A2 arg2) {
        Base *p = new C(arg1,arg2);
        p->_from_construct = true;
        if(p->_name==NULL) {
            p->register_object(new_name(p));
        }
        return p->_name;
    }

    template<class C, typename A1, typename A2, typename A3> 
    static const char *construct(A1 arg1, A2 arg2, A3 arg3) {
        Base *p = new C(arg1,arg2,arg3);
        p->_from_construct = true;
        if(p->_name==NULL) {
            p->register_object(new_name(p));
        }
        return p->_name;
    }

    template<class C, typename A1, typename A2, typename A3, typename A4> 
    static const char *construct(A1 arg1, A2 arg2, A3 arg3, A4 arg4) {
        Base *p = new C(arg1,arg2,arg3,arg4);
        p->_from_construct = true;
        if(p->_name==NULL) {
            p->register_object(new_name(p));
        }
        return p->_name;
    }
#endif

};

/* Macro MBED_OBJECT_NAME_MAX
 *  The maximum size of object name (including terminating null byte)
 *  that will be recognised when using fopen to open a FileLike
 *  object, or when using the rpc function.
 */ 
#define MBED_OBJECT_NAME_MAX 32

/* Macro MBED_METHOD_NAME_MAX
 *  The maximum size of rpc method name (including terminating null
 *  byte) that will be recognised by the rpc function (in rpc.h).
 */ 
#define MBED_METHOD_NAME_MAX 32

} // namespace mbed

#endif

