///\file

/******************************************************************************
The MIT License(MIT)

Embedded Template Library.
https://github.com/ETLCPP/etl
https://www.etlcpp.com

Copyright(c) 2014 John Wellbelove

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

/*[[[cog
import cog
cog.outl("#if 0")
]]]*/
/*[[[end]]]*/
#error THIS HEADER IS A GENERATOR. DO NOT INCLUDE.
/*[[[cog
import cog
cog.outl("#endif")
]]]*/
/*[[[end]]]*/

/*[[[cog
import cog
cog.outl("//***************************************************************************")
cog.outl("// THIS FILE HAS BEEN AUTO GENERATED. DO NOT EDIT THIS FILE.")
cog.outl("//***************************************************************************")
]]]*/
/*[[[end]]]*/

//***************************************************************************
// To generate to header file, run this at the command line.
// Note: You will need Python and COG installed.
//
// python -m cogapp -d -e -olargest.h -DNTypes=<n> largest_generator.h
// Where <n> is the number of types to support.
//
// e.g.
// To generate handlers for up to 16 types...
// python -m cogapp -d -e -olargest.h -DNTypes=16 largest_generator.h
//
// See generate.bat
//***************************************************************************

#ifndef ETL_LARGEST_INCLUDED
#define ETL_LARGEST_INCLUDED

///\defgroup largest largest
///\ingroup utilities

#include "platform.h"
#include "smallest.h"
#include "static_assert.h"
#include "type_traits.h"

namespace etl {
#if ETL_USING_CPP11 && !defined(ETL_LARGEST_TYPE_FORCE_CPP03_IMPLEMENTATION)
//***************************************************************************
/// Template to determine the largest type and size.
/// Defines 'value_type' which is the type of the largest parameter.
/// Defines 'size' which is the size of the largest parameter.
///\ingroup largest
//***************************************************************************
template <typename T1, typename... TRest>
class largest_type {
 private:
  // Define 'largest_other' as 'largest_type' with all but the first parameter.
  using largest_other = typename largest_type<TRest...>::type;

 public:
  // Set 'type' to be the largest of the first parameter and any of the others.
  // This is recursive.
  using type = typename etl::conditional<
      (etl::size_of<T1>::value >
       etl::size_of<largest_other>::value),  // Boolean
      T1,                                    // TrueType
      largest_other>                         // FalseType
      ::type;                                // The largest type of the two.

  // The size of the largest type.
  enum { size = etl::size_of<type>::value };
};

//***************************************************************************
// Specialisation for one template parameter.
//***************************************************************************
template <typename T1>
class largest_type<T1> {
 public:
  using type = T1;

  enum { size = etl::size_of<type>::value };
};

#if ETL_USING_CPP11
template <typename... T>
using largest_type_t = typename largest_type<T...>::type;
#endif

#if ETL_USING_CPP17
template <typename... T>
constexpr size_t largest_type_v = largest_type<T...>::size;
#endif

#else
/*[[[cog
  import cog
  cog.outl("//***************************************************************************")
  cog.outl("/// Template to determine the largest type and size.")
  cog.outl("/// Supports up to %s types." % NTypes)
  cog.outl("/// Defines 'value_type' which is the type of the largest parameter.")
  cog.outl("/// Defines 'size' which is the size of the largest parameter.")
  cog.outl("///\ingroup largest")
  cog.outl("//***************************************************************************")
  cog.out("template <typename T1, ")
  for n in range(2, int(NTypes)):
      cog.out("typename T%s = void, " % n)
      if n % 4 == 0:
          cog.outl("")
          cog.out("          ")
  cog.outl("typename T%s = void>" % int(NTypes))
  cog.outl("struct largest_type")
  cog.outl("{")
  cog.outl("  // Define 'largest_other' as 'largest_type' with all but the first parameter. ")
  cog.out("  typedef typename largest_type<")
  for n in range(2, int(NTypes)):
      cog.out("T%s, " % n)
      if n % 16 == 0:
          cog.outl("")
          cog.out("                                ")
  cog.outl("T%s>::type largest_other;" % int(NTypes))
  cog.outl("")
  cog.outl("  // Set 'type' to be the largest of the first parameter and any of the others.")
  cog.outl("  // This is recursive.")
  cog.outl("  typedef typename etl::conditional<(sizeof(T1) > sizeof(largest_other)), // Boolean")
  cog.outl("                                     T1,                                  // TrueType")
  cog.outl("                                     largest_other>                       // FalseType")
  cog.outl("                                     ::type type;                         // The largest type of the two.")
  cog.outl("")
  cog.outl("  // The size of the largest type.")
  cog.outl("  enum")
  cog.outl("  {")
  cog.outl("    size = sizeof(type)")
  cog.outl("  };")
  cog.outl("};")
  cog.outl("")
  cog.outl("//***************************************************************************")
  cog.outl("// Specialisation for one template parameter.")
  cog.outl("//***************************************************************************")
  cog.outl("template <typename T1>")
  cog.out("struct largest_type<T1,   ")
  for n in range(2, int(NTypes)):
      cog.out("void, ")
      if n % 8 == 0:
          cog.outl("")
          cog.out("                    ")
  cog.outl("void>")
  cog.outl("{")
  cog.outl("  typedef T1 type;")
  cog.outl("")
  cog.outl("  enum")
  cog.outl("  {")
  cog.outl("    size = sizeof(type)")
  cog.outl("  };")
  cog.outl("};")
  ]]]*/
/*[[[end]]]*/
#endif

#if ETL_USING_CPP11 && \
    !defined(ETL_LARGEST_ALIGNMENT_FORCE_CPP03_IMPLEMENTATION)
//***************************************************************************
/// Template to determine the largest alignment.
/// Defines <b>value</b> which is the largest alignment of all the parameters.
///\ingroup largest
//***************************************************************************
template <typename T1, typename... TRest>
struct largest_alignment {
  // Define 'largest_other' as 'largest_type' with all but the first parameter.
  using largest_other = typename largest_alignment<TRest...>::type;

  // Set 'type' to be the largest of the first parameter and any of the others.
  // This is recursive.
  using type = typename etl::conditional<
      (etl::alignment_of<T1>::value >
       etl::alignment_of<largest_other>::value),  // Boolean
      T1,                                         // TrueType
      largest_other>                              // FalseType
      ::type;  // The largest type of the two.

  // The largest alignment.
  enum { value = etl::alignment_of<type>::value };
};

//***************************************************************************
// Specialisation for one template parameter.
//***************************************************************************
template <typename T1>
struct largest_alignment<T1> {
  typedef T1 type;

  enum { value = etl::alignment_of<type>::value };
};

#if ETL_USING_CPP17
template <typename... T>
inline constexpr size_t largest_alignment_v = largest_alignment<T...>::value;
#endif

#else
/*[[[cog
  import cog
  cog.outl("//***************************************************************************")
  cog.outl("/// Template to determine the largest alignment.")
  cog.outl("/// Supports up to %s types." % int(NTypes))
  cog.outl("/// Defines <b>value</b> which is the largest alignment of all the parameters.")
  cog.outl("///\ingroup largest")
  cog.outl("//***************************************************************************")
  cog.out("template <typename T1, ")
  for n in range(2, int(NTypes)):
      cog.out("typename T%s = void, " % n)
      if n % 4 == 0:
          cog.outl("")
          cog.out("          ")
  cog.outl("typename T%s = void>" % int(NTypes))
  cog.outl("struct largest_alignment")
  cog.outl("{")
  cog.outl("  // Define 'largest_other' as 'largest_type' with all but the first parameter. ")
  cog.out("  typedef typename largest_alignment<")
  for n in range(2, int(NTypes)):
      cog.out("T%s, " % n)
      if n % 16 == 0:
          cog.outl("")
          cog.out("                                ")
  cog.outl("T%s>::type largest_other;" % int(NTypes))
  cog.outl("")
  cog.outl("  // Set 'type' to be the largest of the first parameter and any of the others.")
  cog.outl("  // This is recursive.")
  cog.outl("  typedef typename etl::conditional<(etl::alignment_of<T1>::value > etl::alignment_of<largest_other>::value), // Boolean")
  cog.outl("                                     T1,                                                                      // TrueType")
  cog.outl("                                     largest_other>                                                           // FalseType")
  cog.outl("                                     ::type type;                                                             // The largest type of the two.")
  cog.outl("")
  cog.outl("  // The largest alignment.")
  cog.outl("  enum")
  cog.outl("  {")
  cog.outl("    value = etl::alignment_of<type>::value")
  cog.outl("  };")
  cog.outl("};")
  cog.outl("")
  cog.outl("//***************************************************************************")
  cog.outl("// Specialisation for one template parameter.")
  cog.outl("//***************************************************************************")
  cog.outl("template <typename T1>")
  cog.out("struct largest_alignment<T1,   ")
  for n in range(2, int(NTypes)):
      cog.out("void, ")
      if n % 8 == 0:
          cog.outl("")
          cog.out("                         ")
  cog.outl("void>")
  cog.outl("{")
  cog.outl("  typedef T1 type;")
  cog.outl("")
  cog.outl("  enum")
  cog.outl("  {")
  cog.outl("    value = etl::alignment_of<type>::value")
  cog.outl("  };")
  cog.outl("};")
  ]]]*/
/*[[[end]]]*/
#endif

//***************************************************************************
/// Defines a type that is as larger or larger than the specified type.
/// Will return the specified type is there is not a larger type.
///\ingroup largest
//***************************************************************************
template <typename T>
struct larger_int_type {
  ETL_STATIC_ASSERT(etl::is_integral<T>::value, "Must be an integral type");

  typedef typename etl::smallest_int_for_bits<
      etl::integral_limits<typename etl::make_signed<T>::type>::bits + 1>::type
      type;
};

#if ETL_USING_CPP11
template <typename T>
using larger_int_type_t = typename larger_int_type<T>::type;
#endif

//***************************************************************************
/// Defines a type that is as larger or larger than the specified type.
/// Will return the specified type is there is not a larger type.
///\ingroup largest
//***************************************************************************
template <typename T>
struct larger_uint_type {
  ETL_STATIC_ASSERT(etl::is_integral<T>::value, "Must be an integral type");

  typedef typename etl::smallest_uint_for_bits<
      etl::integral_limits<typename etl::make_unsigned<T>::type>::bits +
      1>::type type;
};

#if ETL_USING_CPP11
template <typename T>
using larger_uint_type_t = typename larger_uint_type<T>::type;
#endif

//***************************************************************************
/// Defines a type that is as larger or larger than the specified type.
/// Will return the specified type is there is not a larger type.
/// The returned type will be of the same sign.
///\ingroup largest
//***************************************************************************
template <typename T, bool IS_SIGNED = etl::is_signed<T>::value>
struct larger_type;

template <typename T>
struct larger_type<T, false> {
  ETL_STATIC_ASSERT(etl::is_integral<T>::value, "Must be an integral type");

  typedef typename etl::smallest_uint_for_bits<etl::integral_limits<T>::bits +
                                               1>::type type;
};

template <typename T>
struct larger_type<T, true> {
  ETL_STATIC_ASSERT(etl::is_integral<T>::value, "Must be an integral type");

  typedef typename etl::smallest_int_for_bits<etl::integral_limits<T>::bits +
                                              1>::type type;
};

#if ETL_USING_CPP11
template <typename T>
using larger_type_t = typename larger_type<T>::type;
#endif

#if ETL_USING_CPP11 && !defined(ETL_LARGEST_FORCE_CPP03_IMPLEMENTATION)
//***************************************************************************
/// Template to determine the largest type, size and alignment.
/// Defines <b>value</b> which is the largest type, size and alignment of all the parameters.
///\ingroup largest
//***************************************************************************
template <typename... T>
struct largest {
  using type = typename etl::largest_type<T...>::type;

  enum {
    size = etl::largest_type<T...>::size,
    alignment = etl::largest_alignment<T...>::value
  };
};

#if ETL_USING_CPP11
template <typename... T>
using largest_t = typename largest<T...>::type;
#endif

#if ETL_USING_CPP17
template <typename... T>
inline constexpr size_t largest_size = largest<T...>::size;
#endif

#else
/*[[[cog
  import cog
  cog.outl("//***************************************************************************")
  cog.outl("/// Template to determine the largest type, size and alignment.")
  cog.outl("/// Supports up to %s types." % NTypes)
  cog.outl("/// Defines <b>value</b> which is the largest type, size and alignment of all the parameters.")
  cog.outl("///\ingroup largest")
  cog.outl("//***************************************************************************")
  cog.out("template <typename T1, ")
  for n in range(2, int(NTypes)):
      cog.out("typename T%s = void, " % n)
      if n % 4 == 0:
          cog.outl("")
          cog.out("          ")
  cog.outl("typename T%s = void>" % NTypes)
  cog.outl("struct largest")
  cog.outl("{")
  cog.out("  typedef typename etl::largest_type<")
  for n in range(1, int(NTypes)):
      cog.out("T%s, " % n)
      if n % 16 == 0:
          cog.outl("")
          cog.out("                                     ")
  cog.outl("T%s>::type type;" % NTypes)
  cog.outl("")
  cog.outl("  enum")
  cog.outl("  {")
  cog.out("    size      = etl::largest_type<")
  for n in range(1, int(NTypes)):
      cog.out("T%s, " % n)
      if n % 16 == 0:
          cog.outl("")
          cog.out("                                  ")
  cog.outl("T%s>::size," % NTypes)
  cog.out("    alignment = etl::largest_alignment<")
  for n in range(1, int(NTypes)):
      cog.out("T%s, " % n)
      if n % 16 == 0:
          cog.outl("")
          cog.out("                                       ")
  cog.outl("T%s>::value" % NTypes)
  cog.outl("  };")
  cog.outl("};")
  ]]]*/
/*[[[end]]]*/
#endif
}  // namespace etl

#endif
