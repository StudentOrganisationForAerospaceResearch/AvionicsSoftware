/******************************************************************************
The MIT License(MIT)

Embedded Template Library.
https://github.com/ETLCPP/etl
https://www.etlcpp.com

Copyright(c) 2018 John Wellbelove

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

#ifndef ETL_TYPE_SELECT_INCLUDED
#define ETL_TYPE_SELECT_INCLUDED

#include "null_type.h"
#include "platform.h"
#include "static_assert.h"
#include "type_traits.h"

#if 0
#error THIS HEADER IS A GENERATOR. DO NOT INCLUDE.
#endif

//***************************************************************************
// THIS FILE HAS BEEN AUTO GENERATED. DO NOT EDIT THIS FILE.
//***************************************************************************

namespace etl {
#if ETL_USING_CPP11 && !defined(ETL_TYPE_SELECT_FORCE_CPP03_IMPLEMENTATION)
//***************************************************************************
// Variadic version.
//***************************************************************************
template <typename... TTypes>
struct type_select {
 private:
  //***********************************
  template <size_t ID, size_t N, typename T1, typename... TRest>
  struct type_select_helper {
    using type = typename etl::conditional<
        ID == N, T1,
        typename type_select_helper<ID, N + 1, TRest...>::type>::type;
  };

  //***********************************
  template <size_t ID, size_t N, typename T1>
  struct type_select_helper<ID, N, T1> {
    using type = T1;
  };

 public:
  template <size_t ID>
  struct select {
    static_assert(ID < sizeof...(TTypes), "Illegal type_select::select index");

    using type = typename type_select_helper<ID, 0, TTypes...>::type;
  };

  template <size_t ID>
  using select_t = typename select<ID>::type;
};

//***************************************************************************
// Select type alias
//***************************************************************************
template <size_t N, typename... TTypes>
using type_select_t =
    typename etl::type_select<TTypes...>::template select_t<N>;

#else

//***************************************************************************
// For 16 types.
//***************************************************************************
template <typename T0, typename T1 = void, typename T2 = void,
          typename T3 = void, typename T4 = void, typename T5 = void,
          typename T6 = void, typename T7 = void, typename T8 = void,
          typename T9 = void, typename T10 = void, typename T11 = void,
          typename T12 = void, typename T13 = void, typename T14 = void,
          typename T15 = void>
struct type_select {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8,
                                        typename etl::conditional<
                                            ID == 9, T9,
                                            typename etl::conditional<
                                                ID == 10, T10,
                                                typename etl::conditional<
                                                    ID == 11, T11,
                                                    typename etl::conditional<
                                                        ID == 12, T12,
                                                        typename etl::conditional<
                                                            ID == 13, T13,
                                                            typename etl::conditional<
                                                                ID == 14, T14,
                                                                typename etl::
                                                                    conditional<
                                                                        ID ==
                                                                            15,
                                                                        T15,
                                                                        etl::null_type<
                                                                            0>>::
                                                                        type>::
                                                                type>::type>::
                                                        type>::type>::type>::
                                            type>::type>::type>::type>::type>::
                        type>::type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 16, "Invalid ID");
  };
};

//***************************************************************************
// For 15 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8, typename T9,
          typename T10, typename T11, typename T12, typename T13, typename T14>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13,
                   T14> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8,
                                        typename etl::conditional<
                                            ID == 9, T9,
                                            typename etl::conditional<
                                                ID == 10, T10,
                                                typename etl::conditional<
                                                    ID == 11, T11,
                                                    typename etl::conditional<
                                                        ID == 12, T12,
                                                        typename etl::conditional<
                                                            ID == 13, T13,
                                                            typename etl::
                                                                conditional<
                                                                    ID == 14,
                                                                    T14,
                                                                    etl::
                                                                        null_type<
                                                                            0>>::
                                                                    type>::
                                                            type>::type>::
                                                    type>::type>::type>::type>::
                                    type>::type>::type>::type>::type>::type>::
            type>::type type;

    ETL_STATIC_ASSERT(ID < 15, "Invalid ID");
  };
};

//***************************************************************************
// For 14 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8, typename T9,
          typename T10, typename T11, typename T12, typename T13>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8,
                                        typename etl::conditional<
                                            ID == 9, T9,
                                            typename etl::conditional<
                                                ID == 10, T10,
                                                typename etl::conditional<
                                                    ID == 11, T11,
                                                    typename etl::conditional<
                                                        ID == 12, T12,
                                                        typename etl::
                                                            conditional<
                                                                ID == 13, T13,
                                                                etl::null_type<
                                                                    0>>::type>::
                                                        type>::type>::type>::
                                            type>::type>::type>::type>::type>::
                        type>::type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 14, "Invalid ID");
  };
};

//***************************************************************************
// For 13 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8, typename T9,
          typename T10, typename T11, typename T12>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8,
                                        typename etl::conditional<
                                            ID == 9, T9,
                                            typename etl::conditional<
                                                ID == 10, T10,
                                                typename etl::conditional<
                                                    ID == 11, T11,
                                                    typename etl::conditional<
                                                        ID == 12, T12,
                                                        etl::null_type<0>>::
                                                        type>::type>::type>::
                                            type>::type>::type>::type>::type>::
                        type>::type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 13, "Invalid ID");
  };
};

//***************************************************************************
// For 12 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8, typename T9,
          typename T10, typename T11>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8,
                                        typename etl::conditional<
                                            ID == 9, T9,
                                            typename etl::conditional<
                                                ID == 10, T10,
                                                typename etl::conditional<
                                                    ID == 11, T11,
                                                    etl::null_type<0>>::type>::
                                                type>::type>::type>::type>::
                                type>::type>::type>::type>::type>::type>::type
        type;

    ETL_STATIC_ASSERT(ID < 12, "Invalid ID");
  };
};

//***************************************************************************
// For 11 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8, typename T9,
          typename T10>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8,
                                        typename etl::conditional<
                                            ID == 9, T9,
                                            typename etl::conditional<
                                                ID == 10, T10,
                                                etl::null_type<0>>::type>::
                                            type>::type>::type>::type>::type>::
                        type>::type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 11, "Invalid ID");
  };
};

//***************************************************************************
// For 10 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8, typename T9>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8,
                                        typename etl::conditional<
                                            ID == 9, T9, etl::null_type<0>>::
                                            type>::type>::type>::type>::type>::
                        type>::type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 10, "Invalid ID");
  };
};

//***************************************************************************
// For 9 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7, T8> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7,
                                    typename etl::conditional<
                                        ID == 8, T8, etl::null_type<0>>::type>::
                                    type>::type>::type>::type>::type>::type>::
            type>::type type;

    ETL_STATIC_ASSERT(ID < 9, "Invalid ID");
  };
};

//***************************************************************************
// For 8 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7>
struct type_select<T0, T1, T2, T3, T4, T5, T6, T7> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6,
                                typename etl::conditional<
                                    ID == 7, T7, etl::null_type<0>>::type>::
                                type>::type>::type>::type>::type>::type>::type
        type;

    ETL_STATIC_ASSERT(ID < 8, "Invalid ID");
  };
};

//***************************************************************************
// For 7 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6>
struct type_select<T0, T1, T2, T3, T4, T5, T6> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5,
                            typename etl::conditional<
                                ID == 6, T6, etl::null_type<0>>::type>::type>::
                        type>::type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 7, "Invalid ID");
  };
};

//***************************************************************************
// For 6 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4,
          typename T5>
struct type_select<T0, T1, T2, T3, T4, T5> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<
                        ID == 4, T4,
                        typename etl::conditional<
                            ID == 5, T5, etl::null_type<0>>::type>::type>::
                    type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 6, "Invalid ID");
  };
};

//***************************************************************************
// For 5 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3, typename T4>
struct type_select<T0, T1, T2, T3, T4> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<
                    ID == 3, T3,
                    typename etl::conditional<ID == 4, T4, etl::null_type<0>>::
                        type>::type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 5, "Invalid ID");
  };
};

//***************************************************************************
// For 4 types.
//***************************************************************************
template <typename T0, typename T1, typename T2, typename T3>
struct type_select<T0, T1, T2, T3> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<
                ID == 2, T2,
                typename etl::conditional<ID == 3, T3, etl::null_type<0>>::
                    type>::type>::type>::type type;

    ETL_STATIC_ASSERT(ID < 4, "Invalid ID");
  };
};

//***************************************************************************
// For 3 types.
//***************************************************************************
template <typename T0, typename T1, typename T2>
struct type_select<T0, T1, T2> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<
            ID == 1, T1,
            typename etl::conditional<ID == 2, T2, etl::null_type<0>>::type>::
            type>::type type;

    ETL_STATIC_ASSERT(ID < 3, "Invalid ID");
  };
};

//***************************************************************************
// For 2 types.
//***************************************************************************
template <typename T0, typename T1>
struct type_select<T0, T1> {
 public:
  template <const size_t ID>
  struct select {
    typedef typename etl::conditional<
        ID == 0, T0,
        typename etl::conditional<ID == 1, T1, etl::null_type<0>>::type>::type
        type;

    ETL_STATIC_ASSERT(ID < 2, "Invalid ID");
  };
};

//***************************************************************************
// For 1 types.
//***************************************************************************
template <typename T0>
struct type_select<T0> {
 public:
  template <const size_t ID>
  struct select {
    typedef
        typename etl::conditional<ID == 0, T0, etl::null_type<0>>::type type;

    ETL_STATIC_ASSERT(ID < 1, "Invalid ID");
  };
};
#endif
}  // namespace etl

#endif
