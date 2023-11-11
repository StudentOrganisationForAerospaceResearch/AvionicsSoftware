///\file

/******************************************************************************
The MIT License(MIT)

Embedded Template Library.
https://github.com/ETLCPP/etl
https://www.etlcpp.com

Copyright(c) 2019 John Wellbelove

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

#ifndef ETL_TO_U16STRING_INCLUDED
#define ETL_TO_U16STRING_INCLUDED

///\ingroup string

#include "platform.h"
#include "private/to_string_helper.h"
#include "type_traits.h"
#include "u16format_spec.h"
#include "u16string.h"

namespace etl {
//***************************************************************************
/// Default format spec.
/// !etl::iu16string && !etl::u16string_view
//***************************************************************************
template <typename T>
typename etl::enable_if<!etl::is_same<T, etl::iu16string>::value && !etl::is_same<T, etl::u16string_view>::value,
                        const etl::iu16string&>::type
to_string(const T value, etl::iu16string& str, bool append = false) {
    etl::u16format_spec format;

    return private_to_string::to_string(value, str, format, append);
}

//***************************************************************************
/// Supplied format spec.
/// !etl::iu16string && !etl::u16string_view
//***************************************************************************
template <typename T>
typename etl::enable_if<!etl::is_same<T, etl::iu16string>::value && !etl::is_same<T, etl::u16string_view>::value,
                        const etl::iu16string&>::type
to_string(const T value, etl::iu16string& str, const etl::u16format_spec& format, bool append = false) {
    return private_to_string::to_string(value, str, format, append);
}

//***************************************************************************
/// Default format spec.
/// !etl::iu16string && !etl::u16string_view
//***************************************************************************
template <typename T>
typename etl::enable_if<!etl::is_same<T, etl::iu16string>::value && !etl::is_same<T, etl::u16string_view>::value,
                        const etl::iu16string&>::type
to_string(const T value, uint32_t denominator_exponent, etl::iu16string& str, bool append = false) {
    etl::u16format_spec format;

    return private_to_string::to_string(value, denominator_exponent, str, format, append);
}

//***************************************************************************
/// Supplied format spec.
/// !etl::u16string_view && !etl::u16string_view
//***************************************************************************
template <typename T>
typename etl::enable_if<!etl::is_same<T, etl::iu16string>::value && !etl::is_same<T, etl::u16string_view>::value,
                        const etl::iu16string&>::type
to_string(const T value, uint32_t denominator_exponent, etl::iu16string& str, const etl::u16format_spec& format,
          bool append = false) {
    return private_to_string::to_string(value, denominator_exponent, str, format, append);
}

//***************************************************************************
/// Default format spec.
/// etl::iu16string
//***************************************************************************
template <typename T>
typename etl::enable_if<etl::is_same<T, etl::iu16string>::value, const etl::iu16string&>::type to_string(
    const T& value, etl::iu16string& str, bool append = false) {
    etl::u16format_spec format;

    private_to_string::add_string(value, str, format, append);

    return str;
}

//***************************************************************************
/// Supplied format spec.
/// etl::iu16string
//***************************************************************************
template <typename T>
typename etl::enable_if<etl::is_same<T, etl::iu16string>::value, const etl::iu16string&>::type to_string(
    const etl::iu16string& value, T& str, const etl::u16format_spec& format, bool append = false) {
    private_to_string::add_string(value, str, format, append);

    return str;
}

//***************************************************************************
/// Default format spec.
/// etl::u16string_view
//***************************************************************************
template <typename T>
typename etl::enable_if<etl::is_same<T, etl::u16string_view>::value, const etl::iu16string&>::type to_string(
    T value, etl::iu16string& str, bool append = false) {
    etl::u16format_spec format;

    private_to_string::add_string_view(value, str, format, append);

    return str;
}

//***************************************************************************
/// Supplied format spec.
/// etl::u16string_view
//***************************************************************************
template <typename T>
typename etl::enable_if<etl::is_same<T, etl::u16string_view>::value, const etl::iu16string&>::type to_string(
    T value, etl::iu16string& str, const etl::u16format_spec& format, bool append = false) {
    private_to_string::add_string_view(value, str, format, append);

    return str;
}
}  // namespace etl

#endif
