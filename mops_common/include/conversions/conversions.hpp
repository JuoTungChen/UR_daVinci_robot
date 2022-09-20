#pragma once

#include <type_traits>

template<typename T>
struct assert_false : std::false_type
{
};

template<typename To, typename From>
To convert_to(const From&)
{
    static_assert(assert_false<To>::value, "Unsupported conversion");
}
