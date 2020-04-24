#pragma once

#include <stdexcept>
#include <utility>

enum PairIndex : std::size_t
{
    LEFT = 0,
    RIGHT = 1,
};

template<typename T>
struct Pair
{
    T left;
    T right;

    constexpr Pair() = default;

    // Construct from left+right items
    constexpr Pair(const T& l, const T& r)
        : left(l)
        , right(r)
    {
    }

    constexpr Pair(T&& l, T&& r)
        : left(std::move(l))
        , right(std::move(r))
    {
    }

    // Construct by forwarding parameters to the ctor of T
    template<typename U,
             typename V,
             typename = std::enable_if_t<std::is_constructible_v<T, U> && std::is_constructible_v<T, V>>>
    constexpr Pair(U&& x, V&& y)
        : left(std::forward<U>(x))
        , right(std::forward<V>(y))
    {
    }

    // Construct from other Pair
    constexpr Pair(const Pair<T>&) = default;
    constexpr Pair(Pair<T>&&) = default;

    // Assign from other Pair
    Pair<T>& operator=(const Pair<T>&) = default;
    Pair<T>& operator=(Pair<T>&&) = default;

    // Get left/right item by index
    T& operator[](std::size_t idx)
    {
        switch (idx) {
        case PairIndex::LEFT:
            return left;
        case PairIndex::RIGHT:
            return right;
        default:
            throw std::out_of_range("Bad index");
        }
    }

    const T& operator[](std::size_t idx) const
    {
        switch (idx) {
        case PairIndex::LEFT:
            return left;
        case PairIndex::RIGHT:
            return right;
        default:
            throw std::out_of_range("Bad index");
        }
    }

    // For use with algorithms / range-based for
    T* begin()
    {
        return &left;
    }

    const T* cbegin() const
    {
        return &left;
    }

    T* end()
    {
        return &right;
    }

    const T* cend() const
    {
        return &right;
    }

    // Apply f to left and right
    template<typename F, typename R = std::result_of_t<F && (T&)>>
    std::enable_if_t<std::is_void_v<R>, void> apply(F&& f)
    {
        std::forward<F>(f)(left);
        std::forward<F>(f)(right);
    }

    // Apply f to left and right and return the result of each call as a Pair
    template<typename F, typename R = std::result_of_t<F && (T&)>>
    constexpr std::enable_if_t<!std::is_void_v<R>, Pair<R>> apply(F&& f)
    {
        return {std::forward<F>(f)(left), std::forward<F>(f)(right)};
    }
};
