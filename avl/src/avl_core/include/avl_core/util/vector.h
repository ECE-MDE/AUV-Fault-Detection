//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for manipulation of std::vectors
//==============================================================================

#ifndef VECTOR_H
#define VECTOR_H

// C++ includes
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <sstream>
#include <cmath>

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        vec_from_string
// Description: Converts a string containing delimited numbers to a vector of
//              doubles.
// Arguments:   - str: String containing vector elements.
//              - delim: Delimiter between numbers.
// Returns:     Vector of doubles from the input string.
//------------------------------------------------------------------------------
std::vector<double> vec_from_string(const std::string& str,
    const char& delim = ',');

//==============================================================================
//                          TEMPLATE FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        subvector
// Description: Gets a subvector from a larger vector, starting at the specified
//              index to the end of th einput vector.
// Arguments:   - vect: vector from which to get a subvector
//              - start: starting index of subvector
// Returns:     The subvector of the input vector.
//------------------------------------------------------------------------------
template<typename T>
std::vector<T> subvector(std::vector<T> vect, size_t start)
{

    // Check that the start index is within the input vector
    if (start >= vect.size())
    {
        std::stringstream ss;
        ss << "subvector: start index [" << start << "] "
           << "past end of length [" << vect.size() << "] vector";
        throw std::runtime_error(ss.str());
    }

    typename std::vector<T>::const_iterator first = vect.begin() + start;
    typename std::vector<T>::const_iterator last = vect.end();
    std::vector<T> sub_vector(first, last);
    return sub_vector;

}

//------------------------------------------------------------------------------
// Name:        subvector
// Description: Gets a subvector from a larger vector, starting at the specified
//              index with the specified number of elements.
// Arguments:   - vect: vector from which to get a subvector
//              - start: starting index of subvector
//              - num: number of elements in subvector
// Returns:     The subvector of the input vector.
//------------------------------------------------------------------------------
template<typename T>
std::vector<T> subvector(std::vector<T> vect, size_t start, size_t num)
{

    // Check that the start index is within the input vector
    if (start >= vect.size())
    {
        std::stringstream ss;
        ss << "subvector: start index [" << start << "] "
           << "past end of length [" << vect.size() << "] vector";
        throw std::runtime_error(ss.str());
    }

    // Check that the input vector has enough elements to create the subvector
    if (vect.size() < num)
    {
        std::stringstream ss;
        ss << "subvector: input vector length [" << vect.size() << "] "
           << "less than requested subvector length [" << num << "]";
        throw std::runtime_error(ss.str());
    }

    typename std::vector<T>::const_iterator first = vect.begin() + start;
    typename std::vector<T>::const_iterator last = first + num;
    std::vector<T> sub_vector(first, last);
    return sub_vector;

}

//------------------------------------------------------------------------------
// Name:        append
// Description: Appends the second vector onto the end of the first vector.
// Arguments:   - vector0: vector to append to
//              - vector1: vector to be appended
//------------------------------------------------------------------------------
template<typename T>
void append(std::vector<T>& vector0, std::vector<T> vector1)
{
    vector0.insert(vector0.end(), vector1.begin(), vector1.end());
}

//------------------------------------------------------------------------------
// Name:        remove
// Description: Removes a specified number of elements from a vector, starting
//              at the specified index.
// Arguments:   - vector: vector from which to remove elements
//              - start: starting index of removal
//              - num: number of elements to remove
// Returns:     Vector with specified elements removed.
//------------------------------------------------------------------------------
template<typename T>
void remove(std::vector<T>& vector, int start, int num)
{
    vector.erase(vector.begin() + start, vector.begin() + start + num );
}

//------------------------------------------------------------------------------
// Name:        remove_elements
// Description: Removes all instances of the given element from the vector.
// Arguments:   - vector: vector from which to remove elements
//              - elem: element to remove
// Returns:     Vector with specified elements removed.
//------------------------------------------------------------------------------
template<typename T>
void remove_elements(std::vector<T>& vec, T elem)
{
    vec.erase(std::remove(vec.begin(), vec.end(), elem), vec.end());
}

//------------------------------------------------------------------------------
// Name:        find_subvector
// Description: Finds the start index of all instances of a subvector in a
//              vector.
// Arguments:   - vec: vector in which to find instances of the subvector
//              - sub: subvector to find in the vector
// Returns:     Vector of start indicies of the subvector found in the vector.
//------------------------------------------------------------------------------
template<typename T>
std::vector<size_t> find_subvector(const std::vector<T>& vec,
                                   const std::vector<T>& sub)
{
    std::vector<size_t> result = {};
    auto it = vec.begin();
    while ((it = std::search(it, vec.end(), sub.begin(), sub.end())) != vec.end())
        result.push_back(std::distance(vec.begin(), it++));
    return result;
}

//------------------------------------------------------------------------------
// Name:        has_element
// Description: Checks whether a vector contains a specific element.
// Arguments:   - vec: Vector in which to check for the element.
//              - elem: Element to check vector for.
// Returns:     True if the vector contains the element, false if it does not.
//------------------------------------------------------------------------------
template<typename T>
bool has_element(const std::vector<T>& vec, const T& elem)
{
    return std::find(vec.begin(), vec.end(), elem) != vec.end();
}

//------------------------------------------------------------------------------
// Name:        cast_all
// Description: Casts all elements of a vector to a new type.
// Arguments:   - v1: Vector of elements to be cast to the new type.
// Returns:     Vector of elements with cast types.
//------------------------------------------------------------------------------
template <typename T1, typename T2>
std::vector<T2> cast_all(std::vector<T1> v1)
{
    std::vector<T2> v2;
    for (T1 element : v1)
        v2.push_back(static_cast<T2>(element));
    return v2;
}

} // namespace avl

#endif // VECTOR_H
