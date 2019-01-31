/*****************************************************************************
  File: circular_vector_buffer.hpp

  Version: 1.0
  Author: Carlos Faria <carlosfaria89@gmail.com>
  Maintainer: Carlos Faria <carlosfaria89@gmail.com>

  Copyright (C) 2018 Carlos André de Oliveira Faria. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef CIRCULAR_VECTOR_BUFFER_HPP
#define CIRCULAR_VECTOR_BUFFER_HPP

#include <algorithm>
#include <cstddef>
#include <cassert>
#include <stdexcept>
#include <deque>

/*!
 * \brief The CircularBuffers class
 * Holds a vector of std::deque objects that always keep the n-last elements.
 * Produced objects are queued to the front, and all queues updated synchronously.
 * Objects indexed to n+1 are deleted, so the buffer will only hold exactly n-elements.
 * The circular buffer object has no default constructor.
 * The size of the circular buffer (n) and the number of buffers are to be specified at object creation
 */
template <typename T>
class CircularBuffers {

public:
    typedef std::deque<T>& reference;
    typedef const std::deque<T>& const_reference;
    typedef std::deque<T>* pointer;
    typedef const std::deque<T>* const_pointer;

    CircularBuffers() = delete;

    explicit CircularBuffers(size_t num_buffers, size_t buffer_capacity);
    explicit CircularBuffers(size_t num_buffers, size_t buffer_capacity, T value);
    ~CircularBuffers();

    void swap(CircularBuffers<T>& lhs, CircularBuffers<T>& rhs);
    void swap(CircularBuffers<T>& rhs);

    const_reference operator[](size_t index) const;
    reference operator[](size_t index);
    const_reference at(size_t index) const;
    reference at(size_t index);

    void push(const std::vector<T> &item);
    bool full();
    size_t getCapacity();

private:
    std::vector< std::deque<T> > m_buffers;
    size_t m_capacity = 0;

};

template <typename T>
CircularBuffers<T>::CircularBuffers(size_t num_buffers, size_t buffer_capacity) :
    m_capacity(buffer_capacity)
{
    m_buffers.assign(num_buffers, std::deque<T>(buffer_capacity));
}

template <typename T>
CircularBuffers<T>::CircularBuffers(size_t num_buffers, size_t buffer_capacity, T value) :
    m_capacity(buffer_capacity)
{
    m_buffers.assign(num_buffers, std::deque<T>(buffer_capacity, value));
}

template <typename T>
CircularBuffers<T>::~CircularBuffers() {
    if(!m_buffers.empty())
    {
        m_buffers.clear();
    }
}

template <typename T>
void CircularBuffers<T>::swap(CircularBuffers<T>& lhs, CircularBuffers<T>& rhs)
{
    lhs.swap(rhs);
}

template <typename T>
void CircularBuffers<T>::swap(CircularBuffers<T>& rhs)
{
    using std::swap;
    swap(m_buffers,  rhs.m_buffers);
    swap(m_capacity, rhs.m_capacity);
}

template<typename T>
typename CircularBuffers<T>::const_reference
CircularBuffers<T>::operator[](size_t index) const
{
    return m_buffers[index];
}

template<typename T>
typename CircularBuffers<T>::reference
CircularBuffers<T>::operator[](size_t index)
{
    return m_buffers[index];
}

template<typename T>
typename CircularBuffers<T>::const_reference
CircularBuffers<T>::at(size_t index) const
{
    return m_buffers.at(index);
}

template<typename T>
typename CircularBuffers<T>::reference
CircularBuffers<T>::at(size_t index)
{
    return m_buffers.at(index);
}

template<typename T>
void CircularBuffers<T>::push(const std::vector<T> &items)
{
    static const std::out_of_range ex("The number of items to push do not match the number of buffers.");

    if(m_buffers.size() != items.size())
        throw ex;

//    for(size_t i=0; i<items.size(); ++i) {
//        m_buffers.at(i).push_front(items.at(i));

//        while(m_buffers.at(i).size() > m_capacity) {
//            m_buffers.at(i).pop_back();
//        }
//    }

    for(size_t i=0; i<items.size(); ++i) {
        m_buffers.at(i).push_back(items.at(i));

        while(m_buffers.at(i).size() > m_capacity) {
            m_buffers.at(i).pop_front();
        }
    }
}

template<typename T>
bool CircularBuffers<T>::full()
{
    bool full = true;
    for(std::deque<T> &dq : m_buffers) {
        full &= dq.size() == m_capacity;
    }
    return full;
}

template<typename T>
size_t CircularBuffers<T>::getCapacity()
{
    return this->m_capacity;
}

#endif //CIRCULAR_VECTOR_BUFFER_HPP
