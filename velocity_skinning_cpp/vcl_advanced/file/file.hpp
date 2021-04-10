#pragma once

#include "vcl/vcl.hpp"

#include <sstream>

namespace vcl
{

template <typename T>
std::istream& read_from_stream(std::istream& stream, T& data)
{
    stream >> data;
    return stream;
}

template <typename T, size_t N>
std::istream& read_from_stream(std::istream& stream, buffer_stack<T,N>& data)
{
    for(size_t k=0; k<N && stream.good(); ++k)
        read_from_stream(stream,data[k]);
    return stream;
}

template <typename T>
std::istream& read_line_from_stream(std::istream& stream, T& data)
{
    if(stream.good()) {
        std::string line;
        std::getline(stream, line);
        if(stream.good())
        {
            std::istringstream stream_line(line);
            read_from_stream(stream_line, data);
        }
    }

    return stream;
}

template <typename T>
std::istream& read_from_stream(std::istream& stream, buffer<T>& data)
{
    while(stream.good()) {
        T temp;
        read_from_stream(stream, temp);
        if(stream.good())
            data.push_back(temp);
    }

    return stream;
}

template <typename T>
std::istream& read_from_stream_per_line(std::istream& stream, buffer<T>& data)
{
    while(stream.good()) {
        T temp;
        read_line_from_stream(stream, temp);
        if(stream.good())
            data.push_back(temp);
    }

    return stream;
}





template <typename T>
T& read_from_file(std::string const& filename, T& data)
{
    assert_file_exist(filename);

    // Open file with pointer at last position
    std::ifstream stream(filename);
    assert_vcl_no_msg(stream.is_open());

    read_from_stream(stream, data);

    stream.close();
    assert_vcl_no_msg(!stream.is_open());

    return data;
}

template <typename T>
buffer<buffer<T>>& read_from_file(std::string const& filename, buffer<buffer<T>>& data)
{
    assert_file_exist(filename);

    // Open file with pointer at last position
    std::ifstream stream(filename);
    assert_vcl_no_msg(stream.is_open());

    read_from_stream_per_line(stream, data);

    stream.close();
    assert_vcl_no_msg(!stream.is_open());

    return data;
}

/*
template <typename T>
T& read_from_file(std::string const& filename, T& data)
{
    assert_file_exist(filename);

    // Open file with pointer at last position
    std::ifstream stream(filename);
    assert_vcl_no_msg(stream.is_open());

    read_from_stream(stream, data);

    stream.close();
    assert_vcl_no_msg(!stream.is_open());

    return data;
}
*/



template <typename T, typename READER> buffer<T> file_reader_as_buffer(std::string const& filename,
                                                                       T const&,
                                                                       READER const& reader)
{
    assert_file_exist(filename);

    // Open file with pointer at last position
    std::ifstream stream(filename);
    assert_vcl_no_msg(stream.is_open());

    buffer<T> data;
    while(stream.good())
    {
        T value;
        reader(stream, value);
        if(stream.good())
            data.push_back(value);
    }

    stream.close();
    assert_vcl_no_msg(!stream.is_open());

    return data;
}

/*
template <typename T, typename READER> buffer<T> file_reader_as_buffer_generic(const std::string& filename,
                                                                               READER const& reader);

// Read per line, then per entity
template <typename T, typename READER> buffer<buffer<T>> file_reader_as_bbuffer_generic(const std::string& filename,
                                                                                        READER const& reader);

template <typename T> struct file_reader_as_buffer { static vcl::buffer<T> read(std::string const& filename); };

template <typename T, size_t N> struct file_reader_as_buffer<buffer_stack<T,N> > { static vcl::buffer<buffer_stack<T,N> > read(std::string const& filename); };

template <typename T, typename READER> struct file_reader_as_bbuffer
{
    static buffer<buffer<T>> read(std::string const& filename, READER const& reader=[](std::ifstream& stream,T& value){stream>>value;})
    {
    return file_reader_as_bbuffer_generic<T,READER>(filename, reader);
    }
};
*/

}

// Implementation

namespace vcl
{
/*
template <typename T, typename READER> buffer<T> stream_reader_as_buffer_generic(std::istream& stream, READER const& reader)
{
    buffer<T> data;
    while(stream.good())
    {
        T value;
        reader(stream, value);
        if(stream.good())
            data.push_back(value);
    }
    return data;
}

template <typename T, typename READER> buffer<T> file_reader_as_buffer_generic(const std::string& filename, READER const& reader)
{
    assert_file_exist(filename);

    // Open file with pointer at last position
    std::ifstream stream(filename);
    assert_vcl_no_msg(stream.is_open());

    buffer<T> data = stream_reader_as_buffer_generic<T,READER>(stream, reader);

    stream.close();
    assert_vcl_no_msg(!stream.is_open());

    return data;
}

template <typename T, typename READER> buffer<buffer<T>> file_reader_as_bbuffer_generic(const std::string& filename,
                                                                                 READER const& reader)
{

    buffer<std::string> lines = read_file_text_lines(filename);
    size_t const N_lines = lines.size();

    buffer<buffer<T> > data(N_lines);
    for(size_t k=0; k<N_lines; ++k)
    {
        std::istringstream stream(lines[k]);
        data[k] = stream_reader_as_buffer_generic<T,READER>(stream, reader);
    }
    return data;
}

template <typename T>
buffer<T> file_reader_as_buffer<T>::read(std::string const& filename)
{
    return file_reader_as_buffer_generic<T>(filename, [](std::ifstream& stream,T& value){stream>>value;});
}


template <typename T, size_t N>
buffer<buffer_stack<T,N> > file_reader_as_buffer<buffer_stack<T,N> >::read(const std::string& filename)
{
    return file_reader_as_buffer_generic<buffer_stack<T,N> >
            (filename,
             [](std::istream& stream, buffer_stack<T,N>& value)
    {
        for(size_t k=0; k<N && stream.good(); ++k){
            stream >> value[k];
        }
    });
}
*/

}
