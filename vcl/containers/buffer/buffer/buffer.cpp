#include "buffer.hpp"

namespace vcl
{

#if !defined(VCL_NO_DEBUG) && defined(SOURCE_LOCATION)
std::string buffer_access_error_message(size_t index, size_t size_buffer, std::experimental::source_location const& location_caller)
{
    std::string error_message  = "Try to access buffer["+str(index)+"], while the index exceed the size of the buffer (buffer.size()="+str(size_buffer)+").\n";
    if(size_buffer==0)
        error_message += "-> Hint: The buffer doesn't has a zero size - you may have forgotten to resize the buffer before using it; Or you may want to use buffer.push_back() instead.\n";

    error_message += "\nThe error was itself caused by the call of this function\n";
    error_message += "-----------------------------------------------------\n";
    error_message += "- File: "+str(location_caller.file_name())+"\n";
    error_message += "- Function: "+str(location_caller.function_name())+"\n";
    error_message += "- Line "+str(location_caller.line());
    return error_message;
}
#endif

}
