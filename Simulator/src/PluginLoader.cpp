#include "PluginLoader.h"
#include <dlfcn.h>
#include <iostream>

namespace simulator {

PluginLoader::~PluginLoader() {
    closeLibraries();
}

bool PluginLoader::loadLibrary(const std::string& path) {
    // Clear any existing error
    dlerror();
    
    // Load the library
    void* handle = dlopen(path.c_str(), RTLD_LAZY | RTLD_LOCAL);
    
    if (!handle) {
        const char* error = dlerror();
        std::cerr << "Error loading plugin '" << path << "': " 
                  << (error ? error : "Unknown error") << std::endl;
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(mtx_);
        handles_.push_back(handle);
    }
    return true;
}

void PluginLoader::closeLibraries() {
    for (auto it = handles_.rbegin(); it != handles_.rend(); ++it) {
        if (*it) {
            dlclose(*it);
        }
    }
    handles_.clear();
}

} // namespace simulator
