#include <algorithm>
#include <dlfcn.h>
#include <string>

#include <gtest/gtest.h>

#include "finenav_core/map/detail/map_registry.hpp"

TEST(PluginRegistration, SharedLibraryRegistersStvlMapByName) {
    void* handle = dlopen(STVL_MAP_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
    ASSERT_NE(handle, nullptr) << dlerror();

    const auto names = finenav::core::detail::MapRegistry::instance().mapNames();
    EXPECT_NE(std::find(names.begin(), names.end(), "stvl_map"), names.end());

    // Keep the DSO loaded until process exit: the registry stores factories
    // whose code belongs to this library.
}
