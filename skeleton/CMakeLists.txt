cmake_minimum_required(VERSION 3.16.0 FATAL_ERROR)

set(CMAKE_SYSTEM_VERSION 10.0 CACHE STRING "" FORCE)

project(game CXX)

################################################################################
# Set target arch type if empty. Visual studio solution generator provides it.
################################################################################
if(NOT CMAKE_VS_PLATFORM_NAME)
    set(CMAKE_VS_PLATFORM_NAME "x64")
endif()
message("${CMAKE_VS_PLATFORM_NAME} architecture in use")

if(NOT ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x64"))
    message(FATAL_ERROR "${CMAKE_VS_PLATFORM_NAME} arch is not supported!")
endif()

################################################################################
# Global configuration types
################################################################################
set(CMAKE_CONFIGURATION_TYPES
    "Checked"
    "Debug"
    "Profile"
    "Release"
    CACHE STRING "" FORCE
)

################################################################################
# Global compiler options
################################################################################
if(MSVC)
    # remove default flags provided with CMake for MSVC
    set(CMAKE_CXX_FLAGS "")
    set(CMAKE_CXX_FLAGS_CHECKED "")
    set(CMAKE_CXX_FLAGS_DEBUG "")
    set(CMAKE_CXX_FLAGS_PROFILE "")
    set(CMAKE_CXX_FLAGS_RELEASE "")
endif()

################################################################################
# Global linker options
################################################################################
if(MSVC)
    # remove default flags provided with CMake for MSVC
    set(CMAKE_EXE_LINKER_FLAGS "")
    set(CMAKE_MODULE_LINKER_FLAGS "")
    set(CMAKE_SHARED_LINKER_FLAGS "")
    set(CMAKE_STATIC_LINKER_FLAGS "")
    set(CMAKE_EXE_LINKER_FLAGS_CHECKED "${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS_CHECKED "${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS_CHECKED "${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_STATIC_LINKER_FLAGS_CHECKED "${CMAKE_STATIC_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "${CMAKE_STATIC_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS_PROFILE "${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS_PROFILE "${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS_PROFILE "${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_STATIC_LINKER_FLAGS_PROFILE "${CMAKE_STATIC_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS}")
endif()

################################################################################
# Nuget packages function stub.
################################################################################
function(use_package TARGET PACKAGE VERSION)
    message(WARNING "No implementation of use_package. Create yours. "
                    "Package \"${PACKAGE}\" with version \"${VERSION}\" "
                    "for target \"${TARGET}\" is ignored!")
endfunction()

################################################################################
# Common utils
################################################################################
include(CMake/Utils.cmake)

################################################################################
# Additional Global Settings(add specific info there)
################################################################################
include(CMake/GlobalSettingsInclude.cmake OPTIONAL)

################################################################################
# Use solution folders feature
################################################################################
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################
# Sub-projects
################################################################################



























set(PROJECT_NAME game)

################################################################################
# Source groups
################################################################################
set(no_group_source_files
    "callbacks.cpp"
    "callbacks.hpp"
    "core.hpp"
    "main.cpp"
    "objects/mass_particle.cpp"
    "objects/mass_particle.hpp"
    "objects/particle.cpp"
    "objects/particle.hpp"
    "objects/projectile.cpp"
    "objects/projectile.hpp"
    "RenderUtils.cpp"
    "RenderUtils.hpp"
    "systems/particle_storage.cpp"
    "systems/particle_storage.hpp"
    "systems/particle_generator.cpp"
    "systems/particle_generator.hpp"
    "systems/particle_system.cpp"
    "systems/particle_system.hpp"
    "systems/force_composer.hpp"
    "generators/gravity_generator.cpp"
    "generators/gravity_generator.hpp"
    "generators/wind_generator.cpp"
    "generators/wind_generator.hpp"
    "generators/explosion_generator.cpp"
    "generators/explosion_generator.hpp"
    "generators/spring/spring_force_generator.cpp"
    "generators/spring/spring_force_generator.hpp"
    "generators/spring/buoyancy_generator.cpp"
    "generators/spring/buoyancy_generator.hpp"
    "types/particle_defs.hpp"
    "types/v3_f32.cpp"
    "types/v3_f32.hpp"
)
source_group("" FILES ${no_group_source_files})

set(Render
    "Render/Camera.cpp"
    "Render/Camera.h"
    "Render/Render.cpp"
    "Render/Render.h"
)
source_group("Render" FILES ${Render})

set(ALL_FILES
    ${no_group_source_files}
    ${Render}
)

################################################################################
# Target
################################################################################
add_executable(${PROJECT_NAME} ${ALL_FILES})

use_props(${PROJECT_NAME} "${CMAKE_CONFIGURATION_TYPES}" "${DEFAULT_CXX_PROPS}")
################################################################################
# Target name
################################################################################
set_target_properties(${PROJECT_NAME} PROPERTIES
    TARGET_NAME_CHECKED "${PROJECT_NAME}CHECKED"
    TARGET_NAME_DEBUG   "${PROJECT_NAME}DEBUG"
    TARGET_NAME_PROFILE "${PROJECT_NAME}PROFILE"
    TARGET_NAME_RELEASE "${PROJECT_NAME}"
)
################################################################################
# Output directory
################################################################################
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_DIRECTORY_CHECKED "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
    OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
    OUTPUT_DIRECTORY_PROFILE "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
    OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_CHECKED "${OUTPUT_DIRECTORY}"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${OUTPUT_DIRECTORY}"
    ARCHIVE_OUTPUT_DIRECTORY_PROFILE "${OUTPUT_DIRECTORY}"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    ARCHIVE_OUTPUT_NAME_CHECKED "${TARGET_NAME}"
    ARCHIVE_OUTPUT_NAME_DEBUG   "${TARGET_NAME}"
    ARCHIVE_OUTPUT_NAME_PROFILE "${TARGET_NAME}"
    ARCHIVE_OUTPUT_NAME_RELEASE "${TARGET_NAME}"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    PDB_OUTPUT_DIRECTORY         "${OUTPUT_DIRECTORY}"
)
################################################################################
# MSVC runtime library
################################################################################
get_property(MSVC_RUNTIME_LIBRARY_DEFAULT TARGET ${PROJECT_NAME} PROPERTY MSVC_RUNTIME_LIBRARY)
string(CONCAT "MSVC_RUNTIME_LIBRARY_STR"
    $<$<CONFIG:checked>:
        MultiThreaded
    >
    $<$<CONFIG:debug>:
        MultiThreadedDebug
    >
    $<$<CONFIG:profile>:
        MultiThreaded
    >
    $<$<CONFIG:release>:
        MultiThreaded
    >
    $<$<NOT:$<OR:$<CONFIG:checked>,$<CONFIG:debug>,$<CONFIG:profile>,$<CONFIG:release>>>:${MSVC_RUNTIME_LIBRARY_DEFAULT}>
)
set_target_properties(${PROJECT_NAME} PROPERTIES MSVC_RUNTIME_LIBRARY ${MSVC_RUNTIME_LIBRARY_STR})

################################################################################
# Include directories
################################################################################
target_include_directories(${PROJECT_NAME} PUBLIC
    "$ENV{PHYSXPATH}/PhysX_3.4/Include;"
    "$ENV{PHYSXPATH}/PxShared/include;"
    "$ENV{PHYSXPATH}/PxShared/src/foundation/include;"
    "$ENV{PHYSXPATH}/PxShared/src/fastxml/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/Graphics/include/win32/GL;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysX-3.4/PhysX_3.4/Include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysX-3.4/PxShared/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysX-3.4/PxShared/src/foundation/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysX-3.4/PxShared/src/fastxml/include"
)

################################################################################
# Compile definitions
################################################################################
target_compile_definitions(${PROJECT_NAME} PRIVATE
    "$<$<CONFIG:checked>:"
        "NDEBUG;"
        "PX_CHECKED=1;"
        "PX_NVTX=1;"
        "PX_SUPPORT_PVD=1"
    ">"
    "$<$<CONFIG:debug>:"
        "_DEBUG;"
        "PX_DEBUG=1;"
        "PX_CHECKED=1;"
        "PX_NVTX=1;"
        "PX_SUPPORT_PVD=1"
    ">"
    "$<$<CONFIG:profile>:"
        "NDEBUG;"
        "PX_PROFILE=1;"
        "PX_NVTX=1;"
        "PX_SUPPORT_PVD=1"
    ">"
    "$<$<CONFIG:release>:"
        "NDEBUG;"
        "PX_SUPPORT_PVD=0"
    ">"
    "_HAS_EXCEPTIONS=0;"
    "WIN32;"
    "WIN64;"
    "_CRT_SECURE_NO_DEPRECATE;"
    "_CRT_NONSTDC_NO_DEPRECATE;"
    "_WINSOCK_DEPRECATED_NO_WARNINGS;"
    "PHYSX_PROFILE_SDK;"
    "RENDER_SNIPPET"
)

################################################################################
# Compile and link options
################################################################################
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:checked>:
            /Ox
        >
        $<$<CONFIG:debug>:
            /Od;
            /RTCu
        >
        $<$<CONFIG:profile>:
            /Ox
        >
        $<$<CONFIG:release>:
            /Ox
        >
        /GF;
        /W4;
        /WX-;
        /Zi;
        /fp:fast;
        /GR-;
        /MP;
        /Wall;
        /wd4514;
        /wd4820;
        /wd4127;
        /wd4710;
        /wd4711;
        /wd4435;
        /wd4577;
        /wd4464;
        /wd4350;
        /wd4668;
        /wd4365;
        /wd4548;
        /d2Zi+;
        /GS-;
        /Y-
    )
    target_link_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:checked>:
            /LIBPATH:../../../Lib/vc14win64;
            PhysX3CommonCHECKED_x64.lib;
            PhysX3CHECKED_x64.lib;
            PhysX3CookingCHECKED_x64.lib;
            PhysX3CharacterKinematicCHECKED_x64.lib;
            PhysX3ExtensionsCHECKED.lib;
            PxTaskCHECKED_x64.lib;
            PxFoundationCHECKED_x64.lib;
            PsFastXmlCHECKED_x64.lib;
            PxPvdSDKCHECKED_x64.lib;
            /LIBPATH:../../lib/vc14win64;
            UtilsCHECKED.lib
        >
        $<$<CONFIG:profile>:
            /INCREMENTAL:NO;
            /LIBPATH:../../../Lib/vc14win64;
            PhysX3CommonPROFILE_x64.lib;
            PhysX3PROFILE_x64.lib;
            PhysX3CookingPROFILE_x64.lib;
            PhysX3CharacterKinematicPROFILE_x64.lib;
            PhysX3ExtensionsPROFILE.lib;
            PxTaskPROFILE_x64.lib;
            PxFoundationPROFILE_x64.lib;
            PsFastXmlPROFILE_x64.lib;
            PxPvdSDKPROFILE_x64.lib;
            /LIBPATH:../../lib/vc14win64;
            UtilsPROFILE.lib
        >
        $<$<CONFIG:release>:
            /INCREMENTAL:NO;
            /LIBPATH:../../../Lib/vc14win64;
            PhysX3Common_x64.lib;
            PhysX3_x64.lib;
            PhysX3Cooking_x64.lib;
            PhysX3CharacterKinematic_x64.lib;
            PhysX3Extensions.lib;
            PxTask_x64.lib;
            PxFoundation_x64.lib;
            PsFastXml_x64.lib;
            PxPvdSDK_x64.lib;
            /LIBPATH:../../lib/vc14win64;
            Utils.lib
        >
        /DEBUG;
        /MACHINE:X64;
        /SUBSYSTEM:CONSOLE
    )
endif()

################################################################################
# Dependencies
################################################################################
set(ADDITIONAL_LIBRARY_DEPENDENCIES
    "$<$<CONFIG:checked>:"
        "PhysX3CommonCHECKED_x64;"
        "PhysX3CHECKED_x64;"
        "PhysX3CookingCHECKED_x64;"
        "PhysX3CharacterKinematicCHECKED_x64;"
        "PhysX3ExtensionsCHECKED;"
        "PxTaskCHECKED_x64;"
        "PxFoundationCHECKED_x64;"
        "PsFastXmlCHECKED_x64;"
        "PxPvdSDKCHECKED_x64;"
        "UtilsCHECKED"
    ">"
    "$<$<CONFIG:debug>:"
        "PhysX3CommonDEBUG_x64;"
        "PhysX3DEBUG_x64;"
        "PhysX3CookingDEBUG_x64;"
        "PhysX3CharacterKinematicDEBUG_x64;"
        "PhysX3ExtensionsDEBUG;"
        "PxTaskDEBUG_x64;"
        "PxFoundationDEBUG_x64;"
        "PsFastXmlDEBUG_x64;"
        "PxPvdSDKDEBUG_x64;"
        "UtilsDEBUG"
    ">"
    "$<$<CONFIG:profile>:"
        "PhysX3CommonPROFILE_x64;"
        "PhysX3PROFILE_x64;"
        "PhysX3CookingPROFILE_x64;"
        "PhysX3CharacterKinematicPROFILE_x64;"
        "PhysX3ExtensionsPROFILE;"
        "PxTaskPROFILE_x64;"
        "PxFoundationPROFILE_x64;"
        "PsFastXmlPROFILE_x64;"
        "PxPvdSDKPROFILE_x64;"
        "UtilsPROFILE"
    ">"
    "$<$<CONFIG:release>:"
        "PhysX3Common_x64;"
        "PhysX3_x64;"
        "PhysX3Cooking_x64;"
        "PhysX3CharacterKinematic_x64;"
        "PhysX3Extensions;"
        "PxTask_x64;"
        "PxFoundation_x64;"
        "PsFastXml_x64;"
        "PxPvdSDK_x64;"
        "Utils"
    ">"
    "Winmm;"
    "OpenGL32;"
    "glut32"
)
target_link_libraries(${PROJECT_NAME} PRIVATE "${ADDITIONAL_LIBRARY_DEPENDENCIES}")

target_link_directories(${PROJECT_NAME} PRIVATE
    "$ENV{PHYSXPATH}/PhysX_3.4/Common/lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/lib/vc14win64;"
    "$ENV{PHYSXPATH}/PxShared/lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/Graphics/lib/win64/glut;"
    "$ENV{PHYSXPATH}/PhysX_3.4/Lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysX-3.4/PhysX_3.4/Lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysX-3.4/PxShared/lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/PhysX-3.4/PhysX_3.4/Common/lib/vc14win64"
)

