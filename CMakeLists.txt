cmake_minimum_required(VERSION 3.15...3.30)

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_SOURCE_DIR}/bin>)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_SOURCE_DIR}/bin>)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_SOURCE_DIR}/bin>)

project(SimPhy VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(
    GLOB_RECURSE
	SIMPHY_SOURCES
	CMAKE_CONFIGURE_DEPENDS
	skeleton/*.cpp
)
    
add_executable(
    ${PROJECT_NAME}
    skeleton/main.cpp
    ${SIMPHY_SOURCES}
)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    skeleton/
)

get_property(
    MSVC_RUNTIME_LIBRARY_DEFAULT
    TARGET
    ${PROJECT_NAME}
    PROPERTY
    MSVC_RUNTIME_LIBRARY
)
string(
    CONCAT
    "MSVC_RUNTIME_LIBRARY_STR"
    $<$<CONFIG:Debug>:
        MultiThreadedDebug
    >
    $<$<CONFIG:Release>:
        MultiThreaded
    >
    $<$<NOT:$<OR:$<CONFIG:Debug>,$<CONFIG:Release>>>:${MSVC_RUNTIME_LIBRARY_DEFAULT}>
)
set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
    MSVC_RUNTIME_LIBRARY
    ${MSVC_RUNTIME_LIBRARY_STR}
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    "$ENV{PHYSXPATH}/PhysX_3.4/Include;"
    "$ENV{PHYSXPATH}/PxShared/include;"
    "$ENV{PHYSXPATH}/PxShared/src/foundation/include;"
    "$ENV{PHYSXPATH}/PxShared/src/fastxml/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Graphics/include/win32/GL;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/PhysX-3.4/PhysX_3.4/Include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/PhysX-3.4/PxShared/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/PhysX-3.4/PxShared/src/foundation/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/PhysX-3.4/PxShared/src/fastxml/include"
)

target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE
    "$<$<CONFIG:Debug>:"
        "_DEBUG;"
        "PX_DEBUG=1;"
        "PX_CHECKED=1;"
        "PX_NVTX=1;"
        "PX_SUPPORT_PVD=1"
    ">"
    "$<$<CONFIG:Release>:"
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

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:Debug>:
            /Od;
            /RTCu
        >
        $<$<CONFIG:Release>:
            /Ox
        >
        /W4;
        /Zi;
    )
    target_link_options(
        ${PROJECT_NAME}
        PRIVATE
        $<$<CONFIG:Release>:
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

set(
    ADDITIONAL_LIBRARY_DEPENDENCIES
    "$<$<CONFIG:Debug>:"
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
    "$<$<CONFIG:Release>:"
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
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
    "${ADDITIONAL_LIBRARY_DEPENDENCIES}"
)

target_link_directories(${PROJECT_NAME} PRIVATE
    "$ENV{PHYSXPATH}/PhysX_3.4/Common/lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/lib/vc14win64;"
    "$ENV{PHYSXPATH}/PxShared/lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Graphics/lib/win64/glut;"
    "$ENV{PHYSXPATH}/PhysX_3.4/Lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/PhysX-3.4/PhysX_3.4/Lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/PhysX-3.4/PxShared/lib/vc14win64;"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/PhysX-3.4/PhysX_3.4/Common/lib/vc14win64"
)