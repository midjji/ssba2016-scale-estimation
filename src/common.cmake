# common defines
set(line "<================================================================================>")
macro(PrintList list prepend)
    foreach(el ${list})
        message("${prepend} -- ${el}")
    endforeach()
endmacro()
macro(add_subdirectory_if_exists dir)
        if(EXISTS "${dir}/CMakeLists.txt")
                add_subdirectory(${dir})
                else()
                message("subdirectory not found? ${dir}")
        endif()
endmacro()

# Build configuration
macro(BuildConfig)
# Change the default build type from Debug to Release

# The CACHE STRING logic here and elsewhere is needed to force CMake
# to pay attention to the value of these variables.(for override)
    if(NOT CMAKE_BUILD_TYPE)
        MESSAGE("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
        set(CMAKE_BUILD_TYPE Release CACHE STRING
            "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."  FORCE)
    else()
        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            MESSAGE("\n${line}")
            MESSAGE("\n-- Build type: Debug. Performance will be terrible!")
            MESSAGE("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
            MESSAGE("-- Add -DCMAKE_BUILD_TYPE=RelWithDebInfo to the CMake command line to get an faster build with symbols(-g).")
            MESSAGE("\n${line}")
        endif()
    endif()

# compiler specific:
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(languageused "-std=c++11")
        add_definitions(${languageused})

    endif()



# WINDOWS EXTRA DEFINES
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
add_definitions(-D__STDC_LIMIT_MACROS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP8")
endif()

endmacro()



macro(WarningConfig)
    #todo add compiler specific...
    option(WExtrawarnings "Extra warnings" ON)
    option(WError "Warnings are errors" ON)
    if(CMAKE_COMPILER_IS_GNUCXX)  # GCC
        if(WError)
            set(warningoptions "${warningoptions}-Werror ")
        endif()


        if(WExtrawarnings)# GCC is not strict enough by default, so enable most of the warnings.

            set(warn "${warn} -Wall")
            set(warn "${warn} -Wextra")
            set(warn "${warn} -Wno-unknown-pragmas")
            set(warn "${warn} -Wno-sign-compare")
            set(warn "${warn} -Wno-unused-parameter")
            set(warn "${warn} -Wno-missing-field-initializers")
            set(warn "${warn} -Wno-unused")
            set(warn "${warn} -Wno-unused-function")
            set(warn "${warn} -Wno-unused-label")
            set(warn "${warn} -Wno-unused-parameter")
            set(warn "${warn} -Wno-unused-value")
            set(warn "${warn} -Wno-unused-variable")
            set(warn "${warn} -Wno-unused-but-set-parameter")
            set(warn "${warn} -Wno-unused-but-set-variable")
            set(warningoptions "${warningoptions}${warn}")
            # disable annoying ones ...
            list(APPEND warningoptions )
        endif()

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${warningoptions}")
        # also no "and" or "or" ! for msvc
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-operator-names")
    endif()
endmacro()


macro(OptimizationConfig)
    if(CMAKE_COMPILER_IS_GNUCXX)  # GCC
        set(CMAKE_CXX_FLAGS_RELEASE "-march=native -mtune=native -O3 -Ofast -DNDEBUG")
    else()
        message("TODO: fix opt options on this compiler")
    endif()
endmacro()












if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()
function(cmessage)
if(ARGV STREQUAL "")
_message("")
else()
  list(GET ARGV 0 MessageType)
  if(MessageType STREQUAL FATAL_ERROR OR MessageType STREQUAL SEND_ERROR)
    list(REMOVE_AT ARGV 0)
    _message(${MessageType} "${BoldRed}${ARGV}${ColourReset}")
  elseif(MessageType STREQUAL WARNING)
    list(REMOVE_AT ARGV 0)
    _message(${MessageType} "${BoldYellow}${ARGV}${ColourReset}")
  elseif(MessageType STREQUAL AUTHOR_WARNING)
    list(REMOVE_AT ARGV 0)
    _message(${MessageType} "${BoldCyan}${ARGV}${ColourReset}")
  elseif(MessageType STREQUAL STATUS)
    list(REMOVE_AT ARGV 0)
    _message(${MessageType} "${Green}${ARGV}${ColourReset}")
  elseif(MessageType STREQUAL silentwarning)
    list(REMOVE_AT ARGV 0)
    _message("${BoldRed}${ARGV}${ColourReset}")
  else()
    _message("${ARGV}")
  endif()
endif()
endfunction()




# find opencv macro which fixes the damned opencv bad names and ensures its convenient to link to it even if it fails to find it automatically
macro(Add_Package_OpenCV3)
    find_package( OpenCV 3 REQUIRED )
    if(OpenCV_FOUND)
        message("-- Found OpenCV version ${OpenCV_VERSION}")
        set(OpenCV_LIBRARIES ${OpenCV_LIBS})# fixes the name

    # pulls in cuda options, mark them as advanced...
    mark_as_advanced(FORCE CUDA_BUILD_CUBIN)
    mark_as_advanced(FORCE CUDA_BUILD_EMULATION)
    mark_as_advanced(FORCE CUDA_HOST_COMPILER)
    mark_as_advanced(FORCE CUDA_SDK_ROOT_DIR)
    mark_as_advanced(FORCE CUDA_SEPARABLE_COMPILATION)
    mark_as_advanced(FORCE CUDA_TOOLKIT_ROOT_DIR)
    mark_as_advanced(FORCE CUDA_VERBOSE_BUILD)
    mark_as_advanced(FORCE CUDA_USE_STATIC_CUDA_RUNTIME)
    mark_as_advanced(FORCE CUDA_dl_LIBRARY)
    mark_as_advanced(FORCE CUDA_rt_LIBRARY)

    message("-- Include directories: ${OpenCV_INCLUDE_DIRS}")
    message("-- OpenCV_Libraries:  ")
    PrintList("${OpenCV_LIBRARIES}" "    ")
    list(APPEND LIBS ${OpenCV_LIBRARIES})
    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
endif()
endmacro()


macro(Find_Package_OSG)

    find_package(OpenSceneGraph 3.2 REQUIRED osgDB osgUtil osgViewer osgGA osgWidget )
    if(OPENSCENEGRAPH_FOUND)
        message("Found OSG Version: ?")        

            mark_as_advanced(FORCE OPENTHREADS_INCLUDE_DIR)
            mark_as_advanced(FORCE OPENTHREADS_LIBRARY)
            mark_as_advanced(FORCE OPENTHREADS_LIBRARY_DEBUG)
            mark_as_advanced(FORCE OSGDB_INCLUDE_DIR)
            mark_as_advanced(FORCE OSGDB_LIBRARY     )
            mark_as_advanced(FORCE OSGDB_LIBRARY_DEBUG)
            mark_as_advanced(FORCE OSGGA_INCLUDE_DIR   )
            mark_as_advanced(FORCE OSGGA_LIBRARY        )
            mark_as_advanced(FORCE OSGGA_LIBRARY_DEBUG   )
            mark_as_advanced(FORCE OSGUTIL_INCLUDE_DIR )
            mark_as_advanced(FORCE OSGUTIL_LIBRARY      )
            mark_as_advanced(FORCE OSGUTIL_LIBRARY_DEBUG )
            mark_as_advanced(FORCE OSGVIEWER_INCLUDE_DIR  )
            mark_as_advanced(FORCE OSGVIEWER_LIBRARY       )
            mark_as_advanced(FORCE OSGVIEWER_LIBRARY_DEBUG  )
            mark_as_advanced(FORCE OSGWIDGET_INCLUDE_DIR  )
            mark_as_advanced(FORCE OSGWIDGET_LIBRARY )
            mark_as_advanced(FORCE OSGWIDGET_LIBRARY_DEBUG   )
            mark_as_advanced(FORCE OSG_INCLUDE_DIR  )
            mark_as_advanced(FORCE OSG_LIBRARY         )
            mark_as_advanced(FORCE OSG_LIBRARY_DEBUG)

    else()
    message("OSG not found")
    set(OPENSCENEGRAPH_INCLUDE_DIR "OFF" CACHE FILEPATH "Filepath to OSG include folder")
    set(tofind "osgDB;osgUtil;osgViewer;osgGA;osgWidget;osg;OpenThreads")
        set(OPENSCENEGRAPH_LIBRARIES "")
        foreach(pth ${tofind})
            set(OPENSCENEGRAPH_libpath_${pth} "OFF" CACHE FILEPATH "Filepath to ${pth}")
            if(EXISTS ${OPENSCENEGRAPH_libpath_${pth}})
                list(APPEND OPENSCENEGRAPH_LIBRARIES ${OPENSCENEGRAPH_libpath_${pth}})
            endif()
        endforeach()
    endif()
    message("OpenSceneGraph include directories: ${OPENSCENEGRAPH_INCLUDE_DIR}")
    message("OpenSceneGraph libraries: ")
    PrintList("${OPENSCENEGRAPH_LIBRARIES}" "    ")

endmacro()


macro(Add_Package_Ceres)
    find_package(Ceres REQUIRED)
    if(Ceres_FOUND)
        message("Found Ceres Version: ${CERES_VERSION}")    
    endif()


    message("Ceres include directories: ${CERES_INCLUDE_DIRS}")
    message("Ceres libraries: ")
    PrintList("${CERES_LIBRARIES}" "    ")
    list(APPEND LIBS ${CERES_LIBRARIES})
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
endmacro()







