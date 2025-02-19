# cmake/modules/FindPortAudio.cmake
find_path(PORTAUDIO_INCLUDE_DIR
    NAMES portaudio.h
    PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
    /sw/include
)

find_library(PORTAUDIO_LIBRARY
    NAMES portaudio
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
)

set(PORTAUDIO_LIBRARIES ${PORTAUDIO_LIBRARY})
set(PORTAUDIO_INCLUDE_DIRS ${PORTAUDIO_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PortAudio DEFAULT_MSG PORTAUDIO_LIBRARY PORTAUDIO_INCLUDE_DIR)

mark_as_advanced(PORTAUDIO_INCLUDE_DIR PORTAUDIO_LIBRARY)
