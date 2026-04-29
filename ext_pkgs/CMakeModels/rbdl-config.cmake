set(RBDL_INSTALL_PREFIX "/opt/rbdl")
find_path (RBDL_INCLUDE_DIR rbdl/rbdl.h
  PATHS
  ${RBDL_INSTALL_PREFIX}/include
  NO_DEFAULT_PATH
)
find_library(RBDL_LIBRARY
    NAMES rbdl
    PATHS "${RBDL_INSTALL_PREFIX}/lib"
    NO_DEFAULT_PATH
)

# URDF READER
find_path(RBDL_URDFREADER_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
  PATHS
  ${RBDL_INSTALL_PREFIX}/include
  NO_DEFAULT_PATH
)

find_library(RBDL_URDFREADER_LIBRARY rbdl_urdfreader
  PATHS
  ${RBDL_INSTALL_PREFIX}/lib
  NO_DEFAULT_PATH
)

if(RBDL_LIBRARY AND EXISTS "${RBDL_INCLUDE_DIR}/rbdl/rbdl.h")
  set(RBDL_FOUND TRUE)
else()
  set(RBDL_FOUND FALSE)
endif()
