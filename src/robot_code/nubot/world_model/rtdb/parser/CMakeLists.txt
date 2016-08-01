# src/libs/rtdb/parser

# Define where is the rtdb_user.h

set(AGENT_CURRENT nubot$ENV{AGENT})
SET(RTDB_USER_H_FILE   /home/${AGENT_CURRENT}/nubot_ws/src/nubot/world_model/include/nubot/rtdb/rtdb_user.h)
SET(RTDB_INI_FILE      /home/${AGENT_CURRENT}/nubot_ws/src/nubot/world_model/config/rtdb.ini )
SET(RTDB_Config_FILE   /home/${AGENT_CURRENT}/nubot_ws/src/nubot/world_model/config/rtdb.config )

include_directories(${world_model_includes})

# For XRTDB CFLAGS: Include all directories inside source dir AND in include_directories
SET( RTDB_INCLUDE_DIRS )
GET_PROPERTY(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
STRING(LENGTH "${CMAKE_SOURCE_DIR}" srcdir_len)	# get cmake source dir string length
FOREACH(dir ${dirs})
	STRING( LENGTH "${dir}" dir_len)
	IF( ${dir_len} GREATER ${srcdir_len} OR ${dir_len} EQUAL ${srcdir_len} )
		STRING(SUBSTRING ${dir} 0 ${srcdir_len} cmpare_str)
		IF( ${cmpare_str} STREQUAL ${CMAKE_SOURCE_DIR} )
			SET( RTDB_INCLUDE_DIRS "${RTDB_INCLUDE_DIRS} -I${dir}")
		ENDIF()
	ENDIF()
ENDFOREACH()
#MESSAGE( STATUS "out='${RTDB_INCLUDE_DIRS}'")



CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/rtdb_configuration.h.in
	${CMAKE_CURRENT_BINARY_DIR}/rtdb_configuration.h
)

SET( rtdbparser_SRC
	xrtdb.c
	rtdb_errors.c
	rtdb_functions.c
	rtdb_user_creator.c
	rtdb_ini_creator.c
	
	xrtdb.tab.c
)

include_directories(${CMAKE_CURRENT_BINARY_DIR} )

ADD_EXECUTABLE ( xrtdb ${rtdbparser_SRC} )
#SET_TARGET_PROPERTIES( xrtdb PROPERTIES LINKER_LANGUAGE C)
