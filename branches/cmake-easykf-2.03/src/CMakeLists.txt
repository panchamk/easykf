# Generate the list of source files
# using GLOB does not RECURSE
file(
	GLOB
	source_files
	*
)

#set(LIBRARY_OUTPUT_PATH lib/${CMAKE_BUILD_TYPE})

# Define our library 
add_library(
	easykf
	SHARED
	${source_files}
)

