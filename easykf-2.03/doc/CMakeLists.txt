FIND_PACKAGE(Doxygen)
IF(NOT DOXYGEN_FOUND)
    message("Doxygen not found, I will not generate/install the documentation")
ELSE()
   configure_file(Doxyfile.in Doxyfile)

   SET(DOXYGEN_INPUT ${CMAKE_BINARY_DIR}/doc/Doxyfile)
   SET(APIDOC_DIR ${CMAKE_BINARY_DIR}/doc CACHE PATH "API documentation directory")
   SET(DOXYGEN_OUTPUT ${APIDOC_DIR}/html/index.html)

   ADD_CUSTOM_COMMAND(
     OUTPUT ${DOXYGEN_OUTPUT}
     COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
     COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
     COMMAND ${CMAKE_COMMAND} -E echo "Done."
     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
     DEPENDS ${DOXYGEN_INPUT}
  )

  ADD_CUSTOM_TARGET(apidoc ALL DEPENDS ${DOXYGEN_OUTPUT})

  FILE(GLOB_RECURSE doc_files RELATIVE ${APIDOC_DIR}/html ${APIDOC_DIR}/html/*)
  FOREACH(file ${doc_files})
     GET_FILENAME_COMPONENT(file_prefix ${file} PATH)
     INSTALL(FILES ${APIDOC_DIR}/html/${file}
             DESTINATION share/doc/easykf/html/${file_prefix})
  ENDFOREACH(file)

ENDIF()

