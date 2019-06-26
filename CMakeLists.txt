cmake_minimum_required(VERSION 2.8.12)

#####################################
# SCOTSv2 optimal controller determinizer
#####################################
#Find the source files
file (GLOB_RECURSE _SourceFiles "./ext/optdet/*.cc")
#Copy the source files
file (COPY ${_SourceFiles}
	DESTINATION "${CMAKE_BINARY_DIR}/ext/optdet/"
	NO_SOURCE_PERMISSIONS)
#Add the LibraryLink library source folder
add_subdirectory(ext/optdet/)

#Find the source files of the splitter
file (GLOB_RECURSE _SourceFiles2 "./src/*.cc")
#Copy the source files
file (COPY ${_SourceFiles2}
	DESTINATION "${CMAKE_BINARY_DIR}/src/"
	NO_SOURCE_PERMISSIONS)
#Add the LibraryLink library source folder
add_subdirectory(src/)

#Find the template files of the wrapper
file (GLOB_RECURSE _SourceFiles3 "./src/template.vhd")
#Copy the source files
file (COPY ${_SourceFiles3}
	DESTINATION "${CMAKE_BINARY_DIR}/src/templates/")

#Find the target file of abc
file (GLOB_RECURSE _TargeFiles "./ext/abc-master/abc")
#Copy the target file
file (COPY ${_TargeFiles}
	DESTINATION "${CMAKE_BINARY_DIR}/ext/abc/")
