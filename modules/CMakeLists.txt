# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
# ------------------------------------------------------------------------------

project(all_modules LANGUAGES CXX)

# ----------------------------------------------------
# Process dir by dir, solving build dependencies
#
# ----------------------------------------------------
set(VISITED_DIRS "")

function(process_module_dir dir)
	mola_message_verbose("Processing: ${dir}")
	list(APPEND VISITED_DIRS ${dir})

	set(VAR_NAME "BUILD_MODULE_${dir}")
	set(${VAR_NAME} ON CACHE BOOL "Build MOLA module: ${dir}")
	#mark_as_advanced(FORCE ${VAR_NAME})

  # If enabled in cmake options, and not already included from the root script:
	if (${VAR_NAME} AND NOT TARGET "${dir}")
		# First, process dependencies:

		set(meta_file "${CMAKE_CURRENT_LIST_DIR}/${dir}/mola-module.yml")
		if (EXISTS ${meta_file})
			file(READ ${meta_file} contents)
			string(REGEX REPLACE ";" "\\\\;" contents "${contents}")
			string(REGEX REPLACE "\n" ";" contents "${contents}")
			string(REGEX REPLACE "build-dep:([.^;]*)" "\\1" build_deps "${contents}")

			mola_message_verbose("${dir} has build_deps ==> ${build_deps}")
			foreach(d ${build_deps})
				string(REGEX REPLACE "(build-dep:)" "" d "${d}")
				string(STRIP "${d}" d)
				if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${d}")
					mola_message_verbose("  Processing dep: ${d}")
					# visit, if not done already:
					list(APPEND VISITED_DIRS )
					list(FIND VISITED_DIRS ${d} idx)
					if (NOT ${idx} GREATER -1)
						process_module_dir("${d}")
					endif()
				endif()
			endforeach()
		endif()

		# Now, process the module:
		add_subdirectory(${dir})
	endif()
endfunction()



# mola-common: already included from top-level cmake script

# Enumerate all subdirs and process them:
list_subdirectories(LIST_MODULE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
mola_message_verbose("LIST_MODULE_DIRS: ${LIST_MODULE_DIRS}")

foreach(dir ${LIST_MODULE_DIRS})
	process_module_dir(${dir})
endforeach(dir)
