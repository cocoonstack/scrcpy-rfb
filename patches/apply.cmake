# Applied by FetchContent's patch step with the libvncserver source tree as
# the working directory. Idempotent: a patch that already applies in reverse
# is skipped so reconfigures don't fail.
file(GLOB patches "${CMAKE_CURRENT_LIST_DIR}/*.patch")
list(SORT patches)
foreach(patch IN LISTS patches)
  execute_process(
    COMMAND git apply --reverse --check "${patch}"
    RESULT_VARIABLE already_applied
    OUTPUT_QUIET ERROR_QUIET
  )
  if(already_applied EQUAL 0)
    continue()
  endif()
  execute_process(
    COMMAND git apply --whitespace=nowarn "${patch}"
    RESULT_VARIABLE result
  )
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "failed to apply ${patch}")
  endif()
endforeach()
