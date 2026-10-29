# Project-wide warning flags. Included after Dependencies.cmake, so they only
# reach our own targets and not the fetched third-party code.
if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8)
  add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN)
  if(PSIM_WERROR)
    add_compile_options(/WX)
  endif()
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  if(PSIM_WERROR)
    add_compile_options(-Werror)
  endif()
endif()
