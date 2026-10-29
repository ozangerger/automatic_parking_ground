# AddressSanitizer + UndefinedBehaviorSanitizer for the project's own targets.
# Sanitizer runs count on Linux only (PLAN.md section 11).
if(PSIM_SANITIZE)
  if(MSVC)
    message(FATAL_ERROR "PSIM_SANITIZE is not supported with MSVC")
  endif()
  set(psim_sanitizer_flags -fsanitize=address,undefined -fno-sanitize-recover=undefined
                           -fno-omit-frame-pointer)
  add_compile_options(${psim_sanitizer_flags})
  add_link_options(${psim_sanitizer_flags})
endif()
