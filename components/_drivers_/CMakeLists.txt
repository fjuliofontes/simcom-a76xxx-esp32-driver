# include paths of this components.
set(COMPONENT_ADD_INCLUDEDIRS simcom/include
                              )

# source files of this components.
set(COMPONENT_SRCDIRS simcom/src
                      )

set(COMPONENT_REQUIRES _config_)

# add this components, this will define a CMake library target.
register_component()

# standard CMake function can be used to specify dependencies. ${COMPONENT_TARGET} is defined
# from esp-idf when you call register_component, by default it's idf_component_<folder_name>.
target_link_libraries(${COMPONENT_TARGET} PRIVATE idf::driver
                                                  idf::esp_timer)
