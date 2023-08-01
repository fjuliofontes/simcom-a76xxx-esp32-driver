### # include paths of this components.
### set(COMPONENT_ADD_INCLUDEDIRS algorithm/include
###                               )
### 
### # source files of this components.
### set(COMPONENT_SRCDIRS algorithm/src
###                       )
### 
### set(COMPONENT_REQUIRES _common_
###                        _config_
###                        _drivers_
###                        )
### # add this components, this will define a CMake library target.
### register_component()
### 
### target_link_libraries(${COMPONENT_TARGET} PRIVATE idf::driver
###                                                   idf::esp_timer)