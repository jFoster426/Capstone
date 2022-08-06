idf_component_register(SRCS "bt_st.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)