cmake_minimum_required(VERSION 2.6)
PROJECT(boost-asio-examples)

# Usage:
#   cmake .. -DCMAKE_INCLUDE_PATH=/home/ott/exp/include -DCMAKE_LIBRARY_PATH=/home/ott/exp/lib
#       -DBOOST_INCLUDEDIR=/home/ott/exp/include -DBOOST_LIBRARYDIR=/home/ott/exp/lib
#

SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost 1.46.1 REQUIRED COMPONENTS system thread regex system-mt thread-mt regex-mt)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

SET(USED_LIBS ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

ADD_EXECUTABLE(chat_server chat_server.cpp)
TARGET_LINK_LIBRARIES(chat_server ${USED_LIBS}  pthread)

ADD_EXECUTABLE(chat_client chat_client.cpp)
TARGET_LINK_LIBRARIES(chat_client ${USED_LIBS}  pthread)

ADD_EXECUTABLE(daytime_server daytime_server.cpp)
TARGET_LINK_LIBRARIES(daytime_server ${USED_LIBS}  pthread)

ADD_EXECUTABLE(sync_daytime_server sync_daytime_server.cpp)
TARGET_LINK_LIBRARIES(sync_daytime_server ${USED_LIBS}  pthread)

ADD_EXECUTABLE(daytime_client daytime_client.cpp)
TARGET_LINK_LIBRARIES(daytime_client ${USED_LIBS}  pthread)
