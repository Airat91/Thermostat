set(SOURCES ${SOURCES}
   ${SOURCES_PREFIX}/croutine.c
   ${SOURCES_PREFIX}/event_groups.c
   ${SOURCES_PREFIX}/CMakeLists.txt
   ${SOURCES_PREFIX}/list.c
   ${SOURCES_PREFIX}/queue.c
   ${SOURCES_PREFIX}/stream_buffer.c
   ${SOURCES_PREFIX}/tasks.c
   ${SOURCES_PREFIX}/timers.c
   ${SOURCES_PREFIX}/portable/heap_4.c
   ${SOURCES_PREFIX}/portable/port.c
   ${SOURCES_PREFIX}/cmsis_os/cmsis_os.c
   PARENT_SCOPE)

