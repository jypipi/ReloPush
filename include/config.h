// Jeeho Ahn
// CMake will automatically fill config.h as formatted below
#ifndef CONFIG_H_IN
#define CONFIG_H_IN

#define PROJECT_NAME ReloPush-Rebuild //project name will be written here

#define STR(x) #x //for stringizing

//cmake source dir will be written here
// Windows path:
// #define CMAKE_SOURCE_DIR STR(/mnt/c/Users/Jeffrey Chen/Desktop/NL-2-Actions/ReloPush)

// Docker path:
// #define CMAKE_SOURCE_DIR STR(/ws/NL-2-Actions/ReloPush)

// Lab Desktop path (Linux 20.04):
#define CMAKE_SOURCE_DIR STR(/home/jeffrey/NL-2-Actions/ReloPush)

#endif // CONFIG_H_IN
