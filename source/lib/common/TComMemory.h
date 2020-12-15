#pragma once

#if defined(_WIN32)
// clang-format off
#include <windows.h>
#include <psapi.h>
// clang-format on

/**
 * Return the peak physical memory usage in Bytes for current process
 */
static size_t getPeakMemory() {
  PROCESS_MEMORY_COUNTERS pmc;
  GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
  return (size_t)pmc.PeakWorkingSetSize;
}

/**
 * Return the current physical memory usage in Bytes for current process
 */
static size_t getCurrentMemory() {
  PROCESS_MEMORY_COUNTERS pmc;
  GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
  return (size_t)pmc.WorkingSetSize;
}

#elif defined(__linux__) || defined(__linux) || defined(linux) || defined(_gnu_linux__)
#include <stdio.h>

static int64_t parseLine(char* line) {
  // This assumes that a digit will be found and the line ends in " Kb".
  int i = strlen(line);
  const char* p = line;
  while (*p < '0' || *p > '9')
    p++;
  line[i - 3] = '\0';
  int64_t result = atol(p);
  return result;
}

/**
 * Return the peak physical memory usage in Bytes for current process
 */
static size_t getCurrentMemory() {
  FILE* file = fopen("/proc/self/status", "r");
  size_t result = 0;
  char line[128];

  if (file != NULL) {
    while (fgets(line, 128, file) != NULL) {
      if (strncmp(line, "VmSize:", 7) == 0) {
        result = parseLine(line);
        break;
      }
    }
    fclose(file);
  }
  return (size_t)(result * 1024);
}

static size_t getPeakMemory() {
  FILE* file = fopen("/proc/self/status", "r");
  size_t result = 0;
  char line[128];

  if (file != NULL) {
    while (fgets(line, 128, file) != NULL) {
      if (strncmp(line, "VmPeak:", 7) == 0) {
        result = parseLine(line);
        break;
      }
    }
    fclose(file);
  }
  return (size_t)(result * 1024);
}

#else
/**
 * Return the peak physical memory usage in Bytes for current process
 */
static size_t getPeakMemory() {
  return (size_t)0;
}

/**
 * Return the current physical memory usage in Bytes for current process
 */
static size_t getCurrentMemory() {
  return (size_t)0;
}

#endif
