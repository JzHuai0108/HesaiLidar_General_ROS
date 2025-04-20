/******************************************************************************
 * Copyright 2018 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <cstdio>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <ctime>

inline void get_current_time(int& hour, int& min, int& sec, int& millis) {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t tt = system_clock::to_time_t(now);
    struct tm *ptm = localtime(&tt);

    hour = ptm->tm_hour;
    min = ptm->tm_min;
    sec = ptm->tm_sec;
    millis = static_cast<int>(ms.count());
}

class TranceFunc {
public:
    TranceFunc(const char* file, const char* func)
        : m_cFile(file), m_cFunc(func) {
        int hour, min, sec, millis;
        get_current_time(hour, min, sec, millis);
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10zu ->[File:%s Function:%s ]\n",
               hour, min, sec, millis, getpid(),
               std::hash<std::thread::id>()(std::this_thread::get_id()),
               m_cFile, m_cFunc);
    }

    ~TranceFunc() {
        int hour, min, sec, millis;
        get_current_time(hour, min, sec, millis);
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10zu <-[File:%s Function:%s ]\n",
               hour, min, sec, millis, getpid(),
               std::hash<std::thread::id>()(std::this_thread::get_id()),
               m_cFile, m_cFunc);
    }

    const char* m_cFile;
    const char* m_cFunc;
};

#define LOG_D(format, ...) {\
    int hour, min, sec, millis;\
    get_current_time(hour, min, sec, millis);\
    printf("[D] %02d:%02d:%02d.%03d pid:%d tid:%10zu File:%s Function:%s Line:%d " format "\n",\
           hour, min, sec, millis, getpid(),\
           std::hash<std::thread::id>()(std::this_thread::get_id()),\
           __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_E(format, ...) {\
    int hour, min, sec, millis;\
    get_current_time(hour, min, sec, millis);\
    printf("[E] %02d:%02d:%02d.%03d pid:%d tid:%10zu File:%s Function:%s Line:%d " format "\n",\
           hour, min, sec, millis, getpid(),\
           std::hash<std::thread::id>()(std::this_thread::get_id()),\
           __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_FUNC() TranceFunc tf(__FILE__, __FUNCTION__)