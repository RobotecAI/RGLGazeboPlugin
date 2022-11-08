// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
#define NO_MANGLING extern "C"
#else // NOT __cplusplus
#define NO_MANGLING
#endif

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
  #define RGL_VISIBLE __attribute__ ((dllexport))
 #else
  #define RGL_VISIBLE __declspec(dllexport)
 #endif // __GNUC__
#else
#define RGL_VISIBLE __attribute__ ((visibility("default")))
#if __GNUC__ >= 4
#define RGL_VISIBLE __attribute__ ((visibility("default")))
#else
#define RGL_VISIBLE
#endif
#endif // _WIN32 || __CYGWIN__

#define RGL_API NO_MANGLING RGL_VISIBLE
