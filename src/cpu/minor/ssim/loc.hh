#pragma once

enum LOC {
#define MACRO(x) x
#include "loc.def"
#undef MACRO
};

extern const char *LOC_NAME[LOC::TOTAL + 1];