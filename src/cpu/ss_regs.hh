#ifndef __SS_REGS_HH__
#define __SS_REGS_HH__

// Don't forget to add to SSRegNames array
// ... someday I'll do the #macro trick to automate
// Yes! The magic is applied!

enum SSRegIdx {
#define MACRO(x) SS_##x,
#include "ss_reg.def"
#undef MACRO
};

const char* const SSRegNames[] = {
#define MACRO(x) #x,
#include "ss_reg.def"
#undef MACRO
};

enum SSCmdIdx {
#define MACRO(x) SS_##x,
#include "ss_cmd.def"
#undef MACRO
};


const char* const SSCmdNames[] = {
#define MACRO(x) #x,
#include "ss_cmd.def"
#undef MACRO
};

#endif
