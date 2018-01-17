#ifndef __SD_REGS_HH__
#define __SD_REGS_HH__

//Don't forget to add to SDRegNames array
//... someday I'll do the #macro trick to automate -- don't judge me
enum SDRegIdx {
   SD_MEM_ADDR, SD_INDEX_ADDR, SD_ACCESS_SIZE, SD_STRIDE, SD_SCRATCH_ADDR,
   SD_NUM_STRIDES, SD_NUM_ELEM, SD_CFG_SIZE,
   SD_NUM_BYTES, SD_CONSTANT, SD_IN_PORT, SD_OUT_PORT,
   SD_IND_PORT, SD_IND_TYPE, SD_WAIT_MASK, SD_SHIFT_BYTES, SD_GARBAGE, SD_GARB_ELEM, 
   SD_NUM_ELEM2, SD_CONSTANT2, SD_FLAGS, SD_CONTEXT, SD_REPEAT, SD_REPEAT_STRETCH,
   SD_NUM_REG
};

const char* const SDRegNames[] = {
   "MEM_ADDR", "INDEX_ADDR", "ACCESS_SIZE", "STRIDE", "SCRATCH_ADDR",
   "NUM_STRIDES", "NUM_ELEM", "CFG_SIZE",
   "NUM_BYTES", "CONSTANT", "IN_PORT", "OUT_PORT",
   "IND_PORT", "IND_TYPE", "WAIT_MASK", "SHIFT_BYTES", "GARBAGE", "SD_GARB_ELEM", 
   "SD_NUM_ELEM2", "SD_CONSTANT2", "SD_FLAGS", "SD_CONTEXT", "SD_REPEAT", "SD_REPEAT_STRETCH",
   "NUM_REG"
  };

enum SDCmdIdx {
    SB_BEGIN_ROI,
    SB_END_ROI,
    SB_STATS,
    SB_CFG,
    SB_CFG_PORT,
    SB_CTX,
    SB_FILL_MODE,
    SB_MEM_SCR,
    SB_MEM_PRT,
    SB_SCR_PRT,
    SB_SCR_MEM,
    SB_PRT_SCR,
    SB_PRT_MEM,
    SB_PRT_PRT,
    SB_IND_PRT,
    SB_PRT_IND,
    SB_CNS_PRT,
    SB_SET_ITERS,
    SB_GARB,
    SB_WAIT,
    SB_NUM_CMDS
};


const char* const SDCmdNames[] = {
    "SB_BEGIN_ROI",
    "SB_END_ROI",
    "SB_STATS", 
    "SB_CFG",
    "SB_CFG_PORT",
    "SB_CTX",
    "SB_FILL_MODE",
    "SB_MEM_SCR",
    "SB_MEM_PRT",
    "SB_SCR_PRT",
    "SB_SCR_MEM",
    "SB_PRT_SCR",
    "SB_PRT_MEM",
    "SB_PRT_PRT",
    "SB_IND_PRT",
    "SB_PRT_IND",
    "SB_CNS_PRT",
    "SB_SET_ITERS",
    "SB_GARB",
    "SB_WAIT",
    "SB_NUM_CMDS" };

#endif
