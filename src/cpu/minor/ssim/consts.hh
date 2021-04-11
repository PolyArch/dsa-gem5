#pragma once

typedef uint64_t addr_t;

#define SBDT uint64_t           //cgra datatype
#define SSWORD uint8_t          //dgra datatype
#define DATA_WIDTH sizeof(SBDT)
#define SCRATCH_SIZE (16384) //size in bytes -- 16KB
// #define SCRATCH_SIZE (32768) //size in bytes -- 16KB
#define SPU_NET_PACKET_SIZE 64
#define NUM_SCRATCH_BANKS 64

#define LSCRATCH_SIZE (1 << 20) //size in bytes -- 16KB
#define NUM_SCRATCH_BANKS 64
#define MAX_BANK_BUFFER_SIZE 64 // 8
// #define NUM_SPU_CORES 64 // for global address space

#define SB_TIMING

#define DEFAULT_FIFO_LEN 15
#define DEFAULT_IND_ROB_SIZE 8


//#define NUM_ACCEL 8
#define NUM_ACCEL_TOTAL (NUM_ACCEL+1)
#define SHARED_SP (NUM_ACCEL)
#define ACCEL_MASK ((1 << NUM_ACCEL) - 1)
#define SHARED_MASK (ACCEL_MASK+1)

#define SD_TRANSFERS_ALLOWED 22

#define MEM_WIDTH (64)
#define MEM_MASK ~(MEM_WIDTH-1)

#define SCR_WIDTH (64)
#define SCR_MASK ~(SCR_WIDTH-1)

#define PORT_WIDTH (64)
#define VP_LEN (64)

#define MAX_MEM_REQS (100)
#define CGRA_FIFO_LEN (32)

#define NUM_IN_PORTS  (32)
#define NUM_OUT_PORTS (32)

#define START_IND_PORTS (23)
// #define START_IND_PORTS (21)
#define STOP_IND_PORTS  (32)

#define NET_ADDR_PORT (21)
#define NET_VAL_PORT (22)

#define ATOMIC_ADDR_PORT (27)
#define ATOMIC_BYTES_PORT (28)
#define BYTES_PORT_DATA_WIDTH (2)
#define ATOMIC_ADDR_DATA_WIDTH (2)
#define MAX_ATOM_REQ_QUEUE_SIZE (64)

// #define NET_ADDR_PORT (25)
// #define NET_VAL_PORT (32)

//Convenience ports for these functions
// #define MEM_SCR_PORT (23)
// #define SCR_MEM_PORT (24)

#define REPEAT_FXPNT (3)

#define MAX_WAIT (1000) //max cycles to wait for forward progress

#define MEM_WR_STREAM (127)
#define CONFIG_STREAM (128)

#define NUM_GROUPS (6)

//bit std::vectors for sb_wait
#define WAIT_SCR_WR       1 //wait for just scratch
#define WAIT_CMP          2 //wait for everything to complete
#define WAIT_SCR_RD       4 //wait for all reads to complete (not impl)
#define WAIT_SCR_RD_Q     8 //wait for all reads to be de-queued (not impl)
#define WAIT_MEM_WR       16//wait for all writes to complete (not impl)
#define WAIT_SCR_ATOMIC   32//wait for all atomics to be done, delay the core
#define WAIT_SCR_WR_DF    64//wait for N remote writes to be done, delay the core
#define GLOBAL_WAIT       128//wait for all cores (threads) to be done
#define STREAM_WAIT       66//wait only for streams to be done


//fill modes
#define NO_FILL                0
#define POST_ZERO_FILL         1
#define PRE_ZERO_FILL          2
#define STRIDE_ZERO_FILL       3
#define STRIDE_DISCARD_FILL    4

//datatype encodings
#define T64 0
#define T32 1
#define T16 2
#define T08 3

#define NO_PADDING (~0ull)
