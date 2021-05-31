#pragma once

#include <sys/time.h>
#include <cstdint>
#include <vector>

#include "cpu/static_inst.hh"


class ssim_t;
class accel_t;
class base_stream_t;

namespace dsa {
namespace stat {

/*!
 * \brief Statistics of the host controller.
 */
struct Host {
  /*!
   * \brief The time of starting simulation.
   */
  timeval sim_time[2];
  /*!
   * \brief The starting CPU time stamp.
   */
  int64_t sim_cycle[2];
  /*!
   * \brief The number of all the instructions issued in ROI.
   */
  int64_t insts_issued{0};
  /*!
   * \brief The number of all the instructions discarded in ROI.
   *        I guess this is because of incorrect branch prediction?
   */
  int64_t insts_discarded{0};
  /*!
   * \brief The number control instruction issued.
   */
  int64_t ctrl_instructions{0};
  /*!
   * \brief The number of intrinsics issued.
   */
  int64_t ctrl_intrinsics{0};
  /*!
   * \brief The simulation host to be counted.
   */
  ssim_t &parent;
  /*!
   * \brief Construct a statistics for the given simulation host.
   * \param parent The simulation host to be counted.
   */
  Host(ssim_t &parent);
  /*!
   * \brief Count an instruction executed in the host CPU.
   * \param inst The instruction to be counted.
   */
  void countHostInst(StaticInstPtr inst, bool discard);
  /*!
   * \brief The time collapse for simulation in seconds.
   */
  double timeElapsed();
  /*!
   * \brief The CPU cycle collapse for simulation.
   */
  double cycleElapsed();

 private:
  /*!
   * \brief If we are in ROI.
   */
  bool roi_{false};

 public:
  bool roi(); 
  bool roi(bool val);
};

/*!
 * \brief Statistics of the accelerator.
 */
struct Accelerator {
  /*!
   * \brief The number of commands issued to coordinate data streams.
   */
  int64_t commands_issued{0};
  /*!
   * \brief The number of compute instance on the spatial architecture.
   */
  int64_t instance_cnt{0};
  /*!
   * \brief We to blame for this cycle of iteration.
   */
  enum Blame {
    #define MACRO(x) x,
    #include "./blame.def"
    #undef MACRO
  };
  static const char *BlameStr[Blame::UNKNOWN + 1];
  Blame blame{Blame::UNKNOWN};
  int64_t blame_count[Blame::UNKNOWN + 1];
  struct Traffic {
    /*!
     * \brief The number of reuqests that caused this traffic
     */
    int num_requests{0};
    /*!
     * \brief Data traffic in bytes.
     */
    int64_t traffic{0};
  };
  /*!
   * \brief The memory traffic between each memory and the spatial architecture.
   *        0 is the main memory, and each following is the scratch pad.
   */
  Traffic traffic[2][LOC::TOTAL];
  /*!
   * \brief The accelerator to which this instance belongs.
   */
  accel_t &parent;

  Accelerator(accel_t &t);

  /*!
   * \brief Count the data traffic made by the stream.
   * \param is_input If this traffic is from an input stream.
   *                 If -1, it is both an input and output stream, typically for recurrence.
   * \param unit The unit this data is destinated or sourced from.
   * \param delta The data traffic made this time.
   */
  void countDataTraffic(int is_input, LOC unit, int delta);

  /*!
   * \brief A convinience wrapper for access ROI in host statistics.
   */
  bool roi();

  /*!
   * \brief Cycle breakdown.
   */
  void blameCycle();
};

}
}