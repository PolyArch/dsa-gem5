#pragma once

#include <string>
#include <vector>

#include "dsa/debug.h"
#include "dsa/arch/model.h"
#include "dsa/mapper/schedule.h"


namespace dsa {
namespace sim {

/*!
 * \brief The configuration wrapper of the loaded bitstream.
 */
struct BitstreamWrapper {

  BitstreamWrapper() {}

#define GATHER_PORTS(vp, res)                               \
  for (auto &elem : sched->ssdfg()->vp) {                   \
    res.emplace_back(&elem, sched->vecPortOf(&elem)); \
  }
  BitstreamWrapper(Schedule *sched_) : sched(sched_) {
    GATHER_PORTS(vins, ports[1]);
    GATHER_PORTS(vouts, ports[0]);
  }
#undef GATHER_PORTS

  /*!
   * \brief The information of a configuration port.
   */
  struct PortInfo {
    /*!
     * \brief The name of the port.
     */
    dsa::dfg::VectorPort *vp;
    /*!
     * \brief The port id of the port.
     */
    int port;

    PortInfo(dsa::dfg::VectorPort *v, int p) : vp(v), port(p) {}
  };

  /*!
   * \brief The schedule to be wrapped.
   */
  Schedule *sched{nullptr};
  /*!
   * \brief The port id's of output/input ports.
   */
  std::vector<PortInfo> ports[2];

  /*!
   * \brief Find if the port is actively configured.
   * 
   * \param is_input Input port or output.
   * \param port The port to be looked for.
   */
  PortInfo *FindPort(bool is_input, int port) {
    auto &pis = ports[is_input];
    auto cond = [port] (const PortInfo &pi) { return port == pi.port; };
    auto iter = std::find_if(pis.begin(), pis.end(), cond);
    return iter == pis.end() ? nullptr : &(*iter);
  }

  /*!
   * \brief Find the input port.
   */
  std::vector<PortInfo> &iports() { return ports[1]; }
  
  /*!
   * \brief Find the output port.
   */
  std::vector<PortInfo> &oports() { return ports[0]; }

  /*!
   * \brief The name of the string.
   * \param is_input The output/input we want to find.
   * \param port The id of the port.
   */
  std::string name(bool is_input, int port) {
    auto pi = FindPort(is_input, port);
    if (pi) {
      return pi->vp->name();
    }
    DSA_WARNING << port << " not in configuration use!";
    return "port: " + std::to_string(port);
  }

  SSDfg *dfg() {
    return sched ? sched->ssdfg() : nullptr;
  }
};

}
}
