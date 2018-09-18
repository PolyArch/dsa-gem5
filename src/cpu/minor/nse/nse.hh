#include <time.h>
#include <cstdint>
#include <iostream>
#include "../ssim/ssim.hh"

class nse_t 
{
  friend class ticker_t;


public:
  // NSE Interface
  // nse_t(Minor::LSQ* lsq);
  // Minor::LSQ _lsq;
  nse_t(ssim_t* ssim);

  ssim_t _ssim; // underscore means private members?

};

