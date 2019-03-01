//
// Timer that uses clock_gettime()
//
// Based on timer.hpp from Pinocchio
//

#include <sys/time.h>
#include <iostream>
#include <stack>

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth) 

/* Return the time spent in secs. */
inline double operator-(const struct timespec & t1,const struct timespec & t0)
{
   /* TODO: double check the double conversion from long (on 64x). */
   return double(t1.tv_sec - t0.tv_sec)+1e-9*double(t1.tv_nsec - t0.tv_nsec);
}

struct TicToc
{
   enum Unit { S = 1, MS = 1000, US = 1000000, NS = 1000000000 };
   Unit DEFAULT_UNIT;
   static std::string unitName(Unit u)
   {
      switch(u) { case S: return "s"; case MS: return "ms"; case US: return "us"; case NS: return "ns"; }
      return "";
   }

   std::stack<struct timespec> stack;
   mutable struct timespec t0;

   TicToc( Unit def = MS ) : DEFAULT_UNIT(def) {}

   inline void tic() {
      stack.push(t0);
      clock_gettime(CLOCK_MONOTONIC,&(stack.top()));
   }

   inline double toc(const Unit factor)
   {
      clock_gettime(CLOCK_MONOTONIC,&t0);
      double dt = (t0-stack.top())*factor;
      stack.pop();
      return dt;
   }
   inline void toc(std::ostream & os, double SMOOTH=1)
   {
      os << toc(DEFAULT_UNIT)/SMOOTH << " " << unitName(DEFAULT_UNIT) << std::endl;
   }
};

