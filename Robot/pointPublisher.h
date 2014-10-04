#ifndef _POINTPUBLISHER_H_
#define _POINTPUBLISHER_H_

#include <list>
#include "tp1/ccoord.h"

// Publishes a list of points to the cmd_pos topic 
void publishPoints(std::list<tp1::ccoord> points);

#endif

