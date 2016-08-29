/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#ifndef TRASHNODE_H
#define TRASHNODE_H

#include "nodes/tweval.h"


// TODO not use define
#if 1
#define Trashnode vrptc::nodes::Tweval
#else


namespace vrptc {
namespace nodes {

class Tweval;

class Trashnode : public Tweval {
};


}
}

#endif

#endif
