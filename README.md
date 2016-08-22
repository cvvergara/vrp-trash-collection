VRP Garbage Collection
============================================

The Capacitated, Multitrip with Time Windows Vehicle Routing Problem of Garbage Collection

Deposit: Starting and Ending sites from where a vehicle departs.
Containers: Object used to hold garbage.
Dump: Sites where the garbage is delivered.
Vehicle: Garbage collector.

A vehicle's route:

Deposit -> Containers -> Dump -> [Containers -> Dump] -> Deposit

Restrictions:
* Vehicle:
  * Time window:
    * Driver shift start.
    * Driver's shift end.
  * One dimensional ``Capacity``.
  * Capacity can be vary between vehicles.
  * Goes to the ``Dump`` when reached the maximum capacity except the last trip.
    * Even when a better route can be found with `not full` vehicle.
  * Vehicles are empty at the ``Deposit`` and after departure of the ``Dump``
  * Has to return to the original ``Deposit``
  * Can not make `U turns`
  * If a Driver has time


Deposit:
  * Has a (lat , lon) location.
  * opens at 0
  * closes at infinity
  * As a Starting Site
    * can have a service time  (currently is fixed to 0)
  * As an Ending Site
    * can have a service time  (currently is fixed to 0)
    
Dump:
  * Has a (lat , lon) location.
  * opens at 0
  * closes at infinity
  * Has a service time.

Container:
  * Has a (lat , lon) location.
  * Has an opening time
  * Has an closing time
  * Has a Capacity
    * The units must match the vehicles Capacity units.

Others:
  * Visit Dump inmediately after Deposit is forbidden
  * Visit Deposit inmediately after a Container is forbidden

Implied restrictions:
  * City topoogy


From the point of view of the driver:
Deposit and dump:
  - Opens at Driver's shift time start.
  - Closes at Driver's shift time end.

Data from Montevideo City is used for this development.




# Dependencies

Currently:

 * glog - For logging
 * OSRM - Local shared server installed for coverage area.


# TODO

- Update To be used with OSRM v5
  - When the project started OSRM 0.3.3 was used.
- Clean Code
  - Many trial & error were kept in the code.
- Update code to C++14
- Documentation

