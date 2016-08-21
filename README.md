Garbage collection in montevideo
============================================

# A little Code Cleanup

The project started to be a bit of everything.
But there is a lot o unsued code.

First clean up Garbage collection code, And post the clean up in the public repository.
(Specially for the FOSS4G)

Then a cmake.



# Original comments:

## C++ Classes for solving various vehicle routing problems

 * baseClasses - Classes shared betwen the various problems
 * trash-collection - solve a trash collection problem. This has multiple depots with a single vehicle at each, they go to various containers and collect the trash and unloads at a dump before returning to its home depot. There is support for multiple dump sites, vehicle capacity and time windows.
 * vrpdptw - A single depot multiple vehicle pick and delivery problem with time windows.
 * more to come ...

# THIS IS A WORK IN PROGRESS

# Dependencies

We current this have dependencies in our development environment for

 * libgd2 - used for generating plots of the routes
 * libcurl - used via curlpp to access OSRM engine
 * curlpp - a C++ wrapper around libcurl to talk to OSRM engine https://code.google.com/p/curlpp/
 * OSRM - we have a local server installed for our coverage area. This is used for comput drive times and routes https://github.com/Project-OSRM/osrm-backend

