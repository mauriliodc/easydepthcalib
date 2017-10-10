#ifndef DRIVER_NODELET_H
#define DRIVER_NODELET_H

// Ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// easydepthcalib
#include "driver.h"


class DriverNodelet : public nodelet::Nodelet
{

    // ================================
    // ======== PUBLIC METHODS ========
    // ================================

public:

    DriverNodelet();
    ~DriverNodelet();

private:

    /*!
      * @brief Serves as a psuedo constructor for nodelets.
      *
      * This function needs to do the MINIMUM amount of work
      * to get the nodelet running.  Nodelets should not call
      * blocking functions here for a significant period of time.
      */
    void onInit();

    /**
     * @brief driver    Object of the driver class.
     */
    Driver *driver;
};

#endif //DRIVER_NODELET_H
