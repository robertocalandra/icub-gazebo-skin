/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_SKINDRIVER_H
#define GAZEBOYARP_SKINDRIVER_H

#include <gazebo/sensors/sensors.hh>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <boost/shared_ptr.hpp>

namespace yarp {
    namespace dev {
        class GazeboYarpSkinDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class ContactSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}


extern int yarpSkinChannelsNumber;
extern  std::string YarpSkinScopedName;

class yarp::dev::GazeboYarpSkinDriver: 
    public yarp::dev::IAnalogSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpSkinDriver();
    virtual ~GazeboYarpSkinDriver();
    
    void onUpdate(const gazebo::common::UpdateInfo& /*_info*/);
	
    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ANALOG SENSOR
    virtual int read(yarp::sig::Vector& out);
    virtual int getState(int channel);
    virtual int getChannels();
    virtual int calibrateChannel(int channel, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int channel);
        int getIndex(std::string cname);
    int getForceValue(double force);

    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


private:
    yarp::sig::Vector m_skinData; //buffer for contact sensor data
    yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
    yarp::os::Semaphore m_dataMutex; //mutex for accessing the data
    gazebo::sensors::ContactSensor* m_parentSensor;
    gazebo::sensors::ContactSensorPtr parentSensor;
    gazebo::event::ConnectionPtr m_updateConnection;

};

#endif // GAZEBOYARP_CONTACTDRIVER_H
