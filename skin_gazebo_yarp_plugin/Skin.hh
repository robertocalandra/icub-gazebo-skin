/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef GAZEBOYARP_SKIN_HH
#define GAZEBOYARP_SKIN_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/PolyDriver.h>

#include <string>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
    namespace sensors {
        class ContactSensor;
    }

    /// \class GazeboYarpSkin
    /// Gazebo Plugin emulating the yarp imu device in Gazebo.
    ///
    /// This plugin instantiate a yarp skin driver for the Gazebo simulator
    /// and instantiate a network wrapper 
    /// to expose the sensor on the yarp network.
    ///
    /// It can be configurated using the yarpConfigurationFile sdf tag,
    /// that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the controlBoard
    ///
    /// The parameter that the yarpConfigurationFile must contain are:
    ///  <TABLE>
    ///  <TR><TD> name </TD><TD> Port name to assign to the wrapper to this device. </TD></TR>
    ///  <TR><TD> period </TD><TD> Update period (in s) of yarp port that publish the measure. </TD></TR>
    ///  </TABLE>
    /// If the required parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::ServerInertial wrapper .
    ///
    class GazeboYarpSkin : public SensorPlugin
    {
    public:
        GazeboYarpSkin();
        virtual ~GazeboYarpSkin();
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        yarp::os::Property m_parameters; 
		yarp::dev::PolyDriver m_skinWrapper;
        yarp::dev::PolyDriver m_skinDriver;
		yarp::dev::IMultipleWrapper* m_iWrap;

        std::string m_sensorName;
    };
}

#endif
