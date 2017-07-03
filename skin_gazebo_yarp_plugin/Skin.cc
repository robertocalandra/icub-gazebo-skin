/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "Skin.hh"
#include "SkinDriver.h"
#include "Handler.hh"
#include "common.h"

#include <gazebo/sensors/ContactSensor.hh>

#include <yarp/dev/ServerInertial.h>
//#include <yarp/dev/MultipleWrapper.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpSkin)

namespace gazebo {
    
GazeboYarpSkin::GazeboYarpSkin() : SensorPlugin(), m_iWrap(0)
{
}

GazeboYarpSkin::~GazeboYarpSkin()
{
    std::cout << "*** GazeboYarpSkin closing ***" << std::endl;
    if(m_iWrap) { m_iWrap->detachAll(); m_iWrap = 0; }
    if( m_skinWrapper.isValid() ) m_skinWrapper.close();
    if( m_skinDriver.isValid() ) m_skinDriver.close();
    
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
    yarp::os::Network::fini();
}

void GazeboYarpSkin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        std::cerr << "GazeboYarpSkin::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
        return;
    }
    
    std::cout << "*** GazeboYarpSkin plugin started ***" << std::endl;
    
    if (!_sensor) {
        gzerr << "GazeboYarpSkin plugin requires a ContactSensor." << std::endl;
        return;
    }
    
    _sensor->SetActive(true);
    
    //Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpSkinDriver>
                                       ("gazebo_skin", "analogServer", "GazeboYarpSkinDriver"));
							
   
    
    //Getting .ini configuration file from sdf
	::yarp::os::Property wrapper_properties;
    ::yarp::os::Property driver_properties;
    bool configuration_loaded = false;
    
    if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str())) {
            std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl;
            configuration_loaded = true;
        }
    }
	driver_properties=m_parameters;
    wrapper_properties = driver_properties;
    if (!configuration_loaded) {

        std::cout << "File .ini not found, quitting"  << std::endl;
        return;
    }
    
    m_sensorName = _sensor->GetScopedName();
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());
    
    m_parameters.put(YarpSkinScopedName.c_str(), m_sensorName.c_str());
	driver_properties.put(YarpSkinScopedName.c_str(), m_sensorName.c_str());
    
	//Open the wrapper
    //Force the wrapper to be of type "analogServer" (it make sense? probably no)
	wrapper_properties.put("device","analogServer");

    if( m_skinWrapper.open(wrapper_properties) ) {
        std::cout<<"GazeboYarpSkin Plugin: correcly opened GazeboYarpSkin wrapper"<<std::endl;
    } else {
        std::cout<<"GazeboYarpSkin Plugin failed: error in opening yarp driver wrapper"<<std::endl;
        return;
    }

    ///Open the driver
    //Force the device to be of type "gazebo_forcetorque" (it make sense? probably yes)

	driver_properties.put("device","gazebo_skin");
	driver_properties.put("gazeboYarpPluginsRobotName","icubGazeboSim");

    if( m_skinDriver.open(driver_properties) ) {
        std::cout<<"GazeboYarpSkin Plugin: correcly opened GazeboYarpSkin"<<std::endl;
    } else {
        std::cout<<"GazeboYarpSkin Plugin failed: error in opening yarp driver"<<std::endl;
        return;
    }

    //Attach the driver to the wrapper
    ::yarp::dev::PolyDriverList driver_list;
    
    if( !m_skinWrapper.view(m_iWrap) ) {
        std::cerr << "GazeboYarpSkin : error in loading wrapper" << std::endl;
        return;
    }
    
    driver_list.push(&m_skinDriver,"dummy");
    
    if( m_iWrap->attachAll(driver_list) ) {
        std::cerr << "GazeboYarpSkin : wrapper was connected with driver " << std::endl;
    } else {
        std::cerr << "GazeboYarpSkin : error in connecting wrapper and device " << std::endl;
    }
}
    
}
