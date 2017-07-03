/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "SkinDriver.h"
#include "Handler.hh"
#include <boost/math/special_functions/gamma.hpp>
#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <math.h>

using namespace yarp::dev;

int yarpSkinChannelsNumber = 768; //The Skin sensors
std::string YarpSkinScopedName = "sensorScopedName";

GazeboYarpSkinDriver::GazeboYarpSkinDriver() {}
GazeboYarpSkinDriver::~GazeboYarpSkinDriver() {}

/**
 *
 * Export a force/torque sensor.
 * 
 * \todo check Skin data
 */
void GazeboYarpSkinDriver::onUpdate(const gazebo::common::UpdateInfo& /*_info*/)
{
   
// Get all the contacts.
  gazebo::msgs::Contacts contacts;
  contacts = m_parentSensor->GetContacts();
  std::vector<double> skinSensors(yarpSkinChannelsNumber,255);
  float mforce =0.0;
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
	  {
		//std::cout << "Collision between[" << contacts.contact(i).collision1()<< "] and [" << contacts.contact(i).collision2() << "]\n";
		/*
		 * Control output
		std::cout << "    xyz wrench 0 - body1-force:" << contacts.contact(i).wrench(0).body_1_wrench().force().x() << " - " << contacts.contact(i).wrench(0).body_1_wrench().force().y() << " - "<< contacts.contact(i).wrench(0).body_1_wrench().force().z() << "\n";
		std::cout << "    xyz wrench 0 - body2-force:" 	<< contacts.contact(i).wrench(0).body_2_wrench().force().x() << " - " << contacts.contact(i).wrench(0).body_2_wrench().force().y() << " - "<< contacts.contact(i).wrench(0).body_2_wrench().force().z() << "\n";
		*/
		
		//Calculate length of the force vector
		 mforce = sqrt (pow(contacts.contact(i).wrench(0).body_1_wrench().force().x(),2)+pow(contacts.contact(i).wrench(0).body_1_wrench().force().y(),2) + pow(contacts.contact(i).wrench(0).body_1_wrench().force().x(),2));

		//idea read collisions and extract collision id
		std::string cname = contacts.contact(i).collision1();
		int index1 = GazeboYarpSkinDriver::getIndex(cname);

		//element 1 
		if (index1!=-1){
			skinSensors[index1] = (int)getForceValue(mforce);
		}
		else {
			//element 2	
			std::string cname2 = contacts.contact(i).collision2();
			int index2 = GazeboYarpSkinDriver::getIndex(cname2);
			if (index2!=-1){
				skinSensors[index2] = (int)getForceValue(mforce);
			}
		}
	}
 
    /** \todo ensure that the timestamp is the right one */
    /** \todo TODO use GetLastMeasureTime, not GetLastUpdateTime */
    m_lastTimestamp.update(this->m_parentSensor->GetLastUpdateTime().Double());
    
    int i=0;
    m_dataMutex.wait();

    for (i = 0; i < yarpSkinChannelsNumber; i++) {
        m_skinData[0 + i] = skinSensors[i];
    }

    m_dataMutex.post();
    return;
}

    
//DEVICE DRIVER
bool GazeboYarpSkinDriver::open(yarp::os::Searchable& config)
{
    std::cout << "GazeboYarpSkinDriver::open() called" << std::endl;
    
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpSkinScopedName.c_str()).asString().c_str());
    std::cout << "GazeboYarpSkinDriver::open is looking for sensor " << sensorScopedName << "..." << std::endl;
    
    m_parentSensor = (gazebo::sensors::ContactSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);


    if (!m_parentSensor)
    {
        std::cout << "Error, Contact sensor was not found" << std::endl;
        return AS_ERROR;
    }
	YarpSkinScopedName =sensorScopedName;
	//mutex 
    m_dataMutex.wait();
    m_skinData.resize(yarpSkinChannelsNumber, 0.0);
    m_dataMutex.post();

    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpSkinDriver::onUpdate, this, _1));
  
    std::cout << "GazeboYarpSkinDriver::open() returning true" << std::endl;
    return true;
}

bool GazeboYarpSkinDriver::close()
{
    if (this->m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
    }
    return true;
}
    
//ANALOG SENSOR
int GazeboYarpSkinDriver::read(yarp::sig::Vector& out)
{
    ///< \todo TODO in my opinion the reader should care of passing a vector of the proper dimension to the driver, but apparently this is not the case
    /*
    if( (int)m_ContactData.size() != yarpContactChannelsNumber ||
        (int)out.size() != yarpContactChannelsNumber ) {
        return AS_ERROR;
    }
    */
    
  /* if ((int)m_contactData.size() != yarpContactChannelsNumber) {
        return AS_ERROR;
   }*/
   
   if ((int)out.size() != yarpSkinChannelsNumber) {
       out.resize(yarpSkinChannelsNumber);
   }
    
    m_dataMutex.wait();
    out = m_skinData;
    m_dataMutex.post();
    
    return AS_OK;
}

int GazeboYarpSkinDriver::getChannels()
{
    return yarpSkinChannelsNumber;
}

int GazeboYarpSkinDriver::getState(int ch)
{
    return AS_OK;
}

int GazeboYarpSkinDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpSkinDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int GazeboYarpSkinDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int GazeboYarpSkinDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}
int GazeboYarpSkinDriver::getIndex(std::string cname){
	
	std::size_t found = cname.find("box_collision");
	int index =-1;
	if (found!=std::string::npos){
		std::string substring = cname.substr (found+13);
		index = strtol(substring.c_str(),NULL,0);
		if (index != 0 && index <= yarpSkinChannelsNumber) {
			index=index-1;
		}
	}
	return index;
}
int GazeboYarpSkinDriver::getForceValue(double force){
	//beta function
	double forceValue = 255*boost::math::ibeta(force,9,0.8);
	//std::cout << forceValue << " beta forceValue "<<  '\n';
	
	//std::cout << forceValue << "  forceValue "<<  '\n';
	return forceValue;
	
}
	
//PRECISELY TIMED
yarp::os::Stamp GazeboYarpSkinDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}
