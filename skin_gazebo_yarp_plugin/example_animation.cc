/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 7.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->SetTranslation(math::Vector3(-0.2, -0.2, 0.3));
        key->SetRotation(math::Quaternion(0, 0, 0));
        
         

        // set waypoint location after 1 seconds
        key = anim->CreateKeyFrame(1.0);
        key->SetTranslation(math::Vector3(-0.0, -0.2, 0.7));
        key->SetRotation(math::Quaternion(0, 0, 0.0));

//contact points
        key = anim->CreateKeyFrame(2.5);
         key->SetTranslation(math::Vector3(-0.0, -0.11, 0.7));
        key->SetRotation(math::Quaternion(0, 0, 0.0));
        
        /*key = anim->CreateKeyFrame(2.6);
         key->SetTranslation(math::Vector3(-0.0, -0.10, 0.7));
        key->SetRotation(math::Quaternion(0, 0, 1.5707));
        */
        key = anim->CreateKeyFrame(2.8);
         key->SetTranslation(math::Vector3(-0.1, -0.10, 0.7));
        key->SetRotation(math::Quaternion(0, 0, 0));
        
        key = anim->CreateKeyFrame(3.3);
        key->SetTranslation(math::Vector3(-0.2, -0.2, 0.3));
        key->SetRotation(math::Quaternion(0, 0, 0));
        
        
          key = anim->CreateKeyFrame(4.5);
        key->SetTranslation(math::Vector3(0.30, -0.01, 0.6));
        key->SetRotation(math::Quaternion(0, 0, 0));
        
        
         key = anim->CreateKeyFrame(5.0);
        key->SetTranslation(math::Vector3(0.40, -0.21, 0.6));
        key->SetRotation(math::Quaternion(0, 0, 0));
        
        
        key = anim->CreateKeyFrame(6.0);
        key->SetTranslation(math::Vector3(-0.2, -0.2, 0.3));
        key->SetRotation(math::Quaternion(0, 0, 0));
        
     /*   key = anim->CreateKeyFrame(3.0);
       key->SetTranslation(math::Vector3(-0.1, -0.1, 0.4));
        key->SetRotation(math::Quaternion(0, 0, 0));
        
        key = anim->CreateKeyFrame(5.0);
        key->SetTranslation(math::Vector3(-0.2, -0.2, 0.3));
        key->SetRotation(math::Quaternion(0, 0, 0));
*/
/*
        key = anim->CreateKeyFrame(6.0);
        key->SetTranslation(math::Vector3(2, 2, 2));
        key->SetRotation(math::Quaternion(0, 0, 1.5707));


        key = anim->CreateKeyFrame(8.0);
        key->SetTranslation(math::Vector3(1, -1, 1));
        key->SetRotation(math::Quaternion(0, 0, 1.5707));

        // set final location equal to starting location
        key = anim->CreateKeyFrame(1);
        key->SetTranslation(math::Vector3(1, 1, 1));
        key->SetRotation(math::Quaternion(0, 0, 0));
*/
        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}
