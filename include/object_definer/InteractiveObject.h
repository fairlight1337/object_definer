/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Institute for Artificial Intelligence,
 *  Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Jan Winkler */


#ifndef __INTERACTIVE_OBJECT_H__
#define __INTERACTIVE_OBJECT_H__


// System
#include <string>
#include <iostream>
#include <mutex>
#include <memory>

// ROS
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>


namespace object_definer {
  typedef struct {
    std::string strLabel;
    std::string strIdentifier;
    std::string strParameter;
    uint8_t unMenuEntryID;
    uint8_t unParentID;
  } InteractiveMenuEntry;
  
  typedef struct {
    std::string strObject;
    std::string strCommand;
    std::string strParameter;
  } InteractiveObjectCallbackResult;
  
  class InteractiveObject {
  private:
    ros::NodeHandle m_nhHandle;
    ros::Subscriber m_subPoseUpdate;
    unsigned int m_unInteractionMode;
    visualization_msgs::Marker m_mkrMarker;
    interactive_markers::MenuHandler m_mhMenu;
    visualization_msgs::InteractiveMarker m_imMarker;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> m_imsServer;
    visualization_msgs::InteractiveMarkerControl m_imcControl;
    std::list<InteractiveMenuEntry> m_lstMenuEntries;
    std::list<InteractiveObjectCallbackResult> m_lstCallbackResults;
    std::mutex m_mtxCallbackResults;
    
  public:
    InteractiveObject(std::string strName);
    ~InteractiveObject();
    
    bool insertIntoServer(std::shared_ptr<interactive_markers::InteractiveMarkerServer> imsServer);
    bool removeFromServer();
    void clickCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void eventCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    void setMarker(visualization_msgs::Marker mkrMarker);
    void setSize(float fWidth, float fDepth, float fHeight);
    void setPose(std::string strFixedFrame, geometry_msgs::Pose posPose, bool bUpdateServer = true);
    void setPose(geometry_msgs::Pose posPose, bool bUpdateServer = true);
    
    geometry_msgs::Pose pose();
    
    float width();
    float depth();
    float height();
    
    std::string name();
    unsigned int addMenuEntry(std::string strLabel, std::string strIdentifier, std::string strParameter = "", unsigned int unParentID = 0);
    void removeMenuEntry(std::string strIdentifier, std::string strParameter = "");
    void setMenuCheckBox(unsigned int unMenuEntry, int nState);
    void clearMenuEntries();
    
    std::list<InteractiveObjectCallbackResult> callbackResults();
    
    void changeControlMode(unsigned int unInteractionMode);
    unsigned int controlMode();
    
    void subscribeToPoseUpdate(std::string strFeedbackNode);
    void poseUpdateCallback(const visualization_msgs::InteractiveMarkerFeedback feedback);
    
    InteractiveMenuEntry menuEntry(std::string strIdentifier, std::string strParameter = "");
  };
}


#endif /* __INTERACTIVE_OBJECT_H__ */
