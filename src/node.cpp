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


// System
#include <iostream>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

// Private
#include <object_definer/InteractiveObject.h>
#include <object_definer/OwlIndividual.h>


// Global variables
std::map<std::shared_ptr<object_definer::InteractiveObject>, std::string> g_mapGraspType;


// Functions
unsigned int shape(std::string strShape) {
  if(strShape == "cube") {
    return visualization_msgs::Marker::CUBE;
  } else if(strShape == "sphere") {
    return visualization_msgs::Marker::SPHERE;
  } else if(strShape == "cylinder") {
    return visualization_msgs::Marker::CYLINDER;
  }
}


std::string shapeString(unsigned int unShape) {
  switch(unShape) {
  case visualization_msgs::Marker::CUBE: return "cube"; break;
  case visualization_msgs::Marker::SPHERE: return "sphere"; break;
  case visualization_msgs::Marker::CYLINDER: return "cylinder"; break;
  };
}


unsigned int controlMode(std::string strIdentifier) {
  if(strIdentifier == "MOVE_AXIS") {
    return visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  } else if(strIdentifier == "ROTATE_AXIS") {
    return visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  } else if(strIdentifier == "MOVE_PLANE") {
    return visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  } else if(strIdentifier == "MOVE_ROTATE") {
    return visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  } else if(strIdentifier == "MOVE_3D") {
    return visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  } else if(strIdentifier == "ROTATE_3D") {
    return visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
  } else if(strIdentifier == "MOVE_ROTATE_3D") {
    return visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  }
}


std::string controlModeString(unsigned int unMode) {
  switch(unMode) {
  case visualization_msgs::InteractiveMarkerControl::MOVE_AXIS: return "MOVE_AXIS"; break;
  case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS: return "ROTATE_AXIS"; break;
  case visualization_msgs::InteractiveMarkerControl::MOVE_PLANE: return "MOVE_PLANE"; break;
  case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE: return "MOVE_ROTATE"; break;
  case visualization_msgs::InteractiveMarkerControl::MOVE_3D: return "MOVE_3D"; break;
  case visualization_msgs::InteractiveMarkerControl::ROTATE_3D: return "ROTATE_3D"; break;
  case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D: return "MOVE_ROTATE_3D"; break;
  };
}


std::shared_ptr<object_definer::InteractiveObject> loadObject(std::string strObjectOWLFile) {
  std::shared_ptr<object_definer::InteractiveObject> ioObject = std::make_shared<object_definer::InteractiveObject>("object");
  
  visualization_msgs::Marker mkrMarker;
  /* BEGIN ADAPT */
  mkrMarker.type = visualization_msgs::Marker::CUBE;
  /* END ADAPT */
  
  /* BEGIN ADAPT */
  mkrMarker.color.r = 1.0;
  mkrMarker.color.g = 1.0;
  mkrMarker.color.b = 1.0;
  mkrMarker.color.a = 1.0;
  /* END ADAPT */
  
  mkrMarker.scale.x = 1.0;
  mkrMarker.scale.y = 1.0;
  mkrMarker.scale.z = 1.0;
  
  ioObject->setMarker(mkrMarker);
  
  /* BEGIN ADAPT */
  ioObject->setSize(0.1, 0.1, 0.1);
  /* END ADAPT */
  
  geometry_msgs::Pose psPose;
  psPose.position.x = 0;
  psPose.position.y = 0;
  psPose.position.z = 0;
  psPose.orientation.x = 0;
  psPose.orientation.y = 0;
  psPose.orientation.z = 0;
  psPose.orientation.w = 1;
  
  ioObject->setPose("object", psPose);
  
  ioObject->addMenuEntry("Add new handle", "add_handle");
  
  unsigned int unShape = ioObject->addMenuEntry("Shape", "shape_menu");
  ioObject->addMenuEntry("Cube", "change_shape", "cube", unShape);
  ioObject->addMenuEntry("Sphere", "change_shape", "sphere", unShape);
  ioObject->addMenuEntry("Cylinder", "change_shape", "cylinder", unShape);
  
  unsigned int unExport = ioObject->addMenuEntry("Export", "export_menu");
  ioObject->addMenuEntry("OWL (" + ioObject->name() + ".owl)", "export", "owl", unExport);
  
  ioObject->setMenuCheckBox(ioObject->menuEntry("change_shape", "cube").unMenuEntryID, 1);
  
  return ioObject;
}


void setGraspType(std::shared_ptr<object_definer::InteractiveObject> ioObject, std::string strGraspType) {
  g_mapGraspType[ioObject] = strGraspType;
}


std::string graspType(std::shared_ptr<object_definer::InteractiveObject> ioObject) {
  return g_mapGraspType[ioObject];
}


std::shared_ptr<object_definer::InteractiveObject> makeHandle(std::string strHandleName) {
  std::shared_ptr<object_definer::InteractiveObject> ioHandleNew = std::make_shared<object_definer::InteractiveObject>(strHandleName);
  
  visualization_msgs::Marker mkrMarker;
  /* BEGIN ADAPT */
  mkrMarker.type = visualization_msgs::Marker::ARROW;
  /* END ADAPT */
  
  /* BEGIN ADAPT */
  mkrMarker.color.r = 1.0;
  mkrMarker.color.g = 0.0;
  mkrMarker.color.b = 0.0;
  mkrMarker.color.a = 1.0;
  /* END ADAPT */
  
  mkrMarker.scale.x = 1.0;
  mkrMarker.scale.y = 1.0;
  mkrMarker.scale.z = 1.0;
  
  ioHandleNew->setMarker(mkrMarker);
  
  /* BEGIN ADAPT */
  ioHandleNew->setSize(0.2, 0.05, 0.05);
  /* END ADAPT */
  
  geometry_msgs::Pose psPose;
  psPose.position.x = -0.3;
  psPose.position.y = 0;
  psPose.position.z = 0;
  psPose.orientation.x = 0;
  psPose.orientation.y = 0;
  psPose.orientation.z = 0;
  psPose.orientation.w = 1;
  
  ioHandleNew->setPose("object", psPose);
  ioHandleNew->addMenuEntry("Remove", "remove_handle", strHandleName);
  unsigned int unControlMode = ioHandleNew->addMenuEntry("Control Mode", "control_mode_menu");
  ioHandleNew->addMenuEntry("Move Axis", "change_control_mode", "MOVE_AXIS", unControlMode);
  ioHandleNew->addMenuEntry("Rotate Axis", "change_control_mode", "ROTATE_AXIS", unControlMode);
  ioHandleNew->addMenuEntry("Move Plane", "change_control_mode", "MOVE_PLANE", unControlMode);
  ioHandleNew->addMenuEntry("Move/Rotate", "change_control_mode", "MOVE_ROTATE", unControlMode);
  ioHandleNew->addMenuEntry("Move 3D", "change_control_mode", "MOVE_3D", unControlMode);
  ioHandleNew->addMenuEntry("Rotate 3D", "change_control_mode", "ROTATE_3D", unControlMode);
  ioHandleNew->addMenuEntry("Move/Rotate 3D", "change_control_mode", "MOVE_ROTATE_3D", unControlMode);
  
  ioHandleNew->changeControlMode(controlMode("MOVE_AXIS"));
  ioHandleNew->setMenuCheckBox(ioHandleNew->menuEntry("change_control_mode", "MOVE_AXIS").unMenuEntryID, 1);
  
  unsigned int unGraspTypeMenu = ioHandleNew->addMenuEntry("Grasp Type", "grasp_type_menu");
  ioHandleNew->addMenuEntry("Push", "change_grasp_type", "push", unGraspTypeMenu);
  
  setGraspType(ioHandleNew, "push");
  ioHandleNew->setMenuCheckBox(ioHandleNew->menuEntry("change_grasp_type", "push").unMenuEntryID, 1);
  
  return ioHandleNew;
}


std::string str(double dValue) {
  std::stringstream sts;
  sts << dValue;
  
  return sts.str();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "object_definer");
  ROS_INFO("Started object definer.");
  
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> imsServer = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_definer", "", false);
  
  std::string strObjectOWLFile = ""; // TODO(winkler): Add this later.
  std::shared_ptr<object_definer::InteractiveObject> ioObject = loadObject(strObjectOWLFile);
  
  if(ioObject) {
    //ROS_INFO("Successfully loaded object file.");
    ioObject->insertIntoServer(imsServer);
    
    std::list< std::shared_ptr<object_definer::InteractiveObject> > lstHandles;
    
    while(ros::ok()) {
      ros::spinOnce();
      
      std::list<object_definer::InteractiveObjectCallbackResult> lstCallbacks = ioObject->callbackResults();
      for(object_definer::InteractiveObjectCallbackResult iocrResult : lstCallbacks) {
	if(iocrResult.strCommand == "add_handle") {
	  std::stringstream sts;
	  sts << "handle_";
	  sts << lstHandles.size();
	  
	  std::shared_ptr<object_definer::InteractiveObject> ioHandleNew = makeHandle(sts.str());
	  
	  if(ioHandleNew) {
	    lstHandles.push_back(ioHandleNew);
	    
	    ioHandleNew->insertIntoServer(imsServer);
	    ioHandleNew->subscribeToPoseUpdate("interactive_definer");
	  }
	} else if(iocrResult.strCommand == "export") {
	  if(iocrResult.strParameter == "owl") {
	    std::ofstream ofOWL(ioObject->name() + ".owl", std::ofstream::out);
	    std::string strFile = "";
	    
	    object_definer::OwlIndividual oiObject;
	    oiObject.setID("&knowrob;" + ioObject->name());
	    oiObject.setType("&knowrob;HumanScaleObject");
	    
	    oiObject.addDataProperty("knowrob:shape", "&xsd;string", shapeString(ioObject->shape()));
	    
	    oiObject.addDataProperty("knowrob:widthOfObject", "&xsd;double", str(ioObject->width()));
	    oiObject.addDataProperty("knowrob:depthOfObject", "&xsd;double", str(ioObject->depth()));
	    oiObject.addDataProperty("knowrob:heightOfObject", "&xsd;double", str(ioObject->height()));
	    
	    for(std::shared_ptr<object_definer::InteractiveObject> ioHandle : lstHandles) {
	      object_definer::OwlIndividual oiHandle;
	      oiHandle.setID(ioObject->name() + "_" + ioHandle->name());
	      oiHandle.setType("&knowrob;SemanticHandle");
	      oiHandle.addContentProperty("knowrob:graspType", graspType(ioHandle));
	      
	      oiObject.addResourceProperty("knowrob:semanticHandle", oiHandle.id());
	      
	      object_definer::OwlIndividual oiHandlePose;
	      oiHandlePose.setID(ioObject->name() + "_" + ioHandle->name() + "_pose");
	      oiHandlePose.setType("&knowrob;RotationMatrix3D");
	      
	      geometry_msgs::Pose psHandlePose = ioHandle->pose();
	      tf::Pose tfPose;
	      tf::poseMsgToTF(psHandlePose, tfPose);
	      
	      tf::Matrix3x3 mxRotation = tfPose.getBasis();
	      tf::Vector3 vcOrigin = tfPose.getOrigin();
	      
	      for(int nI = 0; nI < 4; nI++) {
		for(int nJ = 0; nJ < 4; nJ++) {
		  std::stringstream sts;
		  sts << "knowrob:m";
		  sts << nI;
		  sts << nJ;
		  
		  double dValue = 0.0;
		  
		  if(nI < 3) {
		    if(nJ == 3) {
		      dValue = vcOrigin[nI];
		    } else {
		      dValue = mxRotation[nI][nJ];
		    }
		  } else {
		    if(nJ == 3) {
		      dValue = 1.0;
		    } else {
		      dValue = 0.0;
		    }
		  }
		  
		  oiHandlePose.addDataProperty(sts.str(), "http://www.w3.org/2001/XMLSchema#double", str(dValue));
		}
	      }
	      
	      oiHandle.addResourceProperty("knowrob:handlePose", oiHandlePose.id());
	      
	      strFile += oiHandle.print();
	      strFile += oiHandlePose.print();
	    }
	    
	    strFile = oiObject.print() + strFile;
	    
	    strFile = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\
<!DOCTYPE rdf:RDF [\n\
    <!ENTITY owl \"http://www.w3.org/2002/07/owl#\">\n\
    <!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\">\n\
    <!ENTITY knowrob \"http://knowrob.org/kb/knowrob.owl#\">\n\
    <!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\">\n\
    <!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">\n\
    <!ENTITY log \"http://knowrob.org/kb/cram_log.owl#\">\n\
]>\n\
\n\
<rdf:RDF xmlns=\"http://knowrob.org/kb/cram_log.owl#\"\n\
     xml:base=\"http://knowrob.org/kb/cram_log.owl\"\n\
     xmlns:owl=\"http://www.w3.org/2002/07/owl#\"\n\
     xmlns:xsd=\"http://www.w3.org/2001/XMLSchema#\"\n\
     xmlns:knowrob=\"http://knowrob.org/kb/knowrob.owl#\"\n\
     xmlns:rdfs=\"http://www.w3.org/2000/01/rdf-schema#\"\n\
     xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\"\n\
     xmlns:log=\"http://knowrob.org/kb/cram_log.owl#\">\n\
    \n\
    <owl:Ontology rdf:about=\"http://knowrob.org/kb/cram_log.owl\">\n\
        <owl:imports rdf:resource=\"package://knowrob_common/owl/knowrob.owl\"/>\n\
    </owl:Ontology>\n\
    \n\
" + strFile + "</rdf:RDF>";
	    
	    ofOWL << strFile;
	    ofOWL.close();
	  }
	} else if(iocrResult.strCommand == "change_shape") {
	  ioObject->setMenuCheckBox(ioObject->menuEntry(iocrResult.strCommand, shapeString(ioObject->shape())).unMenuEntryID, -1);
	  ioObject->setShape(shape(iocrResult.strParameter));
	  
	  ioObject->setMenuCheckBox(ioObject->menuEntry(iocrResult.strCommand, iocrResult.strParameter).unMenuEntryID, 1);
	}
      }
      
      for(std::shared_ptr<object_definer::InteractiveObject> ioHandle : lstHandles) {
	std::list<object_definer::InteractiveObjectCallbackResult> lstCallbacks = ioHandle->callbackResults();
	bool bListChanged = false;
	
	for(object_definer::InteractiveObjectCallbackResult iocrResult : lstCallbacks) {
	  if(iocrResult.strCommand == "remove_handle") {
	    for(std::list< std::shared_ptr<object_definer::InteractiveObject> >::iterator itHandle = lstHandles.begin();
		itHandle != lstHandles.end(); itHandle++) {
	      if(*itHandle == ioHandle) {
		lstHandles.erase(itHandle);
		ioHandle->removeFromServer();
		bListChanged = true;
		
		break;
	      }
	    }
	  } else if(iocrResult.strCommand == "change_control_mode") {
	    ioHandle->setMenuCheckBox(ioHandle->menuEntry(iocrResult.strCommand, controlModeString(ioHandle->controlMode())).unMenuEntryID, -1);
	    ioHandle->changeControlMode(controlMode(iocrResult.strParameter));
	    
	    ioHandle->setMenuCheckBox(ioHandle->menuEntry(iocrResult.strCommand, iocrResult.strParameter).unMenuEntryID, 1);
	  } else if(iocrResult.strCommand == "change_grasp_type") {
	    ioHandle->setMenuCheckBox(ioHandle->menuEntry(iocrResult.strCommand, graspType(ioHandle)).unMenuEntryID, -1);
	    setGraspType(ioHandle, iocrResult.strParameter);
	    
	    ioHandle->setMenuCheckBox(ioHandle->menuEntry(iocrResult.strCommand, iocrResult.strParameter).unMenuEntryID, 1);
	  }
	}
	
	if(bListChanged) {
	  break;
	}
      }
    }
    
    ROS_INFO("Exiting.");
  } else {
    ROS_ERROR("Failed to load object file.");
  }
  
  return EXIT_SUCCESS;
}
