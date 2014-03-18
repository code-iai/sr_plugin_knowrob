#include <bs_plugin_knowrob/PluginKnowRob.h>


namespace beliefstate {
  namespace plugins {
    PluginKnowRob::PluginKnowRob() {
      m_prlgProlog = NULL;
      m_expOwl = NULL;
      
      this->addDependency("ros");
      this->setDevelopmentPlugin(true);
      this->setPluginVersion("0.25b");
    }
    
    PluginKnowRob::~PluginKnowRob() {
      if(m_prlgProlog) {
	delete m_prlgProlog;
      }
      
      if(m_expOwl) {
	delete m_expOwl;
      }
    }
    
    Result PluginKnowRob::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      CDesignator* cdConfig = this->getIndividualConfig();
      
      string strJSONService = cdConfig->stringValue("json-service");
      if(strJSONService == "") {
	strJSONService = "/json_prolog";
      }
      
      this->info("Waiting for JSONProlog node '" + strJSONService + "'");
      m_prlgProlog = new Prolog(strJSONService);
      m_expOwl = new CExporterOwl();
      
      string strSemanticsDescriptorFile = cdConfig->stringValue("semantics-descriptor-file");
      
      if(strSemanticsDescriptorFile != "") {
	if(m_expOwl->loadSemanticsDescriptorFile(strSemanticsDescriptorFile) == false) {
	  this->warn("Failed to load semantics descriptor file '" + strSemanticsDescriptorFile + "'.");
	}
      } else {
	this->warn("No semantics descriptor file was specified.");
      }
      
      float fWaitDuration = -1;
      
      if(cdConfig->childForKey("wait-for-service-duration")) {
	fWaitDuration = cdConfig->floatValue("wait-for-service-duration");
      }
      
      bool bInitOK = false;
      
      if(ros::ok()) {
	if(fWaitDuration == -1) {
	  bInitOK = m_prlgProlog->waitForServer();
	} else {
	  bInitOK = m_prlgProlog->waitForServer(ros::Duration(fWaitDuration));
	}
      
	if(bInitOK) {
	  // Plan node control events
	  this->setSubscribedToEvent("symbolic-begin-context", true);
	  this->setSubscribedToEvent("symbolic-end-context", true);
	  this->setSubscribedToEvent("symbolic-set-subcontext", true);
	  this->setSubscribedToEvent("symbolic-add-image", true);
	  this->setSubscribedToEvent("symbolic-equate-designators", true);
	  this->setSubscribedToEvent("symbolic-add-failure", true);
	  this->setSubscribedToEvent("symbolic-create-designator", true);
	  this->setSubscribedToEvent("symbolic-add-designator", true);
	  this->setSubscribedToEvent("symbolic-set-perception-request", true);
	  this->setSubscribedToEvent("symbolic-set-perception-result", true);
	  
	  this->setSubscribedToEvent("experiment-start", true);
	  
	  // TODO(winkler): Fully implement these events
	  this->setSubscribedToEvent("symbolic-set-object-acted-on", true);
	  this->setSubscribedToEvent("symbolic-set-detected-object", true);
	} else {
	  resInit.bSuccess = false;
	}
      } else {
	resInit.bSuccess = false;
      }
      
      return resInit;
    }
    
    Result PluginKnowRob::deinit() {
      return defaultResult();
    }
    
    Result PluginKnowRob::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginKnowRob::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "symbolic-begin-context") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  string strOWLClass = m_expOwl->owlClassForNode(ndNode, false, true);
	  string strTaskContextDescription = ndNode->title();
	  string strTimeStart = ndNode->metaInformation()->stringValue("time-start");
	  string strPreviousAction = "_";
	  
	  Node* ndPrevious = ndNode->previousNode();
	  if(ndPrevious) {
	    strPreviousAction = ndPrevious->metaInformation()->stringValue("action-instance");
	  }
	  
	  if(strPreviousAction == "") {
	    this->warn("The previous node exists but does not supply an ID. Assuming '_'. This is potentially problematic.");
	    strPreviousAction = "_";
	  }
	  
	  string strQuery = "cram_start_action(" +
	    strOWLClass + ", " +
	    "'" + strTaskContextDescription + "', " +
	    strTimeStart + ", " +
	    "'" + strPreviousAction + "', ACTIONINSTANCE)";
	  
	  bool bSuccess;
	  PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  
	  if(bSuccess) {
	    // TODO(winkler): Interprete the pbBdgs binding for `?actin'
	    // here.
	    string strActionInstance = pbBdgs["ACTIONINSTANCE"]; // Get this action instance
	    
	    // from the prolog query
	    // result.
	    ndNode->metaInformation()->setValue("action-instance", strActionInstance);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-end-context") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	  string strTimeEnd = ndNode->metaInformation()->stringValue("time-end");
	  
	  string strQuery = "cram_finish_action(" +
	    string("'") + strActionInstance + string("', ") +
	    strTimeEnd + ")";
	  
	  bool bSuccess;
	  PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	}
      } else if(evEvent.strEventName == "symbolic-set-subcontext") {
	if(evEvent.lstNodes.size() > 1) {
	  Node* ndParent = evEvent.lstNodes.front();
	  evEvent.lstNodes.pop_front();
	  Node* ndChild = evEvent.lstNodes.front();
	  
	  string strActionInstanceParent = ndParent->metaInformation()->stringValue("action-instance");
	  string strActionInstanceChild = ndChild->metaInformation()->stringValue("action-instance");
	  
	  string strQuery = "cram_set_subaction(" +
	    string("'") + strActionInstanceParent + string("', ") +
	    string("'") + strActionInstanceChild + "')";
	  
	  bool bSuccess;
	  PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	}
      } else if(evEvent.strEventName == "symbolic-add-image") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	  string strFilename = evEvent.cdDesignator->stringValue("filename");
	  
	  string strQuery = "cram_add_image_to_event(" +
	    string("'") + strActionInstance + string("', ") +
	    string("'") + strFilename + string("')");
	  
	  bool bSuccess;
	  PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	}
      } else if(evEvent.strEventName == "symbolic-equate-designators") {
	if(evEvent.cdDesignator) {
	  string strParentID = evEvent.cdDesignator->stringValue("parent-id");
	  string strChildID = evEvent.cdDesignator->stringValue("child-id");
	  string strEquationTime = evEvent.cdDesignator->stringValue("equation-time");
	  
	  if(strParentID != "" && strChildID != "") {
	    string strQuery = "cram_equate_designators(" +
	      string("'") + strParentID + string("', ") +
	      string("'") + strChildID + string("', ") +
	      strEquationTime + string(")");
	    
	    bool bSuccess;
	    PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  } else {
	    this->warn("Cannot equate designators '" + strParentID + "' and '" + strChildID + "'.");
	  }
	}
      } else if(evEvent.strEventName == "symbolic-add-failure") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    Node* ndNode = evEvent.lstNodes.front();
	    
	    string strCondition = evEvent.cdDesignator->stringValue("condition");
	    string strTimeFail = evEvent.cdDesignator->stringValue("time-failure");
	    string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	    
	    string strQuery = "cram_add_failure_to_action(" +
	      string("'") + strActionInstance + "', " +
	      "'" + m_expOwl->failureClassForCondition(strCondition) + "', " +
	      "'" + m_expOwl->owlEscapeString(strCondition) + "', " +
	      strTimeFail + ", " +
	      "FAILUREINSTANCE" +
	      ")";
	    
	    bool bSuccess;
	    PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-add-designator") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    CDesignator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	    string strID = cdDesig->stringValue("_id");
	    
	    Node* ndNode = evEvent.lstNodes.front();
	    string strOwlClass = m_expOwl->owlClassForNode(ndNode);
	    string strAnnotation = cdDesig->stringValue("_annotation");
	    
	    // NOTE(winkler): Check if the designator already
	    // exists. Only existing designators may be added. This
	    // could be changed and an implicit `create' could be done
	    // here, as it only modifies local data and we can do the
	    // appropriate prolog calls from here. Right now, this is
	    // not the case.
	    if(m_mapDesignatorInstanceMapping.count(strID) == 0) {
	      this->warn("Adding designator that was not created previously. Its created implicitly now. You might want to look into this.");
	      this->addDesignator(cdDesig);
	    }
	    
	    if(m_mapDesignatorInstanceMapping.count(strID) == 1) {
	      // Check if this node is related to perception or acting
	      // on objects. If it is, leave it alone and trigger a new
	      // event with the appropriate type. If it is not, add the
	      // given designator as a regular designator for the given
	      // node.
	      if(strAnnotation == "perception-request") {
		Event evPercReq = defaultEvent("symbolic-set-perception-request");
		evPercReq.cdDesignator = new CDesignator(cdDesig);
		evPercReq.lstNodes = evEvent.lstNodes;
		this->deployEvent(evPercReq);
	      } else if(strAnnotation == "perception-result") {
		// TODO(winkler): These two must be distinguished. Talk to Moritz about it.
		Event evPercRes = defaultEvent("symbolic-set-perception-result");
		evPercRes.cdDesignator = new CDesignator(cdDesig);
		evPercRes.lstNodes = evEvent.lstNodes;
		this->deployEvent(evPercRes);
		
		Event evDetObj = defaultEvent("symbolic-set-detected-object");
		evDetObj.cdDesignator = new CDesignator(cdDesig);
		evDetObj.lstNodes = evEvent.lstNodes;
		this->deployEvent(evDetObj);
	      } else if(strAnnotation == "object-acted-on") {
		Event evObjActedOn = defaultEvent("symbolic-set-object-acted-on");
		evObjActedOn.cdDesignator = new CDesignator(cdDesig);
		evObjActedOn.lstNodes = evEvent.lstNodes;
		this->deployEvent(evObjActedOn);
	      } else {
		string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
		string strDesignatorInstance = m_mapDesignatorInstanceMapping[strID];
		
		string strQuery = "cram_add_desig_to_action(" +
		  string("'") + strActionInstance + "', " +
		  "'" + strDesignatorInstance + "'" +
		  ")";
		
		bool bSuccess;
		PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	      }
	    }
	  }
	}
      } else if(evEvent.strEventName == "symbolic-create-designator") {
	if(evEvent.cdDesignator) {
	  CDesignator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	  // Make sure this designator is not yet added.
	  if(m_mapDesignatorInstanceMapping.count(cdDesig->stringValue("_id")) == 0) {
	    // Its not in the map.
	    this->addDesignator(cdDesig);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-set-object-acted-on") {
	if(evEvent.cdDesignator) {
	  this->warn("Setting object acted on via json queries is not yet implemented!");
	  // TODO(winkler): Implement this
	}
      } else if(evEvent.strEventName == "symbolic-set-detected-object") {
	if(evEvent.cdDesignator) {
	  this->warn("Setting detected object via json queries is not yet implemented!");
	  // TODO(winkler): Implement this
	}
      } else if(evEvent.strEventName == "symbolic-set-perception-request") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    CDesignator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	    string strID = cdDesig->stringValue("_id");
	    Node* ndNode = evEvent.lstNodes.front();
	    string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	    string strDesigID = m_mapDesignatorInstanceMapping[strID];
	    
	    string strQuery = "cram_set_perception_request(" +
	      string("'") + strActionInstance + "', " +
	      string("'") + strDesigID + "'"
	      + ")";
	    
	    bool bSuccess;
	    PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-set-perception-result") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    CDesignator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	    string strID = cdDesig->stringValue("_id");
	    Node* ndNode = evEvent.lstNodes.front();
	    string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	    string strDesigID = m_mapDesignatorInstanceMapping[strID];
	    
	    string strQuery = "cram_set_perception_result(" +
	      string("'") + strActionInstance + "', " +
	      string("'") + strDesigID + "'"
	      + ")";
	    
	    bool bSuccess;
	    PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  }
	}
      } else if(evEvent.strEventName == "experiment-start") {
	// TODO: Start experiment here.
	string strExperimentName = "";
	string strFirstNodeName = "";
	
	string strQuery = "start_experiment('" +
	  strExperimentName + "', " +
	  strFirstNodeName + ")";
	
	bool bSuccess;
	PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
      }
    }
    
    bool PluginKnowRob::addDesignator(CDesignator* cdDesig) {
      string strID = cdDesig->stringValue("_id");
      string strType = (cdDesig->type() == ACTION ? "ACTION" : (cdDesig->type() == LOCATION ? "LOCATION" : "OBJECT"));
      
      string strQuery = "cram_create_desig(" +
	string("'") + strType + "', " +
	"DESIGNATORINSTANCE"
	+ ")";
      
      bool bSuccess;
      PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
      
      if(bSuccess) {
	string strDesignatorInstance = pbBdgs["DESIGNATORINSTANCE"];
	m_mapDesignatorInstanceMapping[strID] = strDesignatorInstance;
      }
      
      return bSuccess;
    }
    
    PrologBindings PluginKnowRob::assertQuery(string strQuery, bool& bSuccess) {
      PrologBindings pbBdgs;
      
      try {
	pbBdgs = m_prlgProlog->once(strQuery);
	bSuccess = true;
	
	this->info("Query successful: " + strQuery);
	
	map<string, PrologValue> mapBdgs = pbBdgs;
	
	for(map<string, PrologValue>::iterator itBdg = mapBdgs.begin();
	    itBdg != mapBdgs.end();
	    itBdg++) {
	  string strName = (*itBdg).first;
	  string strContent = pbBdgs[strName];
	  
	  this->info("  " + strName + " = '" + strContent + "'");
	}
      } catch(PrologQueryProxy::QueryError qe) {
	this->warn("Query error: " + string(qe.what()));
	this->warn("While querying for: " + strQuery);
	bSuccess = false;
      }
      
      return pbBdgs;
    }
  }
  
  extern "C" plugins::PluginKnowRob* createInstance() {
    return new plugins::PluginKnowRob();
  }
  
  extern "C" void destroyInstance(plugins::PluginKnowRob* icDestroy) {
    delete icDestroy;
  }
}
