#include <sr_plugin_knowrob/PluginKnowRob.h>


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_prlgProlog = NULL;
      m_expOwl = NULL;
      m_bConnectionLess = false;
      
      this->addDependency("ros");
      this->setDevelopmentPlugin(false);
      this->setPluginVersion("0.25b");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_prlgProlog) {
	delete m_prlgProlog;
      }
      
      if(m_expOwl) {
	delete m_expOwl;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      Designator* cdConfig = this->getIndividualConfig();
      
      std::string strJSONService = cdConfig->stringValue("json-service");
      if(strJSONService == "") {
	strJSONService = "/json_prolog";
      }
      
      this->info("Waiting for JSONProlog node '" + strJSONService + "'");
      m_prlgProlog = new json_prolog::Prolog(strJSONService);
      m_expOwl = new CExporterOwl();
      
      std::string strSemanticsDescriptorFile = cdConfig->stringValue("semantics-descriptor-file");
      
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
      
      m_bConnectionLess = cdConfig->floatValue("connection-less");
      
      bool bInitOK = false;
      bool bConnectionOK = false;
      
      if(m_bConnectionLess) {
	this->info("The KnowRob plugin is operating in connection-less mode.");
	
	bConnectionOK = true;
      } else {
	bConnectionOK = ros::ok();
      }
      
      if(bConnectionOK) {
	if(m_bConnectionLess) {
	  bInitOK = true;
	} else {
	  if(fWaitDuration == -1) {
	    bInitOK = m_prlgProlog->waitForServer();
	  } else {
	    bInitOK = m_prlgProlog->waitForServer(ros::Duration(fWaitDuration));
	  }
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
	  this->setSubscribedToEvent("symbolic-set-node-success", true);
	  
	  this->setOffersService("resolve-designator-knowrob-live-id", true);
	} else {
	  resInit.bSuccess = false;
	}
      } else {
	resInit.bSuccess = false;
      }
      
      return resInit;
    }
    
    Result PLUGIN_CLASS::deinit() {
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    Event PLUGIN_CLASS::consumeServiceEvent(ServiceEvent seEvent) {
      Event evReturn = this->Plugin::consumeServiceEvent(seEvent);
      
      if(seEvent.siServiceIdentifier == SI_REQUEST) {
	if(seEvent.strServiceName == "resolve-designator-knowrob-live-id") {
	  ServiceEvent seResponse = eventInResponseTo(seEvent);
	  Designator* cdRequest = seEvent.cdDesignator;
	  
	  seResponse.bPreserve = true;
	  Designator* cdResponse = new Designator();
	  cdResponse->setType(Designator::DesignatorType::ACTION);
	  cdResponse->setValue("knowrob-live-id", m_mapDesignatorInstanceMapping[cdRequest->stringValue("designator-id")]);
	  
	  seResponse.cdDesignator = cdResponse;
	  this->deployServiceEvent(seResponse);
	}
      }
      
      return evReturn;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "symbolic-begin-context") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  std::string strOWLClass = m_expOwl->owlClassForNode(ndNode, false, true);
	  std::string strTaskContextDescription = ndNode->title();
	  std::string strTimeStart = ndNode->metaInformation()->stringValue("time-start");
	  std::string strPreviousAction = "_";
	  
	  Node* ndPrevious = ndNode->previousNode();
	  if(ndPrevious) {
	    strPreviousAction = ndPrevious->metaInformation()->stringValue("action-instance");
	  }
	  
	  if(strPreviousAction == "") {
	    this->warn("The previous node exists but does not supply an ID. Assuming '_'. This is potentially problematic.");
	    strPreviousAction = "_";
	  }
	  
	  std::string strQuery = "cram_start_action(" +
	    strOWLClass + ", " +
	    "'" + strTaskContextDescription + "', " +
	    strTimeStart + ", " +
	    "'" + strPreviousAction + "', ACTIONINSTANCE)";
	  
	  bool bSuccess;
	  json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  
	  if(bSuccess) {
	    std::string strActionInstance = "_";
	    
	    if(!m_bConnectionLess) {
	      std::string strActionInstancePB = pbBdgs["ACTIONINSTANCE"];
	      strActionInstance = strActionInstancePB;
	    }
	    
	    ndNode->metaInformation()->setValue("action-instance", strActionInstance);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-end-context") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  std::string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	  std::string strTimeEnd = ndNode->metaInformation()->stringValue("time-end");
	  
	  std::string strQuery = "cram_finish_action(" +
	    std::string("'") + strActionInstance + std::string("', ") +
	    strTimeEnd + ")";
	  
	  bool bSuccess;
	  json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	}
      } else if(evEvent.strEventName == "symbolic-set-subcontext") {
	if(evEvent.lstNodes.size() > 1) {
	  Node* ndParent = evEvent.lstNodes.front();
	  evEvent.lstNodes.pop_front();
	  Node* ndChild = evEvent.lstNodes.front();
	  
	  if(ndParent && ndChild) {
	    std::string strActionInstanceParent = ndParent->metaInformation()->stringValue("action-instance");
	    std::string strActionInstanceChild = ndChild->metaInformation()->stringValue("action-instance");
	  
	    std::string strQuery = "cram_set_subaction(" +
	      std::string("'") + strActionInstanceParent + std::string("', ") +
	      std::string("'") + strActionInstanceChild + "')";
	  
	    bool bSuccess;
	    json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  } else {
	    if(!ndParent) {
	      this->warn("The former parent for `set-symbolic-subcontext` was not set. This probably means that this context is on top-level. Right now, no meta data individual is created here; you might want to consider this when your log-queries are not working.");
	    }
	  }
	}
      } else if(evEvent.strEventName == "symbolic-add-image") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  std::string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	  std::string strFilename = evEvent.cdDesignator->stringValue("filename");
	  
	  std::string strQuery = "cram_add_image_to_event(" +
	    std::string("'") + strActionInstance + std::string("', ") +
	    std::string("'") + strFilename + std::string("')");
	  
	  bool bSuccess;
	  json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	}
      } else if(evEvent.strEventName == "symbolic-equate-designators") {
	if(evEvent.cdDesignator) {
	  std::string strParentID = evEvent.cdDesignator->stringValue("parent-id");
	  std::string strChildID = evEvent.cdDesignator->stringValue("child-id");
	  std::string strEquationTime = evEvent.cdDesignator->stringValue("equation-time");
	  
	  if(strParentID != "" && strChildID != "") {
	    std::string strQuery = "cram_equate_designators(" +
	      std::string("'") + strParentID + std::string("', ") +
	      std::string("'") + strChildID + std::string("', ") +
	      strEquationTime + std::string(")");
	    
	    bool bSuccess;
	    json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  } else {
	    this->warn("Cannot equate designators '" + strParentID + "' and '" + strChildID + "'.");
	  }
	}
      } else if(evEvent.strEventName == "symbolic-add-failure") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    Node* ndNode = evEvent.lstNodes.front();
	    
	    std::string strCondition = evEvent.cdDesignator->stringValue("condition");
	    std::string strTimeFail = evEvent.cdDesignator->stringValue("time-failure");
	    std::string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	    
	    std::string strQuery = "cram_add_failure_to_action(" +
	      std::string("'") + strActionInstance + "', " +
	      "'" + m_expOwl->failureClassForCondition(strCondition) + "', " +
	      "'" + m_expOwl->owlEscapeString(strCondition) + "', " +
	      strTimeFail + ", " +
	      "FAILUREINSTANCE" +
	      ")";
	    
	    bool bSuccess;
	    json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-add-designator") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    Designator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	    std::string strID = cdDesig->stringValue("_id");
	    
	    Node* ndNode = evEvent.lstNodes.front();
	    std::string strOwlClass = m_expOwl->owlClassForNode(ndNode);
	    std::string strAnnotation = evEvent.strAnnotation;//cdDesig->stringValue("_annotation");
	    
	    if(m_mapDesignatorInstanceMapping.count(strID) == 0) {
	      this->warn("Adding designator that was not created previously. Its created implicitly now. You might want to look into this.");
	      this->addDesignator(cdDesig);
	    }
	    
	    std::string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	    std::string strDesignatorInstance = m_mapDesignatorInstanceMapping[strID];
	    
	    std::string strDesigPurpose = m_expOwl->resolveDesignatorAnnotationTagName(strAnnotation);
	    std::string strQuery = "cram_add_desig_to_action(" +
	      std::string("'") + strActionInstance + "', " +
	      "knowrob:" + strDesigPurpose + ", " +
	      "'" + strDesignatorInstance + "')";
	    
	    bool bSuccess;
	    json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-create-designator") {
	if(evEvent.cdDesignator) {
	  Designator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	  // Make sure this designator is not yet added.
	  if(m_mapDesignatorInstanceMapping.count(cdDesig->stringValue("_id")) == 0) {
	    // Its not in the map. So add it.
	    this->addDesignator(cdDesig);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-set-perception-request") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    Designator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	    std::string strID = cdDesig->stringValue("_id");
	    Node* ndNode = evEvent.lstNodes.front();
	    std::string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	    std::string strDesigID = m_mapDesignatorInstanceMapping[strID];
	    
	    std::string strQuery = "cram_set_perception_request(" +
	      std::string("'") + strActionInstance + "', " +
	      std::string("'") + strDesigID + "'"
	      + ")";
	    
	    bool bSuccess;
	    json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-set-perception-result") {
	if(evEvent.cdDesignator) {
	  if(evEvent.lstNodes.size() > 0) {
	    Designator* cdDesig = evEvent.cdDesignator; // NOTE(winkler): Convenience.
	    std::string strID = cdDesig->stringValue("_id");
	    Node* ndNode = evEvent.lstNodes.front();
	    std::string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	    std::string strDesigID = m_mapDesignatorInstanceMapping[strID];
	    
	    std::string strQuery = "cram_set_perception_result(" +
	      std::string("'") + strActionInstance + "', " +
	      std::string("'") + strDesigID + "'"
	      + ")";
	    
	    bool bSuccess;
	    json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-set-node-success") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndSet = evEvent.lstNodes.front();
	  bool bTaskSuccess = ndSet->success();
	  
	  std::string strActionInstance = ndSet->metaInformation()->stringValue("action-instance");
	  
	  std::string strQuery = "cram_set_success(" + strActionInstance + ", " + std::string(bTaskSuccess ? "true" : "false") + ")";
	}
      }
    }
    
    bool PLUGIN_CLASS::addDesignator(Designator* cdDesig) {
      std::string strID = cdDesig->stringValue("_id");
      std::string strType = "http://knowrob.org/kb/knowrob.owl#";
      
      switch(cdDesig->type()) {
      case Designator::DesignatorType::ACTION:
	strType += "CRAMActionDesignator";
	break;
	
      case Designator::DesignatorType::LOCATION:
	strType += "CRAMLocationDesignator";
	break;
	
      case Designator::DesignatorType::OBJECT:
	strType += "CRAMObjectDesignator";
	break;
	
      default:
	strType += "CRAMDesignator";
	break;
      }
      
      std::string strQuery = "cram_create_desig(" +
	std::string("'") + strType + "', " +
	"'http://knowrob.org/kb/cram_log.owl#" + strID + "')";
      
      bool bSuccess;
      json_prolog::PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
      
      if(bSuccess) {
	//string strDesignatorInstance = pbBdgs["DESIGNATORINSTANCE"];
	m_mapDesignatorInstanceMapping[strID] = "http://knowrob.org/kb/cram_log.owl#" + strID;
      }
      
      return bSuccess;
    }
    
    json_prolog::PrologBindings PLUGIN_CLASS::assertQuery(std::string strQuery, bool& bSuccess) {
      json_prolog::PrologBindings pbBdgs;
      
      if(m_bConnectionLess) {
	this->info("Connectionless query: " + strQuery);
	bSuccess = true;
      } else {
	try {
	  pbBdgs = m_prlgProlog->once(strQuery);
	  bSuccess = true;
	  
	  this->info("Query successful: " + strQuery);
	  
	  std::map<std::string, json_prolog::PrologValue> mapBdgs = pbBdgs;
	  
	  for(std::pair<std::string, json_prolog::PrologValue> prBinding : mapBdgs) {
	    std::string strName = prBinding.first;
	    std::string strContent = pbBdgs[strName];
	    
	    this->info("  " + strName + " = '" + strContent + "'");
	  }
	} catch(json_prolog::PrologQueryProxy::QueryError qe) {
	  this->warn("Query error: " + std::string(qe.what()));
	  this->warn("While querying for: " + strQuery);
	  bSuccess = false;
	}
      }
      
      return pbBdgs;
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
