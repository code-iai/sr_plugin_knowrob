#include <bs_plugin_knowrob/PluginKnowRob.h>


namespace beliefstate {
  namespace plugins {
    PluginKnowRob::PluginKnowRob() {
      m_prlgProlog = NULL;
      m_expOwl = NULL;
      
      this->addDependency("ros");
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
      
      m_prlgProlog = new Prolog("/knowrob");
      m_expOwl = new CExporterOwl();
      
      bool bInitOK = m_prlgProlog->waitForServer(ros::Duration(5));
      
      if(bInitOK) {
	// Plan node control events
	this->setSubscribedToEvent("symbolic-begin-context", true);
	this->setSubscribedToEvent("symbolic-end-context", true);
	this->setSubscribedToEvent("symbolic-set-subcontext", true);
	this->setSubscribedToEvent("symbolic-add-image", true);
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
	  
	  string strQuery = "cram_start_action(" +
	    strOWLClass + ", " +
	    "'" + strTaskContextDescription + "', " +
	    strTimeStart + ", " +
	    strPreviousAction + ", ACTIONINSTANCE)";
	  
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
	    string("'") + strActionInstanceChild + string("', ");
	  
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
	    string("'") + strFilename + string(")");
	  
	  bool bSuccess;
	  PrologBindings pbBdgs = this->assertQuery(strQuery, bSuccess);
	}
      }
    }
    
    PrologBindings PluginKnowRob::assertQuery(string strQuery, bool& bSuccess) {
      PrologBindings pbBdgs;
      
      try {
	pbBdgs = m_prlgProlog->once(strQuery);
	bSuccess = true;
	
	this->info("Query successful: " + strQuery);
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
