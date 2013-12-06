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
	  
	  string strOWLClass = m_expOwl->owlClassForNode(ndNode);
	  string strTaskContextDescription = ndNode->title();
	  string strTimeStart = ndNode->metaInformation()->stringValue("time-start");
	  string strPreviousAction = "_";
	  
	  Node* ndPrevious = ndNode->previousNode();
	  if(ndPrevious) {
	    strPreviousAction = ndPrevious->metaInformation()->stringValue("action-instance");
	  }
	  
	  string strQuery = "cram_start_action(" +
	    strOWLClass + ", " +
	    strTaskContextDescription + ", " +
	    strTimeStart + ", " +
	    strPreviousAction + ", _)";
	  
	  cout << strQuery << endl;
	  
	  PrologBindings pbBdgs = m_prlgProlog->once(strQuery);
	  
	  // TODO(winkler): Interprete the pbBdgs binding for `?actin'
	  // here. Also, fill the missing information with OWL specific
	  // data from the symbolic log.
	  
	  string strActionInstance = ""; // Get this action instance
					 // from the prolog query
					 // result.
	  ndNode->metaInformation()->setValue("action-instance", strActionInstance);
	}
      } else if(evEvent.strEventName == "symbolic-end-context") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  string strActionInstance = ndNode->metaInformation()->stringValue("action-instance");
	  string strTimeEnd = ndNode->metaInformation()->stringValue("time-end");
	  
	  string strQuery = "cram_finish_action(" +
	    strActionInstance + ", " +
	    strTimeEnd + ")";
	  
	  cout << strQuery << endl;
	  
	  PrologBindings pbBdgs = m_prlgProlog->once(strQuery);
	  
	  // TODO(winkler): Fill the missing information with OWL
	  // specific data from the symbolic log.
	}
      }
    }
  }
  
  extern "C" plugins::PluginKnowRob* createInstance() {
    return new plugins::PluginKnowRob();
  }
  
  extern "C" void destroyInstance(plugins::PluginKnowRob* icDestroy) {
    delete icDestroy;
  }
}
