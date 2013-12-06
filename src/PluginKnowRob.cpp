#include <bs_plugin_knowrob/PluginKnowRob.h>


namespace beliefstate {
  namespace plugins {
    PluginKnowRob::PluginKnowRob() {
      m_prlgProlog = NULL;
      
      this->addDependency("ros");
    }
    
    PluginKnowRob::~PluginKnowRob() {
      if(m_prlgProlog) {
	delete m_prlgProlog;
      }
    }
    
    Result PluginKnowRob::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      m_prlgProlog = new Prolog();
      
      if(m_prlgProlog->waitForServer(ros::Duration(5))) {
	// Plan node control events
	this->setSubscribedToEvent("begin-context", true);
	this->setSubscribedToEvent("end-context", true);
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
      if(evEvent.strEventName == "begin-context") {
	string strOWLClass = "";
	string strTaskContextDescription = "";
	string strTimeStart = "";
	string strPreviousAction = "";
	
	PrologBindings pbBdgs = m_prlgProlog->once("cram_start_action(" +
						   strOWLClass + ", " +
						   strTaskContextDescription + ", " +
						   strTimeStart + ", " +
						   strPreviousAction + ", ?actin)");
	
	// TODO(winkler): Interprete the pbBdgs binding for `?actin'
	// here. Also, fill the missing information with OWL specific
	// data from the symbolic log.
      } else if(evEvent.strEventName == "end-context") {
	string strActionInstance = "";
	string strTimeEnd = "";
	
	PrologBindings pbBdgs = m_prlgProlog->once("cram_finish_action(" +
						   strActionInstance + ", " +
						   strTimeEnd + ")");
	
	// TODO(winkler): Fill the missing information with OWL
	// specific data from the symbolic log.
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
