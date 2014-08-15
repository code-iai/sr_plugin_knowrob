#ifndef __PLUGIN_KNOWROB_H__
#define __PLUGIN_KNOWROB_H__


#define PLUGIN_CLASS PluginKnowRob


// System
#include <cstdlib>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <map>

// json_prolog
#include <json_prolog/prolog.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>
#include <plugins/owlexporter/CExporterOwl.h>


namespace beliefstate {
  namespace plugins {
    class PLUGIN_CLASS : public Plugin {
    private:
      CExporterOwl* m_expOwl;
      json_prolog::Prolog* m_prlgProlog;
      std::map<std::string, std::string> m_mapDesignatorInstanceMapping;
      bool m_bConnectionLess;
      
    public:
      PluginKnowRob();
      ~PluginKnowRob();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      
      bool addDesignator(CDesignator* cdDesig);
      json_prolog::PrologBindings assertQuery(std::string strQuery, bool& bSuccess);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_KNOWROB_H__ */
