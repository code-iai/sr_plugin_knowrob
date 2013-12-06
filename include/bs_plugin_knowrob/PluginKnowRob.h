#ifndef __PLUGIN_KNOWROB_H__
#define __PLUGIN_KNOWROB_H__


// System
#include <cstdlib>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

// json_prolog
#include <json_prolog/prolog.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>
#include <plugins/owlexporter/CExporterOwl.h>

using namespace std;
using namespace json_prolog;


namespace beliefstate {
  namespace plugins {
    class PluginKnowRob : public Plugin {
    private:
      CExporterOwl* m_expOwl;
      Prolog* m_prlgProlog;
      
    public:
      PluginKnowRob();
      ~PluginKnowRob();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      
      PrologBindings assertQuery(string strQuery, bool& bSuccess);
    };
  }
  
  extern "C" plugins::PluginKnowRob* createInstance();
  extern "C" void destroyInstance(plugins::PluginKnowRob* icDestroy);
}


#endif /* __PLUGIN_KNOWROB_H__ */
