using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using org.apache.log4j;
using System.Collections.ObjectModel;
using java.util;
using eu.anorien.mhl;


namespace SimpleMHTTest
{
    class SimpleWatcher : eu.anorien.mhl.Watcher
    {
        private static org.apache.log4j.Logger logger = org.apache.log4j.Logger.getLogger("SimpleWatcher");
        private Set facts = new HashSet();

        public void newFact(eu.anorien.mhl.Fact fact)
        {
            facts.add(fact);
        }

        public void newFacts(Collection clctn)
        {
            for(java.util.Iterator it = clctn.iterator(); it.hasNext();)
            {
                newFact((Fact) it.next());
            }
        }

        public void removedFact(eu.anorien.mhl.Fact fact)
        {
            facts.remove(fact);
        }

        public void removedFacts(Collection clctn)
        {
            for (java.util.Iterator it = clctn.iterator(); it.hasNext(); )
            {
                removedFact((Fact)it.next());
            }
        }

        public void newEvent(eu.anorien.mhl.Event eve)
        {
        }

        public void newEvents(Collection clctn)
        {
        }

        public void removedEvent(eu.anorien.mhl.Event eve)
        {
        }

        public void removedEvents(Collection clctn)
        {
        }

        public void confirmedEvent(eu.anorien.mhl.Event eve)
        {
        }

        public void confirmedEvents(Collection clctn)
        {
        }

        public void bestHypothesis(eu.anorien.mhl.Hypothesis hpths)
        {
        }

        public Set getFacts()
        {
            return facts;
        }
    }
}
