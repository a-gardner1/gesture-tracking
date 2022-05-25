using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using eu.anorien.mhl;
using eu.anorien.mhl.generator;
using eu.anorien.mhl.pruner;
using org.apache.log4j;
using java.util;
using se.liu.isy.control.assignmentproblem;
using java.awt.geom;
using java.net;

namespace SimpleMHTTest
{
    class Tracker
    {
        private static Logger logger = Logger.getLogger("Tracker");
        private MHLService service;
        private Factory factory;
        private HypothesesManager hm;
        private SimpleWatcher watcher;
        private long time = 0;
        private long targetIdGen = 0;
        private int maxNumLeaves, maxDepth, timeUndetected, bestK;
        private double probUndetected, probNewTarget, probFalseAlarm;

        public Tracker(int maxNumLeaves, int maxDepth, int timeUndetected, int bestK, double probUndetected, double probNewTarget, double probFalseAlarm)
        {
            this.maxNumLeaves = maxNumLeaves;
            this.maxDepth = maxDepth;
            this.timeUndetected = timeUndetected;
            this.bestK = bestK;
            this.probUndetected = probUndetected;
            this.probNewTarget = probNewTarget;
            this.probFalseAlarm = probFalseAlarm;
            //var type = typeof(MHLService);
            //var types = AppDomain.CurrentDomain.GetAssemblies()
            //    .SelectMany(s => s.GetTypes())
            //    .Where(p => type.IsAssignableFrom(p) && !p.IsInterface && !p.IsAbstract)
            //    .Select(x => Activator.CreateInstance(x));
            ServiceLoader serviceLoader = ServiceLoader.load(java.lang.Class.forName("eu.anorien.mhl.MHLService"));
            //service = (eu.anorien.mhl.MHLService)types.GetEnumerator().Current;
            service = (MHLService) serviceLoader.iterator().next();
            factory = service.getFactory();
            hm = factory.newHypothesesManager();
            
            PrunerFactory prunerFactory = factory.getPrunerFactory();

            hm.setPruner(prunerFactory.newCompositePruner(new Pruner[]{
                    prunerFactory.newBestKPruner(maxNumLeaves),
                    prunerFactory.newTreeDepthPruner(maxDepth)}));

            watcher = new SimpleWatcher();
            hm.register(watcher);
        }

        public void newScan(List<Point2D> m)
        {
            time++;
            java.util.Map map = new java.util.HashMap();
            Set isolatedTargets = new HashSet(watcher.getFacts());

            createGroups(m, map, isolatedTargets);

            generateHypForGroups(map);

            generateHypIsolatedTargets(isolatedTargets);
        }

        class HypGenerator : HypothesesGenerator 
        {
            Factory factory;
            private long time, timeUndetected;
            private double probUndetected;
            public HypGenerator(Factory factory, long time, long timeUndetected, double probUndetected)
            {
                this.factory = factory;
                this.time = time;
                this.timeUndetected = timeUndetected;
                this.probUndetected = probUndetected;
            }

            public GeneratedHypotheses generate(Set set, Set provFacts)
            {
                List generatedHypothesisList = new ArrayList();
                if (provFacts.isEmpty()) {
                    generatedHypothesisList.add(factory.newGeneratedHypothesis(1, new HashSet(), new HashSet()));
                } else {
                    Set newFacts = new HashSet();
                    TargetFact target = (TargetFact) provFacts.iterator().next();
                    if (time - target.getLastDetection() <= timeUndetected) {
                        TargetFact targetUpdate = new TargetFact(target.getId(), target.getLastDetection(), target.getX() + target.getVelocityX(), target.getY() + target.getVelocityY(), target.getVelocityX(), target.getVelocityY());
                        newFacts.add(targetUpdate);
                    }
                    generatedHypothesisList.add(factory.newGeneratedHypothesis(probUndetected, new HashSet(), newFacts));
                }
                return factory.newGeneratedHypotheses(generatedHypothesisList);
            }
        }

        private void generateHypIsolatedTargets(java.util.Set isolatedTargets) {
            for(java.util.Iterator it = isolatedTargets.iterator(); it.hasNext();) {
                Fact fact = (Fact) it.next();
                Set reqFacts = new HashSet();
                reqFacts.add(fact);
                hm.generateHypotheses(new HypGenerator(factory, time, timeUndetected, probUndetected), new HashSet(), reqFacts);
            }
        }

        class HypGenerator2 : HypothesesGenerator 
        {
            Factory factory;
            private long time, timeUndetected, targetIdGen;
            private double probFalseAlarm, probNewTarget, probUndetected;
            private int bestK;
            Map.Entry entry;
            public HypGenerator2(Factory factory, long time, 
                long timeUndetected, double probFalseAlarm, 
                double probNewTarget, int bestK,
                long targetIdGen, double probUndetected,
                Map.Entry entry)
            {
                this.factory = factory;
                this.time = time;
                this.timeUndetected = timeUndetected;
                this.probFalseAlarm = probFalseAlarm;
                this.probNewTarget = probNewTarget;
                this.entry = entry;
                this.bestK = bestK;
                this.targetIdGen = targetIdGen;
                this.probUndetected = probUndetected;
            }

            public GeneratedHypotheses generate(Set set, Set provFacts)
            {
                List measurements = new ArrayList((Collection) entry.getKey());
                    List targets = new ArrayList(provFacts);
                    double[][] costMatrix = new double[measurements.size()][]; 
                    for (int i = 0; i < measurements.size(); i++) {
                        costMatrix[i] = new double[targets.size() + measurements.size() * 2];
                        Point2D measurement = (Point2D) measurements.get(i);
                        for (int j = 0; j < targets.size(); j++) {
                            costMatrix[i][j] = ((TargetFact) targets.get(j)).measurementProbability(measurement);
                        }
                        costMatrix[i][targets.size() + i] = probFalseAlarm;
                        costMatrix[i][ targets.size() + measurements.size() + i] = probNewTarget;
                    }
                    List generatedHypothesesList = new ArrayList();
                    MurtyAlgorithm.MurtyAlgorithmResult result = MurtyAlgorithm.solve(costMatrix, bestK);
                    for (int solution = 0; solution < result.getCustomer2Item().Length; solution++) {
                        int[] assignments = result.getCustomer2Item()[solution];
                        double hypProb = 1;
                        Set newFacts = new HashSet();
                        for (int measurementNumber = 0; measurementNumber < assignments.Length; measurementNumber++) {
                            int assignment = assignments[measurementNumber];
                            Point2D measurement = (Point2D) measurements.get(measurementNumber);
                            hypProb *= costMatrix[measurementNumber][assignment];
                            if (assignment < targets.size()) {
                                TargetFact target = (TargetFact) targets.get(assignment);
                                TargetFact targetUpdate = new TargetFact(target.getId(), time, measurement.getX(), measurement.getY(), measurement.getX() - target.getX(), measurement.getY() - target.getY());
                                newFacts.add(targetUpdate);
                            } else if (assignment < targets.size() + measurements.size()) {
                                // False Alarm
                            } else {
                                TargetFact newTarget = new TargetFact(
                                        targetIdGen++,
                                        time, measurement.getX(), measurement.getY(), 0, 0);
                                newFacts.add(newTarget);
                            }
                        }
                        int[] item2Customer = result.getItem2Customer()[solution];
                        for (int i = 0; i < targets.size(); i++) {
                            if (item2Customer[i] == -1) {
                                TargetFact target = (TargetFact) targets.get(i);
                                if (time - target.getLastDetection() <= timeUndetected) {
                                    TargetFact targetUpdate = new TargetFact(
                                            target.getId(),
                                            target.getLastDetection(),
                                            target.getX() + target.getVelocityX(),
                                            target.getY() + target.getVelocityY(),
                                            target.getVelocityX(),
                                            target.getVelocityY());
                                    newFacts.add(targetUpdate);
                                }
                                hypProb *= probUndetected;
                            }
                        }
                        generatedHypothesesList.add(factory.newGeneratedHypothesis(hypProb, new HashSet(), newFacts));
                    }
                    return factory.newGeneratedHypotheses(generatedHypothesesList);
            }
        }

    private void generateHypForGroups(Map map) {
        for (java.util.Iterator it = map.entrySet().iterator(); it.hasNext();) {
            Map.Entry entry = (Map.Entry) it.next();
            hm.generateHypotheses(new HypGenerator2(factory, time, 
                timeUndetected, probFalseAlarm, 
                probNewTarget, bestK, 
                targetIdGen, probUndetected, 
                entry), new HashSet(), (Set) entry.getValue());
        }
    }

    private void createGroups(List<Point2D> m, Map map, Set isolatedTargets) {
        {
            foreach(Point2D measurement in m) {
                HashSet targets = new HashSet();
                HashSet measurements = new HashSet();
                measurements.add(measurement);
                map.put(measurements, targets);
                for (java.util.Iterator it = watcher.getFacts().iterator(); it.hasNext();) {
                    Fact fact = (Fact) it.next();
                    TargetFact target = (TargetFact) fact;
                    if (target.measurementInGate(measurement)) {
                        targets.add(fact);
                        isolatedTargets.remove(fact);
                    }
                }
            }
            while (true) {
            outter: {}
                for (java.util.Iterator it = map.entrySet().iterator(); it.hasNext();) {
                    Map.Entry e1 = (Map.Entry) it.next();
                    for (java.util.Iterator it2 = map.entrySet().iterator(); it2.hasNext();) {
                        Map.Entry e2 = (Map.Entry)it2.next();
                        if (!e1.equals(e2)) {
                            if (interset((Set) e1.getValue(), (Set) e2.getValue())) {
                                HashSet newFacts = new HashSet();
                                newFacts.addAll((Set) e1.getValue());
                                newFacts.addAll((Set) e2.getValue());
                                HashSet newMeasurements = new HashSet();
                                newMeasurements.addAll((Set)e1.getKey());
                                newMeasurements.addAll((Set) e2.getKey());
                                map.put(newMeasurements, newFacts);
                                map.remove(e1.getKey());
                                map.remove(e2.getKey());
                                goto outter;
                            }
                        }
                    }
                }
                break;
            }
        }
    }

    private bool interset(Set s1, Set s2) {
        for (java.util.Iterator it = s2.iterator(); it.hasNext();) {
            if (s1.contains(it.next())) {
                return true;
            }
        }
        for (java.util.Iterator it = s1.iterator(); it.hasNext();) {
            if (s2.contains(it.next())) {
                return true;
            }
        }
        return false;
    }

    public Hypothesis getBestHypothesis() {
        return hm.getBestHypothesis();
    }
    }
}
