package pack_technical;

import pack_1.Launcher;
import pack_AI.AI_manager;
import pack_AI.AI_type;
import pack_boids.Boid_generic;
import pack_boids.Boid_standard;
import processing.core.PApplet;
import processing.core.PVector;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.*;

public class EnviromentalSimulation extends Thread {
    ArrayList<Boid_generic> defenders;
    ArrayList<Boid_generic> attackBoids;

    Tree MCT;

    AI_type simulator;
    PApplet parent;
    PatrollingScheme scheme;
    ArrayList<int[]> cords;

    FlockManager flock;
    double startTime = 0;
    int maxTreeDepth = 20;
    int actionCounter = 0;
    boolean treeReady = false;
    boolean dangerClose = false;

    CollisionHandler handler;

    public AI_type getSimulator() {
        return simulator;
    }


    public EnviromentalSimulation(int sns, int ans, int cns, double sw, double aw, double cw, String name, ArrayList<Boid_generic> defenders, PApplet parent, ArrayList<int[]> cords, ArrayList<Boid_generic> attackers, CollisionHandler handler) throws IOException {
        this.parent = parent;
        this.handler = handler;
        this.cords = cords;
        this.defenders = defenders;

        simulator = new AI_type(randFloat(AI_manager.neighbourhoodSeparation_lower_bound, AI_manager.neighbourhoodSeparation_upper_bound), 70, 70, 2.0, 1.2, 0.9f, 0.04f, "Simulator2000");

        defenders = copyTheStateOfAttackBoids(defenders);
        this.attackBoids = copyTheStateOfAttackBoids(attackers);

        this.flock = new FlockManager(parent, true, true);
        this.scheme = new PatrollingScheme(simulator.getWayPointForce());
        for (Boid_generic g : defenders) {
            g.setAi(simulator);
        }

        for (int[] cord : cords) {
            scheme.getWaypoints().add(new PVector(cord[0], cord[1]));
        }
        //FOLLOW THE SIMILLAR WAYPOINT AS DEFENDERS
        float shortestDistance = 3000;
        int counter = 0;
        int positionInTheList = 0;
        for (PVector checkpoint : scheme.getWaypoints()) {
            float distance = PVector.dist(defenders.get(0).getLocation(), checkpoint);
            counter++;
            if (distance < shortestDistance) {
                shortestDistance = distance;
                positionInTheList = counter;
            }
        }

        scheme.setup();

        for (int i = 0; i < positionInTheList + 1; i++) {
            if (!scheme.getIterator().hasNext()) {   // the ! is important
                scheme.setIterator(scheme.getWaypoints().iterator());
            }
            scheme.setCurrWaypoint(scheme.getIterator().next());
        }
        startTime = System.nanoTime();

        MCT = new Tree(new InnerSimulation(simulator, copyTheStateOfAttackBoids(defenders), cords, copyTheStateOfAttackBoids(attackBoids), handler, parent), maxTreeDepth);
        new Thread(this).start();
    }


    public void setAiToInnerSimulation(AI_type t) {
        MCT.root.simulation.setAii(t);
    }


    public boolean isSimulating() {
        return MCT.root.simulation.isSimulating();
    }


    public static float randFloat(float min, float max) {
        Random rand = new Random();
        float result = rand.nextFloat() * (max - min) + min;
        return result;
    }


    public void restartTheSimulation(ArrayList<Boid_generic> attackBoids, ArrayList<Boid_generic> defenders) {
        MCT.root.simulation.restartTheSimulation(attackBoids, defenders);
    }


    public void setSimulating(boolean k) {
        MCT.root.simulation.setSimulating(k);
    }


    public PVector reutrnTargetVecotr() {
        Node<InnerSimulation> bestSim = MCT.bestAvgVal();
        PVector bestVector = bestSim.simulation.MrLeandroVector;
        try {
            MCT.root = new Node(new InnerSimulation(simulator, copyTheStateOfAttackBoids(defenders), cords, copyTheStateOfAttackBoids(attackBoids), handler, parent), 0, "root", 0, 0);
            dangerClose = false;
            //bestSim.parent = null;
            //MCT.root = bestSim;
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.println("Node " + bestSim.name + ": " + bestSim.nodeSimValue + "   " + bestSim.rolloutValue + "   " + bestSim.avgEstimatedValue);
        System.out.println("Target Vector: " + bestVector + "\n");

        if(actionCounter > 10){
            System.gc();
            System.runFinalization();
            actionCounter = 0;
        }else{
            actionCounter++;
        }


        return bestVector;
    }


    public void updateBoids(ArrayList<Boid_generic> defenders, ArrayList<Boid_generic> attacker) {
        this.defenders = copyTheStateOfAttackBoids(defenders);
        this.attackBoids = copyTheStateOfAttackBoids(attacker);
    }


    public void run() {
        while (true) {
            try {
                Node<InnerSimulation> n = MCT.UCT(MCT.root);
                InnerSimulation s = n.simulation;
                InnerSimulation newSim = new InnerSimulation(simulator, s.copyTheStateOfAttackBoids(s.getSimulationClones()), s.cords, s.copyTheStateOfAttackBoids(s.getAttackBoids()), s.handler, s.parent);
                //newSim.restartTheSimulation(newSim.copyTheStateOfAttackBoids(attackBoids), newSim.copyTheStateOfAttackBoids(defenders));
                newSim.run1(n.children.size());
                System.out.println("Rollout avg val: " + newSim.avgReward);

                if(newSim.avgReward < 0){
                    dangerClose = true;
                }else{
                    dangerClose = false;
                }

                double simVal = 0;
                if (newSim.attackBoids.get(0).isHasFailed()) {
                    System.out.println("LOSING NODE");
                    simVal = -100 ;
                } else if (newSim.victory) {
                    System.out.println("WINNING NODE");
                    simVal = 1;
                } else {
                    System.out.println("EVEN NODE");
                    if(!dangerClose) {
                        simVal = 0.5 - (newSim.currentDistance / 6000);
                    }
                }

                String nodeName = n.name + "." + n.children.size();
                //System.out.println("ADDING> " + nodeName + "\nLocation> " + newSim.attackBoids.get(0).location + "\n");
                //System.out.println("UCT: " + n.uct + "\n");
                n.addChild(newSim, simVal, nodeName, newSim.avgReward);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }


    public ArrayList<Boid_generic> copyTheStateOfAttackBoids(ArrayList<Boid_generic> boids) {
        ArrayList<Boid_generic> boidListClone = new ArrayList<>();

        for (Boid_generic boid : boids) {
            Boid_generic bi = new Boid_standard(parent, boid.getLocation().x, boid.getLocation().y, 6, 10);
            bi.setAcceleration(boid.getAcceleration());
            bi.setVelocity(boid.getVelocity());
            bi.setLocation(boid.getLocation());
            boidListClone.add(bi);
        }
        return boidListClone;
    }
}
