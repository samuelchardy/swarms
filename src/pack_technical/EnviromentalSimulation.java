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
    Thread enviroThread = new Thread(this);

    AI_type simulator;
    PApplet parent;
    PatrollingScheme scheme;
    ArrayList<int[]> cords;

    FlockManager flock;
    double startTime = 0;
    int counter = 0;
    boolean noWaitingThreads;
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

        MCT = new Tree(new InnerSimulation(simulator, copyTheStateOfAttackBoids(defenders), cords, copyTheStateOfAttackBoids(attackBoids), handler, parent));
        enviroThread.start();
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
        Node<InnerSimulation> bestSim = MCT.bestAvgVal(MCT.root, MCT.root);
        PVector bestVector = bestSim.simulation.MrLeandroVector;
        try {
            MCT.root = new Node(new InnerSimulation(simulator, copyTheStateOfAttackBoids(defenders), cords, copyTheStateOfAttackBoids(attackBoids), handler, parent), 0, "root");
        } catch (Exception e) {
        }

        System.out.println(bestVector);
        return bestVector;
    }

    public void updateBoids(ArrayList<Boid_generic> defenders, ArrayList<Boid_generic> attacker) {
        this.defenders = copyTheStateOfAttackBoids(defenders);
        this.attackBoids = copyTheStateOfAttackBoids(attacker);
        noWaitingThreads = false;
        try {
            Thread.sleep(5);
        } catch (Exception e) {
            e.printStackTrace();
        }

        new Thread(this).start();
    }

    public void run() {
        noWaitingThreads = true;
        while (noWaitingThreads) {
            try {
                Node<InnerSimulation> n = MCT.UCT(MCT.root, MCT.root);

                //System.out.println("EXPANDED NODE> " + n.name);

                InnerSimulation s = n.simulation;
                InnerSimulation newSim = new InnerSimulation(simulator, s.copyTheStateOfAttackBoids(s.getSimulationClones()), s.cords, s.copyTheStateOfAttackBoids(s.getAttackBoids()), s.handler, s.parent);
                newSim.restartTheSimulation(newSim.copyTheStateOfAttackBoids(attackBoids), newSim.copyTheStateOfAttackBoids(defenders));
                newSim.run1();

                double avgVal = 0;
                if (!newSim.attackBoids.get(0).isHasFailed()) {
                    counter++;
                    avgVal = 1 - Math.sin(newSim.theClosetDistance);
                    System.out.println(counter + "  " + avgVal);
                }

                String nodeName = n.name + "." + n.children.size();
                //System.out.println("\nADDING> " + nodeName + "\nLocation> " + newSim.attackBoids.get(0).location);
                n.addChild(newSim, avgVal, nodeName);

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
