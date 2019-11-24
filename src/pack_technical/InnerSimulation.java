package pack_technical;

import pack_AI.AI_type;
import pack_boids.Boid_generic;
import pack_boids.Boid_standard;
import processing.core.PApplet;
import processing.core.PVector;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

public class InnerSimulation  {
    ArrayList<Boid_generic> attackBoids;
    ArrayList<Boid_generic> SimulationClones;

    public void setAii(AI_type ai) {
        this.ai = ai;
    }

    AI_type ai;
    private int tick =0;
    PApplet parent;
    CollisionHandler handler;
    ArrayList<int[]> cords ;
    ArrayList<int[]> historyOfMovement = new ArrayList<>();
    PatrollingScheme scheme ;

    Integer nextWaypoint;
    Random randG = new Random();
    PVector targetVector = new PVector(0,0);
    PVector MrLeandroVector;

    float theClosetDistance;

    public boolean isSimulating() {
        return simulating;
    }

    public void setSimulating(boolean simulating) {
        this.simulating = simulating;
    }

    boolean simulating=true;

    public void createSimulationsAndRandomVectors(ArrayList<Boid_generic> attackBoids){
        float rand = randG.nextFloat() * 1;
        float rand2 = randG.nextFloat() * 1;
        MrLeandroVector = new PVector(-1+2*rand, -1+2*rand2);
        MrLeandroVector.setMag(0.1f);
    }
    public void restartTheSimulation(ArrayList<Boid_generic> attackBoidss,ArrayList<Boid_generic> defenders ) {
        attackBoids.clear();
        SimulationClones.clear();
        this.attackBoids=copyTheStateOfAttackBoids(attackBoidss);
        this.SimulationClones = copyTheStateOfAttackBoids(defenders);
        scheme.setWaypointforce(ai.getWayPointForce());
        for(Boid_generic g : SimulationClones){
            g.setAi(ai);
        }
        scheme.restartIterator();

        System.out.println(attackBoidss.get(0).getLocation());

        float shortestDistance = 3000;
        int counter = 0;
        int positionInTheList = 0;
        float shortestVectorAngle=0;
        float nextToShortestVectorAngle=0;

        for(int i=0;i<scheme.getWaypoints().size();i++) {
            PVector checkpoint = scheme.getWaypoints().get(i);
            PVector nextCheckPoint = scheme.getWaypoints().get((i+1)%scheme.getWaypoints().size());
            float distance = PVector.dist(SimulationClones.get(0).getLocation(), checkpoint);

            if (distance < shortestDistance) {
                shortestDistance = distance;
                positionInTheList = counter;
                shortestVectorAngle = PVector.angleBetween(SimulationClones.get(0).getLocation(), checkpoint);
                nextToShortestVectorAngle = PVector.angleBetween(SimulationClones.get(0).getLocation(), nextCheckPoint);
            }
            counter++;
        }

        if (shortestVectorAngle < nextToShortestVectorAngle) {
            nextWaypoint = positionInTheList;
        }
        else{
            nextWaypoint = (positionInTheList + 1) % scheme.getWaypoints().size();
        }

        scheme.currentPosition = nextWaypoint;
        createSimulationsAndRandomVectors(attackBoids);
    }


    public InnerSimulation(AI_type ai, ArrayList<Boid_generic> defenders, ArrayList<int[]> cords, ArrayList<Boid_generic> attackers,CollisionHandler handler,PApplet parent) throws IOException {
        this.ai = ai;
        this.parent=parent;
        this.cords= new ArrayList<>(cords);
        this.parent=parent;
        this.attackBoids=copyTheStateOfAttackBoids(attackers);
        this.SimulationClones=copyTheStateOfAttackBoids(defenders);
        this.handler=handler;
        scheme = new PatrollingScheme(ai.getWayPointForce());

        for(int[] cord : cords){
            scheme.getWaypoints().add(new PVector(cord[0],cord[1]));
        }

        //FOLLOW THE SIMILLAR WAYPOINT AS DEFENDERS
        float shortestDistance = 3000;
        int counter = 0;
        int positionInTheList = 0;
        float shortestVectorAngle=0;
        float nextToShortestVectorAngle=0;
        for(int i=0;i<scheme.getWaypoints().size();i++) {
            PVector checkpoint = scheme.getWaypoints().get(i);
            PVector nextCheckPoint = scheme.getWaypoints().get((i+1)%scheme.getWaypoints().size());
            float distance = PVector.dist(SimulationClones.get(0).getLocation(), checkpoint);

            if (distance < shortestDistance) {
                shortestDistance = distance;
                positionInTheList = counter;
                shortestVectorAngle = PVector.angleBetween(SimulationClones.get(0).getLocation(), checkpoint);
                nextToShortestVectorAngle = PVector.angleBetween(SimulationClones.get(0).getLocation(), nextCheckPoint);
            }
            counter++;
        }

        if (shortestVectorAngle < nextToShortestVectorAngle) {
            nextWaypoint = positionInTheList;
        }else{
            nextWaypoint = (positionInTheList + 1) % scheme.getWaypoints().size();
        }

        scheme.currentPosition = nextWaypoint;
        createSimulationsAndRandomVectors(attackBoids);
    }

    public PVector reutrnTargetVecotr(){
        return targetVector;
    }

    public void run1() throws IOException {
        if (simulating) {
            boolean willContinueSimulation = true;
            tick++;
            PVector sumOfMassCentres = new PVector(0, 0);
            int counter = 0;
            PVector theClosest = new PVector(0,0);
            theClosetDistance = 2000;
            float distance = 150;
            boolean CheckVector = false ;

            PVector acceleration = attackBoids.get(0).getAcceleration();
            PVector velocity = attackBoids.get(0).getVelocity();
            PVector location = attackBoids.get(0).getLocation();

            for (Boid_generic b1 : SimulationClones) {
                b1.move(SimulationClones);
                b1.update();
                if (Math.abs(PVector.dist(b1.getLocation(), location)) < 16) {  // was 3
                    attackBoids.get(0).setHasFailed(true);
                }
            }

            if((PVector.dist(location,new PVector(550,500))<=10 || PVector.dist(attackBoids.get(0).getLocation(),location)>=distance /*location.x-50<=0*/) && !attackBoids.get(0).isHasFailed()){
                willContinueSimulation = false;
            }
            velocity.limit(1);
            location.add(velocity.add(acceleration.add(MrLeandroVector)));
            acceleration.mult(0);

            float currentDistance = Math.abs(PVector.dist(location,new PVector(550,500)));
            if (currentDistance < theClosetDistance && !attackBoids.get(0).isHasFailed()) {
                theClosest = MrLeandroVector;
                theClosetDistance = currentDistance;
            }
            if(!attackBoids.get(0).isHasFailed())
                CheckVector = true;

            if(CheckVector) {
                if(!willContinueSimulation)
                    targetVector = theClosest;
            } else {
                willContinueSimulation = false;
            }

            if (!willContinueSimulation)
                simulating = false;

            if (simulating) {
                for (Boid_generic b : SimulationClones) {
                    PVector accelerationB = b.getAcceleration();
                    PVector velocityB = b.getVelocity();
                    PVector locationB = b.getLocation();
                    b.run(SimulationClones, true, true);

                    velocityB.limit(1);
                    locationB.add(velocityB.add(accelerationB.add(scheme.patrol(b.getLocation(), b)/*patrolling.patrol(be.getLocation(),be)*/)));
                    accelerationB.mult(0);

                    sumOfMassCentres = PVector.add(sumOfMassCentres, b.getLocation());
                    counter++;
                }

                PVector mean = PVector.div(sumOfMassCentres, counter);
                if (tick % 10 == 0) {
                    historyOfMovement.add(new int[]{(int) mean.x + 50, (int) mean.y});
                }
            }
        }
    }

    public ArrayList<Boid_generic> copyTheStateOfAttackBoids(ArrayList<Boid_generic> boids) {
        ArrayList<Boid_generic> boidListClone = new ArrayList<>();

        for(Boid_generic boid : boids){
            Boid_generic bi = new Boid_standard(parent,boid.getLocation().x,boid.getLocation().y,6,10);
            bi.setAi(ai);
            bi.setAcceleration(boid.getAcceleration());
            bi.setVelocity(boid.getVelocity());
            bi.setLocation(boid.getLocation());
            boidListClone.add(bi);
        }
        return boidListClone;
    }
}