package pack_technical;

import java.util.LinkedList;


public class Node<InnerSimulation> {
    Node<InnerSimulation> parent;
    LinkedList<Node<InnerSimulation>> children;
    InnerSimulation simulation;

    int timesVisited = 1, depth;
    double avgEstimatedValue = 0, nodeSimValue = 0, rolloutValue;
    double uct = 0;
    String name = "Root";

    /**
     * Constructor of Node, assigns internal values and initialises storage for children.
     *
     * @param simulation
     */
    public Node(InnerSimulation simulation, double simulationValue, String name, int depth, double rolloutValue) {
        this.simulation = simulation;
        this.children = new LinkedList<Node<InnerSimulation>>();
        this.nodeSimValue = simulationValue;
        this.name = name;
        this.depth = depth;
        this.rolloutValue = rolloutValue;
    }

    /**
     * Adds a node to the list of children for the calling parent node.
     *
     * @param child
     * @return
     */
    public Node<InnerSimulation> addChild(InnerSimulation child, double simulationValue, String name, double cRolloutValue) {
        Node<InnerSimulation> childNode = new Node<InnerSimulation>(child, simulationValue, name, this.depth+1, cRolloutValue);
        childNode.parent = this;
        this.children.add(childNode);
        childNode.backPropagate();
        return childNode;
    }

    public void updateUCT() {
        if (parent != null) {
            this.uct = calcUCT(parent.timesVisited);
            parent.backPropagate();
        } else {
            this.uct = calcUCT(this.timesVisited);
        }

        /*
        for(Node<InnerSimulation> child : children){
            child.uct = child.avgEstimatedValue + (1.414 * (Math.sqrt(2 * Math.log(this.timesVisited) / (child.timesVisited))));
        }
         */
    }

    public double calcUCT(int parentVisits) {
        if (parent != null) {
            return this.avgEstimatedValue + (1.414 * (Math.sqrt(2 * Math.log(parentVisits+1) / (this.timesVisited))));
        } else {
            return this.avgEstimatedValue + (1.414 * (Math.sqrt(2 * Math.log(parentVisits+1) / (this.timesVisited))));
        }
    }

    /**
     * Updates the stats of all older generation nodes (father/ grandfather etc) via recursion.
     */
    public void backPropagate() {
        this.avgEstimatedValue = rolloutValue;
        this.timesVisited++;
        if(children.size() > 0) {
            for (Node<InnerSimulation> child : children) {
                this.avgEstimatedValue += (child.avgEstimatedValue / children.size());
            }
        }else{
            this.avgEstimatedValue = nodeSimValue;
        }

        updateUCT();
    }

}