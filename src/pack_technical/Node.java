package pack_technical;

import java.util.LinkedList;


public class Node<InnerSimulation> {
    Node<InnerSimulation> parent;
    LinkedList<Node<InnerSimulation>> children;
    InnerSimulation simulation;

    int timesVisited = 2, depth;
    double avgEstimatedValue = 0, nodeSimValue = 0;
    double uct = 0;
    String name = "Root";

    /**
     * Constructor of Node, assigns internal values and initialises storage for children.
     *
     * @param simulation
     */
    public Node(InnerSimulation simulation, double simulationValue, String name, int depth) {
        this.simulation = simulation;
        this.children = new LinkedList<Node<InnerSimulation>>();
        this.nodeSimValue = simulationValue;
        this.name = name;
        this.depth = depth;
    }

    /**
     * Adds a node to the list of children for the calling parent node.
     *
     * @param child
     * @return
     */
    public Node<InnerSimulation> addChild(InnerSimulation child, double simulationValue, String name) {
        Node<InnerSimulation> childNode = new Node<InnerSimulation>(child, simulationValue, name, this.depth+1);
        childNode.parent = this;
        this.children.add(childNode);
        backPropagate();
        return childNode;
    }

    public void updateUCT() {
        if (parent != null) {
            this.uct = calcUCT(parent.timesVisited);
            parent.backPropagate();
        } else {
            this.uct = calcUCT(this.timesVisited);
        }
    }

    public double calcUCT(int parentVisits) {
        if (parent != null) {
            return this.avgEstimatedValue + (1.414 * Math.sqrt(2 * Math.log(parentVisits) * (this.timesVisited / this.parent.timesVisited)));
        } else {
            return this.avgEstimatedValue + (1.414 * Math.sqrt(2 * Math.log(parentVisits) * (this.timesVisited / this.timesVisited + 1)));
        }
    }

    /**
     * Updates the stats of all older generation nodes (father/ grandfather etc) via recursion.
     */
    public void backPropagate() {
        this.avgEstimatedValue = 0;
        this.timesVisited++;
        for (Node<InnerSimulation> child : children) {
            this.avgEstimatedValue += (child.nodeSimValue / children.size());
        }

        updateUCT();

    }

}