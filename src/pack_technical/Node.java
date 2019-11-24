package pack_technical;

import java.util.List;
import java.util.LinkedList;
import java.util.ArrayList;
import java.lang.Math.*;

//import processing.core.PVector;

public class Node<InnerSimulation> {
    Node<InnerSimulation> parent;
    LinkedList<Node<InnerSimulation>> children;
    InnerSimulation simulation;

    int timesVisited = 2;
    double avgEstimatedValue = 0, nodeSimValue = 0;
    double uct = 0;
    String name = "Root";

    /**
     * Constructor of Node, assigns internal values and initialises storage for children.
     *
     * @param simulation
     */
    public Node(InnerSimulation simulation, double avgEstimatedValue, String name) {
        this.simulation = simulation;
        this.children = new LinkedList<Node<InnerSimulation>>();
        this.avgEstimatedValue = avgEstimatedValue;
        this.nodeSimValue = avgEstimatedValue;
        this.name = name;
        updateUCT();
    }

    /**
     * Adds a node to the list of children for the calling parent node.
     *
     * @param child
     * @return
     */
    public Node<InnerSimulation> addChild(InnerSimulation child, double avgEstimatedValue, String name) {
        Node<InnerSimulation> childNode = new Node<InnerSimulation>(child, avgEstimatedValue, name);
        childNode.parent = this;
        this.children.add(childNode);
        updateUCT();
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
        this.avgEstimatedValue = nodeSimValue;
        this.timesVisited++;
        if (avgEstimatedValue != 0) {
            for (Node<InnerSimulation> child : children) {
                this.avgEstimatedValue += (child.avgEstimatedValue / children.size());
            }
        }
        updateUCT();
    }


    //ACCESSOR

    /*
    public PVector getAction(){
        return simulation.MrLeandroVector;
    }
    */
}