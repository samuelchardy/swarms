package pack_technical;

import processing.core.PVector;

public class Tree {
    Node<InnerSimulation> root;
    int maxTreeDepth;
    int maxNodeChildren = 12;

    public Tree(InnerSimulation rootSim, int maxTreeDepth) {
        this.root = new Node<InnerSimulation>(rootSim, 0, "ROOT", 0, 0);
        this.maxTreeDepth = maxTreeDepth;
    }

    public Node<InnerSimulation> UCT(Node<InnerSimulation> currentNode) {
        if(currentNode.children.size() < maxNodeChildren){
            return currentNode;
        }

        while(true){
            if(currentNode.children.size() < maxNodeChildren){
                return currentNode;
            }

            Node<InnerSimulation> bestNode = null;
            for(Node<InnerSimulation> child : currentNode.children){
                //System.out.println(child.name + "  " + child.uct);
                if(bestNode == null){
                    bestNode = child;
                }else if((bestNode.uct < child.uct) && (child.depth < maxTreeDepth + root.depth) && (child.nodeSimValue != -1) && (child.nodeSimValue != 1)){
                    bestNode = child;
                }
            }
            currentNode = bestNode;
            //System.out.println("BEST: " + bestNode.name + "  " + bestNode.uct);
        }
    }


    public Node<InnerSimulation> bestAvgVal() {
        if(root.children.size() == 0){
            return root;
        }
        double bestNode = root.children.get(0).avgEstimatedValue;
        int bestNodePos = 0;
        for (int i=0; i<root.children.size()-1; i++){
            if(root.children.get(i).avgEstimatedValue > bestNode){
                bestNode = root.children.get(i).avgEstimatedValue;
                bestNodePos = i;
            }
            if(root.children.get(i).nodeSimValue >= 1){
                return root.children.get(i);
            }
        }
        return root.children.get(bestNodePos);
    }
}