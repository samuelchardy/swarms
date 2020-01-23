package pack_technical;

public class Tree {
    Node<InnerSimulation> root;
    int maxTreeDepth;

    public Tree(InnerSimulation rootSim, int maxTreeDepth) {
        this.root = new Node<InnerSimulation>(rootSim, 0, "ROOT", 0);
        this.maxTreeDepth = maxTreeDepth;
    }

    public Node<InnerSimulation> UCT(Node<InnerSimulation> bestNode, Node<InnerSimulation> currentNode) {
        if (currentNode.children.size() > 0) {
            for (Node<InnerSimulation> child : currentNode.children) {
                Node<InnerSimulation> nextNode = UCT(bestNode, child);
                if ((nextNode.uct > bestNode.uct) && (nextNode.depth < maxTreeDepth) && (nextNode.children.size() < 360)) {
                    bestNode = nextNode;
                }
            }
        } else if (currentNode.uct > bestNode.uct) {
            bestNode = currentNode;
        }

        return bestNode;
    }

    public Node<InnerSimulation> bestAvgVal() {
        double bestNode = root.children.get(0).avgEstimatedValue;
        int bestNodePos = 0;
        for (int i=0; i<root.children.size()-1; i++) {
            if(root.children.get(i).avgEstimatedValue > bestNode){
                bestNode = root.children.get(i).avgEstimatedValue;
                bestNodePos = i;
            }
        }
        return root.children.get(bestNodePos);
    }

    public void trimTree(Node<InnerSimulation> newRoot) {
        this.root = newRoot;
    }

}