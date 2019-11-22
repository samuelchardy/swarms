package pack_technical;

public class Tree
{
    Node<InnerSimulation> root;

    public Tree(InnerSimulation rootSim) {
        this.root = new Node<InnerSimulation>(rootSim, 0, "ROOT");
    }

    public Node<InnerSimulation> UCT(Node<InnerSimulation> bestNode, Node<InnerSimulation> currentNode){
        if(currentNode.parent != null) {
            currentNode.updateUCT();
            //System.out.println("CURRENT NODE> " + currentNode.name + "\nagv  " + currentNode.avgEstimatedValue + "\nuct  " + currentNode.uct + "\n     " + currentNode.parent.timesVisited + "\n     " + currentNode.timesVisited);
        }

        if(currentNode.children.size() > 0){
            for(Node<InnerSimulation> child : currentNode.children){
                Node<InnerSimulation> nextNode = UCT(bestNode, child);
                if(nextNode.uct > bestNode.uct){
                    bestNode = nextNode;
                }
            }
        }else if(currentNode.uct > bestNode.uct){
            bestNode = currentNode;
        }

        return bestNode;
    }

    public Node<InnerSimulation> bestAvgVal(Node<InnerSimulation> bestNode, Node<InnerSimulation> currentNode){
        if(currentNode.children.size() > 0){
            for(Node<InnerSimulation> child : currentNode.children){
                Node<InnerSimulation> nextNode = UCT(bestNode, child);
                if(nextNode.avgEstimatedValue > bestNode.avgEstimatedValue){
                    bestNode = nextNode;
                }
            }
        }else{
            if(currentNode.avgEstimatedValue > bestNode.avgEstimatedValue){
                bestNode = currentNode;
            }
        }
        return bestNode;
    }

}