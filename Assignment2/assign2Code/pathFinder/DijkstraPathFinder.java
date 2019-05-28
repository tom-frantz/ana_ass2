package pathFinder;

import map.Coordinate;
import map.PathMap;

import java.util.*;

public class DijkstraPathFinder implements PathFinder {
    public class PriorityQueue {
        private List<PathNode> scanned;
        private List<PathNode> pathNodes;

        public PriorityQueue() {
            this.pathNodes = new ArrayList<>();
            this.scanned = new ArrayList<>();
        }

        public void add(PathNode newNode) {
            for (PathNode pathNode : this.scanned) {
                if (pathNode.point.equals(newNode.point)) {
                    return;
                }
            }

            for (PathNode pathNode : this.pathNodes) {
                if (pathNode.point.equals(newNode.point)) {
                    if (pathNode.weight > newNode.weight) {
                        this.pathNodes.remove(pathNode);
                        this.pathNodes.add(newNode);
                    }
                    return;
                }
            }

            this.pathNodes.add(newNode);
        }

        public PathNode next() {
            PathNode smallest = this.pathNodes.get(0);

            for (PathNode pathNode : this.pathNodes) {
                if (pathNode.weight < smallest.weight) {
                    smallest = pathNode;
                }
            }

            this.pathNodes.remove(smallest);
            this.scanned.add(smallest);
            return smallest;
        }

        public boolean isEmpty() {
            return this.pathNodes.isEmpty();
        }
    }

    private PathMap pathMap;

    private Map<
        Coordinate,
        Map<Coordinate, PathNode>
    > dijkstraCalculations = new HashMap<>();

    public static <E> List<List<E>> recursivePermutate(List<E> paramList) {
        if (paramList.size() == 0) {
            return new ArrayList<>();
        }

		if (paramList.size() == 1) {
			final List<List<E>> returnLists = new ArrayList<>();
			returnLists.add(paramList);
			return returnLists;

		}

		List<List<E>> returnLists = new ArrayList<>();
		for (E fixedItem : paramList) {

			List<E> smallerList = new ArrayList<>(paramList);
			smallerList.remove(fixedItem);
;
			for (List<E> subList : recursivePermutate(smallerList)) {
				subList.add(0, fixedItem);
				returnLists.add(subList);
			}
		}

		return returnLists;
	}

	public List<Coordinate> getFromPermutation(List<Coordinate> permutation) {
        List<Coordinate> path = new ArrayList<>();
        for (int i = 0; i < permutation.size() - 1; i++) {
            Coordinate startCoord = permutation.get(i);
            Coordinate endCoord = permutation.get(i + 1);
            List<Coordinate> subPath = dijkstraCalculations.get(startCoord).get(endCoord).coordinatesTo(i == permutation.size() - 2);

            path.addAll(subPath);
        }
        return path;
    }

    public DijkstraPathFinder(PathMap map) {
        pathMap = map;
    } // end of DijkstraPathFinder()

    @Override
    public List<Coordinate> findPath() {
        for (Coordinate coord : this.pathMap.originCells) {
            dijkstraCalculations.put(coord, initialisePathMap(coord));
        }
        for (Coordinate coord : this.pathMap.waypointCells) {
            dijkstraCalculations.put(coord, initialisePathMap(coord));
        }

        List<List<Coordinate>> permutations = DijkstraPathFinder.recursivePermutate(this.pathMap.waypointCells);
        List<List<Coordinate>> newPermutations = new ArrayList<>();
        for (Coordinate origin : this.pathMap.originCells) {
            for (Coordinate destination : this.pathMap.destCells) {
                if (permutations.size() == 0) {
                    List<Coordinate> newPermutation = new ArrayList<>();
                    newPermutations.add(newPermutation);

                    newPermutation.add(origin);
                    newPermutation.add(destination);
                }

                for (List<Coordinate> waypointPermutation : permutations) {
                    List<Coordinate> newPermutation = new ArrayList<>(waypointPermutation);
                    newPermutations.add(newPermutation);

                    newPermutation.add(0, origin);
                    newPermutation.add(destination);
                }
            }
        }

        int smallestWeight = Integer.MAX_VALUE;
        List<Coordinate> smallestPermutation = null;
        // At this point, all the permutations will be setup C:
        start: for (List<Coordinate> permutation : newPermutations) {
            System.out.println("STARTING NEXT PERMUTATION");
            int weight = 0;
            for (int i = 0; i < permutation.size() - 1; i++) {
                Coordinate startCoord = permutation.get(i);
                Coordinate endCoord = permutation.get(i + 1);
                weight += dijkstraCalculations.get(startCoord).get(endCoord).weight;
                if (weight >= smallestWeight) {
                    System.out.println("WEIGHT IS GREATER");
                    continue start;
                }
            }

            if (weight < smallestWeight) {
                smallestWeight = weight;
                smallestPermutation = permutation;
            }
        }

        return this.getFromPermutation(smallestPermutation);
    }


    @Override
    public int coordinatesExplored() {
        // TODO: Implement (optional)
        return 0;
    }

    private Map<Coordinate, PathNode> initialisePathMap(Coordinate source) {
        PriorityQueue scannedPoints = new PriorityQueue();

        HashMap<Coordinate, PathNode> weights = new HashMap<>();

        Coordinate sourcePoint = new Coordinate(source.getRow(), source.getColumn());
        PathNode sourcePathNode = new PathNode(sourcePoint, 0, new ArrayList<>());

        weights.put(sourcePoint, sourcePathNode);

        List<PathNode> adjacentPathNodes = getAdjacentToPoint(sourcePathNode);
        for (PathNode adjacentPathNode : adjacentPathNodes) {
            scannedPoints.add(adjacentPathNode);
        }

        while (!scannedPoints.isEmpty()) {
            PathNode currentPathNode = scannedPoints.next();
            adjacentPathNodes = getAdjacentToPoint(currentPathNode);
            weights.put(currentPathNode.point, currentPathNode);

            for (PathNode adjacentPathNode : adjacentPathNodes) {
                scannedPoints.add(adjacentPathNode);
            }
        }

        return weights;
    }

    /**
     * Returns all the adjacent points and their weights
     * Weight is calculated as terrain cost + prevNode weight.
     * When adding to the priority queue, an adjacent node may override any
     * current pathNode in it, if the weight is smaller
     *
     * @param pathNode: The PathNode that we want to find the adjacent PathNodes to
     * @return A list of the adjacent PathNodes
     */
    private List<PathNode> getAdjacentToPoint(PathNode pathNode) {
        List<PathNode> adjacentPoints = new ArrayList<>();
        Coordinate point = pathNode.point;

        final int row = point.getRow();
        final int column = point.getColumn();

        final double prevWeight = pathNode.pathTo.size() > 0 ? pathNode.pathTo.get(pathNode.pathTo.size() - 1).weight : 0;

        if (row > 0) {
            Coordinate adj = new Coordinate(row - 1, column);
            adjacentPoints.add(new PathNode(adj, getWeight(adj) + prevWeight, pathNode));
        }
        if (row < this.pathMap.sizeR - 1) {
            Coordinate adj = new Coordinate(row + 1, column);
            adjacentPoints.add(new PathNode(adj, getWeight(adj) + prevWeight, pathNode));
        }

        if (column > 0) {
            Coordinate adj = new Coordinate(row, column - 1);
            adjacentPoints.add(new PathNode(adj, getWeight(adj) + prevWeight, pathNode));
        }
        if (column < this.pathMap.sizeR - 1) {
            Coordinate adj = new Coordinate(row, column + 1);
            adjacentPoints.add(new PathNode(adj, getWeight(adj) + prevWeight, pathNode));
        }

        return adjacentPoints;
    }

    private double getWeight(Coordinate point) {
        final Coordinate coordinate = pathMap.cells[point.getRow()][point.getColumn()];
        return !coordinate.getImpassable() ? coordinate.getTerrainCost() : Double.POSITIVE_INFINITY;
    }

    private class PathNode {
        public Coordinate point; // The current place of it
        public double weight; // The weight to get to the node
        public List<PathNode> pathTo; // The path up to, not including, the node.

        // TODO Consider making pathTo un nullable.
        public PathNode (Coordinate point, double weight, List<PathNode> pathTo) {
            this.point = point;
            this.weight = weight;
            this.pathTo = pathTo;
        }

        public PathNode(Coordinate point, double weight, PathNode prevPathNode) {
            this.point = point;
            this.weight = weight;
            this.pathTo = new ArrayList<>(prevPathNode.pathTo != null ? prevPathNode.pathTo : new ArrayList<>());
            this.pathTo.add(prevPathNode);
        }

        public List<Coordinate> coordinatesTo() {
            return coordinatesTo(true);
        }

        public List<Coordinate> coordinatesTo(boolean inclusiveOfEnd) {
            ArrayList<Coordinate> coordinatesTo = new ArrayList<>();

            for (PathNode node : this.pathTo) {
                coordinatesTo.add(node.point);
            }

            if (inclusiveOfEnd) {
                coordinatesTo.add(this.point);
            }

            return coordinatesTo;
        }
    }

} // end of class DijsktraPathFinder
