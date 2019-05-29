package pathFinder;

import map.Coordinate;
import map.PathMap;

import java.util.*;

public class DijkstraPathFinder implements PathFinder {
    // The pathmap object
    private PathMap pathMap;
    // A hashmap containing the results from the Dijkstra's Calculations.
    // Gets a hashmap of coordinates and PathNodes from a specified starting
    // coordinate
    private Map<
       Coordinate,
       Map<Coordinate, PathNode>
   > dijkstraCalculations = new HashMap<>();

    public DijkstraPathFinder(PathMap map) {
        pathMap = map;
    }

    /**
     * Find all permutations of a list.
     *
     * @param paramList : What list do we want to get all permutations from
     * @param <E>       : What type of elements are in the list (IE, coordinates)
     * @return : A list containing all the possible permutations of the initial
     * list
     */
    private static <E> List<List<E>> recursivePermutations(List<E> paramList) {
        // If empty, return an empty list.
        if (paramList.size() == 0) {
            return new ArrayList<>();
        }

        // If the size is one, return a list with a list of a single element,
        // which is the only element passed to it. Recursive end condition.
        if (paramList.size() == 1) {
            final List<List<E>> returnLists = new ArrayList<>();
            returnLists.add(paramList);
            return returnLists;
        }

        // Otherwise (IE, paramList.size() > 1), recursively find the
        // permutations
        List<List<E>> returnLists = new ArrayList<>();
        for (E fixedItem : paramList) {
            // Select an element to keep fixed (IE, at the start of the list)
            // and get the rest of the combinations recursively
            List<E> smallerList = new ArrayList<>(paramList);
            smallerList.remove(fixedItem);
            ;
            for (List<E> subList : recursivePermutations(smallerList)) {
                // Add back the fixed element to the start of the array, and add
                // the permutation to the return values.
                subList.add(0, fixedItem);
                returnLists.add(subList);
            }
        }

        return returnLists;
    }

    /**
     * For a permutation, get the list of coordinates to the point
     *
     * @param permutation: A list of coordinates, which are either origin points
     *                     or waypoints. (Each coordinate must have a matching
     *                     dijkstraCalculation key)
     * @return : A list of coordinates from the permutation.
     */
    private List<Coordinate> getFromPermutation(List<Coordinate> permutation) {
        List<Coordinate> path = new ArrayList<>();

        for (int i = 0; i < permutation.size() - 1; i++) {
            // Get a start and end coordinate.
            Coordinate startCoord = permutation.get(i);
            Coordinate endCoord = permutation.get(i + 1);

            // Include the end only if it is the last set of two of them.
            final boolean inclusiveOfEnd = i == permutation.size() - 2;
            List<Coordinate> subPath = dijkstraCalculations.get(startCoord).get(endCoord).coordinatesTo(inclusiveOfEnd);

            path.addAll(subPath);
        }
        return path;
    }

    /**
     * Entry to the program.
     *
     * @return : A list of coordinates that is the path desired
     */
    @Override
    public List<Coordinate> findPath() {
        // Initialize the Dijkstra paths from origins and waypoints. Save the
        // data under dijkstraCalculations field.
        for (Coordinate coord : this.pathMap.originCells) {
            dijkstraCalculations.put(coord, initialisePathMap(coord));
        }
        for (Coordinate coord : this.pathMap.waypointCells) {
            dijkstraCalculations.put(coord, initialisePathMap(coord));
        }

        // Generate all permutations of the waypoint cells. (Can be none)
        List<
            List<Coordinate>
        > waypointPermutations = DijkstraPathFinder.recursivePermutations(
            this.pathMap.waypointCells
        );

        // Generate the permutations (incuding origins and destinations)
        List<List<Coordinate>> newPermutations = new ArrayList<>();
        for (Coordinate origin : this.pathMap.originCells) {
            for (Coordinate destination : this.pathMap.destCells) {
                if (waypointPermutations.size() == 0) {
                    // Generate at least combinations of the origins and
                    // destinations if there are no waypoint permutations.
                    List<Coordinate> newPermutation = new ArrayList<>();
                    newPermutations.add(newPermutation);

                    newPermutation.add(origin);
                    newPermutation.add(destination);
                }

                for (
                    List<Coordinate> waypointPermutation :
                    waypointPermutations
                ) {
                    // Generate the combinations with the waypoints.
                    List<Coordinate> newPermutation = new ArrayList<>(
                        waypointPermutation
                    );

                    newPermutations.add(newPermutation);

                    newPermutation.add(0, origin);
                    newPermutation.add(destination);
                }
            }
        }

        // Find the smallest weighted permutation (the answer we're after in all
        // of the cases)
        int smallestWeight = Integer.MAX_VALUE;
        List<Coordinate> smallestPermutation = newPermutations.get(0);

        // At this point, all the permutations will be setup
        permutationLoop:
        for (List<Coordinate> permutation : newPermutations) {
            int weight = 0;
            // Calculate the weight of the permutation
            // ie weight of 0 and 1, 1 and 2, ..., n-2 and n-1, and n-1 to n
            for (int i = 0; i < permutation.size() - 1; i++) {
                Coordinate startCoord = permutation.get(i);
                Coordinate endCoord = permutation.get(i + 1);
                weight += dijkstraCalculations.get(startCoord).get(endCoord).weight;
                // Early break, continue to next permutation if the current one
                // is too big
                if (weight >= smallestWeight) {
                    continue permutationLoop;
                }
            }

            // if it's the smallest, update accordingly.
            if (weight < smallestWeight) {
                smallestWeight = weight;
                smallestPermutation = permutation;
            }
        }

        // Return the list of coordinates for the point.
        return this.getFromPermutation(smallestPermutation);
    }

    @Override
    public int coordinatesExplored() {
        // TODO: Implement (optional)
        return 0;
    }

    /**
     * Generate the dijkstra's algorithm for a desired start point
     *
     * @param source : The starting coordinate
     * @return : A map of coordinates to PathNodes, which contain weight and
     * the pathTe the coordinate.
     */
    private Map<Coordinate, PathNode> initialisePathMap(Coordinate source) {
        // Using a priorityQueue
        PriorityQueue priorityQueue = new PriorityQueue();
        // Return weights for the current Dijkstra
        HashMap<Coordinate, PathNode> weights = new HashMap<>();

        // Add the origin in as 0 weight, and no path to it.
        Coordinate sourcePoint = new Coordinate(source.getRow(), source.getColumn());
        PathNode sourcePathNode = new PathNode(sourcePoint, 0, new ArrayList<>());
        weights.put(sourcePoint, sourcePathNode);

        // Add adjacent point to the queue. It handles duplicates and weight
        // differences.
        addAdjacentToQueue(priorityQueue, sourcePathNode);

        while (!priorityQueue.isEmpty()) {
            // Loop through the queue, adding new PathNodes as needed
            PathNode currentPathNode = priorityQueue.next();
            weights.put(currentPathNode.point, currentPathNode);

            addAdjacentToQueue(priorityQueue, currentPathNode);
        }

        return weights;
    }

    /**
     * Add adjacent points to a priorityQueue.
     *
     * @param priorityQueue  : The Queue to add to
     * @param sourcePathNode : The node to get the adjacent points from
     */
    private void addAdjacentToQueue(PriorityQueue priorityQueue, PathNode sourcePathNode) {
        for (PathNode adjacentPathNode : getAdjacentToPoint(sourcePathNode)) {
            priorityQueue.add(adjacentPathNode);
        }
    }

    /**
     * Returns all the adjacent points and their weights
     * Weight is calculated as terrain cost + prevNode weight.
     * When adding to the priority queue, an adjacent node may override any
     * current pathNode in it, if the weight is smaller
     *
     * @param pathNode : The PathNode that we want to find the adjacent
     *                 PathNodes to
     * @return A list of the adjacent PathNodes
     */
    private List<PathNode> getAdjacentToPoint(PathNode pathNode) {
        List<PathNode> adjacentPoints = new ArrayList<>();
        Coordinate point = pathNode.point;

        final int row = point.getRow();
        final int column = point.getColumn();

        // Get the previous weight of the node.
        final double prevWeight = pathNode.pathTo.size() > 0 ? pathNode.pathTo.get(pathNode.pathTo.size() - 1).weight : 0;

        // direction is an enum, see above for details.
        for (Direction direction : Direction.values()) {
            if (isInBounds(row, column, direction)) {
                Coordinate adj = new Coordinate(
                    row + direction.rowAdjustment,
                    column + direction.colAdjustment
                );
                adjacentPoints.add(
                    new PathNode(adj, getWeight(adj) + prevWeight, pathNode)
                );
            }
        }

        return adjacentPoints;
    }

    /**
     * See if the next probable point (row + rowAdj, col + colAdj) is in bounds
     * of the cell array
     *
     * @param row       : Starting row
     * @param column    : Starting col
     * @param direction : A direction that provides the adjustments.
     * @return : If it is in bounds of the PathMap cells.
     */
    private boolean isInBounds(int row, int column, Direction direction) {
        return row + direction.rowAdjustment >= 0
               && row + direction.rowAdjustment < this.pathMap.sizeR
               && column + direction.colAdjustment >= 0
               && column + direction.colAdjustment < this.pathMap.sizeC;
    }

    /**
     * Get the weight of the cell.
     *
     * @param point : The coordinate in question.
     * @return : infinity if impassable, else, the terrain cost.
     */
    private double getWeight(Coordinate point) {
        final Coordinate coordinate = pathMap.cells[point.getRow()][point.getColumn()];
        return !coordinate.getImpassable() ? coordinate.getTerrainCost() : Double.POSITIVE_INFINITY;
    }

    /**
     * All possible directions (and the appropriate adjustments) for finding
     * neighbouring cells from the PathMap
     */
    private enum Direction {
        RIGHT(0, 1),
        LEFT(0, -1),
        TOP(-1, 0),
        BOTTOM(1, 0);

        // These are the adjustments to get to that cell (IE, right means go one
        // col forwards)
        private final int rowAdjustment;
        private final int colAdjustment;

        Direction(int rowAdjustment, int colAdjustment) {
            this.rowAdjustment = rowAdjustment;
            this.colAdjustment = colAdjustment;
        }
    }

    /**
     * A priority queue.
     * Whenever next is called, it retrieves the PathNode with the smallest
     * total weight from the origin.
     */
    public class PriorityQueue {
        private List<PathNode> usedNodes;
        private List<PathNode> pathNodes;

        PriorityQueue() {
            this.pathNodes = new ArrayList<>();
            this.usedNodes = new ArrayList<>();
        }

        /**
         * Add another node to the priority queue. If it has already been in the
         * queue before (IE, in usedNodes), then ignore it. If it is already in
         * the queue, update the weight if it is smaller.
         *
         * @param newNode : The new node to add.
         */
        void add(PathNode newNode) {
            // Scan to see if it has already been used.
            for (PathNode pathNode : this.usedNodes) {
                if (pathNode.point.equals(newNode.point)) {
                    return;
                }
            }

            // Scan to see if it is already in the queue
            for (PathNode pathNode : this.pathNodes) {
                if (pathNode.point.equals(newNode.point)) {
                    // If it is in the queue, update it according to the
                    // smallest weight of the new node and the current node.
                    if (pathNode.weight > newNode.weight) {
                        this.pathNodes.remove(pathNode);
                        this.pathNodes.add(newNode);
                    }
                    return;
                }
            }

            // Add the node if not found in the used or current nodes.
            this.pathNodes.add(newNode);
        }

        /**
         * Get the next node with the smallest weight from the queue
         *
         * @return : The PathNode with the smallest weight in the queue
         */
        PathNode next() {
            // Set the default smallest
            PathNode smallest = this.pathNodes.get(0);

            // Loop to find the smallest weight
            for (PathNode pathNode : this.pathNodes) {
                if (pathNode.weight < smallest.weight) {
                    smallest = pathNode;
                }
            }

            // Remove from queue, add to used.
            this.pathNodes.remove(smallest);
            this.usedNodes.add(smallest);

            return smallest;
        }

        /**
         * Check if the queue is empty
         *
         * @return : true if the queue is empty
         */
        boolean isEmpty() {
            return this.pathNodes.isEmpty();
        }
    }

    /**
     * The PathNode class
     * <p>
     * Contains weight, coordinate, and the path to the point.
     */
    private class PathNode {
        Coordinate point; // The current place of it
        double weight; // The weight to get to the node
        List<PathNode> pathTo; // The path up to, not including, the node.

        PathNode(Coordinate point, double weight, List<PathNode> pathTo) {
            this.point = point;
            this.weight = weight;
            this.pathTo = pathTo;
        }

        /**
         * Constructor that also builds the previous path from the prevNode
         *
         * @param point:        Coordinate value
         * @param weight:       Weight value
         * @param prevPathNode: The previous pathNode. Builds the new pathTo
         */
        PathNode(Coordinate point, double weight, PathNode prevPathNode) {
            this.point = point;
            this.weight = weight;
            this.pathTo = new ArrayList<>(prevPathNode.pathTo != null ? prevPathNode.pathTo : new ArrayList<>());
            this.pathTo.add(prevPathNode);
        }

        /**
         * Return a list of the coordinates to the current node.
         *
         * @param inclusiveOfEnd : Do you include the last node (IE the
         *                       coordinates of the current point)
         * @return : The path to the point.
         */
        List<Coordinate> coordinatesTo(boolean inclusiveOfEnd) {
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
}
