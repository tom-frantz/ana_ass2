package pathFinder;

import map.Coordinate;
import map.PathMap;

import java.util.*;

public class DijkstraPathFinder implements PathFinder
{
    private final int INVALID_POINT = -1;
    private PathMap pathMap;

    private PathNode[][][] paths;
    // stores the data from djikstra's algorithm for each origin and waypoint
    // first index refers to which cell djikstra's algorithm is applied to
    // first being the origin maps, the rest being waypoints
    // second index refers to the row
    // third index refers to the column

    // index of the paths referring to the source which djikstra's algorithm is being performed on
    private int checkingIndex;


    public DijkstraPathFinder(PathMap map) {
        pathMap = map;
    } // end of DijkstraPathFinder()



    @Override
    public List<Coordinate> findPath() {
        // You can replace this with your favourite list, but note it must be a
        // list type
//        List<Coordinate> path = new ArrayList<Coordinate>();

        // Below creates the data from djikstra's algorithm for each origin and waypoint
        paths = new PathNode[pathMap.waypointCells.size()+pathMap.originCells.size()]
                [pathMap.sizeR][pathMap.sizeC];
        for (int i = 0; i < pathMap.originCells.size(); ++i) {
            checkingIndex = i;
            initialisePathMap(pathMap.originCells.get(i));
        }
        for (int i = 0; i < pathMap.waypointCells.size(); ++i) {
            checkingIndex = i + pathMap.originCells.size();
            initialisePathMap(pathMap.waypointCells.get(i));
        }

        /* for loop used to list all the data in the first map, made for debug purposes
        for (int i = 0; i < pathMap.sizeR; ++i) {
            for (int j =0; j < pathMap.sizeC; ++j) {
                if (paths[0][i][j] == null)
                    System.out.println("Row: " + i + " Col: " + j + " is null");
                else
                    System.out.println("Row: " + i + " Col: " + j + " Source (col,row): (" + paths[0][i][j].c + "," + paths[0][i][j].r + ") Length: " + paths[0][i][j].length);
            }
        }
        */

        LinkedList<Coordinate> path = new LinkedList<>();
        if (pathMap.waypointCells.size() == 0) { // code block run for Task A, B and C
            int sourceIndex = INVALID_POINT;
            int pathLength = INVALID_POINT;
            int destIndex = INVALID_POINT;
            for (int i = 0; i < pathMap.originCells.size(); ++i) {  // loops to check which source and destination
                for (int j = 0; j < pathMap.destCells.size(); ++j){ // combination gives the shortest distance
                    if (paths[i][pathMap.destCells.get(j).getRow()][pathMap.destCells.get(j).getColumn()] != null) {
                        if (pathLength == INVALID_POINT || pathLength >
                                paths[i][pathMap.destCells.get(j).getRow()][pathMap.destCells.get(j).getColumn()].length) {
                            sourceIndex = i;
                            pathLength = paths[i][pathMap.destCells.get(j).getRow()][pathMap.destCells.get(j).getColumn()].length;
                            destIndex = j;
                        }
                    }
                }
            }
            // below code block creates the list of coordinates, working backwards from the destination
            int r = pathMap.destCells.get(destIndex).getRow();
            int c = pathMap.destCells.get(destIndex).getColumn();
            while (r != pathMap.originCells.get(sourceIndex).getRow()
                    || c != pathMap.originCells.get(sourceIndex).getColumn()) {
                path.addFirst(pathMap.cells[r][c]);
                int rTemp = paths[sourceIndex][r][c].r;
                int cTemp = paths[sourceIndex][r][c].c;
                r = rTemp;
                c = cTemp;
            }
            path.addFirst(pathMap.originCells.get(sourceIndex));
        } // end of code block run for Task A, B and C
        return path;
    } // end of findPath()


    @Override
    public int coordinatesExplored() {
        // TODO: Implement (optional)

        // placeholder
        return 0;
    } // end of cellsExplored()

    /**
     *
     * @param source - the coordinate of which the path is being found from
     */
    private void initialisePathMap(Coordinate source) {
        // boolean array to store which  coordinates are used already and shortest path known
        boolean[][] foundCoords = new boolean[pathMap.sizeC][pathMap.sizeR];
        paths[checkingIndex][source.getRow()][source.getColumn()] = new PathNode(0);
        paths[checkingIndex][source.getRow()][source.getColumn()].r = INVALID_POINT;
        paths[checkingIndex][source.getRow()][source.getColumn()].c = INVALID_POINT;
        checkAdjacents(source.getRow(),source.getColumn(), 0);
        foundCoords[source.getRow()][source.getColumn()] = true;
        while (true) {
            int c = INVALID_POINT;
            int r = INVALID_POINT;
            int length = INVALID_POINT;
            for (int i = 0; i < pathMap.sizeR; ++i) {       // loops for coordinates
                for (int j = 0; j < pathMap.sizeC; ++j) {
                    if (!foundCoords[i][j]) {   // checks if it hasn't already been found
                        if (paths[checkingIndex][i][j] != null) {   // checks if null, can't use null values
                            if (length == INVALID_POINT || length > paths[checkingIndex][i][j].length) { // checks if there isn't a better target cell already determined
                                r = i;
                                c = j;
                                length = paths[checkingIndex][i][j].length;
                            }
                        }
                    }
                }
            }
            if (length != INVALID_POINT) {
                checkAdjacents(r, c, length);
                foundCoords[r][c] = true;
            } else {break;}
        }
    }

    /**
     * method checks the neighbours of the current node to see if they are in the array,
     * calls check to see if the node is to be changed
     * @param r - row to be checked
     * @param c - column to be checked
     * @param length - the length taken to get to this point
     */
    private void checkAdjacents(int r, int c, int length) {
        if (r-1 >= 0) {
            if (checkAdd(r-1,c,length)) {
                paths[checkingIndex][r-1][c].r = r;
                paths[checkingIndex][r-1][c].c = c;
            }
        }
        if (r+1 != pathMap.sizeR) {
            if (checkAdd(r+1,c,length)) {
                paths[checkingIndex][r+1][c].r = r;
                paths[checkingIndex][r+1][c].c = c;
            }
        }
        if (c-1 >= 0) {
            if (checkAdd(r,c-1,length)) {
                paths[checkingIndex][r][c-1].r = r;
                paths[checkingIndex][r][c-1].c = c;
            }
        }
        if (c+1 != pathMap.sizeC) {
            if (checkAdd(r,c+1,length)) {
                paths[checkingIndex][r][c+1].r = r;
                paths[checkingIndex][r][c+1].c = c;

            }
        }
    }

    /**
     *
     * @return - returns true if the node is a shorter path in length then what already exists,
     *           additionally will create and set the length to the node if it is a shorter path/there isn't a path
     */
    private boolean checkAdd(int r, int c, int length) {
        if (paths[checkingIndex][r][c] != null) {
            if (paths[checkingIndex][r][c].length <= length + pathMap.cells[r][c].getTerrainCost()) {
                return false;
            }
        }
        if (pathMap.cells[r][c].getImpassable())
            return false;
        paths[checkingIndex][r][c] = new PathNode(length + pathMap.cells[r][c].getTerrainCost());
        return true;

    }

    /**
     * a class used to store the details for the path from a node gathered by the path map
     * has a row and column which contain the source node which this node came from
     */
    private class PathNode {
        public int r; //row of source
        public int c; // column of source
        public int length; // length of path to this node

        public PathNode(int length) {
            this.length = length;
        }
    }

} // end of class DijsktraPathFinder
