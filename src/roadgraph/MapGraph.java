/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 *
 */
public class MapGraph {

    ArrayList<MapNode> nodes;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        nodes = new ArrayList<>();
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return nodes.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        Set<GeographicPoint> vertices = new HashSet<>();

        for (MapNode node : nodes) {
            vertices.add(node.getLocation());
        }

        return vertices;
    }

    /**
     * Get the number of road segments in the graph
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        int size = 0;

        for (MapNode node : nodes) {
            size += node.getEdges().size();
        }
        return size;
    }


    /** Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     * @param location  The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        MapNode newNode = new MapNode(location);

        if (location == null || nodes.contains(newNode)) {
            return false;
        }

        nodes.add(newNode);
        return true;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     * @param from The starting point of the edge
     * @param to The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *   added as nodes to the graph, if any of the arguments is null,
     *   or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {

        MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
        MapNode sourceNode = new MapNode(from);
        MapNode destinationNode = new MapNode(to);

        if (from == null || to == null || !nodes.contains(sourceNode) || !nodes.contains
                (destinationNode) || length < 0 ||
                roadName ==
                        null ||
                roadType == null) {
            throw new IllegalArgumentException();
        }

        nodes.get(nodes.indexOf(sourceNode)).addEdge(edge);

    }


    /** Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal The goal location
     * @return The list of intersections that form the shortest (unweighted)
     *   path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /** Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     *   path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {


        MapNode startNode = new MapNode(start);
        MapNode endNode = new MapNode(goal);

        if(!nodes.contains(startNode) || !nodes.contains(endNode)){
            return null;
        }

        startNode = nodes.get(nodes.indexOf(startNode));
        endNode = nodes.get(nodes.indexOf(endNode));

        Queue<MapNode> queue = new LinkedList<>();
        HashMap<GeographicPoint,GeographicPoint> parent = new HashMap<>();
        HashSet<MapNode> visited = new HashSet<>();

        boolean isFound = searchBFS(startNode, endNode, visited, parent, queue, nodeSearched);

        if (!isFound) {
            return null;
        }

        LinkedList<GeographicPoint> path = constructPath(parent, startNode, endNode);

        return path;
    }

    private  LinkedList<GeographicPoint> constructPath(HashMap<GeographicPoint, GeographicPoint> parent, MapNode
            startNode, MapNode endNode){
        LinkedList<GeographicPoint> path = new LinkedList<>();

        GeographicPoint node = endNode.getLocation();
        path.addFirst(node);

        while (node != startNode.getLocation()) {
            node = parent.get(node);
            path.addFirst(node);
        }

        return path;
    }

    private boolean searchBFS(MapNode startNode, MapNode endNode, HashSet<MapNode> visited, HashMap<GeographicPoint,
            GeographicPoint> parent, Queue<MapNode> queue, Consumer<GeographicPoint> nodeSearched){

        boolean isFound;

        queue.add(startNode);
        visited.add(startNode);

        while (!queue.isEmpty()) {
            MapNode current = queue.poll();
            current = nodes.get(nodes.indexOf(current));

            nodeSearched.accept(current.getLocation());


            if (current == endNode) {
                isFound = true;
                return isFound;
            }

            LinkedList<MapEdge> edges = current.getEdges();

            for (MapEdge edge : edges) {
                MapNode dest = new MapNode(edge.getTo());
                dest = nodes.get(nodes.indexOf(dest));

                if (!visited.contains(dest)) {
                    visited.add(dest);
                    queue.add(dest);
                    parent.put(dest.getLocation(), current.getLocation());
                }
            }

        }

        return false;
    }

    /**
     * Method for debugging
     * @param start
     * @param end
     * @return
     */
    public String printBFS(GeographicPoint start, GeographicPoint end) {

        List<GeographicPoint> path = bfs(start, end);

        String pathToString = "";

        if(path == null) return null;

        for (GeographicPoint point : path) {
            pathToString += point.toString() + "->";
        }

        return pathToString;
    }


    /** Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal The goal location
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /** Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 4

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    /** Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal The goal location
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /** Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 4

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }


    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph firstMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(1, 1);
        GeographicPoint end = new GeographicPoint(8, -1);

        System.out.println(firstMap.printBFS(start, end));

        //MapGraph.printGraph(firstMap);

        // You can use this method for testing.


		/* Here are some test cases you should try before you attempt
         * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/


		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/

    }

}
