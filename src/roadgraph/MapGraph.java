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
 * <p>
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
public class MapGraph {

    private static final String CONST_DIJKSTRA = "dijkstra";
    private static final String CONST_BFS = "bfs";
    private static final String CONST_ASTAR = "astar";
    ArrayList<MapNode> nodes;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        nodes = new ArrayList<>();
    }

    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph firstMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(1, 1);
        GeographicPoint end = new GeographicPoint(8, -1);

        System.out.println(firstMap.printSearch(start, end, CONST_BFS));
        System.out.println(firstMap.printSearch(start, end, CONST_DIJKSTRA));
        System.out.println(firstMap.printSearch(start, end, CONST_ASTAR));
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
		System.out.println(simpleTestMap.printSearch(testStart, testEnd, "dijkstra"));
//		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);



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

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return nodes.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
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
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        int size = 0;

        for (MapNode node : nodes) {
            size += node.getEdges().size();
        }
        return size;
    }

    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
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
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
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

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        MapNode startNode = new MapNode(start);
        MapNode endNode = new MapNode(goal);

        if (!nodes.contains(startNode) || !nodes.contains(endNode)) {
            return null;
        }

        startNode = nodes.get(nodes.indexOf(startNode));
        endNode = nodes.get(nodes.indexOf(endNode));

        Queue<MapNode> queue = new LinkedList<>();
        HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<>();
        HashSet<MapNode> visited = new HashSet<>();

        boolean isFound = searchBFS(startNode, endNode, visited, parent, queue, nodeSearched);

        if (!isFound) {
            return null;
        }

        return constructPath(parent, startNode, endNode);
    }

    private LinkedList<GeographicPoint> constructPath(HashMap<GeographicPoint, GeographicPoint> parent, MapNode
            startNode, MapNode endNode) {
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
            GeographicPoint> parent, Queue<MapNode> queue, Consumer<GeographicPoint> nodeSearched) {

        queue.add(startNode);
        visited.add(startNode);

        while (!queue.isEmpty()) {
            MapNode current = queue.poll();
            current = nodes.get(nodes.indexOf(current));
            nodeSearched.accept(current.getLocation());

            if (current == endNode) {
                return true;
            }

            LinkedList<MapEdge> edges = current.getEdges();
            addNodes(edges, visited, parent, queue, current);
        }

        return false;
    }

    /**
     * Helper method
     *
     * @param edges   A list of the edges of each node
     * @param visited Hashset containing all the visited nodes
     * @param parent  HashMap that maps each node with parent node
     * @param queue   Queue being visited
     * @param current Current node
     */
    private void addNodes(LinkedList<MapEdge> edges, HashSet<MapNode> visited, HashMap<GeographicPoint,
            GeographicPoint> parent, Queue<MapNode> queue, MapNode current) {

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

    /**
     * Method for debugging
     *
     * @param start
     * @param end
     * @return
     */
    public String printSearch(GeographicPoint start, GeographicPoint end, String search) {

        List<GeographicPoint> path = new LinkedList<>();

        if (search.equals(CONST_BFS)) {
            path = bfs(start, end);
        } else if (search.equals(CONST_DIJKSTRA)) {
            path = dijkstra(start, end);
        } else if(search.equals(CONST_ASTAR)){
            path = aStarSearch(start, end);
        }

        String pathToString = "";

        if (path == null)
            return null;

        for (GeographicPoint point : path) {
            pathToString += point.toString() + "->";
        }

        return pathToString;
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        MapNode startNode = new MapNode(start);
        MapNode endNode = new MapNode(goal);

        if (!nodes.contains(startNode) || !nodes.contains(endNode)) {
            return null;
        }

        startNode = nodes.get(nodes.indexOf(startNode));
        endNode = nodes.get(nodes.indexOf(endNode));

        PriorityQueue<MapNode> pQueue = new PriorityQueue<>();
        HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<>();
        HashSet<MapNode> visited = new HashSet<>();
        boolean isFound = searchDijkstra(pQueue, parent, visited, startNode, endNode, nodeSearched);

        if (!isFound) {
            System.out.println("Node not found");
            return null;
        }

        return constructPath(parent, startNode, endNode);

    }

    private boolean searchDijkstra(PriorityQueue<MapNode> pQueue, HashMap<GeographicPoint, GeographicPoint> parent,
                                   HashSet<MapNode> visited, MapNode startNode, MapNode endNode,
                                   Consumer<GeographicPoint> nodeSearched) {
        for (MapNode node : nodes) {
            node.setDistance(Integer.MAX_VALUE);
        }

        pQueue.add(startNode);
        startNode.setDistance(0);

        while (!pQueue.isEmpty()) {
            MapNode current = pQueue.poll();
            visited.add(current);
            nodeSearched.accept(current.getLocation());

            if (current == endNode) {
                return true;
            }
            LinkedList<MapEdge> edges = current.getEdges();
            addNodes(edges, pQueue, current, parent);
        }
        return false;
    }

    private void addNodes(LinkedList<MapEdge> edges, PriorityQueue<MapNode> pQueue, MapNode current,
                          HashMap<GeographicPoint, GeographicPoint> parent) {
        for (MapEdge edge : edges) {
            GeographicPoint dest = edge.getTo();
            MapNode destNode = new MapNode(dest);
            destNode = nodes.get(nodes.indexOf(destNode));
            double weight = edge.getLength();
            double destNodeDistance = destNode.getDistance();

            if (current.getDistance() + weight < destNodeDistance) {
                destNode.setDistance(current.getDistance() + weight);
                pQueue.add(destNode);
                parent.put(dest, current.getLocation());
            }
        }
    }


    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        MapNode startNode = new MapNode(start);
        MapNode endNode = new MapNode(goal);

        if (!nodes.contains(startNode) || !nodes.contains(endNode)) {
            return null;
        }

        startNode = nodes.get(nodes.indexOf(startNode));
        endNode = nodes.get(nodes.indexOf(endNode));

        PriorityQueue<MapNode> pQueue = new PriorityQueue<>();
        HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<>();
        HashSet<MapNode> visited = new HashSet<>();

        boolean isFound = searchAstar(pQueue, parent, visited, startNode, endNode, nodeSearched);


        if (!isFound) {
            System.out.println("Node not found");
            return null;
        }

        return constructPath(parent, startNode, endNode);
    }

    private boolean searchAstar(PriorityQueue<MapNode> pQueue, HashMap<GeographicPoint, GeographicPoint> parent,
                                HashSet<MapNode> visited, MapNode startNode, MapNode endNode,
                                Consumer<GeographicPoint> nodeSearched){
        for (MapNode node : nodes) {
            node.setDistance(Integer.MAX_VALUE);
        }

        pQueue.add(startNode);
        startNode.setDistance(0);

        while (!pQueue.isEmpty()){
            MapNode currentNode = pQueue.poll();
            currentNode = nodes.get(nodes.indexOf(currentNode));
            nodeSearched.accept(currentNode.getLocation());
            visited.add(currentNode);

            if(currentNode == endNode){
                return true;
            }

            LinkedList<MapEdge> edges = currentNode.getEdges();
            addNodes(edges, pQueue, currentNode, endNode, parent);
        }

        return false;
    }

    private void addNodes(LinkedList<MapEdge> edges, PriorityQueue<MapNode> pQueue, MapNode current, MapNode endNode,
                          HashMap<GeographicPoint, GeographicPoint> parent){
        for (MapEdge edge : edges) {
            GeographicPoint dest = edge.getTo();
            double distanceFromEnd = dest.distance(endNode.getLocation());
            MapNode destNode = new MapNode(dest);
            destNode = nodes.get(nodes.indexOf(destNode));
            double weight = edge.getLength();
            double destNodeDistance = destNode.getDistance();
            double distanceFromStart = current.getDistance() + weight;

            if (distanceFromStart + distanceFromEnd < destNodeDistance) {
                destNode.setDistance(distanceFromStart + distanceFromEnd);
                pQueue.add(destNode);
                parent.put(dest, current.getLocation());
            }
        }
    }

}
