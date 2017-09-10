package roadgraph;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.HashMap;
import java.util.HashSet;

import static org.junit.Assert.*;

public class MapGraphTest {
    MapGraph testMap;

    private GeographicPoint existingPoint;
    private GeographicPoint nullPoint;
    private GeographicPoint newPoint;
    private GeographicPoint existingDestination;

    @org.junit.Before
    public void setUp(){
        testMap = new MapGraph();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", testMap);

        existingPoint = new GeographicPoint(4, 2);
        nullPoint = null;
        newPoint = new GeographicPoint(10, 20);
        existingDestination = new GeographicPoint(7, 3);
    }

    @org.junit.Test
    public void getNumVertices() throws Exception {
        assertEquals("Number of vertices", testMap.getNumVertices(), 9);

        testMap.addVertex(existingPoint);
        assertTrue(testMap.getNumVertices() == 9);

        testMap.addVertex(newPoint);
        assertTrue(testMap.getNumVertices() == 10);
    }

    @org.junit.Test
    public void getNumEdges() throws Exception {
        assertEquals("Number of edges", testMap.getNumEdges(), 22 );
    }

    @org.junit.Test
    public void addVertex() throws Exception {

        assertTrue(testMap.addVertex(newPoint));
        assertFalse(testMap.addVertex(existingPoint));
        assertFalse(testMap.addVertex(nullPoint));
    }

    @org.junit.Test
    public void addEdge() throws Exception {

        assertEquals("Number of edges", testMap.getNumEdges(), 22 );
        try {
            testMap.addEdge(newPoint, existingDestination, "Hello", "Hi", 5);
            fail();
        }catch (IllegalArgumentException e){

        }

        try {
            testMap.addEdge(existingPoint, newPoint, "Hey", "Hi", 12);
            fail();
        }catch (IllegalArgumentException e){

        }

        try {
            testMap.addEdge(nullPoint, existingDestination, "Hey", "Hello", 14);
            fail();
        }catch (IllegalArgumentException e){

        }

        testMap.addEdge(existingPoint, existingDestination, "front","Hola hola", 16);
        assertEquals(testMap.getNumEdges(), 23);
    }

    @org.junit.Test
    public void bfs() throws Exception {
    }

    @org.junit.Test
    public void dijkstra() throws Exception {
    }

    @org.junit.Test
    public void aStarSearch() throws Exception {
    }

}