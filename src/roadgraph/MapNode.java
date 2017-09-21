package roadgraph;

import geography.GeographicPoint;

import java.util.*;

public class MapNode implements Comparable<MapNode> {

    private GeographicPoint location;
    private LinkedList<MapEdge> edges = new LinkedList<>();
    private double predictedDistance;

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    private double distance;

    public MapNode(GeographicPoint location) {
        this.location = location;
        this.distance = 0;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public void addEdge(MapEdge edge){
        this.edges.add(edge);
    }

    public LinkedList<MapEdge> getEdges() {
        return edges;
    }

    @Override
    public boolean equals(Object obj) {
        if(obj instanceof  MapNode){
            MapNode node = (MapNode) obj;
            return this.location.equals(node.location);
        }

        return false;
    }

    @Override
    public int compareTo(MapNode o) {
        return Double.compare(this.distance, o.distance);
    }

    public double getPredictedDistance() {
        return predictedDistance;
    }

    public void setPredictedDistance(double predictedDistance) {
        this.predictedDistance = predictedDistance;
    }
}
