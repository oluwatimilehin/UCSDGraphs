package roadgraph;

import geography.GeographicPoint;

import java.util.*;

public class MapNode {

    GeographicPoint location;
    LinkedList<MapEdge> edges = new LinkedList<>();

    public MapNode(GeographicPoint location) {
        this.location = location;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public void addEdge(MapEdge edge){
        edges.add(edge);
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
}
