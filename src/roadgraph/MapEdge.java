package roadgraph;

import geography.GeographicPoint;

import java.util.*;

public class MapEdge {

    private GeographicPoint from;
    private GeographicPoint to;

    public GeographicPoint getFrom() {
        return from;
    }

    public GeographicPoint getTo() {
        return to;
    }

    public String getRoadName() {
        return roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public double getLength() {
        return length;
    }

    private String roadName;
    private String roadType;
    private double length;

    public MapEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
        this.from = from;
        this.to = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }


}
