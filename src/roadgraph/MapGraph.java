/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	//
	Map<GeographicPoint, Intersection> intersections;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		intersections = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return intersections.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return intersections.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return (int) intersections.keySet().stream()
				.flatMap(geoPoint -> intersections.get(geoPoint).getRoads().stream())
				.count();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location == null || intersections.containsKey(location)) {
			return false;
		}

		intersections.put(location, new Intersection(location));
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) throws IllegalArgumentException {

        if (from == null || to == null || roadName == null || roadType == null) {
            throw new IllegalArgumentException("Null params not allowed.");
        }

        if (length < 0) {
            throw new IllegalArgumentException("Length is lesser than 0.");
        }

        if (!intersections.containsKey(from) || !intersections.containsKey(to)) {
            throw new IllegalArgumentException("The from or to points are not present in the graph");
        }

        Intersection intersection = intersections.get(from);
        intersection.addEdge(intersections.get(to), roadName, roadType, length);
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
        Consumer<GeographicPoint> temp = (x) -> {};
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

		// mapping between every node and its parent
        Map<Intersection, Intersection> parents = new HashMap<>();

        Set<Intersection> visitedNodes = new HashSet<>();
        Queue<Intersection> queue = new LinkedList();
        queue.add(intersections.get(start));
        visitedNodes.add(intersections.get(start));

        while (!queue.isEmpty())
        {
            Intersection node = queue.poll();
			nodeSearched.accept(node.getGeographicPoint());
            if (node.getGeographicPoint().equals(goal))
            {
                return getPath(node, parents);
            }

			// iterates on the not yet visited children of this node
			node.getRoads()
				.stream()
				.filter(road -> !visitedNodes.contains(road.getDestination()))
				.forEach(road -> {
					visitedNodes.add(road.getDestination());
					queue.add(road.getDestination());
					parents.put(road.getDestination(), node);
				});
        }

        return null;
    }

	/**
	 * builds a path from the starting node to the destination node
	 * @param destinationIntersection the destination node
	 * @param parents the map contaning all the parents of the nodes
     * @return
     */
	private List<GeographicPoint> getPath(Intersection destinationIntersection, Map<Intersection, Intersection> parents) {
		List<GeographicPoint> path = new ArrayList<>();

		// we build the path starting from the destination node
		path.add(destinationIntersection.getGeographicPoint());

		// and then we go up along the parents till we reach
		// the starting node (the only node not present in the map)
		while (parents.containsKey(destinationIntersection)) {
			destinationIntersection = parents.get(destinationIntersection);
			path.add(destinationIntersection.getGeographicPoint());
		}

		// reverses the list for having the path from
		// the starting node to the destination node
		Collections.reverse(path);

		return path;
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
        Consumer<GeographicPoint> temp = (x) -> {};
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
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<Intersection> queue = new PriorityQueue<>(
				(node1, node2) -> Double.compare(node1.getDistanceFromStart(), node2.getDistanceFromStart()));

		intersections.get(start).setDistanceFromStart(0d);
		queue.add(intersections.get(start));

		return loop(queue, start, goal, nodeSearched);
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
        Consumer<GeographicPoint> temp = (x) -> {};
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
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<Intersection> queue = new PriorityQueue<>(
				(node1, node2) -> Double.compare(node1.getDistanceFromStart() + node1.getGeographicPoint().distance(node2.getGeographicPoint()), node2.getDistanceFromStart() + node1.getGeographicPoint().distance(node2.getGeographicPoint())));

		return loop(queue, start, goal, nodeSearched);
	}


	/**
	 * executes the main loop of the Dijkstra/AStar algorithms.
	 * @param queue to queue to use for storing the nodes of the graph
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from start to goal (including both start and goal). It returns null ifno path is present.
     */
	private List<GeographicPoint> loop(Queue<Intersection> queue, GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

		Set<Intersection> visitedIntersections = new HashSet<>();

		// mapping between every intersection and its parent
		Map<Intersection, Intersection> parents = new HashMap<>();


		// initial setup
		intersections.get(start).setDistanceFromStart(0d);
		queue.add(intersections.get(start));

		while (!queue.isEmpty())
		{
			Intersection currentIntersection = queue.poll();
			nodeSearched.accept(currentIntersection.getGeographicPoint());

			// if this intersection has not yet been visited
			if (!visitedIntersections.contains(currentIntersection)) {

				visitedIntersections.add(currentIntersection);

				// if it's the goal, we're done!
				if (currentIntersection.getGeographicPoint().equals(goal)) {
					return getPath(currentIntersection, parents);
				}

				// loops over the roads starting from this intersection
				for (Road road: currentIntersection.getRoads()) {

					double newDistance = currentIntersection.getDistanceFromStart() + road.getLength();
					Intersection destination = road.getDestination();

					// if going to that destination passing from the current intersection
					// has a lesser distance than a previous computed one, it takes this
					// path as the new best one
					if (newDistance < destination.getDistanceFromStart()){
						destination.setDistanceFromStart(newDistance);
						parents.put(destination, currentIntersection);
						queue.add(destination);
					}
				}
			}
		}

		// no path found from start to goal
		return null;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
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
