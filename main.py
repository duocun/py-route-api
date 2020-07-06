from scipy.spatial.distance import pdist, squareform
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from math import radians, cos, sin, asin, sqrt
from flask import Flask, request
from flask_restful import Resource, Api
from json import dumps
# from flask_jsonpify import jsonify

app = Flask(__name__)
api = Api(app)

# MAX_NODES_PER_CLUSTER = 30
# MIN_NODES_PER_CLUSTER = 15
# DEFAULT_MIN_RADIUS = 0.2
# DEFAULT_MAX_RADIUS = 8
# DEBUG = True


@app.route('/routes', methods=['POST', 'GET'])
def route():
    if request.method == 'POST':
        clustersMap = {}
        for driver_id in request.json.keys():
            clustersMap[driver_id] = [{'orderId': str(x['orderId']), 'lat': x['lat'], 'lng': x['lng']} for x in request.json[driver_id]['orders']];
        return {'routes': get_routes_v2(clustersMap)}
    elif request.method == 'GET':
        return 'duocun route api'
    else:
        return None

def haversine(lonlat1, lonlat2):
    """
    Calculate the great circle distance between two nodes 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lat1, lon1 = lonlat1
    lat2, lon2 = lonlat2
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r

def create_data_model(dm):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = dm  # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def calc_route(dm, orders):
    """Entry point of the program."""

    # Instantiate the data problem.
    data = create_data_model(dm)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        return get_route(manager, routing, solution, orders)
    else:
        return orders

def get_route(manager, routing, solution, orders):
    route = []
    """Prints solution on console."""
    print('Objective: {} km'.format(solution.ObjectiveValue()/1000))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        if(index > 0):
            route.append(orders[index-1])
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}km\n'.format(route_distance/1000)
    return route

def get_routes_v2(cluster_map):
    routes = []
    Z = []
    for driver_id in cluster_map.keys():
        z = [[x['lat'], x['lng']] for x in cluster_map[driver_id]]
        z.insert(0, [43.806047,-79.2379642]) # start point
        dm = squareform(pdist(z, (lambda lat, lng: int(haversine(lat,lng)*1000))))
        Z.append({'driverId': driver_id, 'nodes': cluster_map[driver_id], 'distance_matrix': dm})

    for t in Z:
        print("cluster {}".format(t['driverId']))
        route = calc_route(t['distance_matrix'], t['nodes'])
        route.insert(0, {'orderId':0, 'lat':43.806047, 'lng':-79.2379642}) # start point
        driverId = t['driverId']
        routes.append({'driverId': driverId, 'route': route})
    
    return routes


if __name__ == '__main__':
    # api.add_resource(Routes, '/routes') # Route_1
    app.run(port='5002')