from scipy.spatial.distance import pdist, squareform
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

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
    else if request.method == 'GET':
        return 'duocun route api'
    else:
        return None

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