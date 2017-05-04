#!/usr/bin/python

import rospy
from soma_msgs.msg import SOMAObject
from mongodb_store.message_store import MessageStoreProxy
from soma_manager.srv import *
#from tabulate import tabulate
from prettytable import PrettyTable
import json
import os.path
from quasimodo_msgs.msg import fused_world_state_object

def create_analysis_for_type(type):
    print("making query")
    soma_query = rospy.ServiceProxy('soma/query_objects', SOMAQueryObjs)
    print("done query")

    # Query all observations during the last 30 mins
    query = SOMAQueryObjsRequest()
    query.query_type = 0
    query.objecttypes=[type]
    query.configs = ['quasimodo_retrieval_result']

    #query.uselowertime = True
    #query.usedates = True
    #dt_obj = datetime.now() - timedelta(minutes=UPDATE_INT_MINUTES) #fromtimestamp(query.lowerdate
    #query.lowerhour = dt_obj.hour
    #print "Hour %d" % query.lowerhour
    #query.lowerminutes = dt_obj.minute
    #print "Minute %d" % query.lowerminutes

    response = soma_query(query)

    if not response.objects:
        print("No SOMA objects!")
        return {}, 0, 0

    response.objects = [x for x in response.objects if x.type == type] # for some reason the objecttype doesn't get filtered?

    rooms = {}
    queries = {}
    nbr_results = len(response.objects)

    for x in response.objects:
        ids = x.id.split()

        if x.metadata in queries:
            queries[x.metadata] += 1
        else:
            queries[x.metadata] = 1

        if ids[0] in rooms:
            rooms[ids[0]] += 1
        else:
            rooms[ids[0]] = 1

    nbr_queries = len(queries)

    reversed_dict = {}
    for k, v in rooms.items():
        if v in reversed_dict:
            reversed_dict[v] += 1
        else:
            reversed_dict[v] = 1

    return reversed_dict, nbr_queries, nbr_results

def get_metaroom_segment_stats(data_path):

    with open(os.path.join(data_path, 'segments_summary.json')) as data_file:
        data = json.load(data_file)

    nbr_sweeps = data['value0']['nbr_sweeps']
    nbr_segments = data['value0']['nbr_convex_segments']

    return nbr_sweeps, nbr_segments

def get_database_segment_stats():
    msg_store = MessageStoreProxy(database="world_state", collection="quasimodo")
    resp = msg_store.query(fused_world_state_object._type, message_query={"removed_at": ""})
    return len(resp)

def run(data_path):

    nbr_objects = get_database_segment_stats()
    nbr_sweeps, nbr_segments = get_metaroom_segment_stats(data_path)
    db_rooms, db_nbr_queries, db_nbr_results = create_analysis_for_type("quasimodo_db_result")
    metaroom_rooms, metaroom_nbr_queries, metaroom_nbr_results = create_analysis_for_type("quasimodo_metaroom_result")

    t = PrettyTable(['Entity', 'Quasimodo Model DB Query', 'Quasimodo Metarooms Query'])
    t.add_row(['Number queries:', db_nbr_queries, metaroom_nbr_queries])
    t.add_row(['Number results:', db_nbr_results, metaroom_nbr_results])
    t.add_row(['Indexed sweeps:', '-', nbr_sweeps])
    t.add_row(['Indexed segments:', nbr_objects, nbr_segments])
    for k, v in db_rooms.items():
        t.add_row(["Objects with %d nbr search results: " % k, v, '-'])
    for k, v in metaroom_rooms.items():
        t.add_row(["Sweeps with %d nbr search results: " % k, v, '-'])

    print t

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print "Please provide the semantic maps data path..."
    else:
        run(sys.argv[1])
