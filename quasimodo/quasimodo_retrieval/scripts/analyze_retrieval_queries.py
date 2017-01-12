#!/usr/bin/python

import rospy
from soma_msgs.msg import SOMAObject
from mongodb_store.message_store import MessageStoreProxy
from soma_manager.srv import *
#from tabulate import tabulate
from prettytable import PrettyTable

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
        return

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

    return rooms, nbr_queries, nbr_results

if __name__ == '__main__':
    db_rooms, db_nbr_queries, db_nbr_results = create_analysis_for_type("quasimodo_db_result")
    metaroom_rooms, metaroom_nbr_queries, metaroom_nbr_results = create_analysis_for_type("quasimodo_metaroom_result")

    t = PrettyTable(['Entity', 'Quasimodo Model DB Query', 'Quasimodo Metarooms Query'])
    t.add_row(['Number queries:', db_nbr_queries, metaroom_nbr_queries])
    t.add_row(['Number results:', db_nbr_results, metaroom_nbr_results])
    for k, v in db_rooms.items():
        t.add_row([k, v, 0])
    for k, v in metaroom_rooms.items():
        t.add_row([k, 0, v])

    print t
