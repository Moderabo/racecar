import information

def on_subscribe(client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
    if reason_code_list[0].is_failure:
        print(f"Broker rejected your subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")

def on_unsubscribe(client, userdata, mid, reason_code_list, properties):
    # Be careful, the reason_code_list is only present in MQTTv5.
    # In MQTTv3 it will always be empty
    if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
        print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
    else:
        print(f"Broker replied with failure: {reason_code_list[0]}")
    client.disconnect()

def on_message(client, userdata, message):
    # userdata is the structure we choose to provide, here it's a list()
    if message.topic == "commands":
        information.data_list.append(message.payload)
        information.data_list.pop(0)

    elif message.topic == "data":
        print(message.payload)

    elif message.topic == "cones":
        information.cones.clear()
        message.payload = message.payload.decode("utf-8").rstrip(";")
        cones = message.payload.split(";")
        for cone in cones:
            cone_info_list = cone.split(",")
            cone_tuple = (float(cone_info_list[1])/10,
                          float(cone_info_list[0])/10,
                          float(cone_info_list[2])/10)
            information.cones.append(cone_tuple)

    elif message.topic == "bezier/def":
        information.bezier[0].clear()
        message.payload = message.payload.decode("utf-8").rstrip(";")
        def_points = message.payload.split(";")
        for def_point in def_points:
            def_point_info_list = def_point.split(",")
            def_point_tuple = (float(def_point_info_list[0])/10,
                               float(def_point_info_list[1])/10)
            information.bezier[0].append(def_point_tuple)

    elif message.topic == "bezier/waypoints":
        information.bezier[1].clear()
        message.payload = message.payload.decode("utf-8").rstrip(";")
        def_waypoints = message.payload.split(";")
        for def_waypoint in def_waypoints:
            def_waypoint_info_list = def_waypoint.split(",")
            def_waypoint_tuple = (float(def_waypoint_info_list[0])/10,
                                  float(def_waypoint_info_list[0])/10)
            information.bezier[1].append(def_waypoint_tuple)

    # We only want to process 10 messages
    if len(userdata) >= 10:
        #client.unsubscribe("hi")
        return

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        # we should always subscribe from on_connect callback to be sure
        # our subscribed is persisted across reconnections.
        client.subscribe([("commands",0), ("data", 0), ("cones", 0), ("bezier/def", 0), ("bezier/waypoint", 0)])

def on_publish(client, userdata, mid, reason_code, properties):
    # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
    try:
        userdata.remove(mid)
    except KeyError:
        print("on_publish() is called with a mid not present in unacked_publish")
        print("This is due to an unavoidable race-condition:")
        print("* publish() return the mid of the message sent.")
        print("* mid from publish() is added to unacked_publish by the main thread")
        print("* on_publish() is called by the loop_start thread")
        print("While unlikely (because on_publish() will be called after a network round-trip),")
        print(" this is a race-condition that COULD happen")
        print("")
        print("The best solution to avoid race-condition is using the msg_info from publish()")
        print("We could also try using a list of acknowledged mid rather than removing from pending list,")
        print("but remember that mid could be re-used !")
