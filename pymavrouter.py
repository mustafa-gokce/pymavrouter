import os
import time
import threading
import click
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# always use MAVLink 2.0
os.environ["MAVLINK20"] = "1"

# global variables
terminate = False
vehicle = None
endpoints = []
thread_vehicle = None
thread_endpoints = []


# connect to vehicle
def connection_vehicle(master_connections, connection_timeout=5.0, message_rate=4):
    # get global variables
    global terminate
    global vehicle, endpoints

    # this will run until terminate
    while True:

        # get connection string
        connection_string = master_connections[0]
        connection = connection_string.split(",")
        if len(connection) == 2:
            baud = int(connection[1])
        else:
            baud = 115200
        connection_string = connection[0]

        # rotate connection string list
        master_connections.append(master_connections.pop(0))

        # initiate vehicle
        vehicle = None

        # get the time
        time_start = time.monotonic()

        # try to connect to vehicle
        try:

            # create vehicle connection
            vehicle = utility.mavlink_connection(device=connection_string,
                                                 baud=baud,
                                                 autoreconnect=True,
                                                 force_connected=True,
                                                 udp_timeout=connection_timeout,
                                                 timeout=connection_timeout)

            # user requested all the available streams from vehicle
            if message_rate > 0:

                # wait a heartbeat message from vehicle
                heartbeat = vehicle.wait_heartbeat(timeout=connection_timeout)

                # reconnect to vehicle if no heartbeat received
                if not heartbeat:
                    continue

                # request all available streams from vehicle
                vehicle.mav.request_data_stream_send(target_system=vehicle.target_system,
                                                     target_component=vehicle.target_component,
                                                     req_stream_id=dialect.MAV_DATA_STREAM_ALL,
                                                     req_message_rate=message_rate,
                                                     start_stop=1)

        except Exception as e:

            # there is a vehicle object
            if vehicle is not None:

                # try to close the vehicle
                try:

                    # close the vehicle
                    vehicle.close()

                # unknown error occurred during closing the vehicle
                except Exception as e:

                    # ignore the error
                    pass

                # clear the vehicle connection
                vehicle = None

            # cool down the reconnection
            time.sleep(connection_timeout)

            # continue to reconnection to vehicle
            continue

        # this will run until terminate or vehicle connection lost
        while True:

            # break if termination is requested
            if terminate or time.monotonic() - time_start > connection_timeout:

                # check vehicle is existed
                if vehicle is not None:

                    # try to close the vehicle
                    try:

                        # close the vehicle
                        vehicle.close()

                    # unknown error occurred during closing the vehicle
                    except Exception as e:

                        # ignore the error
                        pass

                    # clear the vehicle connection
                    vehicle = None

                # break if timeout occurred
                break

            # try to receive and send a message from vehicle to endpoint
            try:

                # check vehicle is existed
                if vehicle is not None:

                    # try to receive a message
                    message = vehicle.recv_msg()

                    # received a message within timeout
                    if message is not None:

                        # get the time
                        time_start = time.monotonic()

                        # get message buffer
                        message_buffer = message.get_msgbuf()

                        # for each endpoint on the list
                        for endpoint in endpoints:

                            # check if endpoint exist
                            if endpoint is not None:
                                # send the received message from vehicle to endpoint
                                endpoint.write(message_buffer)

                    # did not receive a message within timeout
                    else:

                        # cool down the message receiving
                        time.sleep(0.01)

                # vehicle does not exist
                else:

                    # break the inner loop
                    break

            # unknown error occurred during receive and send a message from vehicle to endpoint
            except Exception as e:
                pass


# connect to endpoint
def connection_endpoint(index, connection_string, connection_timeout=5):
    # get global variables
    global terminate
    global vehicle, endpoints

    # check connection string is serial connection or not
    connection = connection_string.split(",")
    if len(connection) == 2:
        baud = int(connection[1])
    else:
        baud = 115200
    connection_string = connection[0]

    # this will run until terminate
    while True:

        # initiate endpoint
        endpoint = None

        # get the time
        time_start = time.monotonic()

        # try to connect to endpoint
        try:

            # create endpoint connection
            endpoint = utility.mavlink_connection(device=connection_string,
                                                  baud=baud,
                                                  autoreconnect=True,
                                                  force_connected=True,
                                                  udp_timeout=connection_timeout,
                                                  timeout=connection_timeout)

            # add endpoint connection to list
            endpoints[index] = endpoint

        except Exception as e:

            # there is an endpoint object
            if endpoint is not None:

                # try to close the endpoint
                try:

                    # close the endpoint
                    endpoint.close()

                # unknown error occurred during closing the endpoint
                except Exception as e:

                    # ignore the error
                    pass

                # clear the endpoint connection
                endpoint = None

            # cool down the reconnection
            time.sleep(connection_timeout)

            # continue to reconnection to endpoint
            continue

        # this will run until terminate or endpoint connection lost
        while True:

            # break if termination is requested
            if terminate or time.monotonic() - time_start > connection_timeout:

                # check endpoint is existed
                if endpoint is not None:

                    # try to close the endpoint
                    try:

                        # close the endpoint
                        endpoint.close()

                    # unknown error occurred during closing the endpoint
                    except Exception as e:

                        # ignore the error
                        pass

                    # clear the endpoint connection
                    endpoint = None

                # break if timeout occurred
                break

            # try to receive and send a message from endpoint to vehicle
            try:

                # check endpoint is existed
                if endpoint is not None:

                    # try to receive a message
                    message = endpoint.recv_msg()

                    # received a message within timeout
                    if message is not None:

                        # get the time
                        time_start = time.monotonic()

                        # get message buffer
                        message_buffer = message.get_msgbuf()

                        # check if vehicle exist
                        if vehicle is not None:
                            # send the received message from endpoint to vehicle
                            vehicle.write(message_buffer)

                    # did not receive a message within timeout
                    else:

                        # cool down the message receiving
                        time.sleep(0.01)

                # vehicle does not exist
                else:

                    # break the inner loop
                    break

            # unknown error occurred during receive and send a message from endpoint to vehicle
            except Exception as e:
                pass


@click.command()
@click.option("--master", default="tcp:127.0.0.1:5760", type=click.STRING, required=False,
              help="Plus sign seperated MAVLink connection string list of vehicle.")
@click.option("--out", default="udpout:127.0.0.1:14550", type=click.STRING, required=False,
              help="Plus sign seperated MAVLink connection string list of endpoints.")
@click.option("--timeout", default=5.0, type=click.FloatRange(min=5, clamp=True), required=False,
              help="Try to reconnect after this seconds when no message is received.")
@click.option("--rate", default=4, type=click.IntRange(min=0, clamp=True), required=False,
              help="Message stream that will be requested from vehicle, zero means do not request.")
def main(master, out, timeout, rate):
    # get global variables
    global terminate
    global vehicle, endpoints
    global thread_vehicle, thread_endpoints

    # parse master connection strings
    master_connections = master.split("+")

    # start master connection thread
    thread_vehicle = threading.Thread(target=connection_vehicle, args=(master_connections, timeout, rate))
    thread_vehicle.start()

    # parse all endpoints
    for i, device in enumerate(out.split("+")):
        # resize endpoints list
        endpoints.append(None)

        # start slave connection thread
        connection_thread_endpoint = threading.Thread(target=connection_endpoint,
                                                      args=(i, device, timeout))
        connection_thread_endpoint.start()
        thread_endpoints.append(connection_thread_endpoint)

    # infinite loop
    while True:
        try:
            time.sleep(timeout)
        except KeyboardInterrupt:
            terminate = True
            break


# main entry point
if __name__ == "__main__":
    # run main function
    main()
