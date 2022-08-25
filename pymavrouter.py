import time
import threading
import click
import pymavlink.mavutil

# global variables
terminate = False
vehicle = None
endpoints = []
thread_vehicle = None
thread_endpoints = []


# connect to vehicle
def connection_vehicle(connection_string, connection_timeout=5):
    # get global variables
    global terminate
    global vehicle, endpoints

    # this will run until terminate
    while True:

        # initiate vehicle
        vehicle = None

        # try to connect to vehicle
        try:

            # create vehicle connection
            vehicle = pymavlink.mavutil.mavlink_connection(device=connection_string)

        except Exception as e:

            # there is a vehicle object
            if vehicle is not None:
                # close the vehicle
                vehicle.close()

                # clear the vehicle connection
                vehicle = None

            # cool down the reconnection
            time.sleep(1)

            # continue to reconnection to vehicle
            continue

        # this will run until terminate or vehicle connection lost
        while True:

            # try to receive and send a message from vehicle to endpoint
            try:

                # check vehicle is existed
                if vehicle is not None:

                    # try to receive a message within timeout
                    message = vehicle.recv_match(blocking=True, timeout=connection_timeout)

                    # received a message within timeout
                    if message is not None:
                        # get message buffer
                        message_buffer = message.get_msgbuf()

                        # for each endpoint on the list
                        for endpoint in endpoints:

                            # check if endpoint exist
                            if endpoint is not None:
                                # send the received message from vehicle to endpoint
                                endpoint.write(message_buffer)

                    # not received a message within timeout or user requested program termination
                    if message is None or terminate:

                        # check vehicle is existed
                        if vehicle is not None:
                            # close the vehicle
                            vehicle.close()

                            # clear the vehicle connection
                            vehicle = None

                        # break the inner loop
                        break

                # vehicle does not exist
                else:

                    # break the inner loop
                    break

            # unknown error occurred during receive and send a message from vehicle to endpoint
            except Exception as e:

                # check vehicle is existed
                if vehicle is not None:
                    # close the vehicle
                    vehicle.close()

                    # clear the vehicle connection
                    vehicle = None

                # break the inner loop
                break

        # user requested program termination
        if terminate:

            # check vehicle is existed
            if vehicle is not None:
                # close the vehicle
                vehicle.close()

                # clear the vehicle connection
                vehicle = None

            # break the outer loop
            break


# connect to endpoint
def connection_endpoint(connection_string, connection_timeout=5):
    # get global variables
    global terminate
    global vehicle, endpoints

    # this will run until terminate
    while True:

        # initiate endpoint
        endpoint = None

        # try to connect to endpoint
        try:

            # create endpoint connection
            endpoint = pymavlink.mavutil.mavlink_connection(device=connection_string)

            # add endpoint connection to list
            endpoints.append(endpoint)

        except Exception as e:

            # there is an endpoint object
            if endpoint is not None:

                # remove it from list
                for i in range(len(endpoints)):
                    if endpoint.address == endpoints[i].address:
                        del endpoints[i]

                # close the endpoint
                endpoint.close()

                # clear the endpoint connection
                endpoint = None

            # cool down the reconnection
            time.sleep(1)

            # continue to reconnection to endpoint
            continue

        # this will run until terminate or endpoint connection lost
        while True:

            # try to receive and send a message from endpoint to vehicle
            try:

                # check endpoint is existed
                if endpoint is not None:

                    # try to receive a message within timeout
                    message = endpoint.recv_match(blocking=True, timeout=connection_timeout)

                    # received a message within timeout
                    if message is not None:
                        # get message buffer
                        message_buffer = message.get_msgbuf()

                        # check if vehicle exist
                        if vehicle is not None:
                            # send the received message from endpoint to vehicle
                            vehicle.write(message_buffer)

                    # not received a message within timeout or user requested program termination
                    if message is None or terminate:

                        # check endpoint is existed
                        if endpoint is not None:

                            # remove it from list
                            for i in range(len(endpoints)):
                                if endpoint.address == endpoints[i].address:
                                    del endpoints[i]

                            # close the endpoint
                            endpoint.close()

                            # clear the endpoint connection
                            endpoint = None

                        # break the inner loop
                        break

                # vehicle does not exist
                else:

                    # break the inner loop
                    break

            # unknown error occurred during receive and send a message from endpoint to vehicle
            except Exception as e:

                # check endpoint is existed
                if endpoint is not None:

                    # remove it from list
                    for i in range(len(endpoints)):
                        if endpoint.address == endpoints[i].address:
                            del endpoints[i]

                    # close the endpoint
                    endpoint.close()

                    # clear the endpoint connection
                    endpoint = None

                # break the inner loop
                break

        # user requested program termination
        if terminate:

            # check endpoint is existed
            if endpoint is not None:

                # remove it from list
                for i in range(len(endpoints)):
                    if endpoint.address == endpoints[i].address:
                        del endpoints[i]

                # close the endpoint
                endpoint.close()

                # clear the endpoint connection
                endpoint = None

            # break the outer loop
            break


@click.command()
@click.option("--master", default="tcp:127.0.0.1:5760", type=click.STRING, required=False,
              help="Standard MAVLink connection string of vehicle.")
@click.option("--out", default="udpout:127.0.0.1:14550", type=click.STRING, required=False,
              help="Comma seperated MAVLink connection string list of endpoints.")
@click.option("--timeout", default=5.0, type=click.FloatRange(min=5, clamp=True), required=False,
              help="Try to reconnect after this seconds when no message is received.")
def main(master, out, timeout):
    # get global variables
    global terminate
    global vehicle, endpoints
    global thread_vehicle, thread_endpoints

    # start master connection thread
    thread_vehicle = threading.Thread(target=connection_vehicle, args=(master, timeout))
    thread_vehicle.start()

    for device in out.split(","):
        # start slave connection thread
        connection_thread_endpoint = threading.Thread(target=connection_endpoint,
                                                      args=(device, timeout))
        connection_thread_endpoint.start()
        thread_endpoints.append(connection_thread_endpoint)

    # infinite loop
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            terminate = True
            break


# main entry point
if __name__ == "__main__":
    # run main function
    main()
