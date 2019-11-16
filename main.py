
# A very simple Flask Hello World app for you to get started with...
from __future__ import print_function
from flask import Flask
from flask_cors import CORS
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model():
  """Stores the data for the problem."""
  data = {}
  data['time_matrix'] = [
      [0, 5.6, 8, 12.8, 17.4, 13.8, 13.7, 3.4, 4.2, 11, 16.6, 10.4, 18.1, 44, 7.4, 2.4, 17.8],
      [5.6, 0, 4.7, 17.2, 2.4, 12.4, 12.8, 7.5, 8.8, 10.1, 16.4, 9, 15.9, 37.4, 11.9, 6.5, 22],
      [8, 4.7, 0, 19.5, 3.2, 16.4, 8.1, 9.8, 11.2, 5.4, 13.9, 12.9, 11.2, 41.4, 14.3, 8.9, 25.9],
      [12.8, 17.2, 19.5, 0, 18.9, 16, 25.3, 11, 11.2, 22.6, 27.9, 21.9, 27.8, 31.3, 8.9, 10.9, 18.6],
      [17.4, 2.4, 3.2, 18.9, 0, 14.1, 11.3, 9.2, 10.5, 8.6, 17.1, 10.7, 14.4, 39.1, 13.7, 8.3, 23.7],
      [13.8, 12.4, 16.4, 16, 14.1, 0, 21.1, 15.7, 17, 18.4, 24.6, 3.8, 7.1, 29.1, 20.1, 14.7, 30.5],
      [13.7, 12.8, 8.1, 25.3, 11.3, 21.1, 0, 15.6, 16.9, 4.7, 16.3, 21.3, 14.8, 56.2, 20, 14.6, 31.2],
      [3.4, 7.5, 9.8, 11, 9.2, 15.7, 15.6, 0, 4.8, 12.9, 18.2, 12.2, 20, 42, 5.8, 1, 15.8],
      [4.2, 8.8, 11.2, 11.2, 10.5, 17, 16.9, 4.8, 0, 14.2, 19.6, 13.5, 21.2, 42.3, 6.6, 4.3, 17.7],
      [11, 10.1, 5.4, 22.6, 8.6, 18.4, 4.7, 12.9, 14.2, 0, 13.6, 18.4, 12.1, 53.5, 17.3, 12, 28.2],
      [16.6, 16.4, 13.9, 27.9, 17.1, 24.6, 16.3, 18.2, 19.6, 13.6, 0, 21.2, 20.6, 58.9, 22.7, 17.3, 30.5],
      [10.4, 9, 12.9, 21.9, 10.7, 3.8, 21.3, 12.2, 13.5, 18.4, 21.2, 0, 6.2, 32.9, 16.7, 11.3, 26.3],
      [18.1, 15.9, 11.2, 27.8, 14.4, 7.1, 14.8, 20, 21.2, 12.1, 20.6, 6.2, 0, 36.2, 22.6, 19, 34.8],
      [44, 37.4, 41.4, 31.3, 39.1, 29.1, 56.2, 42, 42.3, 53.5, 58.9, 32.9, 36.2, 0, 39.8, 41.8, 36.8],
      [7.4, 11.9, 14.3, 8.9, 13.7, 20.1, 20, 5.8, 6.6, 17.3, 22.7, 16.7, 22.6, 39.8, 0, 5.7, 13.6],
      [2.4, 6.5, 8.9, 10.9, 8.3, 14.7, 14.6, 1, 4.3, 12, 17.3, 11.3, 19, 41.8, 5.7, 0, 15.7],
      [17.8, 22, 25.9, 18.6, 23.7, 30.5, 31.2, 15.8, 17.7, 28.2, 30.5, 26.3, 34.8, 36.8, 13.6, 15.7, 0]

  ]
  data['time_windows'] = [
      (7, 15),  # 0
      (9, 17),  # 1
      (9, 15),  # 2
      (6, 18),  # 3
      (9, 15),  # 4
      (9, 15),  # 5
      (9, 16),  # 6
      (9, 16),  # 7
      (9, 17),  # 8
      (13, 17),  # 9
      (13, 17), # 10
      (9, 17),  # 11
      (8, 18),  # 12
      (10, 16),  # 13
      (9, 17),  # 14
      (10, 18),  # 15
      (0, 23) # 16

  ]
  data['num_vehicles'] = 10
  data['depot'] = 16
  return data

def print_solution(data, manager, routing, assignment):
    """Prints assignment on console."""
    time_dimension = routing.GetDimensionOrDie('Time')
    solution = {}
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        solution[vehicle_id] = {
            'node':[],
            'timeMin':[],
            'timeMax':[],
            'timeTotal':0,
        }
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            solution[vehicle_id]['node'].append(manager.IndexToNode(index))
            solution[vehicle_id]['timeMin'].append(assignment.Min(time_var))
            solution[vehicle_id]['timeMax'].append(assignment.Max(time_var))
            index = assignment.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        solution[vehicle_id]['node'].append(manager.IndexToNode(index))
        solution[vehicle_id]['timeMin'].append(assignment.Min(time_var))
        solution[vehicle_id]['timeMax'].append(assignment.Max(time_var))

        solution[vehicle_id]['timeTotal'] = (assignment.Max(time_var))

    finalResult = ''
    for vehicleId in solution:
        finalResult += str(vehicleId) + '|'
        for target_list in solution[vehicleId]:
            if target_list != 'timeTotal':
                for items in solution[vehicleId][target_list]:
                    finalResult += str(items) +','
            if target_list == 'timeTotal':
                    finalResult += str(solution[vehicleId][target_list])
            finalResult += '|'
        finalResult += '-'

    print(finalResult)
    return finalResult

def main():
    """Solve the VRP with time windows."""
    data = create_data_model()
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]/10

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    time = 'Time'
    routing.AddDimension(transit_callback_index, 30, 30, False, time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0], data['time_windows'][0][1])
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
        return print_solution(data, manager, routing, assignment)

main()

