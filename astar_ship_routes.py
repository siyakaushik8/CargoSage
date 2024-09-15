import heapq
from math import radians, sin, cos, sqrt, atan2

class Port:
    def __init__(self, name, lat, lon):
        self.name = name
        self.lat = radians(lat)
        self.lon = radians(lon)

def haversine_distance(port1, port2):
    R = 6371  # Earth's radius in kilometers

    dlat = port2.lat - port1.lat
    dlon = port2.lon - port1.lon
    
    a = sin(dlat/2)**2 + cos(port1.lat) * cos(port2.lat) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    return R * c

def fuel_consumption(distance, speed=20):
    # Simplified fuel consumption model
    # Assumes 0.3 tons per 1000 km at 20 knots
    return (distance / 1000) * 0.3 * (speed / 20)**2

def astar(start, goal, get_neighbors):
    def heuristic(port):
        return haversine_distance(port, goal)

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor, distance in get_neighbors(current):
            tentative_g_score = g_score[current] + fuel_consumption(distance)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Create a list of major ports
ports = [
    Port("Shanghai", 31.2304, 121.4737),
    Port("Singapore", 1.2902, 103.8519),
    Port("Ningbo-Zhoushan", 29.8683, 121.5440),
    Port("Shenzhen", 22.5431, 114.0579),
    Port("Guangzhou", 23.1291, 113.2644),
    Port("Busan", 35.1796, 129.0756),
    Port("Qingdao", 36.0671, 120.3826),
    Port("Hong Kong", 22.3193, 114.1694),
    Port("Tianjin", 39.0042, 117.7145),
    Port("Rotterdam", 51.9225, 4.4792),
    Port("Antwerp", 51.2213, 4.3998),
    Port("Hamburg", 53.5511, 9.9937),
    Port("Los Angeles", 33.7288, -118.2620),
    Port("Long Beach", 33.7541, -118.2159),
    Port("New York", 40.7128, -74.0060)
]

# Create a dictionary to store port objects by name for easy lookup
port_dict = {port.name: port for port in ports}

def get_neighbors(port):
    # Allow more neighbors, since shipping routes might connect distant ports
    distances = [(p, haversine_distance(port, p)) for p in ports if p != port]
    return sorted(distances, key=lambda x: x[1])[:10
                                                 ]  # Increase neighbor count


# Example usage
start_port = port_dict["Shanghai"]
end_port = port_dict["Rotterdam"]

path = astar(start_port, end_port, get_neighbors)

if path:
    print("Optimal path:")
    total_distance = 0
    for i in range(len(path) - 1):
        distance = haversine_distance(path[i], path[i+1])
        total_distance += distance
        print(f"{path[i].name} -> {path[i+1].name} ({distance:.2f} km)")
    print(f"\nTotal distance: {total_distance:.2f} km")
    print(f"Estimated fuel consumption: {fuel_consumption(total_distance):.2f} tons")
else:
    print("No path found")