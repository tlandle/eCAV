import carla
import random
import os
from agents.navigation.global_route_planner import GlobalRoutePlanner
from jinja2 import FileSystemLoader, Environment
from ecav.scenario_testing.utils.yaml_utils import load_yaml

# Add a seed value for random number generation
random.seed(42)

def generate_spawn_positions(num_cars, x_range=(100, 600), y_range=(234, 255), z_offset=0.3, spawn=True, dest_proximity=10):
    # Generate random points within the range provided
    positions = []
    for _ in range(num_cars):
        while True:
            x = random.uniform(x_range[0], x_range[1])
            y = random.uniform(y_range[0], y_range[1])
            location = carla.Location(x=x, y=y)

            # Get the nearest waypoint to the location
            waypoint = current_map.get_waypoint(location)
            waypoint_loc = waypoint.transform.location
            new_position = [waypoint_loc.x, waypoint_loc.y, waypoint_loc.z + z_offset, 0, 0, 0]

            # Check if this position is already in use
            if new_position not in positions:
                break  # This position is not in use, so we can break the loop and use it

        if spawn:
            print("Waypoint_loc: ", waypoint_loc.x, waypoint_loc.y)
            positions.append(new_position)
        else:
            # Generate valid destinations
            valid_destinations = filter_spawn_points_for_destinations(num_cars, x, y_range=(137, 154))

            if not valid_destinations:
                raise ValueError(f"No valid destinations ahead of spawn point {x}, {y}")

            # If it's the first destination, choose any
            if not positions:
                first_dest = random.choice(valid_destinations)
                x, y = first_dest[0], first_dest[1]
            else:
                # For subsequent vehicles, their destination is chosen to be within `dest_proximity` of the first destination
                valid_destinations = [point for point in valid_destinations if ((point[0] - positions[0][0]) ** 2 + (point[1] - positions[0][1]) ** 2) ** 0.5 < dest_proximity]
                if not valid_destinations:
                    raise ValueError(f"No valid destinations close to the first destination {positions[0][0]}, {positions[0][1]}")

                destination_point = random.choice(valid_destinations)
                x, y = destination_point[0], destination_point[1]

            positions.append([x, y, waypoint.transform.location.z + z_offset, 0, 0, 0])

    return positions


def filter_spawn_points_for_destinations(num_cars, start_x, y_range=(137, 154)):
    valid_points = []
    for _ in range(num_cars):
        y = random.uniform(y_range[0], y_range[1])
        location = carla.Location(x=start_x, y=y)

        # Get the nearest waypoint to the location
        waypoint = current_map.get_waypoint(location)

        waypoint_loc = waypoint.transform.location
        valid_points.append([waypoint_loc.x, waypoint_loc.y, waypoint_loc.z, 0, 0, 0])
    return valid_points

def choose_template():
    template_dir = os.path.abspath('./templates')
    templates = [file for file in os.listdir(template_dir) if file.endswith('.yaml')]
    for i, template in enumerate(templates, start=1):
        print(f"{i}. {template}")
    choice = int(input("Choose a template by number: "))
    return templates[choice - 1]

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]    

client = carla.Client(CARLA_IP, 2000)
client.set_timeout(10)
world = client.load_world('Town06')
current_map = world.get_map()

num_cars = int(input("Enter the number of vehicles to generate: "))
spawn_positions = generate_spawn_positions(num_cars, x_range=(40, 300), y_range=(130, 155), spawn=True)
# destinations = generate_spawn_positions(num_cars, x_range=(450, 800), y_range=(-255, -234), spawn=False)

# Generate the vehicles as a list of dictionaries
vehicles = []
for i in range(num_cars):
    vehicles.append({
        "spawn_position": spawn_positions[i],
        "destination": [606.87, 145.39, 0],
        # "destination": destinations[i]
    })
# Get the absolute path to the templates directory
template_dir = os.path.abspath('./templates')

# Ask the user to choose a template
template_file = choose_template()

# Load the Jinja2 template and render it
env = Environment(loader=FileSystemLoader(template_dir))
template = env.get_template(template_file)
output_yaml = template.render(vehicles=vehicles)

output_file_name = f"./ecav/scenario_testing/config_yaml/ecloud_4lane_dist_{num_cars}_car.yaml"
with open(output_file_name, "w") as file:
    file.write(output_yaml)

print(f"Generated YAML file with {num_cars} vehicles: {output_file_name}")

# Also create a copy of the chosen template
template_python_file_name = f"{template_file.rsplit('.', 1)[0]}.py"
output_py_file_name = f"./ecav/scenario_testing/ecloud_4lane_dist_{num_cars}_car.py"
with open(f'{template_dir}/{template_python_file_name}', 'r') as source:
    with open(output_py_file_name, 'w') as target:
        target.write(source.read())

print(f"Generated Python file from the template: {output_py_file_name}")
