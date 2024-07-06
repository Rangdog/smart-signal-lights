# import tensorflow as tf
# import traci
# sumoCmd = ["sumo", "-c", "vande.sumocfg"]
# sumoCmdGui = ["sumo-gui", "-c", "vande.sumocfg"]
# # Kết nối với môi trường SUMO
# traci.start(sumoCmdGui)

# # Lấy danh sách IDs của các làn đường
# # lane_ids = traci.lane.getIDList()
# # edge_ids = traci.edge.getIDList()
# # print(edge_ids)
# # print(lane_ids)
# # traci.lane.getLastStepOccupancy("d1GiaoLo_0")
# # print(traci.edge.getLastStepVehicleNumber("GiaoLod1"))
# while traci.simulation.getMinExpectedNumber() > 0:
#     traci.simulationStep()
#     number_vehicles_stopped = 0
#     vehicles = traci.edge.getLastStepVehicleIDs("d1GiaoLo")

#     for vehicle_id in vehicles:
#         WaitingTime = traci.vehicle.getWaitingTime(vehicle_id)
#         print(vehicle_id, WaitingTime)

# # Đóng kết nối với SUMO sau khi hoàn thành
# traci.close()


# # # import tensorflow as tf

# # # # Kiểm tra phiên bản TensorFlow
# # # print("Phiên bản TensorFlow:", tf.__version__)

# # # # Kiểm tra phiên bản CUDA
# # # print("Phiên bản CUDA:", tf.test.is_built_with_cuda())

# # # # Kiểm tra phiên bản cuDNN
# # # print("Phiên bản cuDNN:", tf.test.is_built_with_gpu_support())

# # # # Kiểm tra xem cuDNN đã được cấu hình và TensorFlow nhận diện hay chưa
# # # if tf.test.is_built_with_cuda() and tf.test.is_built_with_gpu_support():
# # #     print("cuDNN đã được cấu hình và TensorFlow nhận diện.")
# # # else:
# # #     print("cuDNN chưa được cấu hình hoặc TensorFlow không nhận diện.")


# # # print("Is GPU available:", tf.config.list_physical_devices('GPU'))


# # base = 1.01
# # exponent = 0.3
# # num_rounds = 3000

# # values = []
# # current_value = 1  # Giá trị ban đầu


# # def exponential_growth(time, base, exponent):
# #     """
# #     Hàm mũ tăng đần theo thời gian.

# #     Args:
# #         time (float): Thời gian.
# #         base (float): Cơ số của hàm mũ.
# #         exponent (float): Số mũ của hàm mũ.

# #     Returns:
# #         float: Giá trị của hàm mũ tại thời điểm time.
# #     """
# #     return base ** (exponent * time)


# # for _ in range(num_rounds):
# #     current_value = exponential_growth(
# #         _, base, exponent)  # Thời gian là 1 (cố định)
# #     values.append(current_value)


# # # In ra giá trị tại 100 vòng
# # total = 0

# # for i, value in enumerate(values):
# #     total += value
# #     print(f"Vòng {i+1}: Giá trị = {value}")
# # print(total)

# # print(89 % 90 + 10)
import xml.etree.ElementTree as ET
import random
from xml.dom import minidom


def create_vehicle_element(id, depart, edges):
    vehicle = ET.Element("vehicle")
    vehicle.set("id", str(id))
    vehicle.set("depart", str(depart))

    route = ET.SubElement(vehicle, "route")
    route.set("edges", edges)

    return vehicle


vehicle_idex = 0


def generate_vehicle_data(num_vehicles, depart_range, edges_list, vehicle_idex):
    vehicle_data = []
    for i in range(num_vehicles):
        vehicle_id = vehicle_idex
        depart_time = random.uniform(depart_range[0], depart_range[1])
        edges = random.choice(edges_list)

        vehicle_info = {
            "id": vehicle_id,
            "depart": depart_time,
            "edges": edges
        }

        vehicle_data.append(vehicle_info)
        vehicle_idex += 1

    return vehicle_data, vehicle_idex


depart_time_range = (0, 1000)
# Thay bằng danh sách edges của bạn
edges_list = ["d1GiaoLo GiaoLod2", "d1GiaoLo GiaoLod3", "d1GiaoLo GiaoLod4", "d2GiaoLo GiaoLod1", "d2GiaoLo GiaoLod3", "d2GiaoLo GiaoLod4",
              "d3GiaoLo GiaoLod1", "d3GiaoLo GiaoLod2", "d3GiaoLo GiaoLod4", "d4GiaoLo GiaoLod1", "d4GiaoLo GiaoLod2", "d4GiaoLo GiaoLod3"]

edges_list_main = ["d1GiaoLo GiaoLod2", "d1GiaoLo GiaoLod3", "d1GiaoLo GiaoLod4",
                   "d4GiaoLo GiaoLod1", "d4GiaoLo GiaoLod2", "d4GiaoLo GiaoLod3"]
edges_list_sub = ["d2GiaoLo GiaoLod1", "d2GiaoLo GiaoLod3", "d2GiaoLo GiaoLod4",
                  "d3GiaoLo GiaoLod1", "d3GiaoLo GiaoLod2", "d3GiaoLo GiaoLod4",]
num_vehicles_main = 1000  # Số lượng phương tiện cần tạo
num_vehicles_sub = 200
vehicle_data = []
vehicles_for_10_cars, vehicle_idex = generate_vehicle_data(
    num_vehicles_main, depart_time_range, edges_list_main, vehicle_idex)
vehicle_data.extend(vehicles_for_10_cars)
print(vehicle_idex)
vehicles_for_1_car, vehicle_idex = generate_vehicle_data(
    num_vehicles_sub, depart_time_range, edges_list_sub, vehicle_idex)
vehicle_data.extend(vehicles_for_1_car)
sorted_vehicle_data = sorted(vehicle_data, key=lambda x: x["depart"])
root = ET.Element("routes")

for vehicle_info in sorted_vehicle_data:
    vehicle = ET.Element("vehicle")
    vehicle.set("id", str(vehicle_info["id"]))
    vehicle.set("depart", str(vehicle_info["depart"]))

    route = ET.SubElement(vehicle, "route")
    route.set("edges", vehicle_info["edges"])

    root.append(vehicle)

tree_str = ET.tostring(root, encoding="utf-8")
parsed_tree = minidom.parseString(tree_str)
formatted_xml = parsed_tree.toprettyxml(indent="    ")
with open("generated_vehicles.xml", "w") as f:
    f.write(formatted_xml)

# root = ET.Element("routes")

# vehicle_id = 0
# time_interval = 10  # Khoảng thời gian giữa các xe

# for i in range(50):  # Ví dụ: Tạo 10 xe
#     depart_time = i * time_interval
#     edges = "d2GiaoLo GiaoLod1"
#     vehicle = create_vehicle_element(vehicle_id, depart_time, edges)
#     root.append(vehicle)
#     vehicle_id += 1

# for i in range(50):  # Ví dụ: Tạo 10 xe
#     depart_time = i * time_interval
#     edges = "d2GiaoLo GiaoLod3"
#     vehicle = create_vehicle_element(vehicle_id, depart_time, edges)
#     root.append(vehicle)
#     vehicle_id += 1

# for i in range(50):  # Ví dụ: Tạo 10 xe
#     depart_time = i * time_interval
#     edges = "d2GiaoLo GiaoLod4"
#     vehicle = create_vehicle_element(vehicle_id, depart_time, edges)
#     root.append(vehicle)
#     vehicle_id += 1

# for i in range(50):  # Ví dụ: Tạo 10 xe
#     depart_time = i * time_interval
#     edges = "d3GiaoLo GiaoLod1"
#     vehicle = create_vehicle_element(vehicle_id, depart_time, edges)
#     root.append(vehicle)
#     vehicle_id += 1

# for i in range(50):  # Ví dụ: Tạo 10 xe
#     depart_time = i * time_interval
#     edges = "d3GiaoLo GiaoLod2"
#     vehicle = create_vehicle_element(vehicle_id, depart_time, edges)
#     root.append(vehicle)
#     vehicle_id += 1

# for i in range(50):  # Ví dụ: Tạo 10 xe
#     depart_time = i * time_interval
#     edges = "d3GiaoLo GiaoLod4"
#     vehicle = create_vehicle_element(vehicle_id, depart_time, edges)
#     root.append(vehicle)
#     vehicle_id += 1

# tree = ET.ElementTree(root)
# tree_str = ET.tostring(root, encoding="utf-8")
# parsed_tree = minidom.parseString(tree_str)
# formatted_xml = parsed_tree.toprettyxml(indent="    ")

# with open("generated_vehicles.xml", "w") as f:
#     f.write(formatted_xml)
