import traci
import numpy as np
import random
from collections import deque
import tensorflow as tf
import time
import xml.etree.ElementTree as ET
from xml.dom import minidom

GAMMA = 0.95
LEARNING_RATE = 0.001
MEMORY_SIZE = 1000000
BATCH_SIZE = 100
EXPLORATION_MAX = 1.0
EXPLORATION_MIN = 0.01
EXPLORATION_DECAY = 0.996
sumoCmd = ["sumo", "-c", "vande.sumocfg"]
sumoCmdGui = ["sumo-gui", "-c", "vande.sumocfg"]
phase1 = ["d2GiaoLo", "d3GiaoLo"]
phase2 = ["d1GiaoLo", "d4GiaoLo"]
actions = ["GGGgrrrrGGGgrrrr",
           "rrrrGGGgrrrrGGGg"]
tfl_second = [10, 15, 20, 25, 30, 35, 40, 45, 50, 55]
vehicles_passed = {}

input_lane = ["d1GiaoLo_0", "d1GiaoLo_1", "d2GiaoLo_0", "d2GiaoLo_1",
              "d3GiaoLo_0", "d3GiaoLo_1", "d4GiaoLo_0", "d4GiaoLo_1"]


def normalize(time, step):
    total = 0
    # if step < 1000:
    #     return total
    for i in range(step, time):
        total += ((i+1)/100)
    return total


def normalize_stopped(x):
    return x*2.5


class TrafficSignalController:
    def __init__(self, trafficlights_id):
        self.trafficlights_id = trafficlights_id

    def take_action(self, action):
        light_state = self.convert_action_to_light_state(action)
        traci.trafficlight.setRedYellowGreenState(
            self.trafficlights_id, light_state)

    def convert_action_to_light_state(self, action):
        if (action < 10):
            return "GGGgrrrrGGGgrrrr"
        else:
            return "rrrrGGGgrrrrGGGg"


class DQNAgent:
    def __init__(self, state_size, action_size, trafficlights_id):
        self.state_size = state_size
        self.action_size = action_size
        self.exploration_rate = EXPLORATION_MAX
        self.memory = deque(maxlen=MEMORY_SIZE)
        self.model = tf.keras.models.load_model('traffic_light_dqn10.h5')
        # self.model = self.build_model()
        self.traffic_signal_controller = TrafficSignalController(
            trafficlights_id)

    def build_model(self):
        model = tf.keras.Sequential()
        model.add(tf.keras.layers.Dense(
            256, input_dim=self.state_size, activation='relu'))
        model.add(tf.keras.layers.Dropout(0.5))  # Thêm lớp Dropout
        # Tăng số lượng lớp Dense
        model.add(tf.keras.layers.Dense(128, activation='relu'))
        model.add(tf.keras.layers.Dropout(0.5))  # Thêm lớp Dropout
        # Tăng số lượng lớp Dense
        model.add(tf.keras.layers.Dense(64, activation='relu'))
        model.add(tf.keras.layers.Dense(self.action_size, activation='linear'))
        optimizer = tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE)
        model.compile(loss='mse', optimizer=optimizer)
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() < self.exploration_rate:
            return random.randrange(self.action_size)
        q_values = self.model.predict(state)
        return np.argmax(q_values[0])

    def experience_replay(self):
        if len(self.memory) < BATCH_SIZE:
            return
        batch = random.sample(self.memory, BATCH_SIZE)
        states_batch = []
        q_values_batch = []
        for state, action, reward, next_state, done in batch:
            q_update = reward
            if not done:
                q_update = (reward + GAMMA *
                            np.amax(self.model.predict(next_state)))
            q_values = self.model.predict(state)
            q_values[0, action] = q_update
            states_batch.append(state)
            q_values_batch.append(q_values)
        states_batch = np.array(states_batch)
        q_values_batch = np.array(q_values_batch)
        for x, y in zip(states_batch, q_values_batch):
            self.model.fit(x, y, epochs=1, verbose=0)
        if self.exploration_rate > EXPLORATION_MIN:
            self.exploration_rate *= EXPLORATION_DECAY


class SumoEnvironment:
    def __init__(self, trafficlights_id):
        # Khởi tạo môi trường SUMO và kết nối đến traci
        # traci.start(sumoCmd)
        traci.start(sumoCmdGui)
        self.time_step = 0
        self.time_window = 5000
        self.last_step_cal = 0
        self.last_phase = 0
        self.traffic_signal_controller = TrafficSignalController(
            trafficlights_id)

    def update(self):
        # Khai báo số xe dừng đèn đỏ
        num_vehicles_stopped = 0
        # Cập nhật thông tin về môi trường SUMO
        traci.simulationStep()
        self.time_step += 1
        # Tính số lượng xe đi qua giao lộ trong khoảng thời gian
        num_vehicles_passed = self.get_num_vehicles()
        if self.current_phase == 0:
            num_vehicles_stopped += self.calculate_stopped_vehicles(
                "d2GiaoLo")
            num_vehicles_stopped += self.calculate_stopped_vehicles(
                "d3GiaoLo")
        elif self.current_phase == 1:
            num_vehicles_stopped += self.calculate_stopped_vehicles(
                "d1GiaoLo")
            num_vehicles_stopped += self.calculate_stopped_vehicles(
                "d4GiaoLo")
        return num_vehicles_passed, num_vehicles_stopped

    def get_num_vehicles(self):
        # Hàm này trả về số lượng xe đi qua giao lộ tại thời điểm hiện tại
        vehicles = traci.edge.getLastStepVehicleIDs("GiaoLod1") + traci.edge.getLastStepVehicleIDs(
            "GiaoLod2") + traci.edge.getLastStepVehicleIDs("GiaoLod3") + traci.edge.getLastStepVehicleIDs("GiaoLod4")
        num_vehicles_passed = 0
        for vehicle_id in vehicles:
            if vehicle_id not in vehicles_passed:
                vehicles_passed[vehicle_id] = 1
                num_vehicles_passed += 1

        return num_vehicles_passed

    def calculate_stopped_vehicles(self, edge_id):
        number_vehicles_stopped = 0
        vehicles = traci.edge.getLastStepVehicleIDs(edge_id)
        for vehicle_id in vehicles:
            WaitingTime = traci.vehicle.getWaitingTime(vehicle_id)
            number_vehicles_stopped += normalize_stopped(WaitingTime)
        return number_vehicles_stopped

    def calculate_stopped_vehicles_2(self, edge_id):
        number_vehicles_stopped = 0
        vehicles = traci.edge.getLastStepVehicleIDs(edge_id)
        for vehicle_id in vehicles:
            WaitingTime = traci.vehicle.getWaitingTime(vehicle_id)
            if (WaitingTime > 0):
                number_vehicles_stopped += 1
        return number_vehicles_stopped

    def set_last_step_cal(self, x):
        self.last_step_cal = x

    def get_reward(self, vehicles_pass, vehicles_stopped_score):
        pen_time = normalize(self.time_step, self.last_step_cal)
        self.set_last_step_cal(self.time_step)
        return (vehicles_pass*10) - vehicles_stopped_score - pen_time

    def get_state(self):
        lane_ids = input_lane
        lane_occupancies = [traci.lane.getLastStepOccupancy(
            lane_id) for lane_id in lane_ids]
        lane_densities = [traci.lane.getLastStepVehicleNumber(
            lane_id) for lane_id in lane_ids]
        lane_velocities = [traci.lane.getLastStepMeanSpeed(
            lane_id) for lane_id in lane_ids]
        # Lấy thông tin về số lượng xe đang dừng đèn đỏ
        num_vehicles_stopped_direction_1 = 0
        num_vehicles_stopped_direction_2 = 0
        for i in phase1:
            num_vehicles_stopped_direction_1 += self.calculate_stopped_vehicles(
                i)
        for i in phase2:
            num_vehicles_stopped_direction_2 += self.calculate_stopped_vehicles(
                i)
        # Ghép thông tin trạng thái thành một numpy array
        state = np.array(lane_occupancies + lane_densities +
                         lane_velocities + [num_vehicles_stopped_direction_1] + [num_vehicles_stopped_direction_2])
        state = np.expand_dims(state, axis=0)
        return state

    def reset(self, trafficlights_id):
        # Reset môi trường Sumo về trạng thái ban đầu
        traci.close()
        traci.start(sumoCmd)
        self.time_step = 0
        # Độ dài của khoảng thời gian (300 giây tương đương 5 phút)
        self.time_window = 5000
        self.traffic_signal_controller = TrafficSignalController(
            trafficlights_id)
        self.current_phase = 0
        self.last_step_cal = 0
        # Lấy trạng thái ban đầu của môi trường
        state = self.get_state()
        return state

    def step(self, action):
        if (action < 10):
            self.current_phase = 0
        else:
            self.current_phase = 1
        num_vehicles_passed, num_vehicles_stopped = self.update()
        self.traffic_signal_controller.take_action(action)
        second = tfl_second[action % 10]
        for _ in range(second):
            x, y = self.update()
            if self.is_done():
                break
            num_vehicles_passed += x
            num_vehicles_stopped += y
        next_state = self.get_state()

        reward = self.get_reward(num_vehicles_passed, num_vehicles_stopped)

        done = self.is_done()
        return next_state, reward, done

    def is_done(self):
        if traci.simulation.getMinExpectedNumber() == 0:
            return True
        return self.time_step >= self.time_window

    def close(self):
        # Đóng kết nối traci khi kết thúc môi trường
        traci.close()


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


def gen_rou_file():
    vehicle_idex = 0
    vehicle_data = []
    num_vehicles_1 = random.randint(300, 1000)
    num_vehicles_2 = random.randint(300, 1000)
    vehicles_for_10_cars, vehicle_idex = generate_vehicle_data(
        num_vehicles_1, depart_time_range, edges_list_main, vehicle_idex)
    vehicle_data.extend(vehicles_for_10_cars)
    vehicles_for_1_car, vehicle_idex = generate_vehicle_data(
        num_vehicles_2, depart_time_range, edges_list_sub, vehicle_idex)
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
    with open("test.xml", "w") as f:
        f.write(formatted_xml)

# sumoCmd = ["sumo", "-c", "vande.sumocfg"]
# traci.start(sumoCmd)

# edge_id = "d1GiaoLo"

# simulation_time = 200
# num_vehicles_list = []

# for _ in range(simulation_time):
#     traci.simulationStep()
#     num_vehicles = traci.edge.getLastStepVehicleNumber(edge_id)
#     num_vehicles_list.append(num_vehicles)


# traci.close()
# print("Thời gian\tSố lượng xe")
# for t, num_vehicles in enumerate(num_vehicles_list):
#     print(f"{t}\t\t{num_vehicles}")

# start_time = time.time()
# env = SumoEnvironment("J11")
# while env.time_step <= env.time_window:
#     print(env.time_step, env.update())
# train
# start_time = time.time()
# env = SumoEnvironment("J11")
# state_size = 26
# action_size = 20
# agent = DQNAgent(state_size, action_size, "J11")
# save_interval = 200
# epochs = 1500
# for epoch in range(epochs):
#     gen_rou_file()
#     vehicles_passed.clear()
#     state = env.reset("J11")  # Khởi tạo trạng thái ban đầu của môi trường
#     total_reward = 0
#     done = False

#     while not done:
#         # Chọn hành động dựa trên trạng thái hiện tại
#         action = agent.act(state)

#         # Thực hiện hành động và nhận lại thông tin từ môi trường
#         next_state, reward, done = env.step(action)

#         # Lưu trữ kinh nghiệm trong bộ nhớ của agent
#         agent.remember(state, action, reward, next_state, done)

#         # Cập nhật trạng thái
#         state = next_state
#         total_reward += reward

#     # Cập nhật mô hình DQN sau mỗi epoch
#     agent.experience_replay()
#     if (epoch + 1) % save_interval == 0:
#         agent.model.save('traffic_light_dqn14.h5')
#     # Giảm tỷ lệ thăm dò theo thời gian

#     # if agent.exploration_rate > EXPLORATION_MIN:
#     #     agent.exploration_rate *= EXPLORATION_DECAY
#     # In kết quả sau mỗi epoch
#     print(f"Epoch: {epoch}, Total Reward: {total_reward}")

# # Lưu mô hình sau khi huấn luyện
# agent.model.save('traffic_light_dqn14.h5')
# end_time = time.time()
# elapsed_time = end_time - start_time
# print("Thời gian chạy là: {} giây".format(elapsed_time))

# Thực thi


def make_decision(model, state):
    action = model.predict(state)
    return action


loaded_model = tf.keras.models.load_model('traffic_light_dqn11.h5')
sumo_env = SumoEnvironment("J11")
time = 0
for step in range(15000):
    state = sumo_env.get_state()
    action = loaded_model.predict(state)[0]
    act = np.argmax(action)
    print(act)
    sumo_env.traffic_signal_controller.take_action(
        act)
    second = tfl_second[act % 10]
    for i in range(second):
        traci.simulationStep()
        if (traci.simulation.getMinExpectedNumber() == 0):
            break

    if (traci.simulation.getMinExpectedNumber() == 0):
        break
    time += 1
# for step in range(150000):
#     traci.simulationStep()
#     if (traci.simulation.getMinExpectedNumber() == 0):
#         break
sumo_env.close()
print(time)
