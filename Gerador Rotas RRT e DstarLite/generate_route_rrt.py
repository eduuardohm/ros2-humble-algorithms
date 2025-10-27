import cv2
import numpy as np
import csv
# from path_planning.rrt import rrt  # do PythonRobotics
from PythonRobotics.PathPlanning.RRT import rrt
import matplotlib.pyplot as plt
import random
import numpy as np

random.seed(45)      # controla o random padrão
np.random.seed(45)   # controla o random do NumPy

# Parâmetros
# pgm_file = "interlagos_map.pgm"
# pgm_file = "Mapas\Interlagos_4000x4000_rota.pgm"
pgm_file = "Mapas\Interlagos_v4_rota.pgm"
# pgm_file = "GeradorRotas\Interlagos 400\interlagos_bg_branco_400.pgm"

resolution = 0.05  # igual ao seu .yaml
origin = [-78.0, -93.0]

# resolution = 0.05 
# origin = [-10.0, -10.0]

# origin = [0.0, 0.0]  # igual ao seu .yaml
start = (3.0, -3.0)  # coordenada em metros [ex: (-3.0, 0.05)]
goal = (-2.0, -8.0)          # coordenada em metros
# start = (145.0, 70.0)
# start = (0.0, 0.0)
# goal = (9.0, 2.0)

# Carrega o mapa e cria grid binário
img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
obstacle_map = img < 128  # preto=obstáculo

# Converte coord. reais para índices de pixel
def world_to_map(pos):
    x, y = pos
    map_x = int((x - origin[0]) / resolution)
    # map_y = int(img.shape[0] - 1 - (y - origin[1]) / resolution)
    map_y = int((y - origin[1]) / resolution)
    return map_x, map_y

# Inverso: pixel para coordenada real
def map_to_world(ix, iy):
    x = ix * resolution + origin[0]
    y = iy * resolution + origin[1]

    # y = (img.shape[0] - 1 - iy) * resolution + origin[1]
    return x, y

# x_origin, y_origin = map_to_world(200, 200)
# print(f"offset mapa grande: {x_origin}, {y_origin}")

ix_start, iy_start = world_to_map(start)
print(f"Start: {ix_start}, {iy_start}")
teste_x, teste_y = map_to_world(ix_start, iy_start)
print(f"Teste: {teste_x}, {teste_y}")

ix_goal, iy_goal = world_to_map(goal)
print(f"Goal: {ix_goal}, {iy_goal}")

# Lista de obstaculos
obstacle_list = []
for iy in range(img.shape[0]):
    for ix in range(img.shape[1]):
        if obstacle_map[iy, ix]:
            # cada obstáculo é (x, y, raio)
            obstacle_list.append((ix, iy, 1))

# RRT
# rrt_path = rrt.planning(
#     start=[ix_start, iy_start],
#     goal=[ix_goal, iy_goal],
#     obstacle_map=obstacle_map,
#     rand_area=[0, img.shape[1], 0, img.shape[0]],
#     max_iter=5000
# )

# ========================
# Configura e executa o RRT
# ========================
print("Iniciando RRT...")

rrt_obj = rrt.RRT(
    start=[ix_start, iy_start],
    goal=[ix_goal, iy_goal],
    rand_area=[0, img.shape[1], 0, img.shape[0]],
    obstacle_list=obstacle_list,
    expand_dis=10.0,
    path_resolution=2.0,
    robot_radius=4.0,
    goal_sample_rate=5,
    max_iter=5000
)

# plt.figure()
# plt.imshow(img, cmap='gray')
# plt.scatter(ix_start, iy_start, c='g', s=80, label='Start')  # verde
# plt.scatter(ix_goal, iy_goal, c='r', s=80, label='Goal')     # vermelho
# plt.title("Mapa com Start e Goal")
# plt.legend()

# Ajusta zoom opcional (mantém proporções parecidas com seu mapa)
# margin = 100
# xmin = min(ix_start, ix_goal) - margin
# xmax = max(ix_start, ix_goal) + margin
# ymin = min(iy_start, iy_goal) - margin
# ymax = max(iy_start, iy_goal) + margin

# plt.xlim(xmin, xmax)
# plt.ylim(ymax, ymin)  # note o inverso (origem da imagem é no canto superior esquerdo)

plt.show()

rrt_path = rrt_obj.planning(animation=False)

if rrt_path is None:
    raise RuntimeError("O RRT não conseguiu encontrar um caminho até o objetivo.")

plt.imshow(img, cmap='gray')
path_pixels = np.array(rrt_path)
plt.plot(path_pixels[:,0], path_pixels[:,1], '-r')
plt.scatter(ix_start, iy_start, c='g', label='Start')
plt.scatter(ix_goal, iy_goal, c='b', label='Goal')

margin = 100
xmin = min(ix_start, ix_goal) - margin
xmax = max(ix_start, ix_goal) + margin
ymin = min(iy_start, iy_goal) - margin
ymax = max(iy_start, iy_goal) + margin

plt.xlim(xmin, xmax)
plt.ylim(ymax, ymin)  # note o inverso

plt.legend()
plt.show()

# Salva rota em CSV
with open('rota_rrt.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for (ix, iy) in reversed(rrt_path):
        x, y = map_to_world(ix, iy)
        writer.writerow([x, y])

print("Caminho salvo como 'rota_rrt.csv'.")
