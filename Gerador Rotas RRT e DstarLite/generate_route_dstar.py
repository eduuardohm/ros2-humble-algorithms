import cv2
import numpy as np
import csv
# from path_planning.d_start_lite import d_star_lite
from PythonRobotics.PathPlanning.DStarLite import d_star_lite
import matplotlib.pyplot as plt
import random

random.seed(45)      # controla o random padrão
np.random.seed(45)   # controla o random do NumPy

# ========================
# Parâmetros - NÃO MODIFICAR!
# ========================
pgm_file = "Mapas/interlagos_v4_rota.pgm"
resolution = 0.05  # igual ao seu .yaml
origin = [-78.0, -93.0]
start = (3.0, -3.0)  # coordenada em metros
goal = (-2.0, -8.0)  # coordenada em metros

# Carrega o mapa e cria grid binário
print("="*50)
print("Algoritmo D* Lite")
print("="*50)
print(f"Carregando mapa: {pgm_file}")
img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError(f"Não foi possível carregar o arquivo {pgm_file}")

obstacle_map = img < 128  # preto=obstáculo

# Aplica dilatação para aumentar a área dos obstáculos (margem de segurança)
# Isso faz o robô ficar mais longe das bordas
kernel_size = 10  # aumenta este valor para mais distância das paredes
kernel = np.ones((kernel_size, kernel_size), np.uint8)
obstacle_map = cv2.dilate(obstacle_map.astype(np.uint8), kernel, iterations=1).astype(bool)
print(f"Margem de segurança aplicada: {kernel_size}x{kernel_size} pixels (~{kernel_size * resolution:.2f} metros)")

# Converte coord. reais para índices de pixel
def world_to_map(pos):
    x, y = pos
    map_x = int((x - origin[0]) / resolution)
    map_y = int((y - origin[1]) / resolution)
    return map_x, map_y

# Inverso: pixel para coordenada real
def map_to_world(ix, iy):
    x = ix * resolution + origin[0]
    y = iy * resolution + origin[1]
    return x, y

ix_start, iy_start = world_to_map(start)
print(f"Start: {start} -> pixel ({ix_start}, {iy_start})")
teste_x, teste_y = map_to_world(ix_start, iy_start)
print(f"Teste conversão inversa: ({teste_x:.2f}, {teste_y:.2f})")

ix_goal, iy_goal = world_to_map(goal)
print(f"Goal: {goal} -> pixel ({ix_goal}, {iy_goal})")

# Extrai posições dos obstáculos
print("Extraindo obstáculos do mapa...")
obstacle_x = []
obstacle_y = []

# Para D* Lite, vamos usar TODOS os obstáculos (sem amostragem)
# para garantir que não passe por cima de linhas finas
print("ATENÇÃO: Usando todos os obstáculos (sem amostragem) - pode demorar!")
for iy in range(img.shape[0]):
    for ix in range(img.shape[1]):
        if obstacle_map[iy, ix]:
            obstacle_x.append(ix)
            obstacle_y.append(iy)

print(f"Total de obstáculos: {len(obstacle_x)}")

# Desativa animação do D* Lite
d_star_lite.show_animation = False

# Cria instância do D* Lite
print("\nIniciando D* Lite...")
print("Nota: D* Lite pode demorar alguns minutos em mapas grandes...")

try:
    dstar = d_star_lite.DStarLite(ox=obstacle_x, oy=obstacle_y)
    
    # Define start e goal (coordenadas do mundo)
    start_node = d_star_lite.Node(x=ix_start, y=iy_start)
    goal_node = d_star_lite.Node(x=ix_goal, y=iy_goal)
    
    # D* Lite detectará obstáculos dinamicamente
    # Para nosso caso estático, não precisamos de obstáculos "spoofed"
    spoofed_ox = []  # obstáculos descobertos dinamicamente (vazio para caso estático)
    spoofed_oy = []
    
    print("Calculando caminho...")
    # Executa o planejamento usando o método main()
    path_found, path_x, path_y = dstar.main(
        start=start_node,
        goal=goal_node,
        spoofed_ox=spoofed_ox,
        spoofed_oy=spoofed_oy
    )
    
    if not path_found:
        raise RuntimeError("D* Lite não conseguiu encontrar um caminho até o objetivo.")
    
    print(f"Caminho encontrado com {len(path_x)} pontos!")
    
except Exception as e:
    print(f"Erro durante o planejamento: {e}")
    raise

# Visualiza o resultado
print("\nGerando visualização...")
plt.figure(figsize=(12, 10))
plt.imshow(img, cmap='gray')
plt.plot(path_x, path_y, '-r', linewidth=2, label='D* Lite Path')
plt.scatter(ix_start, iy_start, c='g', s=100, label='Start', zorder=5)
plt.scatter(ix_goal, iy_goal, c='b', s=100, label='Goal', zorder=5)

# Zoom leve para visualizar melhor o caminho
margin = 300
xmin = min(ix_start, ix_goal) - margin
xmax = max(ix_start, ix_goal) + margin
ymin = min(iy_start, iy_goal) - margin
ymax = max(iy_start, iy_goal) + margin

plt.xlim(xmin, xmax)
plt.ylim(ymax, ymin)  # invertido porque a origem da imagem é no canto superior esquerdo

plt.legend()
plt.title('D* Lite Path Planning Result')
plt.savefig('dstar_path_result.png', dpi=150, bbox_inches='tight')
print("Imagem salva como 'dstar_path_result.png'.")

# Salva rota em CSV
print("Salvando caminho em CSV...")
with open('rota_dstar.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for px, py in zip(path_x, path_y):
        x, y = map_to_world(px, py)
        writer.writerow([x, y])

print("Caminho salvo como 'rota_dstar.csv'.")
print("\n" + "="*50)
print("Concluído!")
print("="*50)

